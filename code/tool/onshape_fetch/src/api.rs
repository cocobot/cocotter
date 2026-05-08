//! HTTP calls to the Onshape REST API.

use anyhow::{anyhow, bail, Context, Result};
use reqwest::blocking::Client;
use reqwest::redirect::Policy;
use reqwest::{StatusCode, Url};
use serde::Deserialize;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::{Duration, Instant};

use crate::auth::{signed_headers, Credentials};
use crate::url::OnshapeRef;

/// Running count of API requests made this process. Used to give the user a
/// clear idea of how much of their quota they are burning.
static API_CALLS: AtomicU32 = AtomicU32::new(0);

pub fn api_calls_so_far() -> u32 {
    API_CALLS.load(Ordering::Relaxed)
}

pub struct Api {
    pub base: String,
    pub creds: Credentials,
    pub client: Client,
}

impl Api {
    pub fn new(base: String, creds: Credentials) -> Result<Self> {
        // We follow 307s manually because Onshape's CDN requires re-signing
        // each hop.
        let client = Client::builder()
            .redirect(Policy::none())
            .timeout(Duration::from_secs(180))
            .build()?;
        Ok(Self { base, creds, client })
    }


    /// Kick off an assembly → glTF translation. Returns the translation id
    /// to poll for completion.
    ///
    /// `flatten=false` preserves the sub-assembly node hierarchy, so a
    /// glob like `*PCB*` also drops the PCB's children. `flatten=true` is
    /// safer with heterogeneous imports but collapses everything to
    /// top-level siblings.
    pub fn start_assembly_gltf_translation(
        &self,
        r: &OnshapeRef,
        flatten: bool,
    ) -> Result<String> {
        let path = format!(
            "/api/v6/assemblies/d/{}/{}/{}/e/{}/translations",
            r.did, r.wvm, r.wvmid, r.eid
        );
        let body = serde_json::json!({
            "formatName": "GLTF",
            "storeInDocument": false,
            "flattenAssemblies": flatten,
            "yAxisIsUp": true,
            "angularTolerance": 0.1090830782496456,
            "distanceTolerance": 0.00012,
            "maximumChordLength": 10.0,
        });
        let body_bytes = serde_json::to_vec(&body)?;
        let resp = self.do_signed_json("POST", &path, "", &body_bytes)?;
        #[derive(Deserialize)]
        struct TxStart { id: String }
        let start: TxStart = serde_json::from_slice(&resp)?;
        Ok(start.id)
    }

    /// Poll a translation until it completes, then download the result. The
    /// `poll_interval` starts at `poll_interval_initial` and doubles up to
    /// `poll_interval_max` — keeps API calls down on long-running exports.
    pub fn wait_and_download_translation(
        &self,
        did: &str,
        translation_id: &str,
        timeout: Duration,
        poll_interval_initial: Duration,
        poll_interval_max: Duration,
    ) -> Result<Vec<u8>> {
        #[derive(Deserialize)]
        struct TxStatus {
            #[serde(rename = "requestState")]
            request_state: String,
            #[serde(rename = "resultExternalDataIds", default, deserialize_with = "null_to_empty")]
            result_external_data_ids: Vec<String>,
            #[serde(rename = "failureReason", default)]
            failure_reason: Option<String>,
        }
        fn null_to_empty<'de, D>(de: D) -> Result<Vec<String>, D::Error>
        where
            D: serde::Deserializer<'de>,
        {
            let v: Option<Vec<String>> = Option::deserialize(de)?;
            Ok(v.unwrap_or_default())
        }

        let poll_path = format!("/api/v6/translations/{}", translation_id);
        let deadline = Instant::now() + timeout;
        let mut interval = poll_interval_initial;

        loop {
            log::info!("sleeping {:?} before next poll (API calls so far: {})",
                interval, api_calls_so_far());
            std::thread::sleep(interval);
            if Instant::now() > deadline {
                bail!(
                    "translation {translation_id} timed out after {:?} (still pending — re-run the script to resume)",
                    timeout
                );
            }
            let body = self.do_signed_json("GET", &poll_path, "", &[])?;
            let status: TxStatus = serde_json::from_slice(&body)?;
            match status.request_state.as_str() {
                "DONE" => {
                    let ext_id = status
                        .result_external_data_ids
                        .first()
                        .ok_or_else(|| anyhow!("translation DONE but no external data id"))?;
                    let dl_path = format!("/api/v6/documents/d/{}/externaldata/{}", did, ext_id);
                    return self.do_signed_binary(
                        "GET",
                        &dl_path,
                        "",
                        "application/json",
                        None,
                    );
                }
                "FAILED" => bail!(
                    "translation {translation_id} FAILED: {}",
                    status.failure_reason.unwrap_or_default()
                ),
                other => {
                    log::info!("translation {translation_id} state: {other}");
                    interval = (interval * 2).min(poll_interval_max);
                }
            }
        }
    }

    /// Signed GET/POST returning the response body as bytes. Follows a single
    /// 307 redirect manually (re-signed against the target host if it's still
    /// Onshape, plain GET otherwise).
    fn do_signed_binary(
        &self,
        method: &str,
        path: &str,
        query: &str,
        content_type: &str,
        body: Option<&[u8]>,
    ) -> Result<Vec<u8>> {
        let url = format!("{}{}{}{}", self.base, path,
            if query.is_empty() { "" } else { "?" }, query);
        let headers = signed_headers(&self.creds, method, path, query, content_type)?;
        let mut req = match method {
            "GET" => self.client.get(&url),
            "POST" => self.client.post(&url),
            m => bail!("unsupported method: {m}"),
        };
        req = req.headers(headers);
        if let Some(b) = body {
            req = req.body(b.to_vec());
        }
        API_CALLS.fetch_add(1, Ordering::Relaxed);
        let resp = req.send().with_context(|| format!("request to {url}"))?;
        let status = resp.status();

        if status.is_redirection() {
            // Follow manually. S3/CloudFront pre-signed URLs don't need our
            // Onshape auth.
            let loc = resp
                .headers()
                .get(reqwest::header::LOCATION)
                .ok_or_else(|| anyhow!("{status} without Location"))?
                .to_str()?
                .to_string();
            log::debug!("following redirect to {loc}");
            let redirected_url = Url::parse(&loc)?;
            let mut r2 = self.client.get(redirected_url.as_str());
            if redirected_url.host_str().unwrap_or("").contains("onshape.com") {
                let p = redirected_url.path();
                let q = redirected_url.query().unwrap_or("");
                let h = signed_headers(&self.creds, "GET", p, q, content_type)?;
                r2 = r2.headers(h);
            }
            let rr = r2.send()?;
            if !rr.status().is_success() {
                bail!("redirect target returned {}", rr.status());
            }
            return Ok(rr.bytes()?.to_vec());
        }

        if !status.is_success() {
            let body = resp.text().unwrap_or_default();
            bail!("{method} {path} → {status}: {body}");
        }
        Ok(resp.bytes()?.to_vec())
    }

    /// Like `do_signed_binary` but always takes / returns JSON bodies (no
    /// redirect expected). Also increments the API call counter.
    fn do_signed_json(
        &self,
        method: &str,
        path: &str,
        query: &str,
        body: &[u8],
    ) -> Result<Vec<u8>> {
        let url = format!("{}{}{}{}", self.base, path,
            if query.is_empty() { "" } else { "?" }, query);
        let headers = signed_headers(&self.creds, method, path, query, "application/json")?;
        let mut req = match method {
            "GET" => self.client.get(&url),
            "POST" => self.client.post(&url),
            m => bail!("unsupported method: {m}"),
        };
        req = req.headers(headers);
        if !body.is_empty() {
            req = req.body(body.to_vec());
        }
        API_CALLS.fetch_add(1, Ordering::Relaxed);
        let resp = req.send().with_context(|| format!("request to {url}"))?;
        let status = resp.status();
        let body = resp.bytes()?.to_vec();
        if status == StatusCode::OK {
            Ok(body)
        } else {
            bail!(
                "{method} {path} → {status}: {}",
                String::from_utf8_lossy(&body)
            )
        }
    }
}
