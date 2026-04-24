//! HMAC-SHA256 request signer for the Onshape REST API.
//!
//! Reference: <https://onshape-public.github.io/docs/auth/apikeys/>.
//!
//! Each request must carry these headers:
//! - `Date`            RFC-1123 timestamp, within 5 minutes of the server clock
//! - `On-Nonce`        random alphanumeric (≥ 16 chars, we use 25)
//! - `Content-Type`    whatever the request actually sends
//! - `Authorization`   `On <accessKey>:HmacSHA256:<base64 sig>`
//!
//! The string to sign is `method\nnonce\ndate\ncontent-type\npath\nquery\n`
//! **lowercased**, where `query` is byte-identical to what is sent.

use anyhow::{anyhow, Result};
use base64::Engine;
use hmac::{Hmac, Mac};
use rand::Rng;
use reqwest::header::{HeaderMap, HeaderValue, AUTHORIZATION, CONTENT_TYPE, DATE};
use sha2::Sha256;
use std::time::SystemTime;

type HmacSha256 = Hmac<Sha256>;

pub struct Credentials {
    pub access_key: String,
    pub secret_key: String,
}

impl Credentials {
    pub fn from_env() -> Result<Self> {
        let access_key = std::env::var("ONSHAPE_ACCESS_KEY")
            .map_err(|_| anyhow!("ONSHAPE_ACCESS_KEY env var is missing"))?;
        let secret_key = std::env::var("ONSHAPE_SECRET_KEY")
            .map_err(|_| anyhow!("ONSHAPE_SECRET_KEY env var is missing"))?;
        Ok(Self { access_key, secret_key })
    }
}

fn random_nonce() -> String {
    const ALPHABET: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
    let mut rng = rand::thread_rng();
    (0..25)
        .map(|_| ALPHABET[rng.gen_range(0..ALPHABET.len())] as char)
        .collect()
}

/// Build all four signed headers for a request. `path` must start with `/` and
/// `query` (may be empty) must byte-match what will actually hit the wire.
pub fn signed_headers(
    creds: &Credentials,
    method: &str,
    path: &str,
    query: &str,
    content_type: &str,
) -> Result<HeaderMap> {
    let nonce = random_nonce();
    let date = httpdate::fmt_http_date(SystemTime::now());

    let string_to_sign = format!(
        "{}\n{}\n{}\n{}\n{}\n{}\n",
        method, nonce, date, content_type, path, query
    )
    .to_lowercase();

    let mut mac = HmacSha256::new_from_slice(creds.secret_key.as_bytes())
        .map_err(|e| anyhow!("hmac key: {e}"))?;
    mac.update(string_to_sign.as_bytes());
    let sig = base64::engine::general_purpose::STANDARD.encode(mac.finalize().into_bytes());

    let auth_value = format!("On {}:HmacSHA256:{}", creds.access_key, sig);

    let mut headers = HeaderMap::new();
    headers.insert(DATE, HeaderValue::from_str(&date)?);
    headers.insert("On-Nonce", HeaderValue::from_str(&nonce)?);
    headers.insert(CONTENT_TYPE, HeaderValue::from_str(content_type)?);
    headers.insert(AUTHORIZATION, HeaderValue::from_str(&auth_value)?);
    Ok(headers)
}
