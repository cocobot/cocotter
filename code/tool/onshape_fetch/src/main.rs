//! Fetch a robot model from an Onshape Assembly as glTF (.glb).
//!
//! Set `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY` in the environment
//! (https://dev-portal.onshape.com/keys to create a pair).
//!
//! API-call discipline: Onshape quotas are small (a few thousand calls
//! total in the free tier). This tool avoids wasted calls by:
//! - Caching the raw Onshape .glb in `<out>.raw` — re-runs skip the
//!   download and simply re-apply the filter (0 API calls). Change your
//!   `--exclude`/`--include` lists freely without re-hitting Onshape.
//! - Persisting the translation id in `<out>.pending` so a re-run resumes
//!   the existing job instead of creating a new one.
//! - Polling with exponential back-off (`--poll-start-secs` → `--poll-max-secs`).
//! - `--force` to ignore both caches and start a fresh translation.

mod api;
mod auth;
mod filter;
mod unpack;
mod url;

use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::{Context, Result};
use clap::Parser;

use crate::api::{api_calls_so_far, Api};
use crate::auth::Credentials;

#[derive(Parser, Debug)]
#[command(name = "onshape_fetch", about = "Fetch a glTF model from an Onshape document")]
struct Cli {
    /// Full Onshape document URL (workspace or version).
    #[arg(long)]
    url: String,

    /// Output .glb path.
    #[arg(long)]
    out: PathBuf,

    /// Glob pattern matching node names to exclude from the output
    /// (repeatable).
    #[arg(long = "exclude")]
    exclude: Vec<String>,

    /// Glob pattern matching node names to include (repeatable). Empty list
    /// means "include everything that is not excluded".
    #[arg(long = "include")]
    include: Vec<String>,

    /// Maximum total time to wait for a translation to complete.
    #[arg(long, default_value_t = 600)]
    timeout_secs: u64,

    /// Initial poll interval in seconds (doubles up to `--poll-max-secs`).
    #[arg(long, default_value_t = 30)]
    poll_start_secs: u64,

    /// Maximum poll interval in seconds.
    #[arg(long, default_value_t = 60)]
    poll_max_secs: u64,

    /// Re-fetch even when the output file already exists.
    #[arg(long)]
    force: bool,

    /// Flatten the Onshape assembly hierarchy (all parts become top-level
    /// siblings in the glTF). Off by default — preserving the hierarchy
    /// lets `--exclude` patterns cascade through children.
    #[arg(long)]
    flatten: bool,

    /// Onshape API base URL (override for enterprise installs).
    #[arg(long, default_value = "https://cad.onshape.com")]
    base_url: String,
}

fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let cli = Cli::parse();

    let raw_path = raw_cache_path(&cli.out);
    let pending_path = pending_path(&cli.out);

    if cli.force {
        let _ = std::fs::remove_file(&raw_path);
        let _ = std::fs::remove_file(&pending_path);
    }

    // Path 1 — raw cache hit: re-apply filter only, no API calls.
    let bytes = if raw_path.exists() {
        log::info!("re-using raw cache {:?} (0 API calls this run)", raw_path);
        std::fs::read(&raw_path).with_context(|| format!("reading {:?}", raw_path))?
    } else {
        // Path 2 — fetch from Onshape.
        let r = url::parse(&cli.url).context("parsing Onshape URL")?;
        log::info!(
            "parsed: did={} {}={} eid={}",
            r.did, r.wvm, r.wvmid, r.eid
        );
        let creds = Credentials::from_env()?;
        let api = Api::new(cli.base_url.clone(), creds)?;

        // Resume an existing translation if one is pending.
        let translation_id = if pending_path.exists() {
            let id = std::fs::read_to_string(&pending_path)
                .with_context(|| format!("reading {:?}", pending_path))?
                .trim()
                .to_string();
            log::info!("resuming pending translation {id} (from {:?})", pending_path);
            id
        } else {
            let id = api.start_assembly_gltf_translation(&r, cli.flatten)?;
            std::fs::create_dir_all(pending_path.parent().unwrap_or(Path::new(".")))?;
            std::fs::write(&pending_path, &id)?;
            log::info!("new translation {id} (cached in {:?})", pending_path);
            id
        };

        let result = api.wait_and_download_translation(
            &r.did,
            &translation_id,
            Duration::from_secs(cli.timeout_secs),
            Duration::from_secs(cli.poll_start_secs),
            Duration::from_secs(cli.poll_max_secs),
        );
        // A FAILED translation is terminal — drop the cached id so a
        // subsequent run creates a fresh job instead of re-polling a dead
        // one.
        let _ = std::fs::remove_file(&pending_path);
        let bytes = result?;
        log::info!("downloaded {} bytes (API calls so far: {})", bytes.len(), api_calls_so_far());

        // Onshape wraps the export in a ZIP when ancillary files (buffers,
        // textures) are present. Unpack right here so the cache is always
        // a raw .glb.
        let glb = if unpack::is_zip(&bytes) {
            log::info!(
                "response is a ZIP ({} bytes); extracting glb",
                bytes.len()
            );
            unpack::extract_glb_from_zip(&bytes)?
        } else {
            bytes
        };

        // Save the extracted glb so later runs can re-filter without
        // hitting the API.
        if let Some(parent) = raw_path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(&raw_path, &glb)
            .with_context(|| format!("caching raw glb at {:?}", raw_path))?;
        glb
    };

    let (filtered, fmt) = if cli.exclude.is_empty() && cli.include.is_empty() {
        (bytes.clone(), filter::detect_format(&bytes))
    } else {
        let (out, fmt) = filter::filter(&bytes, &cli.include, &cli.exclude)?;
        log::info!("post-filter size: {} bytes", out.len());
        (out, fmt)
    };

    // Compact: if the payload is JSON .gltf with data-URI buffers, convert
    // it to a .glb binary — smaller file, faster load, Bevy-friendly.
    let (final_bytes, final_fmt) = match fmt {
        filter::GltfFormat::Gltf => {
            log::info!("compacting .gltf (base64) → .glb (binary)");
            let glb = filter::gltf_json_to_glb(&filtered)?;
            log::info!("compacted size: {} bytes", glb.len());
            (glb, filter::GltfFormat::Glb)
        }
        filter::GltfFormat::Glb => (filtered, filter::GltfFormat::Glb),
    };

    let out_path = match final_fmt {
        filter::GltfFormat::Glb => cli.out.with_extension("glb"),
        filter::GltfFormat::Gltf => cli.out.with_extension("gltf"),
    };
    if out_path != cli.out {
        log::info!("writing to {:?}", out_path);
    }

    if let Some(parent) = out_path.parent() {
        std::fs::create_dir_all(parent)
            .with_context(|| format!("creating {parent:?}"))?;
    }
    std::fs::write(&out_path, &final_bytes)
        .with_context(|| format!("writing {:?}", out_path))?;
    log::info!(
        "wrote {:?}  (total API calls this run: {})",
        out_path, api_calls_so_far()
    );
    Ok(())
}

fn pending_path(out: &Path) -> PathBuf {
    sibling(out, ".pending")
}

fn raw_cache_path(out: &Path) -> PathBuf {
    sibling(out, ".raw")
}

fn sibling(out: &Path, suffix: &str) -> PathBuf {
    let mut p = out.to_path_buf();
    let mut stem = p
        .file_name()
        .map(|s| s.to_os_string())
        .unwrap_or_default();
    stem.push(suffix);
    p.set_file_name(stem);
    p
}
