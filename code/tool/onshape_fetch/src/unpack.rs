//! Onshape's translation result is delivered as a ZIP when the output has
//! ancillary files (buffers, textures). For glTF-Binary exports we still
//! get a `.glb` inside the archive.

use anyhow::{anyhow, bail, Context, Result};
use std::io::{Cursor, Read};

/// Detect if `bytes` is a ZIP (PK\x03\x04 or PK\x05\x06 signature).
pub fn is_zip(bytes: &[u8]) -> bool {
    bytes.len() >= 4 && bytes[0] == b'P' && bytes[1] == b'K'
}

/// Return the first .glb entry inside the archive. If no .glb is found but
/// a .gltf is, returns an error suggesting the export mode may be wrong.
pub fn extract_glb_from_zip(bytes: &[u8]) -> Result<Vec<u8>> {
    let reader = Cursor::new(bytes);
    let mut zip = zip::ZipArchive::new(reader).context("parsing ZIP")?;

    let mut gltf_name: Option<String> = None;
    for i in 0..zip.len() {
        let file = zip.by_index(i)?;
        let name = file.name().to_string();
        let lower = name.to_lowercase();
        if lower.ends_with(".glb") {
            log::info!("extracting {name} from archive ({} bytes)", file.size());
            drop(file);
            let mut entry = zip.by_index(i)?;
            let mut out = Vec::with_capacity(entry.size() as usize);
            entry.read_to_end(&mut out)?;
            return Ok(out);
        }
        if lower.ends_with(".gltf") {
            gltf_name = Some(name);
        }
    }

    if let Some(n) = gltf_name {
        bail!(
            "archive contains a loose .gltf ({n}) and no .glb — request glTF-Binary instead"
        );
    }
    Err(anyhow!("archive contains neither a .glb nor a .gltf entry"))
}
