//! Post-process a downloaded `.glb`: drop scene nodes whose name matches an
//! exclude glob (or don't match an include list).
//!
//! We do NOT compact buffers — orphaned accessor/mesh data stays in the
//! file, which wastes a few KB but keeps the implementation much simpler
//! (no rewriting of offsets).

use anyhow::{anyhow, bail, Context, Result};
use base64::Engine;
use globset::{Glob, GlobSet, GlobSetBuilder};
use serde_json::Value;

const GLB_MAGIC: u32 = 0x46546C67; // "glTF"
const CHUNK_JSON: u32 = 0x4E4F534A; // "JSON"
const CHUNK_BIN: u32 = 0x004E4942; // "BIN\0"

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum GltfFormat {
    /// `.glb` — 12-byte header + JSON chunk [+ BIN chunk].
    Glb,
    /// `.gltf` — standalone JSON document (buffers embedded as data URIs
    /// or external file references).
    Gltf,
}

/// Sniff the format of a glTF payload by looking at its first bytes.
pub fn detect_format(bytes: &[u8]) -> GltfFormat {
    if bytes.len() >= 4 && u32::from_le_bytes(bytes[0..4].try_into().unwrap()) == GLB_MAGIC {
        GltfFormat::Glb
    } else {
        GltfFormat::Gltf
    }
}

/// Apply node filtering to either a .glb or a .gltf payload. The output
/// format matches the input.
pub fn filter(
    input: &[u8],
    include: &[String],
    exclude: &[String],
) -> Result<(Vec<u8>, GltfFormat)> {
    match detect_format(input) {
        GltfFormat::Glb => Ok((filter_glb(input, include, exclude)?, GltfFormat::Glb)),
        GltfFormat::Gltf => Ok((filter_gltf_json(input, include, exclude)?, GltfFormat::Gltf)),
    }
}

/// Filter a standalone `.gltf` JSON document. Buffers (possibly embedded as
/// data URIs) are left intact — only the node tree is pruned.
pub fn filter_gltf_json(
    input: &[u8],
    include: &[String],
    exclude: &[String],
) -> Result<Vec<u8>> {
    let mut doc: Value = serde_json::from_slice(input).context("parsing .gltf JSON")?;
    apply_node_filter(&mut doc, include, exclude)?;
    Ok(serde_json::to_vec(&doc)?)
}

/// Convert a `.gltf` JSON document (buffers embedded as data URIs) into a
/// compacted `.glb` binary:
/// - Walks scenes → nodes to find meshes / accessors / bufferViews that
///   are actually rendered.
/// - Drops every unused accessor, bufferView, mesh, and their data from
///   the combined BIN chunk.
/// - Renumbers references.
///
/// Big win on Onshape exports, which emit one accessor per primitive face
/// (~149 k entries on the galipeur) vast majority of which become orphan
/// once you filter nodes out.
pub fn gltf_json_to_glb(input: &[u8]) -> Result<Vec<u8>> {
    use std::collections::{HashMap, HashSet};

    let mut doc: Value = serde_json::from_slice(input).context("parsing .gltf JSON")?;

    // --- 1. Decode all buffers into a single linear blob. ---
    let mut combined = Vec::<u8>::new();
    let mut buffer_start: Vec<usize> = Vec::new();
    if let Some(buffers) = doc.get("buffers").and_then(|v| v.as_array()) {
        for (i, buf) in buffers.iter().enumerate() {
            buffer_start.push(combined.len());
            let uri = buf
                .get("uri")
                .and_then(|v| v.as_str())
                .ok_or_else(|| anyhow!("buffer {i} has no URI"))?;
            let decoded = decode_data_uri_base64(uri)
                .ok_or_else(|| anyhow!("buffer {i} has a non-base64 URI: {uri}"))?;
            combined.extend_from_slice(&decoded);
            while combined.len() % 4 != 0 {
                combined.push(0);
            }
        }
    }

    let empty = Vec::<Value>::new();
    let nodes = doc.get("nodes").and_then(|v| v.as_array()).unwrap_or(&empty).clone();
    let scenes = doc.get("scenes").and_then(|v| v.as_array()).unwrap_or(&empty).clone();
    let meshes = doc.get("meshes").and_then(|v| v.as_array()).unwrap_or(&empty).clone();
    let accessors = doc.get("accessors").and_then(|v| v.as_array()).unwrap_or(&empty).clone();
    let views = doc.get("bufferViews").and_then(|v| v.as_array()).unwrap_or(&empty).clone();

    // --- 2. Reachable nodes (from every scene root). ---
    let mut reachable_nodes: HashSet<usize> = HashSet::new();
    let mut stack: Vec<usize> = Vec::new();
    for scene in &scenes {
        if let Some(list) = scene.get("nodes").and_then(|v| v.as_array()) {
            for n in list {
                if let Some(i) = n.as_u64() {
                    stack.push(i as usize);
                }
            }
        }
    }
    while let Some(n) = stack.pop() {
        if !reachable_nodes.insert(n) {
            continue;
        }
        if let Some(node) = nodes.get(n) {
            if let Some(children) = node.get("children").and_then(|v| v.as_array()) {
                for c in children {
                    if let Some(i) = c.as_u64() {
                        stack.push(i as usize);
                    }
                }
            }
        }
    }

    // --- 3. Used meshes, accessors, bufferViews. ---
    let mut used_meshes: HashSet<usize> = HashSet::new();
    for &ni in &reachable_nodes {
        if let Some(mid) = nodes.get(ni).and_then(|n| n.get("mesh")).and_then(|v| v.as_u64()) {
            used_meshes.insert(mid as usize);
        }
    }
    let mut used_accessors: HashSet<usize> = HashSet::new();
    for &mi in &used_meshes {
        if let Some(prims) = meshes.get(mi).and_then(|m| m.get("primitives")).and_then(|v| v.as_array()) {
            for prim in prims {
                if let Some(attrs) = prim.get("attributes").and_then(|v| v.as_object()) {
                    for v in attrs.values() {
                        if let Some(i) = v.as_u64() {
                            used_accessors.insert(i as usize);
                        }
                    }
                }
                if let Some(idx) = prim.get("indices").and_then(|v| v.as_u64()) {
                    used_accessors.insert(idx as usize);
                }
            }
        }
    }
    let mut used_views: HashSet<usize> = HashSet::new();
    for &ai in &used_accessors {
        if let Some(v) = accessors.get(ai).and_then(|a| a.get("bufferView")).and_then(|v| v.as_u64()) {
            used_views.insert(v as usize);
        }
    }

    log::info!(
        "compaction: nodes {}/{} reachable, meshes {}/{} kept, accessors {}/{} kept, bufferViews {}/{} kept",
        reachable_nodes.len(), nodes.len(),
        used_meshes.len(), meshes.len(),
        used_accessors.len(), accessors.len(),
        used_views.len(), views.len(),
    );

    // --- 4. Renumber maps for meshes, accessors, views. ---
    let mesh_map: HashMap<usize, usize> = renumber(meshes.len(), |i| used_meshes.contains(&i));
    let accessor_map: HashMap<usize, usize> = renumber(accessors.len(), |i| used_accessors.contains(&i));
    let view_map: HashMap<usize, usize> = renumber(views.len(), |i| used_views.contains(&i));

    // --- 5. Compact BIN: keep only the slices used by kept bufferViews. ---
    let mut new_bin = Vec::<u8>::new();
    let mut new_views: Vec<Value> = Vec::with_capacity(used_views.len());
    // Iterate in original order so view_map ordering matches.
    for (old_i, view) in views.iter().enumerate() {
        if !used_views.contains(&old_i) {
            continue;
        }
        let old_buf = view.get("buffer").and_then(|v| v.as_u64()).unwrap_or(0) as usize;
        let old_off = view.get("byteOffset").and_then(|v| v.as_u64()).unwrap_or(0) as usize;
        let len = view.get("byteLength").and_then(|v| v.as_u64()).unwrap_or(0) as usize;
        let src = buffer_start.get(old_buf).copied().unwrap_or(0) + old_off;
        let slice = combined
            .get(src..src + len)
            .ok_or_else(|| anyhow!("bufferView {old_i} out of bounds in combined BIN"))?;
        let new_off = new_bin.len();
        new_bin.extend_from_slice(slice);
        while new_bin.len() % 4 != 0 {
            new_bin.push(0);
        }
        let mut nv = view.clone();
        nv["buffer"] = serde_json::json!(0);
        nv["byteOffset"] = serde_json::json!(new_off);
        new_views.push(nv);
    }

    // --- 6. Accessors: drop orphans, remap bufferView index. ---
    let new_accessors: Vec<Value> = accessors
        .iter()
        .enumerate()
        .filter_map(|(i, a)| {
            if !used_accessors.contains(&i) {
                return None;
            }
            let mut na = a.clone();
            if let Some(v) = na.get("bufferView").and_then(|v| v.as_u64()) {
                if let Some(new_v) = view_map.get(&(v as usize)) {
                    na["bufferView"] = serde_json::json!(new_v);
                }
            }
            Some(na)
        })
        .collect();

    // --- 7. Meshes: drop orphans, remap accessor indices inside primitives. ---
    let new_meshes: Vec<Value> = meshes
        .iter()
        .enumerate()
        .filter_map(|(i, m)| {
            if !used_meshes.contains(&i) {
                return None;
            }
            let mut nm = m.clone();
            if let Some(prims) = nm.get_mut("primitives").and_then(|v| v.as_array_mut()) {
                for prim in prims.iter_mut() {
                    if let Some(attrs) = prim.get_mut("attributes").and_then(|v| v.as_object_mut()) {
                        for v in attrs.values_mut() {
                            if let Some(i) = v.as_u64() {
                                if let Some(new) = accessor_map.get(&(i as usize)) {
                                    *v = serde_json::json!(new);
                                }
                            }
                        }
                    }
                    if let Some(idx) = prim.get("indices").and_then(|v| v.as_u64()) {
                        if let Some(new) = accessor_map.get(&(idx as usize)) {
                            prim["indices"] = serde_json::json!(new);
                        }
                    }
                }
            }
            Some(nm)
        })
        .collect();

    // --- 8. Nodes: remap mesh refs; drop refs to dropped meshes. ---
    let mut new_nodes = nodes.clone();
    for node in new_nodes.iter_mut() {
        if let Some(mid) = node.get("mesh").and_then(|v| v.as_u64()) {
            match mesh_map.get(&(mid as usize)) {
                Some(new) => node["mesh"] = serde_json::json!(new),
                None => {
                    if let Some(obj) = node.as_object_mut() {
                        obj.remove("mesh");
                    }
                }
            }
        }
    }

    // --- 9. Stitch back into the document. ---
    doc["nodes"] = Value::Array(new_nodes);
    doc["meshes"] = Value::Array(new_meshes);
    doc["accessors"] = Value::Array(new_accessors);
    doc["bufferViews"] = Value::Array(new_views);
    doc["buffers"] = serde_json::json!([{"byteLength": new_bin.len()}]);

    let json_out = serde_json::to_vec(&doc)?;
    write_glb(&json_out, Some(new_bin))
}

fn renumber<F: Fn(usize) -> bool>(n: usize, keep: F) -> std::collections::HashMap<usize, usize> {
    let mut m = std::collections::HashMap::new();
    let mut next = 0usize;
    for i in 0..n {
        if keep(i) {
            m.insert(i, next);
            next += 1;
        }
    }
    m
}

fn decode_data_uri_base64(uri: &str) -> Option<Vec<u8>> {
    // Accept any `data:<mime>;base64,<b64>` form.
    let after_colon = uri.strip_prefix("data:")?;
    let (_mime, rest) = after_colon.split_once(',')?;
    if !_mime.contains(";base64") {
        return None;
    }
    base64::engine::general_purpose::STANDARD.decode(rest).ok()
}

pub fn filter_glb(input: &[u8], include: &[String], exclude: &[String]) -> Result<Vec<u8>> {
    let (json_bytes, bin_chunk) = parse_glb(input)?;
    let mut doc: Value = serde_json::from_slice(&json_bytes)?;
    apply_node_filter(&mut doc, include, exclude)?;
    let new_json = serde_json::to_vec(&doc)?;
    write_glb(&new_json, bin_chunk)
}

fn apply_node_filter(doc: &mut Value, include: &[String], exclude: &[String]) -> Result<()> {
    let include_set = build_globset(include)?;
    let exclude_set = build_globset(exclude)?;

    let nodes = doc
        .get("nodes")
        .and_then(|v| v.as_array())
        .cloned()
        .unwrap_or_default();
    let mut drop = vec![false; nodes.len()];
    for (i, n) in nodes.iter().enumerate() {
        let name = n.get("name").and_then(|v| v.as_str()).unwrap_or("");
        let excluded = !exclude.is_empty() && exclude_set.is_match(name);
        let included = include.is_empty() || include_set.is_match(name);
        drop[i] = excluded || !included;
    }

    let dropped_count = drop.iter().filter(|&&d| d).count();
    if dropped_count == 0 {
        log::info!("no nodes filtered (include={include:?}, exclude={exclude:?})");
    } else {
        log::info!("filtering {dropped_count}/{} nodes", drop.len());
        // Sample up to 20 dropped names so the user can sanity-check the
        // exclude patterns; enable RUST_LOG=debug for the full list.
        let dropped_names: Vec<&str> = drop
            .iter()
            .enumerate()
            .filter_map(|(i, &d)| {
                if !d { return None; }
                nodes.get(i).and_then(|n| n.get("name")).and_then(|v| v.as_str())
            })
            .collect();
        let sample: Vec<&&str> = dropped_names.iter().take(20).collect();
        log::info!("dropped sample: {:?}", sample);
        log::debug!("dropped (full list): {:?}", dropped_names);
    }

    if let Some(scenes) = doc.get_mut("scenes").and_then(|v| v.as_array_mut()) {
        for scene in scenes {
            if let Some(list) = scene.get_mut("nodes").and_then(|v| v.as_array_mut()) {
                list.retain(|idx| {
                    !matches!(idx.as_u64(), Some(i) if drop.get(i as usize).copied().unwrap_or(false))
                });
            }
        }
    }
    if let Some(nodes_mut) = doc.get_mut("nodes").and_then(|v| v.as_array_mut()) {
        for node in nodes_mut.iter_mut() {
            if let Some(children) = node.get_mut("children").and_then(|v| v.as_array_mut()) {
                children.retain(|idx| {
                    !matches!(idx.as_u64(), Some(i) if drop.get(i as usize).copied().unwrap_or(false))
                });
            }
        }
    }
    Ok(())
}

fn build_globset(patterns: &[String]) -> Result<GlobSet> {
    let mut b = GlobSetBuilder::new();
    for p in patterns {
        b.add(Glob::new(p)?);
    }
    Ok(b.build()?)
}

fn parse_glb(input: &[u8]) -> Result<(Vec<u8>, Option<Vec<u8>>)> {
    if input.len() < 12 {
        bail!("not a glb (too short)");
    }
    let magic = u32::from_le_bytes(input[0..4].try_into().unwrap());
    if magic != GLB_MAGIC {
        bail!("not a glb (magic mismatch)");
    }
    let version = u32::from_le_bytes(input[4..8].try_into().unwrap());
    if version != 2 {
        bail!("unsupported glb version {version}");
    }
    let total_len = u32::from_le_bytes(input[8..12].try_into().unwrap()) as usize;
    if total_len > input.len() {
        bail!("truncated glb");
    }

    let mut cursor = 12;
    let mut json_chunk: Option<Vec<u8>> = None;
    let mut bin_chunk: Option<Vec<u8>> = None;

    while cursor + 8 <= total_len {
        let len = u32::from_le_bytes(input[cursor..cursor + 4].try_into().unwrap()) as usize;
        let ty = u32::from_le_bytes(input[cursor + 4..cursor + 8].try_into().unwrap());
        cursor += 8;
        if cursor + len > total_len {
            bail!("chunk overruns glb");
        }
        let data = input[cursor..cursor + len].to_vec();
        cursor += len;
        match ty {
            CHUNK_JSON => json_chunk = Some(data),
            CHUNK_BIN => bin_chunk = Some(data),
            other => log::debug!("ignoring glb chunk 0x{:08x}", other),
        }
    }
    let json = json_chunk.ok_or_else(|| anyhow!("glb has no JSON chunk"))?;
    Ok((json, bin_chunk))
}

fn write_glb(json: &[u8], bin: Option<Vec<u8>>) -> Result<Vec<u8>> {
    let json_padded = pad_to_4(json.to_vec(), 0x20); // space
    let bin_padded = bin.map(|b| pad_to_4(b, 0x00));

    let mut total: u32 = 12 + 8 + json_padded.len() as u32;
    if let Some(b) = &bin_padded {
        total += 8 + b.len() as u32;
    }

    let mut out = Vec::with_capacity(total as usize);
    out.extend_from_slice(&GLB_MAGIC.to_le_bytes());
    out.extend_from_slice(&2u32.to_le_bytes());
    out.extend_from_slice(&total.to_le_bytes());

    out.extend_from_slice(&(json_padded.len() as u32).to_le_bytes());
    out.extend_from_slice(&CHUNK_JSON.to_le_bytes());
    out.extend_from_slice(&json_padded);

    if let Some(b) = bin_padded {
        out.extend_from_slice(&(b.len() as u32).to_le_bytes());
        out.extend_from_slice(&CHUNK_BIN.to_le_bytes());
        out.extend_from_slice(&b);
    }

    Ok(out)
}

fn pad_to_4(mut v: Vec<u8>, fill: u8) -> Vec<u8> {
    while v.len() % 4 != 0 {
        v.push(fill);
    }
    v
}
