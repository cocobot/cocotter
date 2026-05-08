#!/usr/bin/env bash
# Re-export the galipeur robot model from Onshape, split into one base
# .glb plus one .glb per articulated joint. The single Onshape assembly
# is fetched once (cached at $RAW_CACHE) and re-filtered N times — so
# the API quota cost is the same as the previous single-file pipeline.
#
# Configuration lives in galipeur-parts.json (shared with the viewer).
#
# Requires:
#   ONSHAPE_ACCESS_KEY, ONSHAPE_SECRET_KEY  in the environment
#   jq                for reading the JSON config
#   npx (Node.js)     for the gltf-transform optimize pipeline (optional)
#
# Env knobs:
#   NO_OPTIMIZE=1    -- skip gltf-transform (raw-filter only)
#   PARTS="base translation_back arm_back_0 ..."  -- space-separated
#                       subset of part names to (re-)build. Default = all.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOL_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${TOOL_DIR}/../.." && pwd)"

CONFIG="${SCRIPT_DIR}/galipeur-parts.json"

if ! command -v jq >/dev/null 2>&1; then
    echo "ERROR: jq is required. Install it with: sudo dnf install jq" >&2
    exit 1
fi

URL="$(jq -r '.url' "${CONFIG}")"
OUT_DIR="${REPO_ROOT}/$(jq -r '.out_dir' "${CONFIG}")"
RAW_CACHE="${REPO_ROOT}/$(jq -r '.raw_cache' "${CONFIG}")"

mkdir -p "${OUT_DIR}"

# Build --exclude flags from exclude_common array
EXCLUDES_COMMON=()
while IFS= read -r pat; do
    EXCLUDES_COMMON+=(--exclude "$pat")
done < <(jq -r '.exclude_common[]' "${CONFIG}")

# Read part names and their patterns from the JSON
PART_NAMES=()
while IFS= read -r name; do
    PART_NAMES+=("$name")
done < <(jq -r '.parts | keys[]' "${CONFIG}")

# Build --exclude flags for base (union of all part patterns)
EXCLUDES_BASE=()
while IFS= read -r pat; do
    EXCLUDES_BASE+=(--exclude "$pat")
done < <(jq -r '.parts[][] ' "${CONFIG}")

# Build the list of parts to (re-)export.
ALL_PARTS=(base "${PART_NAMES[@]}")
PARTS_TO_BUILD=(${PARTS:-${ALL_PARTS[@]}})

cd "${TOOL_DIR}"

fetch_part() {
    local name="$1"; shift
    local out="${OUT_DIR}/${name}.glb"
    echo
    echo "==> fetching ${name} -> ${out}"
    cargo run --release -- \
        --url "${URL}" \
        --out "${out}" \
        --cache "${RAW_CACHE}" \
        "${EXCLUDES_COMMON[@]}" \
        "$@"

    if [[ -f "${out}" ]]; then
        local size
        size=$(stat -c%s "${out}")
        if [[ "${size}" -lt 1024 ]]; then
            echo "  WARNING: ${name}.glb is only ${size} bytes — likely no nodes matched the include glob. Adjust the patterns."
        else
            echo "  ${name}.glb ${size} bytes"
        fi
    fi
}

for part in "${PARTS_TO_BUILD[@]}"; do
    if [[ "${part}" == "base" ]]; then
        fetch_part base "${EXCLUDES_BASE[@]}"
    else
        # Build --include flags for this part from the JSON
        INCLUDES=()
        while IFS= read -r pat; do
            INCLUDES+=(--include "$pat")
        done < <(jq -r --arg p "${part}" '.parts[$p][]' "${CONFIG}")
        if [[ ${#INCLUDES[@]} -eq 0 ]]; then
            echo "unknown part: ${part}" >&2
            exit 1
        fi
        fetch_part "${part}" "${INCLUDES[@]}"
    fi
done

# ---------- gltf-transform optimize per file ----------
if [[ -n "${NO_OPTIMIZE:-}" ]]; then
    echo "NO_OPTIMIZE set — skipping gltf-transform"
    exit 0
fi
if ! command -v npx >/dev/null 2>&1; then
    echo "npx not found — skipping gltf-transform (install Node.js to enable)"
    exit 0
fi

GLTF_TX="npx --yes @gltf-transform/cli@latest"

optimize_glb() {
    local path="$1"
    local label="$2"
    local tmp="${path%.glb}.tmp.glb"
    if [[ ! -f "${path}" ]]; then
        echo "  skipping ${label} — file missing"
        return 0
    fi
    if [[ "$(stat -c%s "${path}")" -lt 1024 ]]; then
        echo "  skipping ${label} — file too small (probably empty)"
        return 0
    fi
    run_step() {
        local stage="$1"; shift
        echo "  ${label} -> ${stage}"
        if $GLTF_TX "$@" "${path}" "${tmp}"; then
            mv "${tmp}" "${path}"
        else
            echo "     ${stage} failed for ${label}"
            rm -f "${tmp}"
            return 1
        fi
    }
    run_step "optimize"           optimize         --compress false   || return 1
    run_step "palette"            palette          --blockSize 8      || true
    run_step "flatten"            flatten                              || true
    run_step "join"               join                                 || true
    run_step "dedup"              dedup                                || true
    run_step "prune"              prune                                || true
    local size
    size=$(stat -c%s "${path}")
    echo "  ${label} done (${size} bytes)"
}

echo
echo "Running gltf-transform pipeline per part..."
for part in "${PARTS_TO_BUILD[@]}"; do
    optimize_glb "${OUT_DIR}/${part}.glb" "${part}"
done

echo
echo "all parts done under ${OUT_DIR}/"
ls -lh "${OUT_DIR}"/*.glb 2>/dev/null || true
