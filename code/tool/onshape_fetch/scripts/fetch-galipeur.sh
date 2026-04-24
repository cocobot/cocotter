#!/usr/bin/env bash
# Re-export the galipeur robot model from Onshape.
#
# Requires:
#   ONSHAPE_ACCESS_KEY, ONSHAPE_SECRET_KEY in the environment
#
# Usage:
#   ./scripts/fetch-galipeur.sh [extra --flags...]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOL_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${TOOL_DIR}/../.." && pwd)"

URL="https://cad.onshape.com/documents/6231618a00b10e8e63b83ca1/w/f02ecdb370dfe9baea8da063/e/ce716b319ea83399c9124a99"
OUT="${REPO_ROOT}/sim/assets/robots/galipeur.glb"

# Things we never want in the sim mesh. Globs match against glTF node
# names (mirror the Onshape instance / part names). Patterns below come
# from inspecting the actual export.
EXCLUDES=(
    --exclude "*PCB*"
    #--exclude "*pcb*"
    --exclude "RC0603N_VIS"
    --exclude "C_*"
    --exclude "D_*"
    --exclude "*SMD*"
    --exclude "*TSSOP*"
    --exclude "*SOIC_*"
    --exclude "*SOT_*"
    --exclude "*ESP32*"
    --exclude "*PLCC*"
    --exclude "00222*"
    --exclude "*1445050*"
    --exclude "*188275*"
    --exclude "*PinHeader*"
    --exclude "COMPOUND_1"

    #non robot parts
    --exclude "part"
    --exclude "Part 32"
    --exclude "Part 33"
    --exclude "Part 34"
    --exclude "Part 35"
    --exclude "Part 48"
    --exclude "Part 49"
    --exclude "Part 35"
    --exclude "Part 51"
    --exclude "Part 68"
    --exclude "Part 151"
    --exclude "Part 166"
    --exclude "support_galipeur"
    --exclude "encombrement"
    --exclude "sol"
    --exclude "Part 205"
)

# No key check here — the raw cache can be re-filtered offline. Keys are
# only required when the tool actually contacts Onshape, and the binary
# errors out clearly in that case.

cd "${TOOL_DIR}"
cargo run --release -- \
    --url "${URL}" \
    --out "${OUT}" \
    "${EXCLUDES[@]}" \
    "$@"

# Post-process: dedup / join / simplify / meshopt via gltf-transform.
# On the galipeur this shrinks the file by ~30×. Needs npx (Node.js).
# Set NO_OPTIMIZE=1 in the environment to skip.
if [[ -n "${NO_OPTIMIZE:-}" ]]; then
    echo "NO_OPTIMIZE set — skipping gltf-transform"
    exit 0
fi
if ! command -v npx >/dev/null 2>&1; then
    echo "npx not found — skipping gltf-transform (install Node.js to enable)"
    exit 0
fi

echo
echo "Running gltf-transform optimize (this can take a few minutes)..."
TMP="${OUT%.glb}.optimized.tmp.glb"

# Bevy 0.18's gltf loader does NOT support EXT_meshopt_compression
# (tracked in bevy issue #11350). `--compress false` skips the meshopt
# step; geometry simplify is kept (it just reduces triangle count without
# adding extensions). Set NO_OPTIMIZE=1 to skip entirely.
if npx --yes @gltf-transform/cli@latest optimize "${OUT}" "${TMP}" \
        --compress false; then
    mv "${TMP}" "${OUT}"
    echo "optimize done — final file: ${OUT} ($(stat -c%s "${OUT}") bytes)"
else
    echo "gltf-transform optimize failed — keeping the un-optimized file"
    rm -f "${TMP}"
    exit 1
fi
