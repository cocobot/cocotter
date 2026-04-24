#!/usr/bin/env bash
# Serve the repo root over HTTP so the threejs viewer (tool/...) can fetch
# the sim assets (sim/...). Browser file:// can't cross-fetch .glb files.
#
# Usage:
#   ./scripts/serve.sh [port]         # default 8000

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
PORT="${1:-8000}"

VIEWER="tool/onshape_fetch/viewer/viewer.html"
DEFAULT_GLB="sim/assets/robots/galipeur.glb"

cd "${REPO_ROOT}"
echo
echo "serving ${REPO_ROOT}"
echo "  → http://localhost:${PORT}/${VIEWER}"
echo "  → http://localhost:${PORT}/${VIEWER}?url=/${DEFAULT_GLB}"
echo
exec python3 -m http.server "${PORT}"
