#!/usr/bin/env bash
set -euo pipefail

# TAG flash script for ILGA
# Build dir:
#   build/tag_nrf54l15_port
# Notes:
# - Run ./scripts/tag_build.sh first
# - Extra args are passed through to "west flash"
#   Example:
#     ./scripts/tag_flash.sh --recover
#     ./scripts/tag_flash.sh --runner jlink
#     ./scripts/tag_flash.sh --runner nrfutil

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

NCS_VENV="${HOME}/ncs/.venv/bin/activate"
NCS_TOP="${HOME}/ncs/ncs-v3.2.3"
BUILD_DIR="${REPO_ROOT}/build/tag_nrf54l15_port"

if [[ ! -f "${NCS_VENV}" ]]; then
  echo "ERROR: NCS venv not found: ${NCS_VENV}" >&2
  exit 1
fi

if [[ ! -d "${NCS_TOP}" ]]; then
  echo "ERROR: NCS workspace not found: ${NCS_TOP}" >&2
  exit 1
fi

if [[ ! -d "${BUILD_DIR}" ]]; then
  echo "ERROR: build dir not found: ${BUILD_DIR}" >&2
  echo "Run ./scripts/tag_build.sh first." >&2
  exit 1
fi

if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]]; then
  echo "ERROR: build dir exists but does not look initialized: ${BUILD_DIR}" >&2
  echo "Run ./scripts/tag_build.sh first." >&2
  exit 1
fi

# shellcheck disable=SC1090
source "${NCS_VENV}"

cd "${NCS_TOP}"

echo "=== ILGA TAG flash start ==="
echo "BUILD_DIR : ${BUILD_DIR}"
if [[ $# -gt 0 ]]; then
  echo "EXTRA ARGS: $*"
fi
echo

west flash -d "${BUILD_DIR}" "$@"

echo
echo "=== ILGA TAG flash done ==="
