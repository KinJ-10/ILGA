#!/usr/bin/env bash
set -euo pipefail

# TAG build script for ILGA
# Target:
#   TAG/zephyr_apps/current/nrf54l15_port
# Board:
#   nrf54l15dk/nrf54l15/cpuapp

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

NCS_VENV="${HOME}/ncs/.venv/bin/activate"
NCS_TOP="${HOME}/ncs/ncs-v3.2.3"
APP_DIR="${REPO_ROOT}/TAG/zephyr_apps/current/nrf54l15_port"
BUILD_DIR="${REPO_ROOT}/build/tag_nrf54l15_port"
BOARD="nrf54l15dk/nrf54l15/cpuapp"

if [[ ! -f "${NCS_VENV}" ]]; then
  echo "ERROR: NCS venv not found: ${NCS_VENV}" >&2
  exit 1
fi

if [[ ! -d "${NCS_TOP}" ]]; then
  echo "ERROR: NCS workspace not found: ${NCS_TOP}" >&2
  exit 1
fi

if [[ ! -d "${APP_DIR}" ]]; then
  echo "ERROR: TAG app dir not found: ${APP_DIR}" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "${NCS_VENV}"

cd "${NCS_TOP}"

echo "=== ILGA TAG build start ==="
echo "BOARD     : ${BOARD}"
echo "APP_DIR   : ${APP_DIR}"
echo "BUILD_DIR : ${BUILD_DIR}"
echo

west build \
  -p always \
  -b "${BOARD}" \
  "${APP_DIR}" \
  -d "${BUILD_DIR}"

echo
echo "=== ILGA TAG build done ==="
echo "ELF : ${BUILD_DIR}/nrf54l15_port/zephyr/zephyr.elf"
echo "HEX : ${BUILD_DIR}/merged.hex"
