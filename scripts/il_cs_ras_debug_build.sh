#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
NCS_TOP="${IL_CS_NCS_TOP:-${HOME}/ncs/ncs-v3.2.3}"
NCS_VENV="${IL_CS_NCS_VENV:-${HOME}/ncs/.venv/bin/activate}"
INITIATOR_APP="${NCS_TOP}/nrf/samples/bluetooth/channel_sounding_ras_initiator"
DEBUG_DIR="${REPO_ROOT}/LOCATOR/zephyr_apps/current/il_cs_ras_debug"
BUILD_DIR="${REPO_ROOT}/build/il_cs_ras_debug_initiator"
BOARD="nrf54l15dk/nrf54l15/cpuapp"

if [[ $# -ne 0 ]]; then
  echo "Usage: ./scripts/il_cs_ras_debug_build.sh" >&2
  exit 2
fi
for path in "${NCS_VENV}" "${INITIATOR_APP}/src/main.c" \
            "${DEBUG_DIR}/initiator.patch" "${DEBUG_DIR}/il_ras_diag.h" \
            "${DEBUG_DIR}/prj_debug.conf"; do
  if [[ ! -e "${path}" ]]; then
    echo "ERROR: required path not found: ${path}" >&2
    exit 1
  fi
done

mkdir -p "${REPO_ROOT}/build"
STAGE="$(mktemp -d "${REPO_ROOT}/build/il_cs_ras_debug_source.XXXXXX")"
echo "Staging NCS RAS Initiator at ${STAGE}"
cp -a "${INITIATOR_APP}/." "${STAGE}/"
patch --batch --fuzz=0 -d "${STAGE}" -p1 < "${DEBUG_DIR}/initiator.patch"
cp "${DEBUG_DIR}/il_ras_diag.h" "${STAGE}/src/il_ras_diag.h"
cat "${DEBUG_DIR}/prj_debug.conf" >> "${STAGE}/prj.conf"

# shellcheck disable=SC1090
source "${NCS_VENV}"
cd "${NCS_TOP}"
west build -p always -b "${BOARD}" "${STAGE}" -d "${BUILD_DIR}"

echo "Diagnostic LOCATOR: ${BUILD_DIR}/merged.hex"
echo "Original TAG firmware remains in build/il_cs_ras_reflector/merged.hex"
echo "Generated source kept at ${STAGE} for reproducible incremental checks."
