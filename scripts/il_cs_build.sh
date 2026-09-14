#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

NCS_VENV="${HOME}/ncs/.venv/bin/activate"
NCS_TOP="${HOME}/ncs/ncs-v3.2.3"
BOARD="nrf54l15dk/nrf54l15/cpuapp"
RAW_UART_BAUD="230400"

PROFILE="normal"
if [[ $# -gt 1 ]]; then
  echo "Usage: ./scripts/il_cs_build.sh [--raw-diagnostics]" >&2
  exit 2
fi
if [[ $# -eq 1 ]]; then
  if [[ "$1" != "--raw-diagnostics" ]]; then
    echo "Usage: ./scripts/il_cs_build.sh [--raw-diagnostics]" >&2
    exit 2
  fi
  PROFILE="raw-diagnostics"
fi

INITIATOR_APP="${REPO_ROOT}/LOCATOR/zephyr_apps/current/il_cs_initiator"
REFLECTOR_APP="${REPO_ROOT}/TAG/zephyr_apps/current/il_cs_reflector"
if [[ "${PROFILE}" == "raw-diagnostics" ]]; then
  INITIATOR_BUILD="${REPO_ROOT}/build/il_cs_initiator_raw"
  REFLECTOR_BUILD="${REPO_ROOT}/build/il_cs_reflector_raw"
else
  INITIATOR_BUILD="${REPO_ROOT}/build/il_cs_initiator"
  REFLECTOR_BUILD="${REPO_ROOT}/build/il_cs_reflector"
fi

for required_path in "${NCS_VENV}" "${NCS_TOP}" "${INITIATOR_APP}" "${REFLECTOR_APP}"; do
  if [[ ! -e "${required_path}" ]]; then
    echo "ERROR: required path not found: ${required_path}" >&2
    exit 1
  fi
done

# shellcheck disable=SC1090
source "${NCS_VENV}"
cd "${NCS_TOP}"

echo "=== ILGA Channel Sounding Phase 0 build ==="
echo "PROFILE         : ${PROFILE}"
echo "BOARD           : ${BOARD}"
if [[ "${PROFILE}" == "raw-diagnostics" ]]; then
  echo "CONSOLE_BAUD    : ${RAW_UART_BAUD}"
else
  echo "CONSOLE_BAUD    : 115200 (board default)"
fi
echo "INITIATOR_BUILD : ${INITIATOR_BUILD}"
echo "REFLECTOR_BUILD : ${REFLECTOR_BUILD}"

if [[ "${PROFILE}" == "raw-diagnostics" ]]; then
  west build -p always -b "${BOARD}" "${INITIATOR_APP}" -d "${INITIATOR_BUILD}" -- \
    -DEXTRA_CONF_FILE="${INITIATOR_APP}/raw_diagnostics.conf" \
    -DDTC_OVERLAY_FILE="${INITIATOR_APP}/raw_uart_230400.overlay"
  west build -p always -b "${BOARD}" "${REFLECTOR_APP}" -d "${REFLECTOR_BUILD}" -- \
    -DEXTRA_CONF_FILE="${REFLECTOR_APP}/raw_diagnostics.conf" \
    -DDTC_OVERLAY_FILE="${REFLECTOR_APP}/raw_uart_230400.overlay"
else
  west build -p always -b "${BOARD}" "${INITIATOR_APP}" -d "${INITIATOR_BUILD}"
  west build -p always -b "${BOARD}" "${REFLECTOR_APP}" -d "${REFLECTOR_BUILD}"
fi

echo "=== Build complete ==="
echo "Initiator: ${INITIATOR_BUILD}/merged.hex"
echo "Reflector: ${REFLECTOR_BUILD}/merged.hex"
