#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

NCS_VENV="${HOME}/ncs/.venv/bin/activate"
NCS_TOP="${HOME}/ncs/ncs-v3.2.3"
BOARD="nrf54l15dk/nrf54l15/cpuapp"
INITIATOR_APP="${NCS_TOP}/nrf/samples/bluetooth/channel_sounding_ras_initiator"
REFLECTOR_APP="${NCS_TOP}/nrf/samples/bluetooth/channel_sounding_ras_reflector"
INITIATOR_BUILD="${REPO_ROOT}/build/il_cs_ras_initiator"
REFLECTOR_BUILD="${REPO_ROOT}/build/il_cs_ras_reflector"

if [[ $# -ne 0 ]]; then
  echo "Usage: ./scripts/il_cs_ras_build.sh" >&2
  exit 2
fi

for required_path in \
  "${NCS_VENV}" \
  "${INITIATOR_APP}" \
  "${REFLECTOR_APP}"; do
  if [[ ! -e "${required_path}" ]]; then
    echo "ERROR: required NCS v3.2.3 path not found: ${required_path}" >&2
    exit 1
  fi
done

# shellcheck disable=SC1090
source "${NCS_VENV}"
cd "${NCS_TOP}"

echo "=== ILGA Nordic RAS + cs_de comparison build ==="
echo "NCS             : ${NCS_TOP}"
echo "BOARD           : ${BOARD}"
echo "ESTIMATORS      : IFFT / phase_slope / RTT"
echo "INITIATOR_BUILD : ${INITIATOR_BUILD}"
echo "REFLECTOR_BUILD : ${REFLECTOR_BUILD}"

west build -p always -b "${BOARD}" "${INITIATOR_APP}" -d "${INITIATOR_BUILD}"
west build -p always -b "${BOARD}" "${REFLECTOR_APP}" -d "${REFLECTOR_BUILD}"

echo "=== Build complete ==="
echo "LOCATOR / Initiator: ${INITIATOR_BUILD}/merged.hex"
echo "TAG     / Reflector: ${REFLECTOR_BUILD}/merged.hex"
