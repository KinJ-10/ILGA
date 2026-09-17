#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/il_cs_ras_flash_pair.sh <locator-serial> <tag-serial> [--execute]

Without --execute this performs a safe dry run. With --execute, both listed DKs
must be connected and the operator must confirm the complete serial/role map.
This script flashes only the Nordic RAS + cs_de comparison builds.
EOF
}

if [[ $# -lt 2 || $# -gt 3 ]]; then
  usage >&2
  exit 2
fi

LOCATOR_SERIAL="$1"
TAG_SERIAL="$2"
shift 2
MODE="--dry-run"

if [[ $# -eq 1 ]]; then
  if [[ "$1" != "--execute" ]]; then
    usage >&2
    exit 2
  fi
  MODE="--execute"
fi

if [[ ! "${LOCATOR_SERIAL}" =~ ^[0-9]+$ || ! "${TAG_SERIAL}" =~ ^[0-9]+$ ]]; then
  echo "ERROR: DK serial numbers must contain digits only." >&2
  exit 2
fi

if [[ "${LOCATOR_SERIAL}" == "${TAG_SERIAL}" ]]; then
  echo "ERROR: LOCATOR and TAG serial numbers must differ." >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
NCS_VENV="${HOME}/ncs/.venv/bin/activate"
NCS_TOP="${HOME}/ncs/ncs-v3.2.3"
INITIATOR_BUILD="${REPO_ROOT}/build/il_cs_ras_initiator"
REFLECTOR_BUILD="${REPO_ROOT}/build/il_cs_ras_reflector"

for build_dir in "${INITIATOR_BUILD}" "${REFLECTOR_BUILD}"; do
  if [[ ! -f "${build_dir}/merged.hex" || ! -f "${build_dir}/CMakeCache.txt" ]]; then
    echo "ERROR: RAS comparison build is not ready: ${build_dir}" >&2
    echo "Run ./scripts/il_cs_ras_build.sh first." >&2
    exit 1
  fi
done

echo "Role map (verify the physical labels before continuing):"
echo "  Profile             : Nordic RAS + cs_de comparison"
echo "  Console UART        : 115200 baud"
echo "  LOCATOR / Initiator : ${LOCATOR_SERIAL}"
echo "  TAG     / Reflector : ${TAG_SERIAL}"
echo
echo "Commands:"
echo "  west flash --skip-rebuild -d ${INITIATOR_BUILD} --dev-id ${LOCATOR_SERIAL}"
echo "  west flash --skip-rebuild -d ${REFLECTOR_BUILD} --dev-id ${TAG_SERIAL}"

if [[ "${MODE}" != "--execute" ]]; then
  echo
  echo "DRY RUN only. Re-run with --execute after labeling both DKs."
  exit 0
fi

if [[ ! -f "${NCS_VENV}" || ! -d "${NCS_TOP}" ]]; then
  echo "ERROR: NCS v3.2.3 environment was not found." >&2
  exit 1
fi

# shellcheck disable=SC1090
source "${NCS_VENV}"

DEVICE_LIST="$(nrfutil device list)"
printf '%s\n' "${DEVICE_LIST}"
for serial in "${LOCATOR_SERIAL}" "${TAG_SERIAL}"; do
  if ! grep -Fq "${serial}" <<<"${DEVICE_LIST}"; then
    echo "ERROR: serial ${serial} is not present in the connected-device list." >&2
    exit 1
  fi
done

if [[ ! -t 0 ]]; then
  echo "ERROR: interactive confirmation is required for flashing." >&2
  exit 1
fi

EXPECTED="FLASH RAS ${LOCATOR_SERIAL} ${TAG_SERIAL}"
read -r -p "Type '${EXPECTED}' to flash both DKs: " CONFIRMATION
if [[ "${CONFIRMATION}" != "${EXPECTED}" ]]; then
  echo "Cancelled; no device was flashed."
  exit 1
fi

cd "${NCS_TOP}"
west flash --skip-rebuild -d "${INITIATOR_BUILD}" --dev-id "${LOCATOR_SERIAL}"
west flash --skip-rebuild -d "${REFLECTOR_BUILD}" --dev-id "${TAG_SERIAL}"

echo "RAS comparison flash complete. Keep the physical serial/role labels attached."
