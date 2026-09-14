#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/il_cs_flash_pair.sh <initiator-serial> <reflector-serial> \
    [--raw-diagnostics] [--execute]

Without --execute this performs a safe dry run. With --execute, both listed DKs
must be connected and the operator must confirm the complete serial/role map.
Use --raw-diagnostics only when both matching raw diagnostic builds are ready.
EOF
}

if [[ $# -lt 2 || $# -gt 4 ]]; then
  usage >&2
  exit 2
fi

INITIATOR_SERIAL="$1"
REFLECTOR_SERIAL="$2"
shift 2
MODE="--dry-run"
PROFILE="normal"

for arg in "$@"; do
  case "${arg}" in
    --execute)
      MODE="--execute"
      ;;
    --dry-run)
      MODE="--dry-run"
      ;;
    --raw-diagnostics)
      PROFILE="raw-diagnostics"
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! "${INITIATOR_SERIAL}" =~ ^[0-9]+$ || ! "${REFLECTOR_SERIAL}" =~ ^[0-9]+$ ]]; then
  echo "ERROR: DK serial numbers must contain digits only." >&2
  exit 2
fi

if [[ "${INITIATOR_SERIAL}" == "${REFLECTOR_SERIAL}" ]]; then
  echo "ERROR: initiator and reflector serial numbers must differ." >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
NCS_VENV="${HOME}/ncs/.venv/bin/activate"
NCS_TOP="${HOME}/ncs/ncs-v3.2.3"
if [[ "${PROFILE}" == "raw-diagnostics" ]]; then
  INITIATOR_BUILD="${REPO_ROOT}/build/il_cs_initiator_raw"
  REFLECTOR_BUILD="${REPO_ROOT}/build/il_cs_reflector_raw"
else
  INITIATOR_BUILD="${REPO_ROOT}/build/il_cs_initiator"
  REFLECTOR_BUILD="${REPO_ROOT}/build/il_cs_reflector"
fi

for build_dir in "${INITIATOR_BUILD}" "${REFLECTOR_BUILD}"; do
  if [[ ! -f "${build_dir}/CMakeCache.txt" ]]; then
    echo "ERROR: build directory is not ready: ${build_dir}" >&2
    echo "Run ./scripts/il_cs_build.sh first." >&2
    exit 1
  fi
done

echo "Role map (verify the physical labels before continuing):"
echo "  Build profile       : ${PROFILE}"
if [[ "${PROFILE}" == "raw-diagnostics" ]]; then
  echo "  Console UART        : 230400 baud (capture with --raw-diagnostics)"
else
  echo "  Console UART        : 115200 baud (capture with --normal)"
fi
echo "  LOCATOR / Initiator : ${INITIATOR_SERIAL}"
echo "  TAG     / Reflector : ${REFLECTOR_SERIAL}"
echo
echo "Commands:"
echo "  west flash --skip-rebuild -d ${INITIATOR_BUILD} --dev-id ${INITIATOR_SERIAL}"
echo "  west flash --skip-rebuild -d ${REFLECTOR_BUILD} --dev-id ${REFLECTOR_SERIAL}"

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
for serial in "${INITIATOR_SERIAL}" "${REFLECTOR_SERIAL}"; do
  if ! grep -Fq "${serial}" <<<"${DEVICE_LIST}"; then
    echo "ERROR: serial ${serial} is not present in the connected-device list." >&2
    exit 1
  fi
done

if [[ ! -t 0 ]]; then
  echo "ERROR: interactive confirmation is required for flashing." >&2
  exit 1
fi

EXPECTED="FLASH ${INITIATOR_SERIAL} ${REFLECTOR_SERIAL}"
read -r -p "Type '${EXPECTED}' to flash both DKs: " CONFIRMATION
if [[ "${CONFIRMATION}" != "${EXPECTED}" ]]; then
  echo "Cancelled; no device was flashed."
  exit 1
fi

cd "${NCS_TOP}"
west flash --skip-rebuild -d "${INITIATOR_BUILD}" --dev-id "${INITIATOR_SERIAL}"
west flash --skip-rebuild -d "${REFLECTOR_BUILD}" --dev-id "${REFLECTOR_SERIAL}"

echo "Flash complete. Keep the physical serial/role labels attached."
