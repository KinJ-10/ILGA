#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/il_cs_log_capture.sh <initiator|reflector> </dev/ttyACMx> \
    [duration_seconds] (--normal | --raw-diagnostics) [--baud RATE] [--dry-run]

Profiles:
  --normal           Normal firmware console at 115200 baud.
  --raw-diagnostics  RAW diagnostic firmware console at 230400 baud.

The profile is mandatory. --baud is optional, but must match the selected
firmware profile; a mismatch is rejected before opening the serial port.
EOF
}

if [[ $# -lt 3 ]]; then
  usage >&2
  exit 2
fi

ROLE="$1"
PORT="$2"
shift 2

DURATION_SECONDS="120"
if [[ $# -gt 0 && "$1" != --* ]]; then
  DURATION_SECONDS="$1"
  shift
fi

PROFILE=""
EXPECTED_BAUD=""
REQUESTED_BAUD="${BAUD:-}"
DRY_RUN="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --normal)
      if [[ -n "${PROFILE}" ]]; then
        echo "ERROR: specify exactly one capture profile." >&2
        exit 2
      fi
      PROFILE="normal"
      EXPECTED_BAUD="115200"
      shift
      ;;
    --raw-diagnostics)
      if [[ -n "${PROFILE}" ]]; then
        echo "ERROR: specify exactly one capture profile." >&2
        exit 2
      fi
      PROFILE="raw-diagnostics"
      EXPECTED_BAUD="230400"
      shift
      ;;
    --baud)
      if [[ $# -lt 2 ]]; then
        echo "ERROR: --baud requires a value." >&2
        exit 2
      fi
      REQUESTED_BAUD="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "${ROLE}" != "initiator" && "${ROLE}" != "reflector" ]]; then
  usage >&2
  exit 2
fi

if [[ -z "${PROFILE}" ]]; then
  echo "ERROR: capture profile is required; use --normal or --raw-diagnostics." >&2
  exit 2
fi

if [[ ! "${DURATION_SECONDS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: duration_seconds must be a positive integer." >&2
  exit 2
fi

if [[ -z "${REQUESTED_BAUD}" ]]; then
  CAPTURE_BAUD="${EXPECTED_BAUD}"
else
  CAPTURE_BAUD="${REQUESTED_BAUD}"
fi

if [[ ! "${CAPTURE_BAUD}" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: baud rate must be a positive integer." >&2
  exit 2
fi

if [[ "${CAPTURE_BAUD}" != "${EXPECTED_BAUD}" ]]; then
  echo "ERROR: profile ${PROFILE} requires ${EXPECTED_BAUD} baud; requested ${CAPTURE_BAUD}." >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${REPO_ROOT}/logs/il_cs"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
if [[ "${PROFILE}" == "raw-diagnostics" ]]; then
  LOG_FILE="${LOG_DIR}/${ROLE}_raw_230400_${TIMESTAMP}.log"
else
  LOG_FILE="${LOG_DIR}/${ROLE}_${TIMESTAMP}.log"
fi

echo "Capture configuration:"
echo "  Role       : ${ROLE}"
echo "  Profile    : ${PROFILE}"
echo "  Port       : ${PORT}"
echo "  UART       : ${CAPTURE_BAUD} baud, 8-N-1"
echo "  Duration   : ${DURATION_SECONDS}s"
echo "  Output     : ${LOG_FILE}"

if [[ "${DRY_RUN}" == "true" ]]; then
  echo "DRY RUN only; serial port was not opened."
  exit 0
fi

if [[ ! -c "${PORT}" ]]; then
  echo "ERROR: serial port is not a character device: ${PORT}" >&2
  exit 1
fi

CAPTURE_PYTHON="${IL_CS_CAPTURE_PYTHON:-python3}"
if ! command -v "${CAPTURE_PYTHON}" >/dev/null 2>&1; then
  echo "ERROR: python3 is required for serial capture." >&2
  exit 1
fi
if ! "${CAPTURE_PYTHON}" -c 'import serial' >/dev/null 2>&1; then
  echo "ERROR: pyserial is required for serial capture." >&2
  exit 1
fi

mkdir -p "${LOG_DIR}"
echo "Opening SEGGER VCOM with pyserial..."

set +e
"${CAPTURE_PYTHON}" -u -c '
from datetime import datetime
import sys
import time

import serial

port = sys.argv[1]
baud = int(sys.argv[2])
duration = int(sys.argv[3])
deadline = time.monotonic() + duration
pending = bytearray()
output = sys.stdout.buffer

def emit(line):
    timestamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
    output.write(timestamp.encode("ascii") + b"\t" + line)
    output.flush()

with serial.Serial(
    port=port,
    baudrate=baud,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.05,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False,
    exclusive=True,
) as uart:
    while time.monotonic() < deadline:
        chunk = uart.read(uart.in_waiting or 1)
        if not chunk:
            continue
        pending.extend(chunk)
        while True:
            newline = pending.find(b"\n")
            if newline < 0:
                break
            emit(bytes(pending[: newline + 1]))
            del pending[: newline + 1]

if pending:
    emit(bytes(pending))
' "${PORT}" "${CAPTURE_BAUD}" "${DURATION_SECONDS}" |
  tee "${LOG_FILE}"
capture_status=${PIPESTATUS[0]}
set -e

if [[ "${capture_status}" -ne 0 ]]; then
  echo "ERROR: capture failed with status ${capture_status}." >&2
  exit "${capture_status}"
fi

echo "Saved: ${LOG_FILE}"
