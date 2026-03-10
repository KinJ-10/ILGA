#!/usr/bin/env bash
set -euo pipefail

# TAG log capture script for ILGA
# Default:
#   PORT=/dev/ttyACM1
#   BAUD=115200
# Usage:
#   ./scripts/tag_log_capture.sh [timeout_seconds]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PORT="${PORT:-/dev/ttyACM1}"
BAUD="${BAUD:-115200}"
TIMEOUT_SECONDS="${1:-10}"
LOG_DIR="${REPO_ROOT}/logs"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/tag_log_${TIMESTAMP}.log"

if [[ ! "${TIMEOUT_SECONDS}" =~ ^[0-9]+$ ]]; then
  echo "ERROR: timeout_seconds must be an integer: ${TIMEOUT_SECONDS}" >&2
  exit 1
fi

if [[ ! -e "${PORT}" ]]; then
  echo "ERROR: serial port not found: ${PORT}" >&2
  exit 1
fi

mkdir -p "${LOG_DIR}"

echo "=== ILGA TAG log capture start ==="
echo "PORT     : ${PORT}"
echo "BAUD     : ${BAUD}"
echo "TIMEOUT  : ${TIMEOUT_SECONDS}s"
echo "LOG_FILE : ${LOG_FILE}"
echo

stty -F "${PORT}" "${BAUD}" raw -echo

set +e
timeout "${TIMEOUT_SECONDS}s" cat "${PORT}" | tee "${LOG_FILE}"
status=${PIPESTATUS[0]}
set -e

if [[ "${status}" -ne 0 && "${status}" -ne 124 ]]; then
  echo "ERROR: log capture failed with status ${status}" >&2
  exit "${status}"
fi

echo
echo "=== ILGA TAG log capture done ==="
echo "Saved : ${LOG_FILE}"
