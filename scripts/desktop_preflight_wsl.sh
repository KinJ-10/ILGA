#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
NCS_TOP="${NCS_TOP:-${HOME}/ncs/ncs-v3.2.3}"
NCS_VENV="${NCS_VENV:-${HOME}/ncs/.venv/bin/activate}"
EXPECTED_HEAD="289e7fc4191912691ebd610eb259bc22f991cf32"
FAILURES=0
WARNINGS=0

pass() {
  printf '[PASS] %s\n' "$1"
}

warn() {
  printf '[WARN] %s\n' "$1"
  WARNINGS=$((WARNINGS + 1))
}

fail() {
  printf '[FAIL] %s\n' "$1"
  FAILURES=$((FAILURES + 1))
}

printf 'ILGA Codex Desktop preflight (WSL)\n'
printf 'Repo: %s\n\n' "${REPO_ROOT}"

if [[ -n "${WSL_DISTRO_NAME:-}" ]]; then
  pass "WSL2 environment detected: ${WSL_DISTRO_NAME}"
else
  warn "WSL_DISTRO_NAME is empty; run this script inside WSL2"
fi

if command -v git >/dev/null 2>&1; then
  pass "git: $(git --version)"
else
  fail "git is not installed"
fi

if git -C "${REPO_ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  HEAD_SHA="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
  BRANCH="$(git -C "${REPO_ROOT}" branch --show-current)"
  pass "repository detected: branch=${BRANCH:-detached} head=${HEAD_SHA}"
  if git -C "${REPO_ROOT}" merge-base --is-ancestor "${EXPECTED_HEAD}" "${HEAD_SHA}" 2>/dev/null; then
    pass "handover baseline is contained in the current history"
  else
    warn "handover baseline ${EXPECTED_HEAD} is not an ancestor of HEAD; review git log before continuing"
  fi
  if [[ -n "$(git -C "${REPO_ROOT}" status --porcelain)" ]]; then
    warn "working tree has uncommitted changes"
    git -C "${REPO_ROOT}" status --short
  else
    pass "working tree is clean"
  fi
else
  fail "repository metadata is unavailable"
fi

if command -v python3 >/dev/null 2>&1; then
  PYTHON_VERSION="$(python3 -c 'import platform; print(platform.python_version())')"
  pass "system Python: ${PYTHON_VERSION}"
  if [[ "${PYTHON_VERSION}" != 3.12.* ]]; then
    warn "Python 3.12 was the verified Viewer/analyzer environment"
  fi
else
  fail "python3 is not installed"
fi

if [[ -d "${NCS_TOP}" ]]; then
  pass "NCS workspace: ${NCS_TOP}"
else
  fail "NCS v3.2.3 workspace not found: ${NCS_TOP}"
fi

if [[ -f "${NCS_VENV}" ]]; then
  pass "NCS venv: ${NCS_VENV}"
  # shellcheck disable=SC1090
  source "${NCS_VENV}"
  if command -v west >/dev/null 2>&1; then
    WEST_VERSION="$(west --version 2>&1)"
    pass "${WEST_VERSION}"
    if [[ "${WEST_VERSION}" != *"1.5.0"* ]]; then
      warn "west 1.5.0 was used in the verified environment"
    fi
  else
    fail "west is unavailable after activating the NCS venv"
  fi
else
  fail "NCS venv activation script not found: ${NCS_VENV}"
fi

for command_name in cmake ninja; do
  if command -v "${command_name}" >/dev/null 2>&1; then
    pass "${command_name}: $("${command_name}" --version 2>&1 | head -n 1)"
  else
    fail "${command_name} is not available"
  fi
done

for required_path in \
  "TAG/zephyr_apps/current/nrf54l15_port/src/main.c" \
  "VIEWER/python/bmi270_BLE_viewer/recv_bmi270_ble_notify_cli.py" \
  "VIEWER/python/bmi270_BLE_viewer/realtime_plot_ble_v7.py" \
  "VIEWER/python/walking_analyzer/analyze_single_leg_csv.py" \
  "scripts/tag_build.sh" \
  "scripts/tag_flash.sh" \
  "scripts/tag_log_capture.sh"; do
  if [[ -f "${REPO_ROOT}/${required_path}" ]]; then
    pass "file: ${required_path}"
  else
    fail "missing file: ${required_path}"
  fi
done

if command -v python3 >/dev/null 2>&1; then
  if python3 -c 'import ast, pathlib, sys; [ast.parse(pathlib.Path(p).read_text(encoding="utf-8"), filename=p) for p in sys.argv[1:]]' \
    "${REPO_ROOT}/VIEWER/python/bmi270_BLE_viewer/recv_bmi270_ble_notify_cli.py" \
    "${REPO_ROOT}/VIEWER/python/bmi270_BLE_viewer/realtime_plot_ble_v7.py" \
    "${REPO_ROOT}/VIEWER/python/walking_analyzer/analyze_single_leg_csv.py"; then
    pass "Python source syntax"
  else
    fail "Python source syntax check failed"
  fi
fi

if compgen -G '/dev/ttyACM*' >/dev/null; then
  pass "serial ports: $(printf '%s ' /dev/ttyACM*)"
else
  warn "no /dev/ttyACM* device found; connect the nRF54L15 DK before flash/log verification"
fi

printf '\nSummary: failures=%d warnings=%d\n' "${FAILURES}" "${WARNINGS}"
if [[ "${FAILURES}" -ne 0 ]]; then
  exit 1
fi
exit 0
