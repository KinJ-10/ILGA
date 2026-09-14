#!/usr/bin/env python3

from __future__ import annotations

import os
import subprocess
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
CAPTURE_SCRIPT = REPO_ROOT / "scripts" / "il_cs_log_capture.sh"


def run_capture_cli(*arguments: str) -> subprocess.CompletedProcess[str]:
    environment = os.environ.copy()
    environment.pop("BAUD", None)
    return subprocess.run(
        ["bash", str(CAPTURE_SCRIPT), *arguments],
        cwd=REPO_ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )


class IlCsLogCaptureCliTests(unittest.TestCase):
    def test_raw_profile_selects_230400(self) -> None:
        result = run_capture_cli(
            "initiator", "/dev/not-opened", "120", "--raw-diagnostics", "--dry-run"
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Profile    : raw-diagnostics", result.stdout)
        self.assertIn("UART       : 230400 baud, 8-N-1", result.stdout)

    def test_normal_profile_selects_115200(self) -> None:
        result = run_capture_cli(
            "reflector", "/dev/not-opened", "30", "--normal", "--dry-run"
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Profile    : normal", result.stdout)
        self.assertIn("UART       : 115200 baud, 8-N-1", result.stdout)

    def test_profile_is_mandatory(self) -> None:
        result = run_capture_cli("initiator", "/dev/not-opened", "120", "--dry-run")
        self.assertEqual(result.returncode, 2)
        self.assertIn("capture profile is required", result.stderr)

    def test_raw_profile_rejects_115200(self) -> None:
        result = run_capture_cli(
            "initiator",
            "/dev/not-opened",
            "120",
            "--raw-diagnostics",
            "--baud",
            "115200",
            "--dry-run",
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("requires 230400 baud", result.stderr)

    def test_normal_profile_rejects_230400(self) -> None:
        result = run_capture_cli(
            "reflector",
            "/dev/not-opened",
            "--normal",
            "--baud",
            "230400",
            "--dry-run",
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("requires 115200 baud", result.stderr)


if __name__ == "__main__":
    unittest.main()
