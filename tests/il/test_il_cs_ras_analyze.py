from __future__ import annotations

import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[2] / "scripts" / "il_cs_ras_analyze.py"
SPEC = importlib.util.spec_from_file_location("il_cs_ras_analyze", SCRIPT_PATH)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class IlCsRasAnalyzeTests(unittest.TestCase):
    def test_parse_summarize_and_write_outputs(self) -> None:
        capture = "\n".join(
            [
                "2026-09-17T09:00:00+09:00\t<inf> app: unrelated",
                "2026-09-17T09:00:01+09:00\t<inf> app: Latest distance estimates on antenna path 0: ifft: 0.50, phase_slope: 0.60, rtt: 1.20 meters",
                "2026-09-17T09:00:02+09:00\t<inf> app: Latest distance estimates on antenna path 0: ifft: nan, phase_slope: 0.80, rtt: 1.00 meters",
            ]
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            log_path = temp_path / "capture.log"
            output_dir = temp_path / "out"
            log_path.write_text(capture, encoding="utf-8")

            rows = MODULE.parse_log(log_path)
            summary = MODULE.summarize(rows, true_distance_m=0.5)
            MODULE.write_outputs(output_dir, log_path, rows, summary)

            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["host_timestamp"], "2026-09-17T09:00:01+09:00")
            self.assertTrue(math.isnan(rows[1]["ifft_m"]))
            ifft = summary["by_antenna_path"]["0"]["ifft"]
            phase = summary["by_antenna_path"]["0"]["phase_slope"]
            self.assertEqual(ifft["count"], 1)
            self.assertEqual(ifft["rejected_non_finite"], 1)
            self.assertAlmostEqual(ifft["mae_m"], 0.0)
            self.assertAlmostEqual(phase["median_m"], 0.7)
            self.assertTrue((output_dir / "distance_estimates.csv").is_file())
            saved = json.loads((output_dir / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(saved["estimate_rows"], 2)


if __name__ == "__main__":
    unittest.main()
