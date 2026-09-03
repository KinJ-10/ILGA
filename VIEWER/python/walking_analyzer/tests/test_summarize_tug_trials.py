import json
import sys
import tempfile
import unittest
from pathlib import Path


MODULE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_DIR))
import summarize_tug_trials as tug  # noqa: E402


class TugSummaryTests(unittest.TestCase):
    def record(self, name, duration, status="ok"):
        return {
            "trial_name": name,
            "test_type": "tug",
            "timing": {"marked_duration_sec": duration},
            "quality": {"status": status},
        }

    def test_three_trial_statistics_and_phase_limit(self):
        result = tug.summarize(
            [self.record("t1", 9.031), self.record("t2", 8.656), self.record("t3", 8.453)]
        )
        self.assertEqual(result["trial_count"], 3)
        self.assertAlmostEqual(result["mean_sec"], 8.713333333333333)
        self.assertAlmostEqual(result["sd_sec"], 0.2932342635732285)
        self.assertAlmostEqual(result["cv_pct"], 3.3653511504196083)
        self.assertAlmostEqual(result["range_sec"], 0.578)
        self.assertFalse(result["phase_detection"]["implemented"])
        self.assertIn("Foot TAG alone", result["limitations"][0])

    def test_non_tug_or_quality_error_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "non-TUG"):
            tug.summarize([{**self.record("walk", 8.0), "test_type": "walk10m"}])
        with self.assertRaisesRegex(ValueError, "quality error"):
            tug.summarize([self.record("bad", 8.0, status="error")])

    def test_run_writes_machine_and_human_readable_outputs(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            paths = []
            for index, duration in enumerate((9.0, 8.5), start=1):
                path = root / f"t{index}.json"
                path.write_text(json.dumps(self.record(f"t{index}", duration)))
                paths.append(path)
            out = root / "out"
            tug.run(paths, out)
            self.assertTrue((out / "tug_aggregate.json").is_file())
            self.assertTrue((out / "tug_trials.csv").is_file())
            self.assertIn("phase_detection: not implemented", (out / "summary.txt").read_text())


if __name__ == "__main__":
    unittest.main()
