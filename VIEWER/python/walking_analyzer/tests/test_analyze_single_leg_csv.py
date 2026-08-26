import csv
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ANALYZER = Path(__file__).resolve().parents[1] / "analyze_single_leg_csv.py"
CSV_COLUMNS = (
    "seq",
    "ax_mg",
    "ay_mg",
    "az_mg",
    "gx_mdps",
    "gy_mdps",
    "gz_mdps",
)


class AnalyzeSingleLegCsvRegressionTests(unittest.TestCase):
    def make_rows(self, sample_count=100, zero_accel_indices=(), constant_accel=False):
        zero_accel_indices = set(zero_accel_indices)
        rows = []
        for index in range(sample_count):
            if index in zero_accel_indices:
                ax_mg, ay_mg, az_mg = 0, 0, 0
            elif constant_accel:
                ax_mg, ay_mg, az_mg = 1000, 0, 0
            else:
                phase = 2.0 * math.pi * index / 25.0
                ax_mg = round(1000.0 + 150.0 * math.sin(phase), 3)
                ay_mg = round(80.0 * math.sin(phase + 0.5), 3)
                az_mg = round(50.0 * math.cos(phase), 3)

            gyro_phase = 2.0 * math.pi * index / 25.0
            rows.append(
                (
                    1000 + index,
                    ax_mg,
                    ay_mg,
                    az_mg,
                    round(120000.0 * math.sin(gyro_phase), 3),
                    round(20000.0 * math.cos(gyro_phase), 3),
                    0,
                )
            )
        return rows

    def run_analyzer(self, rows):
        temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(temp_dir.cleanup)
        root = Path(temp_dir.name)
        input_csv = root / "fixture.csv"
        output_dir = root / "result"

        with input_csv.open("w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(CSV_COLUMNS)
            writer.writerows(rows)

        result = subprocess.run(
            [
                sys.executable,
                str(ANALYZER),
                str(input_csv),
                "--fs",
                "100",
                "--out-dir",
                str(output_dir),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        return result, output_dir

    def assert_successful_analysis(self, rows):
        result, output_dir = self.run_analyzer(rows)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stderr, "")
        self.assertTrue((output_dir / "summary.json").is_file())
        self.assertTrue((output_dir / "step_events.csv").is_file())
        return json.loads((output_dir / "summary.json").read_text())

    def test_normal_csv_is_analyzed(self):
        summary = self.assert_successful_analysis(self.make_rows())
        self.assertEqual(summary["sample_count"], 100)

    def test_short_zero_acceleration_mix_is_accepted(self):
        summary = self.assert_successful_analysis(
            self.make_rows(zero_accel_indices={20, 40})
        )
        self.assertEqual(summary["sample_count"], 100)

    def test_ten_consecutive_zero_acceleration_samples_are_accepted(self):
        summary = self.assert_successful_analysis(
            self.make_rows(zero_accel_indices=range(30, 40))
        )
        self.assertEqual(summary["sample_count"], 100)

    def test_ninety_five_percent_zero_acceleration_is_rejected(self):
        result, output_dir = self.run_analyzer(
            self.make_rows(zero_accel_indices=range(95))
        )

        self.assertEqual(result.returncode, 2)
        self.assertIn("BMI270 acceleration is zero", result.stderr)
        self.assertIn("95/100 samples (95.0%)", result.stderr)
        self.assertFalse(output_dir.exists())

    def test_constant_nonzero_acceleration_produces_no_peaks(self):
        summary = self.assert_successful_analysis(
            self.make_rows(constant_accel=True)
        )
        self.assertEqual(summary["step_count"], 0)
        self.assertEqual(summary["detection_threshold_mg"], 0.0)


if __name__ == "__main__":
    unittest.main()
