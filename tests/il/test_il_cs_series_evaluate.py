import csv
import importlib.util
import math
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "il_cs_series_evaluate.py"
SPEC = importlib.util.spec_from_file_location("il_cs_series_evaluate", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class IlCsSeriesEvaluateTests(unittest.TestCase):
    def test_circular_search_recovers_known_distance(self):
        distance = 1.75
        slope = -4.0 * math.pi * distance * 1_000_000.0 / MODULE.SPEED_OF_LIGHT_M_PER_S
        samples = [
            (frequency, MODULE.wrap_phase(0.4 + slope * frequency))
            for frequency in (2426.0, 2431.0, 2440.0, 2452.0, 2457.0)
        ]
        estimate, cost = MODULE.search_distance(samples, 5.0, 0.01)
        self.assertAlmostEqual(estimate, distance, places=2)
        self.assertLess(cost, 1e-9)

    def test_rank_values_uses_average_rank_for_ties(self):
        self.assertEqual(MODULE.rank_values([2.0, 1.0, 2.0]), [2.5, 1.0, 2.5])

    def test_parse_series_rejects_bad_value(self):
        with self.assertRaises(Exception):
            MODULE.parse_series("missing-path")

    def test_load_trial_keeps_reused_procedure_counters_separate_by_boot(self):
        summary_fields = [
            "boot_index",
            "procedure_counter",
            "ended",
            "pbr_valid",
            "pbr_distance_m",
            "pbr_residual_rms_rad",
        ]
        pbr_fields = [
            "boot_index",
            "procedure_counter",
            "valid",
            "frequency_mhz",
            "wrapped_phase_rad",
        ]
        with tempfile.TemporaryDirectory() as temp_dir:
            trial_dir = Path(temp_dir)
            with (trial_dir / "procedure_summary.csv").open(
                "w", newline="", encoding="utf-8"
            ) as handle:
                writer = csv.DictWriter(handle, fieldnames=summary_fields)
                writer.writeheader()
                writer.writerows(
                    [
                        {
                            "boot_index": boot,
                            "procedure_counter": 1,
                            "ended": "True",
                            "pbr_valid": 2,
                            "pbr_distance_m": distance,
                            "pbr_residual_rms_rad": 0.01,
                        }
                        for boot, distance in ((0, 1.0), (1, 2.0))
                    ]
                )
            with (trial_dir / "pbr_samples.csv").open(
                "w", newline="", encoding="utf-8"
            ) as handle:
                writer = csv.DictWriter(handle, fieldnames=pbr_fields)
                writer.writeheader()
                for boot, distance in ((0, 1.0), (1, 2.0)):
                    slope = (
                        -4.0
                        * math.pi
                        * distance
                        * 1_000_000.0
                        / MODULE.SPEED_OF_LIGHT_M_PER_S
                    )
                    for frequency in (2426.0, 2457.0):
                        writer.writerow(
                            {
                                "boot_index": boot,
                                "procedure_counter": 1,
                                "valid": "True",
                                "frequency_mhz": frequency,
                                "wrapped_phase_rad": MODULE.wrap_phase(0.4 + slope * frequency),
                            }
                        )

            rows = MODULE.load_trial(0.5, trial_dir, 0, 5.0, 0.01)

        self.assertEqual([row["boot_index"] for row in rows], [0, 1])
        self.assertEqual(
            [row["circular_candidate_distance_m"] for row in rows], [1.0, 2.0]
        )


if __name__ == "__main__":
    unittest.main()
