from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from il_cs_ras_window_evaluate import (  # noqa: E402
    endpoint_calibration,
    median_absolute_deviation,
    rolling_medians,
)


class RasWindowEvaluateTests(unittest.TestCase):
    def test_rolling_median_rejects_single_outlier(self) -> None:
        result = rolling_medians([1.0, 1.1, 9.0, 1.2, 1.3], 5)
        self.assertEqual(result, [1.2])

    def test_rolling_window_must_be_positive_and_odd(self) -> None:
        with self.assertRaises(ValueError):
            rolling_medians([1.0, 2.0], 2)

    def test_endpoint_calibration_holds_out_middle(self) -> None:
        gain, offset = endpoint_calibration(3.0, 0.5, 6.0, 1.5)
        self.assertAlmostEqual(gain * 4.5 + offset, 1.0)

    def test_median_absolute_deviation(self) -> None:
        self.assertEqual(median_absolute_deviation([1.0, 2.0, 3.0]), 1.0)


if __name__ == "__main__":
    unittest.main()
