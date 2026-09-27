from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from il_cs_ras_calibrate import (  # noqa: E402
    classify_distances,
    corrected_distance,
    validate_config,
)


def valid_config() -> dict:
    return {
        "schema_version": 1,
        "estimator": "firmware_phase_slope_m",
        "window": {"kind": "rolling_median", "frames": 5},
        "calibration": {
            "low_anchor": {"true_m": 0.5, "measured_m": 3.0},
            "high_anchor": {"true_m": 1.5, "measured_m": 6.0},
            "gain": 1 / 3, "offset_m": -0.5,
        },
        "quality": {"max_window_output_sd_m": 0.2, "anchor_tolerance_m": 0.2,
                    "min_median_good_tones": 60},
        "zone": {"enter_m": 0.75, "exit_m": 1.25},
    }


class RasCalibrateTests(unittest.TestCase):
    def test_valid_config_and_correction(self) -> None:
        config = valid_config()
        validate_config(config)
        self.assertAlmostEqual(corrected_distance(config, 4.5), 1.0)

    def test_zone_hysteresis_holds_state_between_thresholds(self) -> None:
        states = classify_distances([1.4, 0.7, 1.0, 1.3, 1.0], 0.75, 1.25)
        self.assertEqual(states, ["outside", "inside", "inside", "outside", "outside"])

    def test_zone_thresholds_must_be_inside_calibrated_range(self) -> None:
        config = valid_config()
        config["zone"]["exit_m"] = 2.0
        with self.assertRaises(ValueError):
            validate_config(config)


if __name__ == "__main__":
    unittest.main()
