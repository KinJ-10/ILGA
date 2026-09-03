import sys
import unittest
from pathlib import Path

import numpy as np


MODULE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_DIR))
import compare_stride_marker_candidates as comparison  # noqa: E402


class StrideMarkerCandidateComparisonTests(unittest.TestCase):
    def delayed_waveform(self):
        times = np.arange(0.0, 1.01, 0.01)
        values = np.full(len(times), -80.0)
        peak = 20
        values[: peak + 1] = np.linspace(0.0, 200.0, peak + 1)
        values[peak:58] = np.linspace(200.0, 10.0, 38)
        values[58:66] = np.linspace(10.0, -80.0, 8)
        return values, times, peak

    def test_delayed_zero_cross_fails_baseline_and_passes_extended_window(self):
        values, times, peak = self.delayed_waveform()
        base = comparison.waveform_diagnostic(
            values, times, peak, comparison.CONFIGS[0], observed_step_rate=80.0
        )
        extended = comparison.waveform_diagnostic(
            values, times, peak, comparison.CONFIGS[3], observed_step_rate=80.0
        )
        self.assertFalse(base["formal"])
        self.assertEqual(base["failure_reason"], "no_zero_cross")
        self.assertEqual(base["waveform_shape"], "delayed_zero_cross_after_baseline_window")
        self.assertGreater(base["diagnostic_zero_cross_lag_sec"], 0.30)
        self.assertTrue(extended["formal"])

    def test_speed_adaptive_window_changes_only_below_threshold(self):
        config = next(c for c in comparison.CONFIGS if c.name.startswith("speed_adaptive"))
        self.assertEqual(config.effective_window(85.0), 0.45)
        self.assertEqual(config.effective_window(95.0), 0.30)
        self.assertEqual(config.effective_window(None), 0.30)

    def test_direct_negative_cross_is_explicitly_different_event_feature(self):
        values, times, peak = self.delayed_waveform()
        config = next(c for c in comparison.CONFIGS if c.name.startswith("negative50"))
        result = comparison.waveform_diagnostic(values, times, peak, config, 80.0)
        self.assertTrue(result["formal"])
        self.assertEqual(result["confirmation_min_gx_dps"], -50.0)
        self.assertFalse(config.confirmation_required)


if __name__ == "__main__":
    unittest.main()
