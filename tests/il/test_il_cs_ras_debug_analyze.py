from __future__ import annotations

import io
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from il_cs_ras_debug_analyze import (  # noqa: E402
    C_M_PER_S,
    analyze_frame,
    early_peak,
    fnv1a_lines,
    parse_frames,
    robust_phase_slope,
)
from il_cs_ras_debug_profile import frame_profile, local_maxima  # noqa: E402
from il_cs_ras_debug_compare import endpoint_calibration  # noqa: E402


def synthetic_frame(distance_m: float = 1.0) -> dict:
    tones = {}
    for channel in range(2, 77):
        frequency = (channel - 2) * 1_000_000
        phase = -4 * math.pi * frequency * distance_m / C_M_PER_S
        tones[channel] = [0, 0, 1_000_000, 0,
                          round(1_000_000 * math.cos(phase)),
                          round(1_000_000 * math.sin(phase))]
    return {"header": [10, 42, 1, 0, 0, 7, 3, 1000, 1000, 1000, 1000, 0, 0],
            "tones": tones}


class RasDebugAnalyzeTests(unittest.TestCase):
    def test_complete_frame_and_checksum_required(self) -> None:
        frame = synthetic_frame()
        header = "ILRAS1,S," + ",".join(map(str, frame["header"]))
        lines = [header]
        for channel, values in frame["tones"].items():
            lines.append("ILRAS1,T,10," + str(channel) + "," + ",".join(map(str, values)))
        end = f"ILRAS1,E,10,75,{fnv1a_lines(lines):08x}"
        log = "".join("2026-09-25T00:00:00\t" + row + "\n" for row in [*lines, end])
        with patch.object(Path, "open", return_value=io.StringIO(log)):
            frames, errors = parse_frames(Path("unused.log"))
        self.assertEqual(len(frames), 1)
        self.assertFalse(errors)
        corrupt = log.replace("ILRAS1,T,10,2,0", "ILRAS1,T,10,2,1")
        with patch.object(Path, "open", return_value=io.StringIO(corrupt)):
            frames, errors = parse_frames(Path("unused.log"))
        self.assertFalse(frames)
        self.assertIn("checksum", errors[0]["reason"])

    def test_recalculate_one_meter_from_quantized_iq(self) -> None:
        summary, tones, peaks = analyze_frame(synthetic_frame())
        self.assertEqual(len(tones), 75)
        self.assertTrue(peaks)
        self.assertEqual(summary["good_tone_count"], 75)
        self.assertAlmostEqual(summary["nordic_phase_slope_recalc_m"], 1.0, delta=0.01)
        self.assertAlmostEqual(summary["robust_phase_slope_m"], 1.0, delta=0.01)
        self.assertAlmostEqual(summary["nordic_ifft_recalc_m"], 1.0, delta=0.3)

    def test_early_peak_prefers_earlier_valid_path(self) -> None:
        magnitude = np.full(512, 0.01)
        magnitude[3:6] = [0.3, 0.55, 0.35]
        magnitude[19:22] = [0.6, 1.0, 0.7]
        self.assertEqual(early_peak(magnitude), 4)

    def test_robust_slope_reduces_outlier_error(self) -> None:
        n = np.arange(75)
        z = np.exp(-1j * 4 * math.pi * n * 1_000_000 * 1.0 / C_M_PER_S)
        z[40] = 10 * np.exp(1j * 2.5)
        good = np.ones(75, dtype=bool)
        good[40] = False
        robust, count = robust_phase_slope(z, good)
        self.assertEqual(count, 74)
        self.assertAlmostEqual(robust, 1.0, delta=0.05)

    def test_full_profile_preserves_synthetic_distance_peak(self) -> None:
        profile, selected = frame_profile(synthetic_frame())
        strongest = int(np.argmax(profile[:50]))
        self.assertAlmostEqual(strongest * C_M_PER_S / (2 * 512 * 1_000_000), 1.0, delta=0.3)
        self.assertIn(strongest, local_maxima(profile[:50]))
        self.assertAlmostEqual(selected * C_M_PER_S / (2 * 512 * 1_000_000), 1.0, delta=0.4)

    def test_endpoint_calibration_holds_out_middle_distance(self) -> None:
        gain, offset = endpoint_calibration(1.5, 0.5, 5.0, 1.5)
        self.assertAlmostEqual(gain * 3.25 + offset, 1.0)


if __name__ == "__main__":
    unittest.main()
