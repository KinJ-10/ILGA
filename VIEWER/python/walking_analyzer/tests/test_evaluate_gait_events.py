import argparse
import csv
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np


MODULE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_DIR))
import evaluate_gait_events as evaluator  # noqa: E402


class EvaluateGaitEventsTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.root = Path(self.temp_dir.name)

    def write_csv(self, name, fieldnames, rows):
        path = self.root / name
        with path.open("w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        return path

    def make_sensor(self, missing=()):
        missing = set(missing)
        rows = []
        for index in range(400):
            seq = 10000 + index
            if seq in missing:
                continue
            rows.append({"seq": seq, "gx_mdps": 0})
        return self.write_csv("sensor.csv", ("seq", "gx_mdps"), rows)

    def make_sync(self, points=None, include_phase=True):
        if points is None:
            points = (
                (1.0, 10010, "pre"),
                (1.2, 10030, "pre"),
                (1.4, 10050, "pre"),
                (4.2, 10330, "post"),
                (4.4, 10350, "post"),
                (4.6, 10370, "post"),
            )
        fieldnames = evaluator.SYNC_COLUMNS + (("phase",) if include_phase else ())
        return self.write_csv(
            "sync.csv",
            fieldnames,
            [
                {
                    "trial": "synthetic_trial",
                    "sync_index": index,
                    "video_frame": int(video_time * 120),
                    "video_time_sec": video_time,
                    "seq": seq,
                    "notes": "SYNTHETIC EXAMPLE",
                    **({"phase": phase} if include_phase else {}),
                }
                for index, (video_time, seq, phase) in enumerate(points, start=1)
            ],
        )

    def test_pre_post_sync_covers_evaluation_interval(self):
        sensor = evaluator.load_sensor_csv(self.make_sensor())
        sync, rows = evaluator.fit_sync(sensor, self.make_sync(), "synthetic_trial")
        coverage = evaluator.assess_sync_coverage(sync, rows, 1.8, 4.0)
        self.assertEqual((coverage.pre_point_count, coverage.post_point_count), (3, 3))
        self.assertTrue(coverage.standard_six_point_protocol)
        self.assertAlmostEqual(coverage.video_coverage_ratio, 1.0)
        self.assertAlmostEqual(coverage.sensor_coverage_ratio, 1.0)
        self.assertEqual(coverage.warnings, ())

    def test_legacy_six_point_csv_infers_pre_and_post(self):
        sensor = evaluator.load_sensor_csv(self.make_sensor())
        sync, rows = evaluator.fit_sync(
            sensor, self.make_sync(include_phase=False), "synthetic_trial"
        )
        coverage = evaluator.assess_sync_coverage(sync, rows, 1.8, 4.0)
        self.assertEqual(coverage.inferred_phase_point_count, 6)
        self.assertFalse(coverage.standard_six_point_protocol)
        self.assertTrue(any("phase inferred" in warning for warning in coverage.warnings))

    def test_pre_only_sync_is_rejected_as_insufficient_span(self):
        sensor = evaluator.load_sensor_csv(self.make_sensor())
        path = self.make_sync(
            points=((1.0, 10010, "pre"), (1.2, 10030, "pre"), (1.4, 10050, "pre"))
        )
        sync, rows = evaluator.fit_sync(sensor, path, "synthetic_trial")
        with self.assertRaisesRegex(ValueError, "both pre and post"):
            evaluator.assess_sync_coverage(sync, rows, 1.8, 4.0)

    def test_sync_residual_over_twenty_ms_is_rejected(self):
        sensor = evaluator.load_sensor_csv(self.make_sensor())
        path = self.make_sync(
            points=(
                (1.0, 10010, "pre"),
                (1.2, 10030, "pre"),
                (1.4, 10050, "pre"),
                (4.2, 10330, "post"),
                (4.4, 10350, "post"),
                (4.8, 10370, "post"),
            )
        )
        sync, rows = evaluator.fit_sync(sensor, path, "synthetic_trial")
        with self.assertRaisesRegex(ValueError, "sync residual exceeds limit"):
            evaluator.assess_sync_coverage(sync, rows, 1.8, 4.0)

    def test_tolerance_boundary_is_inclusive(self):
        matches, unmatched_d, unmatched_gt = evaluator.ordered_match([1.1], [1.0], 0.1)
        self.assertEqual(matches, [(0, 0)])
        self.assertEqual(unmatched_d, [])
        self.assertEqual(unmatched_gt, [])

    def test_duplicate_candidate_is_one_to_one(self):
        matches, unmatched_d, unmatched_gt = evaluator.ordered_match(
            [1.98, 2.02], [2.0], 0.1
        )
        self.assertEqual(len(matches), 1)
        self.assertEqual(len(unmatched_d), 1)
        self.assertEqual(unmatched_gt, [])

    def test_unmatched_ground_truth_is_reported(self):
        matches, unmatched_d, unmatched_gt = evaluator.ordered_match([1.0], [1.0, 2.0], 0.1)
        metric = evaluator.metric_row(
            "normal_contact",
            [{"event_video_time_sec": 1.0}],
            [{"video_time_sec": 1.0}, {"video_time_sec": 2.0}],
            matches,
        )
        self.assertEqual(unmatched_d, [])
        self.assertEqual(unmatched_gt, [1])
        self.assertEqual((metric["tp"], metric["fp"], metric["fn"]), (1, 0, 1))

    def test_missing_sample_near_event_is_flagged(self):
        sensor = evaluator.load_sensor_csv(self.make_sensor(missing={10120, 10121}))
        near, nearest, run = evaluator.missing_context(1.20, sensor, 0.10)
        self.assertTrue(near)
        self.assertAlmostEqual(nearest, 0.0)
        self.assertEqual(run, 2)

    def test_fixed_detector_uses_zero_cross_and_negative_confirmation(self):
        rows = []
        for index in range(500):
            time_sec = index / evaluator.FS_HZ
            gx_dps = (
                180.0 * math.exp(-((time_sec - 2.0) / 0.07) ** 2)
                - 120.0 * math.exp(-((time_sec - 2.24) / 0.08) ** 2)
            )
            rows.append({"seq": 10000 + index, "gx_mdps": round(gx_dps * 1000.0, 3)})
        sensor_path = self.write_csv("wave.csv", ("seq", "gx_mdps"), rows)
        sensor = evaluator.load_sensor_csv(sensor_path)
        sync = evaluator.SyncFit(1.0, 0.0, 0.0, 0.0, 3)
        candidates = evaluator.detect_fixed_candidates(sensor, sync, 0.10)
        formal = [row for row in candidates if row["formal"]]
        self.assertEqual(len(formal), 1)
        self.assertGreater(formal[0]["peak_gx_dps"], evaluator.PEAK_HEIGHT_DPS)
        self.assertLessEqual(formal[0]["confirmation_min_gx_dps"], evaluator.CONFIRM_GX_DPS)

    def test_end_to_end_metrics_separate_terminal_and_motion_context(self):
        sensor_path = self.make_sensor()
        sync_path = self.make_sync()
        gt_path = self.write_csv(
            "gt.csv",
            evaluator.GT_COLUMNS,
            [
                {"trial": "synthetic_trial", "event_index": 1, "foot": "left", "video_frame": 180, "video_time_sec": 1.5, "event_type": "normal_contact", "confidence": "high", "notes": "SYNTHETIC"},
                {"trial": "synthetic_trial", "event_index": 2, "foot": "right", "video_frame": 240, "video_time_sec": 2.0, "event_type": "normal_contact", "confidence": "high", "notes": "SYNTHETIC"},
                {"trial": "synthetic_trial", "event_index": 3, "foot": "right", "video_frame": 360, "video_time_sec": 3.0, "event_type": "terminal_contact", "confidence": "high", "notes": "SYNTHETIC"},
                {"trial": "synthetic_trial", "event_index": 4, "foot": "right", "video_frame": 396, "video_time_sec": 3.3, "event_type": "terminal_contact", "confidence": "high", "notes": "SYNTHETIC unmatched GT"},
                {"trial": "synthetic_trial", "event_index": 5, "foot": "right", "video_frame": 420, "video_time_sec": 3.5, "event_type": "adjustment", "confidence": "high", "notes": "SYNTHETIC"},
                {"trial": "synthetic_trial", "event_index": 6, "foot": "left", "video_frame": 456, "video_time_sec": 3.8, "event_type": "turn", "confidence": "high", "notes": "SYNTHETIC"},
            ],
        )
        fake_arms = []
        for index, video_time in enumerate((2.04, 2.10, 3.0, 3.5, 3.8), start=1):
            fake_arms.append(
                {
                    "candidate_index": index,
                    "peak_seq": 10000 + index,
                    "peak_sensor_time_sec": video_time - 0.9,
                    "peak_video_time_sec": video_time,
                    "peak_gx_dps": 120.0,
                    "prominence_dps": 80.0,
                    "zero_cross_sensor_time_sec": video_time - 0.9,
                    "zero_cross_video_time_sec": video_time,
                    "confirmation_min_gx_dps": -80.0,
                    "confirmation_pass": True,
                    "formal": True,
                    "failure_reason": "",
                    "missing_near": False,
                    "nearest_missing_sec": None,
                    "max_missing_run_near": 0,
                }
            )
        args = argparse.Namespace(
            sensor_csv=sensor_path,
            ground_truth_csv=gt_path,
            sync_points_csv=sync_path,
            trial="synthetic_trial",
            out_dir=self.root / "out",
            eval_start_video_sec=1.8,
            terminal_start_video_sec=2.5,
            eval_end_video_sec=4.0,
            tolerance_sec=0.10,
            missing_near_sec=0.10,
        )
        with mock.patch.object(evaluator, "detect_fixed_candidates", return_value=fake_arms):
            summary = evaluator.run(args)

        overall = summary["metrics"][0]
        self.assertEqual((overall["tp"], overall["fp"], overall["fn"]), (2, 3, 1))
        self.assertEqual(summary["false_positive_context"], {"adjustment": 1, "turn": 1, "other": 1})
        self.assertEqual(summary["ground_truth_all_feet_contact_count"], 4)
        self.assertTrue(summary["sync_coverage"]["standard_six_point_protocol"])
        self.assertEqual(summary["sync_coverage"]["pre_point_count"], 3)
        self.assertEqual(summary["sync_coverage"]["post_point_count"], 3)
        self.assertTrue((args.out_dir / "matches.csv").is_file())
        self.assertTrue((args.out_dir / "metrics.csv").is_file())
        self.assertIn("offline zero-phase", json.loads((args.out_dir / "summary.json").read_text())["mode"])


if __name__ == "__main__":
    unittest.main()
