import argparse
import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


MODULE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = MODULE_DIR.parents[2]
sys.path.insert(0, str(MODULE_DIR))
import compute_functional_gait_metrics as metrics  # noqa: E402


class FunctionalGaitMetricsTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.root = Path(self.temp_dir.name)
        self.first_rx = 1_000_000_000

    def write_csv(self, name, fieldnames, rows):
        path = self.root / name
        with path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        return path

    def make_sensor(self, missing=(), invalid=False, rail_indices=()):
        missing = set(missing)
        rail_indices = set(rail_indices)
        rows = []
        for index in range(600):
            seq = 10000 + index
            if seq in missing:
                continue
            rows.append(
                {
                    "seq": seq,
                    "rx_monotonic_ns": self.first_rx + index * 10_000_000,
                    "rx_elapsed_ns": index * 10_000_000,
                    "ax_mg": "bad" if invalid and index == 25 else 10,
                    "ay_mg": 4000 if index in rail_indices else -20,
                    "az_mg": 1000,
                    "gx_mdps": 0,
                    "gy_mdps": 0,
                    "gz_mdps": 0,
                }
            )
        return self.write_csv(
            "sensor.csv",
            (
                "seq",
                "rx_monotonic_ns",
                "rx_elapsed_ns",
                "ax_mg",
                "ay_mg",
                "az_mg",
                "gx_mdps",
                "gy_mdps",
                "gz_mdps",
            ),
            rows,
        )

    def marker_row(self, event, elapsed_sec, index=1, **overrides):
        elapsed_ns = round(elapsed_sec * 1_000_000_000)
        row = {
            "schema_version": 1,
            "trial_name": "synthetic_trial",
            "event": event,
            "event_index": index,
            "marker_monotonic_ns": self.first_rx + elapsed_ns,
            "marker_elapsed_ns": elapsed_ns,
            "source": "operator_key",
            "notes": "SYNTHETIC TEST",
        }
        row.update(overrides)
        return row

    def make_markers(self, rows=None, fieldnames=metrics.MARKER_COLUMNS):
        if rows is None:
            rows = [self.marker_row("START", 1.0), self.marker_row("FINISH", 5.0, 2)]
        return self.write_csv("markers.csv", fieldnames, rows)

    def args(self, sensor=None, markers=None, distance=10.0, out_name="out", **kwargs):
        return argparse.Namespace(
            sensor_csv=sensor or self.make_sensor(),
            markers_csv=markers or self.make_markers(),
            distance_m=distance,
            out_dir=self.root / out_name,
            trial_name=kwargs.get("trial_name"),
            metadata_json=kwargs.get("metadata_json"),
            actual_steps=kwargs.get("actual_steps"),
            actual_steps_unknown=kwargs.get("actual_steps_unknown", False),
            start_foot=kwargs.get("start_foot"),
            end_foot=kwargs.get("end_foot"),
            sensor_foot=kwargs.get("sensor_foot", "right"),
            test_type=kwargs.get("test_type"),
            accel_range_g=4.0,
            gyro_range_dps=1000.0,
        )

    def candidates(self, event_times, missing_near_indices=()):
        rows = []
        for index, event_time in enumerate(event_times, start=1):
            rows.append(
                {
                    "candidate_index": index,
                    "peak_seq": 10000 + round(event_time * 100),
                    "peak_sensor_time_sec": event_time - 0.2,
                    "peak_video_time_sec": event_time - 0.2,
                    "peak_gx_dps": 140.0,
                    "prominence_dps": 80.0,
                    "zero_cross_sensor_time_sec": event_time,
                    "zero_cross_video_time_sec": event_time,
                    "confirmation_min_gx_dps": -80.0,
                    "confirmation_pass": True,
                    "formal": True,
                    "failure_reason": "",
                    "missing_near": index in set(missing_near_indices),
                    "nearest_missing_sec": 0.0 if index in set(missing_near_indices) else None,
                    "max_missing_run_near": 1 if index in set(missing_near_indices) else 0,
                }
            )
        return rows

    def run_with_candidates(self, args, event_times, missing_near_indices=()):
        with mock.patch.object(
            metrics.fixed_detector,
            "detect_fixed_candidates",
            return_value=self.candidates(event_times, missing_near_indices),
        ):
            return metrics.run(args)

    def test_normal_start_finish_outputs_speed_and_period_metrics(self):
        args = self.args(actual_steps=7, start_foot="right", end_foot="right")
        summary = self.run_with_candidates(args, [0.5, 2.0, 3.0, 4.0, 5.5])
        self.assertEqual(summary["quality"]["status"], "ok")
        self.assertAlmostEqual(summary["timing"]["marked_duration_sec"], 4.0)
        self.assertAlmostEqual(summary["timing"]["speed_mps"], 2.5)
        self.assertEqual(summary["stride_metrics"]["stride_marker_count"], 3)
        self.assertAlmostEqual(summary["stride_metrics"]["median_stride_time_sec"], 1.0)
        self.assertAlmostEqual(summary["stride_metrics"]["robust_cadence_steps_per_min"], 120.0)
        self.assertAlmostEqual(summary["stride_metrics"]["raw_cv_pct"], 0.0)
        self.assertTrue(summary["reference_step_evaluation"]["evaluated"])
        self.assertEqual(summary["reference_step_evaluation"]["estimated_minus_actual_steps"], -2)
        finish_after = next(
            row
            for row in summary["boundary_diagnostics"]
            if row["boundary"] == "FINISH"
            and row["side"] == "after"
            and row["event_kind"] == "formal_stride_marker"
        )
        self.assertTrue(finish_after["found"])
        self.assertAlmostEqual(finish_after["signed_time_diff_sec"], 0.5)
        self.assertFalse(finish_after["adopted_as_stride_marker"])
        self.assertEqual(finish_after["exclusion_reason"], "after_finish")
        for name in (
            "summary.json",
            "summary.csv",
            "summary.txt",
            "stride_candidates.csv",
            "stride_markers.csv",
            "stride_intervals.csv",
            "boundary_diagnostics.csv",
        ):
            self.assertTrue((args.out_dir / name).is_file())
        self.assertNotIn("right_initial_contact", (args.out_dir / "summary.txt").read_text())
        text = (args.out_dir / "summary.txt").read_text()
        self.assertIn("stride_marker_count: 3", text)
        self.assertIn("estimated_total_steps_reference_only: 5", text)
        self.assertIn("actual_steps: 7", text)
        self.assertIn("not a detected or confirmed total step count", text)

    def test_missing_finish_is_quality_error_without_speed(self):
        markers = self.make_markers([self.marker_row("START", 1.0)])
        summary = self.run_with_candidates(self.args(markers=markers), [2.0, 3.0])
        self.assertEqual(summary["quality"]["status"], "error")
        self.assertIn("marker_missing_finish", summary["quality"]["errors"])
        self.assertIsNone(summary["timing"]["speed_mps"])
        self.assertIsNone(summary["stride_metrics"]["stride_marker_count"])

    def test_duplicate_start_is_quality_error_without_speed(self):
        markers = self.make_markers(
            [
                self.marker_row("START", 1.0, 1),
                self.marker_row("START", 1.2, 2),
                self.marker_row("FINISH", 5.0, 3),
            ]
        )
        summary = self.run_with_candidates(self.args(markers=markers), [2.0, 3.0])
        self.assertIn("marker_duplicate_start", summary["quality"]["errors"])
        self.assertIsNone(summary["timing"]["speed_mps"])

    def test_capture_rejected_duplicate_note_is_quality_error(self):
        markers = self.make_markers(
            [
                self.marker_row("START", 1.0, 1, notes="duplicate_start_rejected"),
                self.marker_row("FINISH", 5.0, 2),
            ]
        )
        summary = self.run_with_candidates(self.args(markers=markers), [2.0, 3.0])
        self.assertIn("marker_duplicate_start_rejected", summary["quality"]["errors"])
        self.assertIsNone(summary["timing"]["speed_mps"])

    def test_missing_elapsed_clock_reference_is_quality_error(self):
        markers = self.make_markers(
            [
                self.marker_row("START", 1.0, 1, marker_elapsed_ns="", notes="missing_rx_reference"),
                self.marker_row("FINISH", 5.0, 2, marker_elapsed_ns="", notes="missing_rx_reference"),
            ]
        )
        summary = self.run_with_candidates(self.args(markers=markers), [2.0, 3.0])
        self.assertIn("marker_elapsed_missing", summary["quality"]["errors"])
        self.assertIsNone(summary["timing"]["speed_mps"])

    def test_reverse_start_finish_is_quality_error_without_speed(self):
        markers = self.make_markers(
            [self.marker_row("START", 5.0, 1), self.marker_row("FINISH", 1.0, 2)]
        )
        summary = self.run_with_candidates(self.args(markers=markers), [2.0, 3.0])
        self.assertIn("marker_order_invalid", summary["quality"]["errors"])
        self.assertIsNone(summary["timing"]["speed_mps"])

    def test_zero_distance_is_quality_error_without_speed(self):
        summary = self.run_with_candidates(self.args(distance=0.0), [2.0, 3.0])
        self.assertIn("distance_nonpositive_or_nonfinite", summary["quality"]["errors"])
        self.assertIsNone(summary["timing"]["speed_mps"])

    def test_invalid_marker_schema_is_rejected_before_output(self):
        markers = self.make_markers(
            [self.marker_row("START", 1.0, schema_version=2), self.marker_row("FINISH", 5.0, 2)]
        )
        args = self.args(markers=markers)
        with self.assertRaisesRegex(ValueError, "schema_version"):
            metrics.run(args)
        self.assertFalse(args.out_dir.exists())

    def test_marker_count_minimums_control_availability(self):
        one = self.run_with_candidates(self.args(out_name="one"), [2.0])
        self.assertIsNone(one["stride_metrics"]["median_stride_time_sec"])
        self.assertIsNone(one["stride_metrics"]["robust_cadence_steps_per_min"])
        self.assertIsNone(one["stride_metrics"]["sd_stride_time_sec"])
        two = self.run_with_candidates(self.args(out_name="two"), [2.0, 3.0])
        self.assertAlmostEqual(two["stride_metrics"]["median_stride_time_sec"], 1.0)
        self.assertAlmostEqual(two["stride_metrics"]["robust_cadence_steps_per_min"], 120.0)
        self.assertIsNone(two["stride_metrics"]["sd_stride_time_sec"])
        self.assertIsNone(two["stride_metrics"]["raw_cv_pct"])

    def test_nonformal_candidate_keeps_detector_exclusion_reason(self):
        candidate = self.candidates([2.0])[0]
        candidate["formal"] = False
        candidate["confirmation_pass"] = False
        candidate["failure_reason"] = "no_negative_confirmation"
        args = self.args()
        with mock.patch.object(
            metrics.fixed_detector, "detect_fixed_candidates", return_value=[candidate]
        ):
            summary = metrics.run(args)
        with (args.out_dir / "stride_candidates.csv").open() as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(rows[0]["candidate_formal"], "False")
        self.assertEqual(rows[0]["exclusion_reason"], "no_negative_confirmation")
        self.assertEqual(summary["stride_metrics"]["stride_marker_count"], 0)

    def test_double_period_is_flagged_without_calling_it_a_stop(self):
        summary = self.run_with_candidates(self.args(), [1.2, 2.2, 4.2, 4.9])
        stride = summary["stride_metrics"]
        self.assertEqual(stride["interval_outlier_count"], 1)
        self.assertEqual(stride["pause_or_missed_marker_count"], 1)
        self.assertIn("pause_or_missed_marker", summary["quality"]["warnings"])
        with (self.root / "out" / "stride_intervals.csv").open() as handle:
            intervals = list(csv.DictReader(handle))
        flagged = [row for row in intervals if row["pause_or_missed_marker"] == "True"]
        self.assertEqual(len(flagged), 1)

    def test_missing_seq_and_nearby_marker_are_quality_flags(self):
        sensor = self.make_sensor(missing={10200})
        summary = self.run_with_candidates(
            self.args(sensor=sensor), [2.0, 3.0, 4.0], missing_near_indices={1}
        )
        self.assertEqual(summary["sensor_quality"]["missing_seq_count"], 1)
        self.assertIn("missing_seq", summary["quality"]["warnings"])
        self.assertIn("stride_marker_near_missing_seq", summary["quality"]["warnings"])

    def test_invalid_sensor_row_is_rejected_before_output(self):
        args = self.args(sensor=self.make_sensor(invalid=True))
        with self.assertRaisesRegex(ValueError, "invalid ax_mg"):
            metrics.run(args)
        self.assertFalse(args.out_dir.exists())

    def test_metadata_is_used_and_marker_count_is_not_primary_steps(self):
        metadata_path = self.root / "metadata.json"
        metadata_path.write_text(
            json.dumps(
                {
                    "trial_name": "synthetic_trial",
                    "actual_steps": 7,
                    "start_foot": "Left",
                    "end_foot": "Right",
                    "metrics": {
                        "invalid_sensor_samples": 1,
                        "sensor_fault": 0,
                        "marker_valid": 1,
                    },
                }
            )
        )
        summary = self.run_with_candidates(
            self.args(metadata_json=metadata_path), [2.0, 3.0, 4.0]
        )
        reference = summary["reference_step_evaluation"]
        self.assertFalse(reference["is_primary_metric"])
        self.assertEqual(reference["estimated_total_steps_reference_only"], 6)
        self.assertEqual(reference["estimated_minus_actual_steps"], -1)
        self.assertIn("capture_invalid_sensor_samples", summary["quality"]["warnings"])

    def test_sensor_foot_controls_reference_conversion_and_unknown_disables_it(self):
        left = self.run_with_candidates(
            self.args(
                out_name="left_sensor",
                actual_steps=15,
                start_foot="left",
                end_foot="left",
                sensor_foot="left",
            ),
            [2.0, 3.0, 4.0],
        )
        reference = left["reference_step_evaluation"]
        self.assertEqual(reference["sensor_foot"], "left")
        self.assertEqual(reference["expected_sensor_foot_contacts_reference"], 8)
        self.assertEqual(reference["estimated_total_steps_reference_only"], 5)
        self.assertEqual(reference["estimation_formula"], "sensor=left, left->left: 2L-1")

        unknown = self.run_with_candidates(
            self.args(
                out_name="unknown_sensor",
                actual_steps=15,
                start_foot="left",
                end_foot="left",
                sensor_foot="unknown",
            ),
            [2.0, 3.0, 4.0],
        )
        self.assertIsNone(
            unknown["reference_step_evaluation"]["estimated_total_steps_reference_only"]
        )
        self.assertIn(
            "sensor_foot_unknown_reference_steps_disabled", unknown["quality"]["warnings"]
        )

    def test_rail_counts_are_split_inside_and_outside_marked_interval(self):
        sensor = self.make_sensor(rail_indices={50, 200, 250})
        summary = self.run_with_candidates(self.args(sensor=sensor), [2.0, 3.0, 4.0])
        ranges = summary["sensor_quality"]["range_by_interval"]
        self.assertEqual(ranges["full"]["rail_counts"]["ay_mg"], 3)
        self.assertEqual(ranges["marked_interval"]["rail_counts"]["ay_mg"], 2)
        self.assertEqual(ranges["outside_marked_interval"]["rail_counts"]["ay_mg"], 1)
        self.assertAlmostEqual(
            ranges["marked_interval"]["rail_ratios"]["ay_mg"],
            2 / ranges["marked_interval"]["sample_count"],
        )

    def test_tug_outputs_operator_total_time_without_stride_or_phases(self):
        args = self.args(distance=None, test_type="tug", sensor_foot="unknown")
        with mock.patch.object(
            metrics.fixed_detector, "detect_fixed_candidates"
        ) as detector:
            summary = metrics.run(args)
        detector.assert_not_called()
        self.assertEqual(summary["test_type"], "tug")
        self.assertAlmostEqual(summary["timing"]["marked_duration_sec"], 4.0)
        self.assertIsNone(summary["timing"]["speed_mps"])
        self.assertIsNone(summary["stride_metrics"]["stride_marker_count"])
        self.assertTrue(summary["tug_scope"]["total_time_implemented"])
        self.assertFalse(summary["tug_scope"]["phase_detection_implemented"])
        self.assertTrue(summary["tug_scope"]["foot_tag_does_not_define_seat_off_or_sit_contact"])

    def test_metadata_marker_invalid_blocks_speed_and_stride_metrics(self):
        metadata_path = self.root / "invalid_marker_metadata.json"
        metadata_path.write_text(
            json.dumps(
                {
                    "trial_name": "synthetic_trial",
                    "metrics": {"marker_valid": 0},
                }
            )
        )
        summary = self.run_with_candidates(
            self.args(metadata_json=metadata_path), [2.0, 3.0, 4.0]
        )
        self.assertIn("metadata_marker_invalid", summary["quality"]["errors"])
        self.assertIsNone(summary["timing"]["speed_mps"])
        self.assertIsNone(summary["stride_metrics"]["stride_marker_count"])

    def pilot_args(self, suffix, **overrides):
        stem = f"20260902_ga_phase1a_S01_10m_marker_pilot_{suffix}"
        log_root = REPO_ROOT / "logs" / "ble"
        return argparse.Namespace(
            sensor_csv=log_root / f"{stem}.csv",
            markers_csv=log_root / f"{stem}_markers.csv",
            distance_m=10.0,
            out_dir=self.root / f"pilot_{suffix}",
            trial_name=f"S01_10m_marker_pilot_{suffix}",
            metadata_json=log_root / f"{stem}_metadata.json",
            actual_steps=overrides.get("actual_steps"),
            actual_steps_unknown=overrides.get("actual_steps_unknown", False),
            start_foot=overrides.get("start_foot"),
            end_foot=overrides.get("end_foot"),
            sensor_foot=overrides.get("sensor_foot", "right"),
            test_type=overrides.get("test_type"),
            accel_range_g=4.0,
            gyro_range_dps=1000.0,
        )

    @unittest.skipUnless(
        (REPO_ROOT / "logs" / "ble" / "20260902_ga_phase1a_S01_10m_marker_pilot_01.csv").is_file(),
        "pilot_01 fixture is not available",
    )
    def test_pilot_01_regression_ignores_uncertain_step_and_end_foot(self):
        summary = metrics.run(
            self.pilot_args(
                "01",
                actual_steps_unknown=True,
                start_foot="right",
                end_foot="unknown",
            )
        )
        self.assertAlmostEqual(summary["timing"]["marked_duration_sec"], 8.859, places=6)
        self.assertAlmostEqual(summary["timing"]["speed_mps"], 10.0 / 8.859, places=9)
        self.assertEqual(summary["stride_metrics"]["stride_marker_count"], 7)
        self.assertAlmostEqual(
            summary["stride_metrics"]["median_stride_time_sec"],
            1.1784488288731767,
            places=9,
        )
        self.assertAlmostEqual(
            summary["stride_metrics"]["robust_cadence_steps_per_min"],
            101.82877445323021,
            places=8,
        )
        reference = summary["reference_step_evaluation"]
        self.assertIsNone(reference["actual_steps"])
        self.assertEqual(reference["actual_steps_source"], "explicit_unknown")
        self.assertEqual(reference["end_foot"], "unknown")
        self.assertIsNone(reference["estimated_total_steps_reference_only"])
        self.assertFalse(reference["evaluated"])

    @unittest.skipUnless(
        (REPO_ROOT / "logs" / "ble" / "20260902_ga_phase1a_S01_10m_marker_pilot_02.csv").is_file(),
        "pilot_02 fixture is not available",
    )
    def test_pilot_02_regression_and_finish_after_marker(self):
        summary = metrics.run(self.pilot_args("02"))
        self.assertAlmostEqual(summary["timing"]["marked_duration_sec"], 7.937, places=6)
        self.assertAlmostEqual(summary["timing"]["speed_mps"], 10.0 / 7.937, places=9)
        self.assertEqual(summary["stride_metrics"]["stride_marker_count"], 7)
        self.assertAlmostEqual(
            summary["stride_metrics"]["median_stride_time_sec"],
            1.1251708554700288,
            places=9,
        )
        self.assertAlmostEqual(
            summary["stride_metrics"]["robust_cadence_steps_per_min"],
            106.65046949680473,
            places=8,
        )
        self.assertEqual(summary["stride_metrics"]["interval_outlier_count"], 0)
        self.assertEqual(summary["stride_metrics"]["pause_or_missed_marker_count"], 0)
        reference = summary["reference_step_evaluation"]
        self.assertEqual(reference["actual_steps"], 16)
        self.assertEqual(reference["start_foot"], "right")
        self.assertEqual(reference["end_foot"], "left")
        self.assertEqual(reference["estimated_total_steps_reference_only"], 14)
        self.assertEqual(reference["estimated_minus_actual_steps"], -2)
        finish_after = next(
            row
            for row in summary["boundary_diagnostics"]
            if row["boundary"] == "FINISH"
            and row["side"] == "after"
            and row["event_kind"] == "formal_stride_marker"
        )
        self.assertTrue(finish_after["found"])
        self.assertGreater(finish_after["signed_time_diff_sec"], 0.30)
        self.assertLess(finish_after["signed_time_diff_sec"], 0.40)
        self.assertFalse(finish_after["adopted_as_stride_marker"])
        self.assertEqual(finish_after["exclusion_reason"], "after_finish")

    @unittest.skipUnless(
        (
            REPO_ROOT
            / "logs"
            / "ble"
            / "20260903_ga_phase1a_S01_10m_leftsensor_normal_01.csv"
        ).is_file(),
        "left-sensor fixture is not available",
    )
    def test_left_sensor_trial_is_preliminary_and_not_accidental_right_formula_success(self):
        log_root = REPO_ROOT / "logs" / "ble"
        stem = "20260903_ga_phase1a_S01_10m_leftsensor_normal_01"
        args = argparse.Namespace(
            sensor_csv=log_root / f"{stem}.csv",
            markers_csv=log_root / f"{stem}_markers.csv",
            distance_m=10.0,
            out_dir=self.root / "left_sensor_real",
            trial_name="S01_10m_leftsensor_normal_01",
            metadata_json=log_root / f"{stem}_metadata.json",
            actual_steps=None,
            actual_steps_unknown=False,
            start_foot=None,
            end_foot=None,
            sensor_foot="left",
            test_type="walk10m",
            accel_range_g=4.0,
            gyro_range_dps=1000.0,
        )
        summary = metrics.run(args)
        reference = summary["reference_step_evaluation"]
        self.assertEqual(summary["stride_metrics"]["stride_marker_count"], 7)
        self.assertEqual(reference["expected_sensor_foot_contacts_reference"], 8)
        self.assertEqual(reference["estimated_total_steps_reference_only"], 13)
        self.assertEqual(reference["estimated_minus_actual_steps"], -2)


if __name__ == "__main__":
    unittest.main()
