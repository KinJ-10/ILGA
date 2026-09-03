import ast
import contextlib
import csv
import importlib.util
import io
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


VIEWER = Path(__file__).resolve().parents[1] / "recv_bmi270_ble_notify_cli.py"
ANALYZER = (
    Path(__file__).resolve().parents[2]
    / "walking_analyzer"
    / "analyze_single_leg_csv.py"
)

SPEC = importlib.util.spec_from_file_location("recv_bmi270_ble_notify_cli", VIEWER)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def payload(seq: int) -> bytearray:
    return bytearray(
        struct.pack("<BIhhhiii", 1, seq, 100, 200, 1000, 1000, 2000, 3000)
    )


class ReceiveTimestampTests(unittest.TestCase):
    def test_callback_captures_monotonic_ns_before_other_work(self):
        tree = ast.parse(VIEWER.read_text(encoding="utf-8"))
        run_function = next(
            node
            for node in tree.body
            if isinstance(node, ast.AsyncFunctionDef) and node.name == "run"
        )
        callback = next(
            node
            for node in run_function.body
            if isinstance(node, ast.FunctionDef) and node.name == "on_stream"
        )
        first_statement = callback.body[0]
        self.assertIsInstance(first_statement, ast.Assign)
        call = first_statement.value
        self.assertIsInstance(call, ast.Call)
        self.assertIsInstance(call.func, ast.Attribute)
        self.assertEqual(call.func.attr, "monotonic_ns")

    def test_csv_timestamps_and_summary_statistics(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "samples.csv"
            session = MODULE.ReceiveSession(str(csv_path))
            session.start_capture()
            timestamps = (
                1_000_000_000,
                1_010_000_000,
                1_030_000_000,
                1_040_000_000,
            )
            for seq, timestamp_ns in enumerate(timestamps):
                self.assertFalse(session.handle_stream(payload(seq), timestamp_ns))
            session.finish()

            with csv_path.open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))

            self.assertEqual(list(rows[0]), MODULE.CSV_FIELDNAMES)
            self.assertEqual(
                [int(row["rx_elapsed_ns"]) for row in rows],
                [0, 10_000_000, 30_000_000, 40_000_000],
            )

            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                session.print_summary()
            summary = output.getvalue()
            self.assertIn("rx_inter_arrival_median_ns=10000000", summary)
            self.assertIn("rx_inter_arrival_p95_ns=19000000", summary)
            self.assertIn("rx_inter_arrival_max_ns=20000000", summary)
            self.assertIn("rx_timestamp_effective_hz=75.000", summary)

    def test_analyzer_accepts_timestamp_columns(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            csv_path = root / "samples.csv"
            output_dir = root / "analysis"
            with csv_path.open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file, fieldnames=MODULE.CSV_FIELDNAMES)
                writer.writeheader()
                for index in range(100):
                    writer.writerow(
                        {
                            "seq": index,
                            "rx_monotonic_ns": 1_000_000_000 + index * 10_000_000,
                            "rx_elapsed_ns": index * 10_000_000,
                            "ax_mg": 1000,
                            "ay_mg": 0,
                            "az_mg": 0,
                            "gx_mdps": 0,
                            "gy_mdps": 0,
                            "gz_mdps": 0,
                        }
                    )

            result = subprocess.run(
                [
                    sys.executable,
                    str(ANALYZER),
                    str(csv_path),
                    "--fs",
                    "100",
                    "--out-dir",
                    str(output_dir),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue((output_dir / "summary.json").is_file())
            self.assertTrue((output_dir / "step_events.csv").is_file())


class OperatorMarkerTests(unittest.TestCase):
    def test_marker_arguments_are_opt_in(self):
        parser = MODULE.build_arg_parser()
        defaults = parser.parse_args([])
        self.assertIsNone(defaults.markers_csv)
        self.assertIsNone(defaults.trial_name)
        enabled = parser.parse_args(
            ["--markers-csv", "markers.csv", "--trial-name", "trial_01"]
        )
        self.assertEqual(enabled.markers_csv, "markers.csv")
        self.assertEqual(enabled.trial_name, "trial_01")

    def test_valid_markers_use_first_rx_clock_and_schema(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "markers.csv"
            recorder = MODULE.OperatorMarkerRecorder(str(path), "walk_01")
            self.assertTrue(recorder.record_event("START", 1_100_000_000))
            self.assertTrue(recorder.record_event("FINISH", 1_600_000_000))
            status = recorder.status(1_000_000_000)
            self.assertTrue(status.marker_valid)
            self.assertEqual(status.start_count, 1)
            self.assertEqual(status.finish_count, 1)
            self.assertAlmostEqual(status.marked_duration_sec, 0.5)
            recorder.write_csv(1_000_000_000)

            with path.open(newline="", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual(list(rows[0]), MODULE.MARKER_FIELDNAMES)
            self.assertEqual([row["schema_version"] for row in rows], ["1", "1"])
            self.assertEqual([row["trial_name"] for row in rows], ["walk_01", "walk_01"])
            self.assertEqual([row["event"] for row in rows], ["START", "FINISH"])
            self.assertEqual([row["event_index"] for row in rows], ["1", "2"])
            self.assertEqual(
                [int(row["marker_elapsed_ns"]) for row in rows],
                [100_000_000, 600_000_000],
            )
            self.assertEqual([row["source"] for row in rows], ["operator_key", "operator_key"])

    def test_duplicate_is_rejected_without_overwrite(self):
        recorder = MODULE.OperatorMarkerRecorder("unused.csv", "walk_02")
        with contextlib.redirect_stderr(io.StringIO()) as warning:
            self.assertTrue(recorder.record_event("START", 100))
            self.assertFalse(recorder.record_event("START", 200))
        self.assertIn("duplicate START rejected", warning.getvalue())
        recorder.record_event("FINISH", 300)
        status = recorder.status(0)
        self.assertEqual(status.start_count, 1)
        self.assertEqual(status.finish_count, 1)
        self.assertFalse(status.marker_valid)
        self.assertIsNone(status.marked_duration_sec)
        self.assertIn("duplicate_start_rejected", status.issues)

    def test_finish_before_start_and_incomplete_markers_are_invalid(self):
        reverse = MODULE.OperatorMarkerRecorder("unused.csv", "reverse")
        reverse.record_event("FINISH", 100)
        reverse.record_event("START", 200)
        reverse_status = reverse.status(0)
        self.assertFalse(reverse_status.marker_valid)
        self.assertIn("finish_not_after_start", reverse_status.issues)

        before_rx = MODULE.OperatorMarkerRecorder("unused.csv", "before_rx")
        before_rx.record_event("START", 99)
        before_rx.record_event("FINISH", 110)
        before_rx_status = before_rx.status(100)
        self.assertFalse(before_rx_status.marker_valid)
        self.assertIsNone(before_rx_status.marked_duration_sec)
        self.assertIn("marker_before_first_rx", before_rx_status.issues)

        cases = (
            (("START",), "missing_finish"),
            (("FINISH",), "missing_start"),
            ((), "missing_start"),
        )
        for events, expected_issue in cases:
            with self.subTest(events=events):
                recorder = MODULE.OperatorMarkerRecorder("unused.csv", "incomplete")
                for index, event in enumerate(events):
                    recorder.record_event(event, 100 + index)
                status = recorder.status(0)
                self.assertFalse(status.marker_valid)
                self.assertIn(expected_issue, status.issues)

    def test_summary_adds_marker_fields_and_default_remains_disabled(self):
        default_session = MODULE.ReceiveSession(None)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            default_session.print_summary()
        default_summary = output.getvalue()
        self.assertIn("markers_file='None'", default_summary)
        self.assertIn("start_count=0", default_summary)
        self.assertIn("finish_count=0", default_summary)
        self.assertIn("marker_valid=NA", default_summary)
        self.assertIn("marked_duration_sec=NA", default_summary)

        with tempfile.TemporaryDirectory() as temp_dir:
            marker_path = Path(temp_dir) / "markers.csv"
            recorder = MODULE.OperatorMarkerRecorder(str(marker_path), "walk_03")
            session = MODULE.ReceiveSession(None, recorder)
            session.start_capture()
            with contextlib.redirect_stdout(io.StringIO()):
                session.handle_stream(payload(1), 1_000_000_000)
                recorder.record_event("START", 1_100_000_000)
                recorder.record_event("FINISH", 1_600_000_000)
            session.finish()
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                session.print_summary()
            summary = output.getvalue()
            self.assertIn(f"markers_file='{marker_path}'", summary)
            self.assertIn("start_count=1", summary)
            self.assertIn("finish_count=1", summary)
            self.assertIn("marker_valid=1", summary)
            self.assertIn("marked_duration_sec=0.500000", summary)

    def test_key_listener_captures_monotonic_ns_immediately_after_key(self):
        tree = ast.parse(VIEWER.read_text(encoding="utf-8"))
        listener = next(
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == "WindowsMarkerKeyListener"
        )
        run_method = next(
            node
            for node in listener.body
            if isinstance(node, ast.FunctionDef) and node.name == "_run"
        )
        while_loop = next(node for node in ast.walk(run_method) if isinstance(node, ast.While))
        key_assignment_index = next(
            index
            for index, node in enumerate(while_loop.body)
            if isinstance(node, ast.Assign)
            and isinstance(node.targets[0], ast.Name)
            and node.targets[0].id == "key"
        )
        timestamp_assignment = while_loop.body[key_assignment_index + 1]
        self.assertIsInstance(timestamp_assignment, ast.Assign)
        self.assertEqual(timestamp_assignment.targets[0].id, "marker_monotonic_ns")
        self.assertEqual(timestamp_assignment.value.func.attr, "monotonic_ns")


if __name__ == "__main__":
    unittest.main()
