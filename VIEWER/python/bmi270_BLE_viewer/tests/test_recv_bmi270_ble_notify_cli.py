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


if __name__ == "__main__":
    unittest.main()
