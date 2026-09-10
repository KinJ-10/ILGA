"""Lifecycle, metric provenance and local HTTP security without physical BLE."""
import asyncio
import csv
import http.client
import json
import struct
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import capture
import reporting
from gait_demo_server import Server
import gait_demo_server as web

PACKET = struct.Struct("<BIhhhiii")


def packet(seq):
    return bytearray(PACKET.pack(1, seq, 0, 0, 1000, 0, 0, 0))


def summary(count=7, valid=True):
    return {
        "timing": {"marked_duration_sec": 8, "speed_mps": 1.25},
        "stride_metrics": {"stride_marker_count": count, "mean_stride_time_sec": 1.1,
                           "robust_cadence_steps_per_min": None, "inlier_cv_pct": None},
        "reference_step_evaluation": {"actual_steps": 14, "estimated_total_steps_reference_only": None},
        "quality": {"status": "ok" if valid else "error", "flags": []},
    }


class MetricTests(unittest.TestCase):
    def test_provenance_and_unknowns(self):
        view = reporting.view_model(summary(), {})
        self.assertEqual(view["estimated_steps"], 14)
        self.assertAlmostEqual(view["step_length_measured_m"], 10 / 14)
        self.assertAlmostEqual(view["stride_length_reference_m"], 1.375)
        self.assertIsNone(view["cadence"])
        self.assertIsNone(view["cv_pct"])
        self.assertTrue(any("足踏み" in w for w in view["warnings"]))

    def test_invalid_and_no_events(self):
        view = reporting.view_model(summary(valid=False), {})
        for key in ("duration_sec", "speed_mps", "right_stride_candidates", "estimated_steps",
                    "step_length_measured_m", "stride_length_reference_m"):
            self.assertIsNone(view[key], key)
        self.assertEqual(view["actual_steps"], 14)
        self.assertIsNone(reporting.view_model(summary(0), {})["estimated_steps"])

    def test_html_escape(self):
        with tempfile.TemporaryDirectory() as tmp:
            data = summary()
            data["demo"] = reporting.view_model(data, {})
            reporting.render_report(Path(tmp), data, {"trial_name": "<script>alert(1)</script>"})
            page = (Path(tmp) / "report.html").read_text(encoding="utf-8")
            self.assertNotIn("<script>", page)
            self.assertIn("&lt;script&gt;", page)
            self.assertIn("解析できません", page)
            self.assertLess(page.index('<img src="waveform.png"'), page.index('<ul class="cards">'))
            for old in ("stride", "cadence", "CV", "品質", "欠番", "10m歩行の参考結果"):
                self.assertNotIn(old, page)
            self.assertIn("測定状態：確認事項あり", page)

    def test_validation(self):
        for value in (-1, 0, "1.2", True, "nan", "inf"):
            with self.assertRaises(ValueError):
                capture.observations({"actual_steps": value})
        with self.assertRaises(ValueError):
            capture.observations({"sync_pre": -1})


class LifecycleTests(unittest.TestCase):
    def test_invalid_quality_is_exposed(self):
        async def receiver(session):
            session.accept(bytearray(b"bad"))
            for seq in range(10):
                session.accept(bytearray(PACKET.pack(1, seq, 0, 0, 0, 0, 0, 0)))

        with tempfile.TemporaryDirectory() as tmp:
            session = capture.Session(tmp, receiver)
            before = session.snapshot()
            self.assertEqual(before["invalid_sensor_samples"], 0)
            self.assertEqual(before["invalid_payloads"], 0)
            self.assertFalse(before["sensor_fault"])
            session.start({})
            session.thread.join(10)
            self.assertFalse(session.thread.is_alive())
            after = session.snapshot()
            self.assertEqual(after["invalid_sensor_samples"], 10)
            self.assertEqual(after["invalid_payloads"], 1)
            self.assertTrue(after["sensor_fault"])
            metadata = json.loads((session.run / "metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["metrics"]["invalid_sensor_samples"], 10)
            self.assertEqual(metadata["metrics"]["sensor_fault"], 1)

    def test_ble_duration_and_disconnect(self):
        import types
        for disconnected in (False, True):
            with self.subTest(disconnected=disconnected), tempfile.TemporaryDirectory() as tmp:
                class Scanner:
                    @staticmethod
                    async def find_device_by_filter(*args, **kwargs):
                        return object()

                class Client:
                    is_connected = not disconnected

                    def __init__(self, *args, **kwargs):
                        pass

                    async def __aenter__(self):
                        return self

                    async def __aexit__(self, *args):
                        pass

                    async def start_notify(self, uuid, cb):
                        session.accept(packet(1), 1_000_000_000)
                        session.accept(packet(2), 31_000_000_000)

                    async def stop_notify(self, uuid):
                        self.stopped = True

                module = types.SimpleNamespace(BleakScanner=Scanner, BleakClient=Client)
                clock = types.SimpleNamespace(monotonic=lambda: 31, monotonic_ns=lambda: 31_000_000_000)
                with patch.dict(sys.modules, {"bleak": module}), patch.object(capture, "time", clock):
                    session = capture.Session(tmp)
                    session.start({})
                    session.thread.join(10)
                    self.assertFalse(session.thread.is_alive())
                    if disconnected:
                        self.assertIn("通信が切れ", session.error)
                    else:
                        self.assertIsNone(session.error)
                        self.assertFalse(session.stop_event.is_set())  # Automatic 30 second completion.
                    self.assertTrue((session.run / "report.html").exists())

    def test_real_csv_through_capture_pipeline_and_reanalysis(self):
        repo = Path(__file__).resolve().parents[4]
        stem = repo / "logs/ble/20260903_ga_phase1a_S01_10m_fast_right_01"
        with Path(str(stem) + ".csv").open() as handle:
            rows = list(csv.DictReader(handle))
        with Path(str(stem) + "_markers.csv").open() as handle:
            marks = list(csv.DictReader(handle))

        async def receiver(session):
            pending = list(marks)
            for row in rows:
                ns = int(row["rx_monotonic_ns"])
                while pending and int(pending[0]["marker_monotonic_ns"]) < ns:
                    mark = pending.pop(0)
                    session.mark(session.run.name, mark["event"], int(mark["marker_monotonic_ns"]))
                payload = bytearray(PACKET.pack(1, *(int(row[k]) for k in
                    ("seq", "ax_mg", "ay_mg", "az_mg", "gx_mdps", "gy_mdps", "gz_mdps"))))
                session.accept(payload, ns)

        with tempfile.TemporaryDirectory() as tmp:
            session = capture.Session(tmp, receiver)
            run_id = session.start({"actual_steps": 14, "start_foot": "right", "end_foot": "left"})
            session.thread.join(15)
            self.assertEqual(session.state, "completed", session.detail)
            self.assertEqual(session.summary["demo"]["right_stride_candidates"], 7)
            self.assertAlmostEqual(session.summary["demo"]["duration_sec"], 7.579)
            self.assertEqual(session.summary["demo"]["estimated_steps"], 14)
            session.update_observations(run_id, {"actual_steps": ""})
            session.thread.join(15)
            self.assertIsNone(session.summary["demo"]["actual_steps"])
            self.assertIsNone(session.summary["demo"]["step_length_measured_m"])
            self.assertEqual(session.summary["demo"]["estimated_steps"], 14)

    def test_order_clock_dedup_and_partial_retention(self):
        ready, release = threading.Event(), threading.Event()

        async def receiver(session):
            session.accept(packet(100), 1_000_000_000)
            session.accept(packet(100), 1_000_000_000)
            ready.set()
            while not release.is_set():
                await asyncio.sleep(.01)
            session.accept(packet(102), 1_200_000_000)

        with tempfile.TemporaryDirectory() as tmp:
            session = capture.Session(tmp, receiver)
            run_id = session.start({"trial_name": "test"})
            self.assertTrue(ready.wait(2))
            with self.assertRaises(ValueError):
                session.start({})
            with self.assertRaises(ValueError):
                session.mark(run_id, "FINISH", 1_050_000_000)
            with self.assertRaises(ValueError):
                session.mark("stale", "START", 1_050_000_000)
            session.mark(run_id, "START", 1_050_000_000)
            with self.assertRaises(ValueError):
                session.mark(run_id, "START", 1_060_000_000)
            session.mark(run_id, "FINISH", 1_150_000_000)
            session.stop(run_id)
            release.set()
            session.thread.join(10)
            self.assertFalse(session.thread.is_alive())
            with (session.run / "markers.csv").open() as handle:
                markers = list(csv.DictReader(handle))
            self.assertEqual(int(markers[0]["marker_elapsed_ns"]), 50_000_000)
            self.assertEqual(int(markers[1]["marker_elapsed_ns"]), 150_000_000)
            self.assertEqual(session.tracker.completed_samples, 2)
            self.assertEqual(session.tracker.duplicate_packets, 1)
            self.assertEqual(session.tracker.missing_seq_count, 1)
            for name in ("sensor.csv", "markers.csv", "metadata.json", "analysis.json", "waveform.png", "report.html"):
                self.assertTrue((session.run / name).is_file(), name)
            self.assertEqual(session.state, "failed")  # Too short for zero-phase filter.

    def test_scan_failure_retains_six_artifacts(self):
        async def receiver(session):
            raise ValueError("TAGが見つかりません。")
        with tempfile.TemporaryDirectory() as tmp:
            session = capture.Session(tmp, receiver)
            session.start({})
            session.thread.join(10)
            self.assertFalse(session.thread.is_alive())
            self.assertEqual(session.snapshot()["state"], "failed")
            self.assertIn("見つかりません", session.snapshot()["error"])
            for name in ("sensor.csv", "markers.csv", "metadata.json", "analysis.json", "waveform.png", "report.html"):
                self.assertTrue((session.run / name).exists(), name)

    def test_ble_transport_stop_and_decode(self):
        class Scanner:
            @staticmethod
            async def find_device_by_filter(*args, **kwargs):
                return object()

        class Client:
            is_connected = True

            def __init__(self, *args, **kwargs):
                pass

            async def __aenter__(self):
                return self

            async def __aexit__(self, *args):
                pass

            async def start_notify(self, uuid, cb):
                self.callback = cb
                cb(None, packet(10))
                session.stop(session.run.name)

            async def stop_notify(self, uuid):
                self.stopped = True

        import types
        module = types.SimpleNamespace(BleakScanner=Scanner, BleakClient=Client)
        with tempfile.TemporaryDirectory() as tmp, patch.dict(sys.modules, {"bleak": module}):
            session = capture.Session(tmp)
            session.start({})
            session.thread.join(10)
            self.assertEqual(session.tracker.completed_samples, 1)
            self.assertFalse(session.thread.is_alive())


class FolderTests(unittest.TestCase):
    def test_windows_path_conversion(self):
        self.assertEqual(web.windows_folder("/home/in/work/ILGA", platform="posix", distro="Ubuntu"),
                         "//wsl.localhost/Ubuntu/home/in/work/ILGA".replace("/", chr(92)))
        self.assertEqual(web.windows_folder("/mnt/c/Users/MatsL", platform="posix"),
                         "C:/Users/MatsL".replace("/", chr(92)))

    def test_current_folder_http(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(web, "HERE", Path(tmp)):
            root = Path(tmp) / "runs"
            root.mkdir()
            session = capture.Session(root)
            session.run = root / "current"
            session.run.mkdir()
            server = Server(0, session=session)
            runner = threading.Thread(target=server.serve_forever, daemon=True)
            runner.start()

            def request(data):
                client = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=3)
                client.request("POST", "/api/open-folder", json.dumps(data),
                               {"Content-Type": "application/json", "X-ILGA-Token": server.token})
                response = client.getresponse()
                value = response.status, json.loads(response.read())
                client.close()
                return value

            try:
                with patch.object(web, "launch_explorer") as opener:
                    code, response = request({"run_id": "current"})
                    self.assertEqual(code, 200)
                    self.assertTrue(response["save_path"])
                    self.assertIn("Windows", response["message"])
                    opener.assert_called_once_with(session.run.resolve())
                    opener.reset_mock()
                    for bad in ("past", "../current", "../../outside", "%2e%2e/current", None):
                        self.assertEqual(request({"run_id": bad})[0], 400)
                    self.assertEqual(request({"run_id": "current", "path": str(Path(tmp))})[0], 400)
                    session.run = root / "missing"
                    self.assertEqual(request({"run_id": "missing"})[0], 400)
                    session.run = Path(tmp)  # Server-side escaped path must also be rejected.
                    self.assertEqual(request({"run_id": Path(tmp).name})[0], 400)
                    opener.assert_not_called()
                    session.run = root / "current"
                with patch.object(web, "launch_explorer", side_effect=OSError("mock failure")):
                    code, response = request({"run_id": "current"})
                    self.assertEqual(code, 400)
                    self.assertIn("開けませんでした", response["error"])
            finally:
                server.shutdown()
                server.server_close()
                runner.join()


class HTTPTests(unittest.TestCase):
    def test_main_api_and_guards(self):
        release = threading.Event()

        async def receiver(session):
            session.accept(packet(20))
            while not release.is_set():
                await asyncio.sleep(.01)
            session.accept(packet(21))

        with tempfile.TemporaryDirectory() as tmp:
            session = capture.Session(tmp, receiver)
            server = Server(0, session=session)
            runner = threading.Thread(target=server.serve_forever, daemon=True)
            runner.start()

            def request(method, path, body=None, extra=None):
                client = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=3)
                headers = {"Content-Type": "application/json", "X-ILGA-Token": server.token}
                headers.update(extra or {})
                client.request(method, path, json.dumps(body) if body is not None else None, headers)
                response = client.getresponse()
                status, data, content_type = response.status, response.read(), response.getheader("Content-Type")
                client.close()
                return status, data, content_type

            try:
                self.assertEqual(request("GET", "/")[0], 200)
                self.assertEqual(request("GET", "/api/session")[0], 200)
                self.assertEqual(request("POST", "/api/start", {}, {"X-ILGA-Token": ""})[0], 403)
                self.assertEqual(request("POST", "/api/start", {}, {"Origin": "https://example.com"})[0], 403)
                self.assertEqual(request("GET", "/", extra={"Host": "evil.example"})[0], 403)
                self.assertEqual(request("GET", "/runs/%2e%2e/metadata.json")[0], 404)
                self.assertEqual(request("POST", "/api/start", {"duration": "nan"})[0], 400)
                self.assertEqual(request("POST", "/api/start", {"actual_steps": 0})[0], 400)
                self.assertEqual(request("POST", "/api/start", {})[0], 200)
                for _ in range(100):
                    if session.snapshot()["state"] == "recording":
                        break
                    threading.Event().wait(.01)
                run_id = session.run.name
                self.assertEqual(request("POST", "/api/marker", {"run_id": run_id, "event": "START"})[0], 200)
                self.assertEqual(request("POST", "/api/marker", {"run_id": run_id, "event": "START"})[0], 400)
                deadline = time.monotonic() + 1
                while time.monotonic_ns() <= session.markers[0]["marker_monotonic_ns"]:
                    self.assertLess(time.monotonic(), deadline, "PC clock did not advance")
                    threading.Event().wait(.002)
                self.assertEqual(request("POST", "/api/marker", {"run_id": run_id, "event": "FINISH"})[0], 200)
                self.assertEqual(request("POST", "/api/observations", {"run_id": run_id, "actual_steps": 14})[0], 200)
                self.assertEqual(request("POST", "/api/stop", {"run_id": run_id})[0], 200)
                release.set()
                session.thread.join(10)
                status, data, mime = request("GET", f"/runs/{run_id}/analysis.json")
                self.assertEqual(status, 200)
                self.assertIn("application/json", mime)
                self.assertEqual(request("GET", f"/runs/{run_id}/waveform.png")[2], "image/png")
                self.assertEqual(request("POST", "/api/observations", {"run_id": run_id, "actual_steps": ""})[0], 200)
                session.thread.join(10)
                self.assertIsNone(session.metadata["actual_steps"])
            finally:
                release.set()
                session.close()
                server.shutdown()
                server.server_close()
                runner.join()


if __name__ == "__main__":
    unittest.main()
