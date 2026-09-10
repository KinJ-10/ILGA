"""Serialized capture lifecycle; BLE remains on Windows, analysis is offline."""
from __future__ import annotations

import asyncio
import csv
import threading
import time
from datetime import datetime
from pathlib import Path

import reporting

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "bmi270_BLE_viewer"))
from recv_bmi270_ble_notify_cli import (
    CSV_FIELDNAMES, MARKER_FIELDNAMES, DEFAULT_NAME, STREAM_UUID, StreamTracker,
)

ACTIVE = {"connecting", "recording", "stopping", "analyzing"}


def integer(value, label, lower, upper, optional=False):
    if optional and (value is None or value == ""):
        return None
    try:
        if isinstance(value, bool) or str(int(value)) != str(value).strip():
            raise ValueError()
        number = int(value)
        if lower <= number <= upper:
            return number
    except (TypeError, ValueError):
        pass
    raise ValueError(f"{label}は{lower}〜{upper}の整数で入力してください。")


def observations(data):
    result = {"actual_steps": integer(data.get("actual_steps"), "実測歩数", 1, 200, True)}
    result["actual_steps_source"] = "ui_observation"
    result["sync_observed"] = {
        side: integer(data.get("sync_" + side), "足踏み回数", 0, 20, True)
        for side in ("pre", "post")
    }
    for side in ("start", "end"):
        value = data.get(side + "_foot", "unknown")
        if value not in {"right", "left", "unknown"}:
            raise ValueError("開始足・終了足の選択を確認してください。")
        result[side + "_foot"] = value
    return result


async def receive_ble(session):
    from bleak import BleakClient, BleakScanner
    device = await BleakScanner.find_device_by_filter(
        lambda d, a: DEFAULT_NAME in (a.local_name or d.name or ""), timeout=10,
    )
    if session.stop_event.is_set():
        return
    if device is None:
        raise ValueError("TAGが見つかりません。電源とBluetoothを確認し、他の接続を解除してください。")
    disconnected = threading.Event()
    async with BleakClient(device, disconnected_callback=lambda _: disconnected.set(), timeout=12) as client:
        if session.stop_event.is_set():
            return
        await client.start_notify(STREAM_UUID, lambda _, payload: session.accept(payload))
        waiting_since = time.monotonic()
        try:
            while True:
                now = time.monotonic()
                with session.lock:
                    first, last = session.first_ns, session.last_ns
                    fault = session.tracker.sensor_fault
                if fault:
                    raise ValueError("加速度が連続してゼロです。TAGのセンサー接続と電源を確認してください。")
                if disconnected.is_set() or not client.is_connected:
                    raise ValueError("TAGとの通信が切れました。受信済みデータは保存します。")
                if first is None and now - waiting_since > 5:
                    raise ValueError("TAGからデータが届きません。電源とセンサー接続を確認してください。")
                if last is not None and now - last / 1e9 > 3:
                    raise ValueError("データ受信が3秒以上止まりました。通信状態を確認してください。")
                if first is not None and now - first / 1e9 >= session.duration:
                    break
                if session.stop_event.is_set():
                    # A marker timestamp must be covered by the last received sample.
                    finish = session.markers[-1]["marker_monotonic_ns"] if session.markers else 0
                    if not finish or (last is not None and last >= finish) or now - session.stop_at > .8:
                        break
                await asyncio.sleep(.05)
        finally:
            if client.is_connected:
                await client.stop_notify(STREAM_UUID)


class Session:
    def __init__(self, root, receiver=receive_ble):
        self.root = Path(root)
        self.receiver = receiver
        self.lock = threading.RLock()
        self.state = "idle"
        self.run = None
        self.thread = None
        self.detail = "測定を開始できます。"
        self.first_ns = None
        self.last_ns = None
        self.markers = []
        self.stop_event = threading.Event()
        self.tracker = StreamTracker()
        self.duration = 30
        self.summary = None
        self.error = None

    def start(self, data):
        duration = integer(data.get("duration", 30), "取得秒数", 10, 120)
        observed = observations(data)
        title = str(data.get("trial_name", "10m歩行")).strip()
        if not title or len(title) > 80:
            raise ValueError("測定名は1〜80文字で入力してください。")
        with self.lock:
            if self.state in ACTIVE:
                raise ValueError("現在の測定・解析が終わるまでお待ちください。")
            self.run = reporting.new_run(self.root)
            self.duration = duration
            self.first_ns = self.last_ns = None
            self.markers = []
            self.tracker = StreamTracker()
            self.stop_event = threading.Event()
            self.summary = self.error = None
            self.metadata = {
                "schema_version": 1, "trial_name": title, "test_type": "Walk10m",
                "distance_m": 10, "sensor_foot": "right", "demo_profile": reporting.PROFILE,
                "capture_duration_sec": duration, "started_at": datetime.now().astimezone().isoformat(),
                "receive_timestamp_definition": "PC BLE callback entry, not sensor acquisition time",
                "marker_input": "web_button", **observed,
            }
            self.sensor_handle = (self.run / "sensor.csv").open("x", newline="", encoding="utf-8")
            self.writer = csv.DictWriter(self.sensor_handle, fieldnames=CSV_FIELDNAMES)
            self.writer.writeheader()
            self.sensor_handle.flush()
            self.marker_handle = (self.run / "markers.csv").open("x", newline="", encoding="utf-8")
            self.marker_writer = csv.DictWriter(self.marker_handle, fieldnames=MARKER_FIELDNAMES)
            self.marker_writer.writeheader()
            self.marker_handle.flush()
            reporting.write_json(self.run / "metadata.json", self.metadata)
            self.state, self.detail = "connecting", "TAGを探して接続しています。"
            self.thread = threading.Thread(target=self._capture, daemon=True)
            self.thread.start()
            return self.run.name

    def accept(self, payload, timestamp=None):
        timestamp = time.monotonic_ns() if timestamp is None else timestamp
        with self.lock:
            if self.state not in {"connecting", "recording", "stopping"}:
                return
            sample = self.tracker.add_payload(payload)
            if sample is None:
                return
            if self.first_ns is None:
                self.first_ns = timestamp
                if self.state == "connecting":
                    self.state, self.detail = "recording", "受信中です。開始線でSTARTを押してください。"
            self.last_ns = timestamp
            self.writer.writerow({
                **vars(sample), "rx_monotonic_ns": timestamp,
                "rx_elapsed_ns": timestamp - self.first_ns,
            })
            self.sensor_handle.flush()

    def require_run(self, run_id):
        if self.run is None or run_id != self.run.name:
            raise ValueError("表示中の測定が更新されました。ページを再読み込みしてください。")

    def mark(self, run_id, event, timestamp=None):
        with self.lock:
            self.require_run(run_id)
            if self.state != "recording":
                raise ValueError("受信中にSTART・FINISHを記録してください。")
            expected = "START" if not self.markers else ("FINISH" if len(self.markers) == 1 else None)
            if event != expected:
                raise ValueError("START、FINISHの順に各1回押してください。")
            now = time.monotonic_ns() if timestamp is None else timestamp
            if self.first_ns is None or now < self.first_ns or (self.markers and now <= self.markers[-1]["marker_monotonic_ns"]):
                raise ValueError("時刻を確認できません。もう一度操作してください。")
            row = {
                "schema_version": 1, "trial_name": self.metadata["trial_name"], "event": event,
                "event_index": len(self.markers) + 1, "marker_monotonic_ns": now,
                "marker_elapsed_ns": now - self.first_ns,
                # Existing analyzer schema requires operator_key for operator input.
                "source": "operator_key", "notes": "web_button",
            }
            self.marker_writer.writerow(row)
            self.marker_handle.flush()
            self.markers.append(row)
            self.detail = event + "を記録しました。"

    def stop(self, run_id):
        with self.lock:
            self.require_run(run_id)
            if self.state not in {"connecting", "recording", "stopping"}:
                raise ValueError("取得中の測定がありません。")
            if not self.stop_event.is_set():
                self.stop_at = time.monotonic()
                self.stop_event.set()
            self.state, self.detail = "stopping", "取得を停止し、保存・解析します。"

    def update_observations(self, run_id, data):
        observed = observations(data)
        with self.lock:
            self.require_run(run_id)
            if self.state in {"stopping", "analyzing", "connecting", "idle"}:
                raise ValueError("受信中または解析完了後に記録を更新してください。")
            self.metadata.update(observed)
            reporting.write_json(self.run / "metadata.json", self.metadata)
            if self.state == "recording":
                return
            self.state, self.detail = "analyzing", "入力した実測値で結果を更新しています。"
            self.thread = threading.Thread(target=self._analyze, daemon=True)
            self.thread.start()

    def _capture(self):
        try:
            asyncio.run(self.receiver(self))
        except Exception as exc:
            self.error = str(exc) if isinstance(exc, ValueError) else "Bluetooth通信に失敗しました。TAGの電源と接続先を確認してください。"
            (self.run / "diagnostic.log").write_text(repr(exc), encoding="utf-8")
        finally:
            with self.lock:
                self.state, self.detail = "analyzing", "取得したデータを保存・解析しています。"
                self.sensor_handle.close()
                self.marker_handle.close()
                self.metadata.update({
                    "finished_at": datetime.now().astimezone().isoformat(),
                    "first_rx_monotonic_ns": self.first_ns, "last_rx_monotonic_ns": self.last_ns,
                    "capture_error": self.error,
                    "metrics": {
                        "missing_seq": self.tracker.missing_seq_count,
                        "invalid_sensor_samples": self.tracker.invalid_sensor_samples,
                        "sensor_fault": int(self.tracker.sensor_fault),
                        "duplicate_packets": self.tracker.duplicate_packets,
                        "invalid_payloads": self.tracker.invalid_payloads,
                        "sample_count": self.tracker.completed_samples,
                    },
                })
                reporting.write_json(self.run / "metadata.json", self.metadata)
        self._analyze()

    def _analyze(self):
        try:
            summary = reporting.analyze_run(self.run, self.metadata)
            with self.lock:
                self.summary = summary
                failed = summary["demo"]["quality"] == "計算不可" or self.error
                self.state = "failed" if failed else "completed"
                self.detail = self.error or ("結果に計算できない項目があります。保存したレポートを確認してください。" if failed else "保存・解析が完了しました。")
        except Exception as exc:
            with self.lock:
                self.error = "結果の保存に失敗しました。空き容量と保存先を確認してください。受信CSVは測定フォルダーに残しています。"
                self.state, self.detail = "failed", self.error
            (self.run / "diagnostic.log").write_text(repr(exc), encoding="utf-8")

    def snapshot(self):
        with self.lock:
            elapsed = 0 if self.first_ns is None else max(0, ((time.monotonic_ns() if self.state in {"recording", "stopping"} else self.last_ns) - self.first_ns) / 1e9)
            return {
                "state": self.state, "detail": self.detail, "error": self.error,
                "run_id": self.run.name if self.run else None,
                "samples": self.tracker.completed_samples, "missing": self.tracker.missing_seq_count,
                "invalid_sensor_samples": self.tracker.invalid_sensor_samples,
                "invalid_payloads": self.tracker.invalid_payloads,
                "sensor_fault": self.tracker.sensor_fault,
                "elapsed": round(elapsed, 1), "duration": self.duration,
                "effective_hz": round((self.tracker.completed_samples - 1) / elapsed, 1) if elapsed > 0 else None,
                "markers": [r["event"] for r in self.markers],
                "report": f"/runs/{self.run.name}/report.html" if self.run and (self.run / "report.html").exists() else None,
            }

    def close(self):
        if self.state in {"connecting", "recording"}:
            self.stop(self.run.name)
        if self.thread:
            self.thread.join(timeout=25)
