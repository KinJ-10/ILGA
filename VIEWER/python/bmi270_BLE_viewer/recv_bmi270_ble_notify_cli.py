#!/usr/bin/env python3
"""
CLI viewer / logger for ILGA TAG BMI270 BLE notifications.

Purpose:
- Connect to the TAG as a BLE central from a PC
- Subscribe to the combined BMI270 stream characteristic
- Decode the versioned little-endian payload and print it to stdout
- Optionally append analyzer-compatible CSV rows

Default target:
- device name substring: BMI270_BLE_SAMPLE
- stream UUID: 12345678-1234-5678-1234-6789abcdef13

Payload format (23 bytes, no padding):
- version(u8) + seq(u32 LE) + ax/ay/az(i16 LE, mg)
  + gx/gy/gz(i32 LE, mdps)

Examples:
  python recv_bmi270_ble_notify_cli.py
  python recv_bmi270_ble_notify_cli.py --scan-timeout 15
  python recv_bmi270_ble_notify_cli.py --address AA:BB:CC:DD:EE:FF
  python recv_bmi270_ble_notify_cli.py --name BMI270_BLE_SAMPLE
  python recv_bmi270_ble_notify_cli.py --save-csv walk.csv --duration-sec 30

Install:
  pip install -r requirements_ble_cli.txt

Notes:
- Stop with Ctrl+C.
- On Linux, Bleak uses BlueZ. Make sure your BLE adapter is available.
- If the device is already connected to a phone, disconnect it first.
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import signal
import struct
import sys
import time
from dataclasses import dataclass
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from typing import Optional


DEFAULT_NAME = "BMI270_BLE_SAMPLE"
DEFAULT_SCAN_TIMEOUT = 10.0
DEFAULT_PROGRESS_INTERVAL_SEC = 1.0

STREAM_UUID = "12345678-1234-5678-1234-6789abcdef13"
STREAM_VERSION = 1
STREAM_STRUCT = struct.Struct("<BIhhhiii")

CSV_FIELDNAMES = ["seq", "ax_mg", "ay_mg", "az_mg", "gx_mdps", "gy_mdps", "gz_mdps"]


def bleak_version() -> str:
    try:
        return version("bleak")
    except PackageNotFoundError:
        return "unknown"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Receive BMI270 BLE notifications from ILGA TAG.")
    parser.add_argument(
        "--name",
        default=DEFAULT_NAME,
        help=f"BLE device name substring to scan for (default: {DEFAULT_NAME})",
    )
    parser.add_argument(
        "--address",
        default=None,
        help="Explicit BLE address to connect to. If set, scan result is matched by address first.",
    )
    parser.add_argument(
        "--scan-timeout",
        type=float,
        default=DEFAULT_SCAN_TIMEOUT,
        help=f"Scan timeout in seconds (default: {DEFAULT_SCAN_TIMEOUT})",
    )
    parser.add_argument(
        "--save-csv",
        default=None,
        help="Save combined samples to CSV with columns for walking_analyzer.",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=None,
        help="Stop receiving after the given number of seconds.",
    )
    parser.add_argument(
        "--disconnect-on-finish",
        action="store_true",
        help="Explicitly disconnect the BLE client before exit.",
    )
    return parser


@dataclass
class StreamSample:
    seq: int
    ax_mg: int
    ay_mg: int
    az_mg: int
    gx_mdps: int
    gy_mdps: int
    gz_mdps: int


def decode_stream_payload(data: bytearray) -> tuple[Optional[StreamSample], str]:
    if len(data) != STREAM_STRUCT.size:
        return None, "length"

    version_value, seq, ax_mg, ay_mg, az_mg, gx_mdps, gy_mdps, gz_mdps = STREAM_STRUCT.unpack(
        bytes(data)
    )
    if version_value != STREAM_VERSION:
        return None, "version"

    return (
        StreamSample(
            seq=seq,
            ax_mg=ax_mg,
            ay_mg=ay_mg,
            az_mg=az_mg,
            gx_mdps=gx_mdps,
            gy_mdps=gy_mdps,
            gz_mdps=gz_mdps,
        ),
        "",
    )


class CsvSampleWriter:
    def __init__(self, path: str):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._fh, fieldnames=CSV_FIELDNAMES)
        self._writer.writeheader()
        self._fh.flush()

    def write_row(self, row: dict[str, int]) -> None:
        self._writer.writerow(row)

    def close(self) -> None:
        if not self._fh.closed:
            self._fh.flush()
            self._fh.close()


class StreamTracker:
    def __init__(self):
        self.packets = 0
        self.completed_samples = 0
        self.missing_seq_count = 0
        self.duplicate_packets = 0
        self.stale_packets = 0
        self.invalid_payloads = 0
        self.invalid_versions = 0
        self.last_seq: Optional[int] = None

    def add_payload(self, data: bytearray) -> Optional[StreamSample]:
        self.packets += 1
        sample, error = decode_stream_payload(data)
        if sample is None:
            self.invalid_payloads += 1
            if error == "version":
                self.invalid_versions += 1
            print(f"[STREAM] invalid {error}: {len(data)} bytes", file=sys.stderr, flush=True)
            return None

        if self.last_seq is not None:
            delta = (sample.seq - self.last_seq) & 0xFFFFFFFF
            if delta == 0:
                self.duplicate_packets += 1
                return None
            if delta >= 0x80000000:
                self.stale_packets += 1
                return None
            if delta > 1:
                missing = delta - 1
                self.missing_seq_count += missing
                print(
                    f"[RX] gap detected after seq={self.last_seq}: missing={missing}",
                    flush=True,
                )

        self.last_seq = sample.seq
        self.completed_samples += 1
        return sample


class ReceiveSession:
    def __init__(self, save_csv_path: Optional[str]):
        self.save_csv_path = save_csv_path
        self.writer = CsvSampleWriter(save_csv_path) if save_csv_path else None
        self.tracker = StreamTracker()
        self._last_progress_ts = 0.0
        self._capture_start: Optional[float] = None
        self._capture_stop: Optional[float] = None
        self._accepting = False
        self.ble_disconnects = 0
        self.unexpected_disconnects = 0

    def start_capture(self) -> None:
        self._capture_start = time.monotonic()
        self._capture_stop = None
        self._accepting = True

    def stop_capture(self) -> None:
        if self._capture_start is not None and self._capture_stop is None:
            self._capture_stop = time.monotonic()
        self._accepting = False

    def note_disconnect(self, unexpected: bool) -> None:
        self.ble_disconnects += 1
        if unexpected:
            self.unexpected_disconnects += 1
            self.stop_capture()

    def handle_stream(self, data: bytearray) -> None:
        if not self._accepting:
            return
        sample = self.tracker.add_payload(data)
        if sample is None:
            return
        if self.writer is None:
            print(
                f"BMI seq={sample.seq} ax_mg={sample.ax_mg} ay_mg={sample.ay_mg} "
                f"az_mg={sample.az_mg} gx_mdps={sample.gx_mdps} "
                f"gy_mdps={sample.gy_mdps} gz_mdps={sample.gz_mdps}",
                flush=True,
            )
        else:
            self.writer.write_row(
                {
                    "seq": sample.seq,
                    "ax_mg": sample.ax_mg,
                    "ay_mg": sample.ay_mg,
                    "az_mg": sample.az_mg,
                    "gx_mdps": sample.gx_mdps,
                    "gy_mdps": sample.gy_mdps,
                    "gz_mdps": sample.gz_mdps,
                }
            )
        self._print_progress()

    def finish(self) -> None:
        self.stop_capture()
        if self.writer is not None:
            self.writer.close()

    def print_summary(self) -> None:
        elapsed_sec = 0.0
        if self._capture_start is not None and self._capture_stop is not None:
            elapsed_sec = self._capture_stop - self._capture_start
        effective_hz = self.tracker.completed_samples / elapsed_sec if elapsed_sec > 0 else 0.0
        print(
            "[SUMMARY] "
            f"stream_packets={self.tracker.packets} "
            f"completed_samples={self.tracker.completed_samples} "
            f"duration_sec={elapsed_sec:.3f} "
            f"effective_hz={effective_hz:.3f} "
            f"missing_seq={self.tracker.missing_seq_count} "
            f"duplicate_packets={self.tracker.duplicate_packets} "
            f"stale_packets={self.tracker.stale_packets} "
            f"invalid_payloads={self.tracker.invalid_payloads} "
            f"invalid_versions={self.tracker.invalid_versions} "
            f"ble_disconnects={self.ble_disconnects} "
            f"unexpected_disconnects={self.unexpected_disconnects} "
            f"csv='{self.save_csv_path}'",
            flush=True,
        )

    def _print_progress(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self._last_progress_ts) < DEFAULT_PROGRESS_INTERVAL_SEC:
            return
        self._last_progress_ts = now
        print(
            "[RX] "
            f"packets={self.tracker.packets} "
            f"rows={self.tracker.completed_samples} "
            f"missing={self.tracker.missing_seq_count} "
            f"duplicate={self.tracker.duplicate_packets} "
            f"stale={self.tracker.stale_packets} "
            f"invalid={self.tracker.invalid_payloads} "
            f"last_seq={self.tracker.last_seq if self.tracker.last_seq is not None else -1}",
            flush=True,
        )


async def find_device(name_substring: str, address: Optional[str], timeout: float):
    from bleak import BleakScanner

    print(f"[BLE] bleak version: {bleak_version()}")
    print(f"[BLE] scanning for {timeout:.1f}s")

    devices = await BleakScanner.discover(timeout=timeout)
    address_lower = address.lower() if address else None

    for device in devices:
        device_name = (device.name or "").strip()
        device_address = (device.address or "").strip()

        if address_lower and device_address.lower() == address_lower:
            return device

        if not address_lower and name_substring.lower() in device_name.lower():
            return device

    return None


async def wait_for_stop(stop_event: asyncio.Event, duration_sec: Optional[float]) -> bool:
    if duration_sec is None:
        await stop_event.wait()
        return False

    stop_waiter = asyncio.create_task(stop_event.wait())
    duration_waiter = asyncio.create_task(asyncio.sleep(duration_sec))
    pending: set[asyncio.Task[object]] = set()
    try:
        done, pending = await asyncio.wait({stop_waiter, duration_waiter}, return_when=asyncio.FIRST_COMPLETED)
        return duration_waiter in done
    finally:
        for task in pending:
            task.cancel()
        if pending:
            await asyncio.gather(*pending, return_exceptions=True)


async def stop_notify_quietly(client, uuid: str) -> None:
    try:
        await client.stop_notify(uuid)
    except asyncio.CancelledError:
        pass
    except Exception:
        pass


async def disconnect_quietly(client) -> None:
    try:
        if client.is_connected:
            await client.disconnect()
    except asyncio.CancelledError:
        pass
    except Exception:
        pass


async def run(args: argparse.Namespace) -> int:
    try:
        from bleak import BleakClient
        from bleak.exc import BleakError
    except ModuleNotFoundError:
        print("ERROR: bleak is not installed. Run: pip install -r requirements_ble_cli.txt", file=sys.stderr)
        return 1

    device = await find_device(args.name, args.address, args.scan_timeout)
    if device is None:
        if args.address:
            print(f"[BLE] device not found for address: {args.address}", file=sys.stderr)
        else:
            print(f"[BLE] device not found for name substring: {args.name}", file=sys.stderr)
        return 1

    print(f"[BLE] found name='{device.name}' address='{device.address}'")

    session = ReceiveSession(args.save_csv)
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    duration_reached = False
    connected = False
    shutdown_expected = False

    if sys.platform != "win32":
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, stop_event.set)
            except NotImplementedError:
                pass

    def on_disconnect(_client) -> None:
        session.note_disconnect(unexpected=not shutdown_expected)
        print("[BLE] disconnected")
        loop.call_soon_threadsafe(stop_event.set)

    def on_stream(_: int, data: bytearray) -> None:
        session.handle_stream(data)

    try:
        client_kwargs = {}
        if sys.platform == "win32":
            # The combined characteristic is new; bypass stale WinRT GATT cache.
            client_kwargs["winrt"] = {"use_cached_services": False}

        async with BleakClient(
            device, disconnected_callback=on_disconnect, **client_kwargs
        ) as client:
            if not client.is_connected:
                raise BleakError("connect failed")
            connected = True

            print("[BLE] connected")
            print(f"[BLE] subscribe BMI stream {STREAM_UUID}")
            await client.start_notify(STREAM_UUID, on_stream)
            session.start_capture()
            if args.save_csv:
                print(f"[CSV] saving samples to {args.save_csv}")
            print("[BLE] notifications started; press Ctrl+C to stop")

            try:
                duration_reached = await wait_for_stop(stop_event, args.duration_sec)
                if duration_reached:
                    print(f"[BLE] duration reached: {args.duration_sec:.3f}s", flush=True)
                    stop_event.set()
            except KeyboardInterrupt:
                stop_event.set()
                pass
            finally:
                shutdown_expected = True
                session.stop_capture()
                await stop_notify_quietly(client, STREAM_UUID)
                print("[BLE] notifications stopped")
                if args.disconnect_on_finish and client.is_connected:
                    print("[BLE] disconnecting on finish", flush=True)
                    await disconnect_quietly(client)
    except KeyboardInterrupt:
        pass
    except asyncio.CancelledError:
        if not (connected or stop_event.is_set() or duration_reached):
            print("[BLE] error: operation cancelled before normal shutdown", file=sys.stderr)
            return 1
    except Exception as exc:
        print(f"[BLE] error: {exc!r}", file=sys.stderr)
        return 1
    finally:
        session.finish()
        session.print_summary()
        if duration_reached:
            print("[BLE] capture finished by duration", flush=True)

    return 0


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        return 0
    except asyncio.CancelledError:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
