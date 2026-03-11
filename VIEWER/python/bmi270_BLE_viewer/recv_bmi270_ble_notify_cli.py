#!/usr/bin/env python3
"""
CLI viewer / logger for ILGA TAG BMI270 BLE notifications.

Purpose:
- Connect to the TAG as a BLE central from a PC
- Subscribe to ACC/GYR notify characteristics
- Decode little-endian payloads and print them to stdout
- Optionally merge ACC/GYR by seq and append analyzer-compatible CSV rows

Default target:
- device name substring: BMI270_BLE_SAMPLE
- ACC UUID: 12345678-1234-5678-1234-6789abcdef11
- GYR UUID: 12345678-1234-5678-1234-6789abcdef12

Payload format:
- ACC: seq(u32 LE) + ax(i16 LE) + ay(i16 LE) + az(i16 LE), unit=mg
- GYR: seq(u32 LE) + gx(i32 LE) + gy(i32 LE) + gz(i32 LE), unit=mdps

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
DEFAULT_REORDER_WINDOW = 32

ACC_UUID = "12345678-1234-5678-1234-6789abcdef11"
GYR_UUID = "12345678-1234-5678-1234-6789abcdef12"

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
        help="Append merged samples to CSV with columns for walking_analyzer.",
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
class AccSample:
    seq: int
    ax_mg: int
    ay_mg: int
    az_mg: int


@dataclass
class GyrSample:
    seq: int
    gx_mdps: int
    gy_mdps: int
    gz_mdps: int


@dataclass
class PartialSample:
    acc: Optional[AccSample] = None
    gyr: Optional[GyrSample] = None

    @property
    def complete(self) -> bool:
        return self.acc is not None and self.gyr is not None


def decode_acc_payload(data: bytearray) -> Optional[AccSample]:
    if len(data) != 10:
        print(f"[ACC] unexpected payload length: {len(data)} bytes", file=sys.stderr)
        return None

    seq, ax_mg, ay_mg, az_mg = struct.unpack("<Ihhh", bytes(data))
    return AccSample(seq=seq, ax_mg=ax_mg, ay_mg=ay_mg, az_mg=az_mg)


def decode_gyr_payload(data: bytearray) -> Optional[GyrSample]:
    if len(data) != 16:
        print(f"[GYR] unexpected payload length: {len(data)} bytes", file=sys.stderr)
        return None

    seq, gx_mdps, gy_mdps, gz_mdps = struct.unpack("<Iiii", bytes(data))
    return GyrSample(seq=seq, gx_mdps=gx_mdps, gy_mdps=gy_mdps, gz_mdps=gz_mdps)


def print_acc(data: bytearray) -> None:
    sample = decode_acc_payload(data)
    if sample is None:
        return
    print(
        f"ACC seq={sample.seq} ax_mg={sample.ax_mg} ay_mg={sample.ay_mg} az_mg={sample.az_mg}",
        flush=True,
    )


def print_gyr(data: bytearray) -> None:
    sample = decode_gyr_payload(data)
    if sample is None:
        return
    print(
        f"GYR seq={sample.seq} gx_mdps={sample.gx_mdps} gy_mdps={sample.gy_mdps} gz_mdps={sample.gz_mdps}",
        flush=True,
    )


class CsvSampleWriter:
    def __init__(self, path: str):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = self.path.open("w", newline="", encoding="utf-8", buffering=1)
        self._writer = csv.DictWriter(self._fh, fieldnames=CSV_FIELDNAMES)
        self._writer.writeheader()
        self._fh.flush()

    def write_row(self, row: dict[str, int]) -> None:
        self._writer.writerow(row)
        self._fh.flush()

    def close(self) -> None:
        if not self._fh.closed:
            self._fh.flush()
            self._fh.close()


class SampleAssembler:
    def __init__(self, reorder_window: int = DEFAULT_REORDER_WINDOW):
        self.reorder_window = max(1, reorder_window)
        self.pending: dict[int, PartialSample] = {}
        self.acc_packets = 0
        self.gyr_packets = 0
        self.emitted_samples = 0
        self.missing_seq_count = 0
        self.incomplete_discarded = 0
        self.invalid_acc = 0
        self.invalid_gyr = 0
        self.stale_packets = 0
        self._last_emitted_seq: Optional[int] = None
        self._next_emit_seq: Optional[int] = None
        self._max_seen_seq: Optional[int] = None

    @property
    def pending_count(self) -> int:
        return len(self.pending)

    @property
    def emitted_seq_floor(self) -> int:
        if self._last_emitted_seq is None:
            return -1
        return self._last_emitted_seq

    def add_acc_payload(self, data: bytearray) -> list[dict[str, int]]:
        sample = decode_acc_payload(data)
        if sample is None:
            self.invalid_acc += 1
            return []
        self.acc_packets += 1
        return self._store_acc(sample)

    def add_gyr_payload(self, data: bytearray) -> list[dict[str, int]]:
        sample = decode_gyr_payload(data)
        if sample is None:
            self.invalid_gyr += 1
            return []
        self.gyr_packets += 1
        return self._store_gyr(sample)

    def finalize(self) -> tuple[list[dict[str, int]], int]:
        rows = self._flush_ready(force_gap_close=True)
        leftover = len(self.pending)
        self.incomplete_discarded += leftover
        self.pending.clear()
        return rows, leftover

    def _store_acc(self, sample: AccSample) -> list[dict[str, int]]:
        if self._is_stale(sample.seq):
            self.stale_packets += 1
            return []
        entry = self.pending.setdefault(sample.seq, PartialSample())
        entry.acc = sample
        return self._after_update(sample.seq)

    def _store_gyr(self, sample: GyrSample) -> list[dict[str, int]]:
        if self._is_stale(sample.seq):
            self.stale_packets += 1
            return []
        entry = self.pending.setdefault(sample.seq, PartialSample())
        entry.gyr = sample
        return self._after_update(sample.seq)

    def _after_update(self, seq: int) -> list[dict[str, int]]:
        self._max_seen_seq = seq if self._max_seen_seq is None else max(self._max_seen_seq, seq)
        return self._flush_ready(force_gap_close=False)

    def _is_stale(self, seq: int) -> bool:
        return self._next_emit_seq is not None and seq < self._next_emit_seq

    def _flush_ready(self, force_gap_close: bool) -> list[dict[str, int]]:
        rows: list[dict[str, int]] = []
        if self._next_emit_seq is None:
            if not self.pending:
                return rows
            self._next_emit_seq = min(self.pending)

        while True:
            current_seq = self._next_emit_seq
            entry = self.pending.get(current_seq)
            if entry and entry.complete:
                rows.append(self._emit_current(current_seq, entry))
                continue

            if not self._should_advance_gap(current_seq, force_gap_close):
                break

            gap_entry = self.pending.pop(current_seq, None)
            if gap_entry is not None and not gap_entry.complete:
                self.incomplete_discarded += 1
            self.missing_seq_count += 1
            print(f"[RX] gap detected at seq={current_seq}", flush=True)
            self._next_emit_seq += 1

        return rows

    def _emit_current(self, seq: int, entry: PartialSample) -> dict[str, int]:
        acc = entry.acc
        gyr = entry.gyr
        if acc is None or gyr is None:
            raise RuntimeError("internal error: incomplete sample emit attempted")
        self.pending.pop(seq, None)
        self.emitted_samples += 1
        self._last_emitted_seq = seq
        self._next_emit_seq = seq + 1
        return {
            "seq": seq,
            "ax_mg": acc.ax_mg,
            "ay_mg": acc.ay_mg,
            "az_mg": acc.az_mg,
            "gx_mdps": gyr.gx_mdps,
            "gy_mdps": gyr.gy_mdps,
            "gz_mdps": gyr.gz_mdps,
        }

    def _should_advance_gap(self, current_seq: int, force_gap_close: bool) -> bool:
        next_complete_seq = self._next_complete_after(current_seq)
        if next_complete_seq is None:
            return False
        if force_gap_close:
            return True
        if self._max_seen_seq is None:
            return False
        return (self._max_seen_seq - current_seq) >= self.reorder_window

    def _next_complete_after(self, current_seq: int) -> Optional[int]:
        complete_seqs = [seq for seq, entry in self.pending.items() if entry.complete and seq > current_seq]
        if not complete_seqs:
            return None
        return min(complete_seqs)


class ReceiveSession:
    def __init__(self, save_csv_path: Optional[str]):
        self.save_csv_path = save_csv_path
        self.writer = CsvSampleWriter(save_csv_path) if save_csv_path else None
        self.assembler = SampleAssembler()
        self._last_progress_ts = 0.0

    def handle_acc(self, data: bytearray) -> None:
        if self.writer is None:
            print_acc(data)
            return
        self._consume_rows(self.assembler.add_acc_payload(data))

    def handle_gyr(self, data: bytearray) -> None:
        if self.writer is None:
            print_gyr(data)
            return
        self._consume_rows(self.assembler.add_gyr_payload(data))

    def finish(self) -> None:
        if self.writer is None:
            return
        rows, leftover = self.assembler.finalize()
        self._consume_rows(rows, force_progress=True)
        if leftover:
            print(f"[CSV] discarded incomplete buffered samples: {leftover}", flush=True)
        self.writer.close()

    def print_summary(self) -> None:
        if self.writer is None:
            return
        print(
            "[SUMMARY] "
            f"acc_packets={self.assembler.acc_packets} "
            f"gyr_packets={self.assembler.gyr_packets} "
            f"completed_samples={self.assembler.emitted_samples} "
            f"missing_seq={self.assembler.missing_seq_count} "
            f"incomplete_discarded={self.assembler.incomplete_discarded} "
            f"stale_packets={self.assembler.stale_packets} "
            f"invalid_acc={self.assembler.invalid_acc} "
            f"invalid_gyr={self.assembler.invalid_gyr} "
            f"csv='{self.save_csv_path}'",
            flush=True,
        )

    def _consume_rows(self, rows: list[dict[str, int]], force_progress: bool = False) -> None:
        if self.writer is None:
            return
        for row in rows:
            self.writer.write_row(row)
        self._print_progress(force=force_progress or bool(rows))

    def _print_progress(self, force: bool = False) -> None:
        if self.writer is None:
            return
        now = time.monotonic()
        if not force and (now - self._last_progress_ts) < DEFAULT_PROGRESS_INTERVAL_SEC:
            return
        self._last_progress_ts = now
        print(
            "[RX] "
            f"acc={self.assembler.acc_packets} "
            f"gyr={self.assembler.gyr_packets} "
            f"rows={self.assembler.emitted_samples} "
            f"pending={self.assembler.pending_count} "
            f"missing={self.assembler.missing_seq_count} "
            f"last_seq={self.assembler.emitted_seq_floor}",
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

    if sys.platform != "win32":
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, stop_event.set)
            except NotImplementedError:
                pass

    def on_disconnect(_client) -> None:
        print("[BLE] disconnected")
        loop.call_soon_threadsafe(stop_event.set)

    def on_acc(_: int, data: bytearray) -> None:
        session.handle_acc(data)

    def on_gyr(_: int, data: bytearray) -> None:
        session.handle_gyr(data)

    try:
        async with BleakClient(device, disconnected_callback=on_disconnect) as client:
            if not client.is_connected:
                raise BleakError("connect failed")
            connected = True

            print("[BLE] connected")
            print(f"[BLE] subscribe ACC {ACC_UUID}")
            await client.start_notify(ACC_UUID, on_acc)
            print(f"[BLE] subscribe GYR {GYR_UUID}")
            await client.start_notify(GYR_UUID, on_gyr)
            if args.save_csv:
                print(f"[CSV] saving merged samples to {args.save_csv}")
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
                for uuid in (ACC_UUID, GYR_UUID):
                    await stop_notify_quietly(client, uuid)
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
        if args.save_csv:
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
