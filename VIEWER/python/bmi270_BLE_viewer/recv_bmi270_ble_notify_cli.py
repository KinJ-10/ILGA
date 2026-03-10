#!/usr/bin/env python3
"""
CLI viewer for ILGA TAG BMI270 BLE notifications.

Purpose:
- Connect to the TAG as a BLE central from a PC
- Subscribe to ACC/GYR notify characteristics
- Decode little-endian payloads and print them to stdout

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
import signal
import struct
import sys
from importlib.metadata import PackageNotFoundError, version
from typing import Optional


DEFAULT_NAME = "BMI270_BLE_SAMPLE"
DEFAULT_SCAN_TIMEOUT = 10.0

ACC_UUID = "12345678-1234-5678-1234-6789abcdef11"
GYR_UUID = "12345678-1234-5678-1234-6789abcdef12"


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
    return parser


def print_acc(data: bytearray) -> None:
    if len(data) != 10:
        print(f"[ACC] unexpected payload length: {len(data)} bytes", file=sys.stderr)
        return

    seq, ax_mg, ay_mg, az_mg = struct.unpack("<Ihhh", bytes(data))
    print(f"ACC seq={seq} ax_mg={ax_mg} ay_mg={ay_mg} az_mg={az_mg}", flush=True)


def print_gyr(data: bytearray) -> None:
    if len(data) != 16:
        print(f"[GYR] unexpected payload length: {len(data)} bytes", file=sys.stderr)
        return

    seq, gx_mdps, gy_mdps, gz_mdps = struct.unpack("<Iiii", bytes(data))
    print(
        f"GYR seq={seq} gx_mdps={gx_mdps} gy_mdps={gy_mdps} gz_mdps={gz_mdps}",
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

    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()

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
        print_acc(data)

    def on_gyr(_: int, data: bytearray) -> None:
        print_gyr(data)

    try:
        async with BleakClient(device, disconnected_callback=on_disconnect) as client:
            if not client.is_connected:
                raise BleakError("connect failed")

            print("[BLE] connected")
            print(f"[BLE] subscribe ACC {ACC_UUID}")
            await client.start_notify(ACC_UUID, on_acc)
            print(f"[BLE] subscribe GYR {GYR_UUID}")
            await client.start_notify(GYR_UUID, on_gyr)
            print("[BLE] notifications started; press Ctrl+C to stop")

            try:
                await stop_event.wait()
            except KeyboardInterrupt:
                pass
            finally:
                for uuid in (ACC_UUID, GYR_UUID):
                    try:
                        await client.stop_notify(uuid)
                    except Exception:
                        pass
                print("[BLE] notifications stopped")
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[BLE] error: {exc!r}", file=sys.stderr)
        return 1

    return 0


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    return asyncio.run(run(args))


if __name__ == "__main__":
    raise SystemExit(main())
