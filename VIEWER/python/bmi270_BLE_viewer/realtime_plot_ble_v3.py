"""
BLE BMI270 Realtime Viewer (PC side) - v3

Fixes from v2:
- Avoid bleak.__version__ (not present in some installs). Use importlib.metadata if available.
- Suppress FuncAnimation cache warning: cache_frame_data=False
- Keep Bleak API compatibility for service discovery.

What it does:
- Scans for a BLE peripheral by name substring
- Connects and discovers GATT
- Subscribes to two NOTIFY characteristics:
    ACC: seq(u32 LE) + ax/ay/az(i16 LE)  [mg]
    GYR: seq(u32 LE) + gx/gy/gz(i32 LE)  [mdps]
- Shows last 3 seconds in plots (100 Hz expected), with value panels.

Auto-detect rule (UUID suffix):
  ACC: ...ef11 or ...de11
  GYR: ...ef12 or ...de12

Dependencies:
  pip install bleak matplotlib numpy
"""

from __future__ import annotations

import asyncio
import struct
import threading
import time
from dataclasses import dataclass
from queue import Queue, Empty
from typing import Optional, Tuple, List

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from bleak import BleakClient, BleakScanner


# =========================
# User settings
# =========================

DEVICE_NAME_SUBSTRING = "BMI270_BLE_SAMPLE"  # substring OK
SCAN_TIMEOUT_SEC = 10.0

WINDOW_SEC = 3.0
EXPECTED_HZ = 100.0
BUF_LEN = int(WINDOW_SEC * EXPECTED_HZ)  # 300 @100Hz

ACC_Y_LIM_G = (-2.2, 2.2)        # ±2g view
GYR_Y_LIM_DPS = (-200.0, 200.0)  # adjust as needed

G0 = 9.80665  # m/s^2 per 1g


def bleak_version() -> str:
    # Some installs don't expose bleak.__version__. Use importlib.metadata instead.
    try:
        from importlib.metadata import version  # py3.8+
        return version("bleak")
    except Exception:
        return "unknown"


# =========================
# Data model
# =========================

@dataclass
class AccSample:
    t: float
    seq: int
    ax_mg: int
    ay_mg: int
    az_mg: int


@dataclass
class GyrSample:
    t: float
    seq: int
    gx_mdps: int
    gy_mdps: int
    gz_mdps: int


class Ring:
    def __init__(self, length: int):
        self.length = length
        self.t = np.full(length, np.nan, dtype=float)
        self.x = np.full(length, np.nan, dtype=float)
        self.y = np.full(length, np.nan, dtype=float)
        self.z = np.full(length, np.nan, dtype=float)
        self._i = 0
        self._filled = False

    def push(self, t: float, x: float, y: float, z: float) -> None:
        self.t[self._i] = t
        self.x[self._i] = x
        self.y[self._i] = y
        self.z[self._i] = z
        self._i += 1
        if self._i >= self.length:
            self._i = 0
            self._filled = True

    def get_window(self, now: float, window_sec: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if not self._filled and self._i == 0:
            return (np.array([]), np.array([]), np.array([]), np.array([]))

        if self._filled:
            idx = np.concatenate([np.arange(self._i, self.length), np.arange(0, self._i)])
        else:
            idx = np.arange(0, self._i)

        t = self.t[idx]
        x = self.x[idx]
        y = self.y[idx]
        z = self.z[idx]

        mask = (t >= now - window_sec) & np.isfinite(t)
        t = t[mask] - now
        return t, x[mask], y[mask], z[mask]


# =========================
# BLE worker
# =========================

def _uuid_suffix(u: str) -> str:
    u = u.lower().replace("{", "").replace("}", "")
    return u.split("-")[-1]


def _pick_acc_gyr_chars(chars: List) -> Tuple[Optional[str], Optional[str]]:
    acc_uuid = None
    gyr_uuid = None
    for ch in chars:
        if not getattr(ch, "uuid", None):
            continue
        su = _uuid_suffix(ch.uuid)
        if su.endswith("ef11") or su.endswith("de11"):
            acc_uuid = ch.uuid
        if su.endswith("ef12") or su.endswith("de12"):
            gyr_uuid = ch.uuid
    return acc_uuid, gyr_uuid


async def _get_services_compat(client: BleakClient):
    # Newer/older Bleak may have different APIs
    if hasattr(client, "get_services"):
        try:
            return await client.get_services()  # type: ignore[attr-defined]
        except TypeError:
            return client.get_services()  # type: ignore[attr-defined]

    svcs = getattr(client, "services", None)
    if svcs is not None:
        return svcs

    backend = getattr(client, "_backend", None)
    if backend is not None and hasattr(backend, "get_services"):
        return await backend.get_services()

    raise RuntimeError("Could not access GATT services via Bleak. Try: pip install -U bleak")


async def ble_worker(sample_q: Queue, stop_evt: threading.Event) -> None:
    print(f"[BLE] bleak version: {bleak_version()}")
    print(f"[BLE] Scanning for '{DEVICE_NAME_SUBSTRING}' for {SCAN_TIMEOUT_SEC:.1f}s ...")

    device = None
    devices = await BleakScanner.discover(timeout=SCAN_TIMEOUT_SEC)
    for d in devices:
        name = (d.name or "").strip()
        if DEVICE_NAME_SUBSTRING.lower() in name.lower():
            device = d
            break

    if device is None:
        print("[BLE] Device not found.")
        return

    print(f"[BLE] Found: name='{device.name}' address='{device.address}'")

    async with BleakClient(device) as client:
        print("[BLE] Connected.")

        svcs = await _get_services_compat(client)

        all_chars = []
        for s in svcs:
            for ch in s.characteristics:
                all_chars.append(ch)

        acc_uuid, gyr_uuid = _pick_acc_gyr_chars(all_chars)
        if not acc_uuid or not gyr_uuid:
            print("[BLE] Could not auto-detect ACC/GYR notify characteristics.")
            print("[BLE] Notifiable characteristics found:")
            for ch in all_chars:
                props = ",".join(ch.properties)
                if "notify" in ch.properties or "indicate" in ch.properties:
                    print(f"  - {ch.uuid}  props=[{props}]")
            return

        print(f"[BLE] ACC uuid: {acc_uuid}")
        print(f"[BLE] GYR uuid: {gyr_uuid}")

        def on_acc(_: int, data: bytearray):
            if len(data) != 10:
                return
            now = time.time()
            seq, ax, ay, az = struct.unpack("<Ihhh", bytes(data))
            sample_q.put(AccSample(t=now, seq=seq, ax_mg=ax, ay_mg=ay, az_mg=az))

        def on_gyr(_: int, data: bytearray):
            if len(data) != 16:
                return
            now = time.time()
            seq, gx, gy, gz = struct.unpack("<Iiii", bytes(data))
            sample_q.put(GyrSample(t=now, seq=seq, gx_mdps=gx, gy_mdps=gy, gz_mdps=gz))

        await client.start_notify(acc_uuid, on_acc)
        await client.start_notify(gyr_uuid, on_gyr)
        print("[BLE] Notifications started (ACC/GYR).")

        try:
            while not stop_evt.is_set():
                await asyncio.sleep(0.1)
        finally:
            for u in (acc_uuid, gyr_uuid):
                try:
                    await client.stop_notify(u)
                except Exception:
                    pass
            print("[BLE] Notifications stopped.")


def start_ble_thread(sample_q: Queue, stop_evt: threading.Event) -> threading.Thread:
    def runner():
        try:
            asyncio.run(ble_worker(sample_q, stop_evt))
        except Exception as e:
            print(f"[BLE] Worker crashed: {e!r}")

    th = threading.Thread(target=runner, daemon=True)
    th.start()
    return th


# =========================
# Plotting (main thread)
# =========================

def main() -> None:
    sample_q: Queue = Queue()
    stop_evt = threading.Event()

    acc_ring = Ring(BUF_LEN)
    gyr_ring = Ring(BUF_LEN)

    last_acc = (0.0, 0.0, 0.0)
    last_gyr = (0.0, 0.0, 0.0)

    _ = start_ble_thread(sample_q, stop_evt)

    fig = plt.figure(figsize=(10, 7))
    gs = fig.add_gridspec(4, 1, height_ratios=[3, 1, 3, 1])

    ax_acc = fig.add_subplot(gs[0, 0])
    ax_acc_txt = fig.add_subplot(gs[1, 0])
    ax_gyr = fig.add_subplot(gs[2, 0])
    ax_gyr_txt = fig.add_subplot(gs[3, 0])

    for ax in (ax_acc_txt, ax_gyr_txt):
        ax.axis("off")

    (lax,) = ax_acc.plot([], [], label="Ax [g]")
    (lay,) = ax_acc.plot([], [], label="Ay [g]")
    (laz,) = ax_acc.plot([], [], label="Az [g]")
    ax_acc.set_title("Accelerometer (last 3s)")
    ax_acc.set_xlabel("Time [s]")
    ax_acc.set_ylabel("g")
    ax_acc.set_xlim(-WINDOW_SEC, 0)
    ax_acc.set_ylim(*ACC_Y_LIM_G)
    ax_acc.legend(loc="upper left")

    (lgx,) = ax_gyr.plot([], [], label="Gx [dps]")
    (lgy,) = ax_gyr.plot([], [], label="Gy [dps]")
    (lgz,) = ax_gyr.plot([], [], label="Gz [dps]")
    ax_gyr.set_title("Gyroscope (last 3s)")
    ax_gyr.set_xlabel("Time [s]")
    ax_gyr.set_ylabel("dps")
    ax_gyr.set_xlim(-WINDOW_SEC, 0)
    ax_gyr.set_ylim(*GYR_Y_LIM_DPS)
    ax_gyr.legend(loc="upper left")

    acc_text = ax_acc_txt.text(0.01, 0.6, "", fontsize=11, family="monospace")
    acc_text2 = ax_acc_txt.text(0.01, 0.1, "", fontsize=11, family="monospace")
    gyr_text = ax_gyr_txt.text(0.01, 0.35, "", fontsize=11, family="monospace")

    def drain_queue():
        nonlocal last_acc, last_gyr
        while True:
            try:
                item = sample_q.get_nowait()
            except Empty:
                break

            if isinstance(item, AccSample):
                ax_g = item.ax_mg / 1000.0
                ay_g = item.ay_mg / 1000.0
                az_g = item.az_mg / 1000.0
                acc_ring.push(item.t, ax_g, ay_g, az_g)
                last_acc = (ax_g, ay_g, az_g)

            elif isinstance(item, GyrSample):
                gx_dps = item.gx_mdps / 1000.0
                gy_dps = item.gy_mdps / 1000.0
                gz_dps = item.gz_mdps / 1000.0
                gyr_ring.push(item.t, gx_dps, gy_dps, gz_dps)
                last_gyr = (gx_dps, gy_dps, gz_dps)

    def update(_frame):
        drain_queue()
        now = time.time()

        t, x, y, z = acc_ring.get_window(now, WINDOW_SEC)
        lax.set_data(t, x)
        lay.set_data(t, y)
        laz.set_data(t, z)

        ax_g, ay_g, az_g = last_acc
        ax_acc_txt.set_title("acc value")
        acc_text.set_text(f"g   : Ax={ax_g:+.3f}  Ay={ay_g:+.3f}  Az={az_g:+.3f}")
        acc_text2.set_text(f"m/s²: Ax={ax_g*G0:+.3f}  Ay={ay_g*G0:+.3f}  Az={az_g*G0:+.3f}")

        t2, x2, y2, z2 = gyr_ring.get_window(now, WINDOW_SEC)
        lgx.set_data(t2, x2)
        lgy.set_data(t2, y2)
        lgz.set_data(t2, z2)

        gx, gy, gz = last_gyr
        ax_gyr_txt.set_title("gyr value")
        gyr_text.set_text(f"dps : Gx={gx:+.2f}  Gy={gy:+.2f}  Gz={gz:+.2f}")

        return lax, lay, laz, lgx, lgy, lgz, acc_text, acc_text2, gyr_text

    _ = FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)

    def on_close(_evt):
        stop_evt.set()

    fig.canvas.mpl_connect("close_event", on_close)

    plt.tight_layout()
    plt.show()

    stop_evt.set()


if __name__ == "__main__":
    main()
