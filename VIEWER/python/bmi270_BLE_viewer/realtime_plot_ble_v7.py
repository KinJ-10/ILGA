"""
BLE BMI270 Realtime Viewer (PC side) - v7

Changes from v6:
- Plot timestamps use *arrival wall time* (item.t) so the graph updates even if the real rate is not 100 Hz.
- Still uses seq to detect drops (seq gaps).
- Shows estimated rate (based on samples received per second) + drops.

Why this matters:
- In v6 we used seq-based time assuming EXPECTED_HZ=100.
  If your real stream is ~20 Hz, the seq-time drifts behind wall clock and the plot window becomes empty,
  while the numeric panels still update. That's exactly what you observed.
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
from bleak.exc import BleakError


# =========================
# User settings
# =========================
DEVICE_NAME_SUBSTRING = "BMI270_BLE_SAMPLE"  # substring OK
SCAN_TIMEOUT_SEC = 10.0

CONNECT_TIMEOUT_SEC = 20.0
CONNECT_RETRIES = 3
RETRY_DELAY_SEC = 1.0

WINDOW_SEC = 3.0

ACC_Y_LIM_G = (-2.2, 2.2)
GYR_Y_LIM_DPS = (-200.0, 200.0)

G0 = 9.80665

# ACC payload unit mode:
#  - 'mg'         : ax/ay/az are milli-g (1g ≈ 1000)
#  - 'mps2_milli'  : ax/ay/az are milli-(m/s^2) (1g ≈ 9806.65)
ACC_PAYLOAD_MODE = 'mps2_milli'


def bleak_version() -> str:
    try:
        from importlib.metadata import version
        return version("bleak")
    except Exception:
        return "unknown"


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

    raise RuntimeError("Could not access GATT services via Bleak.")


async def _connect_with_retry(client: BleakClient) -> None:
    last_err: Optional[Exception] = None
    for i in range(1, CONNECT_RETRIES + 1):
        try:
            print(f"[BLE] Connecting... (try {i}/{CONNECT_RETRIES})")
            await client.connect(timeout=CONNECT_TIMEOUT_SEC)
            if client.is_connected:
                print("[BLE] Connected.")
                return
            raise BleakError("connect() returned but client.is_connected is False")
        except Exception as e:
            last_err = e
            print(f"[BLE] Connect failed: {e!r}")
            try:
                await client.disconnect()
            except Exception:
                pass
            await asyncio.sleep(RETRY_DELAY_SEC)
    raise BleakError(f"Could not connect after {CONNECT_RETRIES} tries: {last_err!r}")


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
        print("[BLE] Tip: ensure the DK is advertising AND your phone is disconnected.")
        return

    print(f"[BLE] Found: name='{device.name}' address='{device.address}'")

    client = BleakClient(device, timeout=CONNECT_TIMEOUT_SEC)

    try:
        await _connect_with_retry(client)

        # Services discovery retry once
        for s_try in (1, 2):
            try:
                svcs = await _get_services_compat(client)
                break
            except Exception as e:
                print(f"[BLE] Service discovery failed (try {s_try}/2): {e!r}")
                if s_try == 2:
                    raise
                try:
                    await client.disconnect()
                except Exception:
                    pass
                await asyncio.sleep(RETRY_DELAY_SEC)
                await _connect_with_retry(client)

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

    finally:
        try:
            await client.disconnect()
        except Exception:
            pass


def start_ble_thread(sample_q: Queue, stop_evt: threading.Event) -> threading.Thread:
    def runner():
        try:
            asyncio.run(ble_worker(sample_q, stop_evt))
        except Exception as e:
            print(f"[BLE] Worker crashed: {e!r}")
            print("[BLE] Common causes on Windows:")
            print("  - Your phone is still connected to the DK (disconnect / forget on phone).")
            print("  - The DK stopped advertising (reset DK).")
            print("  - Windows cached a bad pairing (remove device in Bluetooth settings, then retry).")

    th = threading.Thread(target=runner, daemon=True)
    th.start()
    return th


def main() -> None:
    sample_q: Queue = Queue()
    stop_evt = threading.Event()

    # Buffer length: keep a bit more than window to be safe at low rates
    buf_len = int(max(300, WINDOW_SEC * 200))  # >= 300 samples
    acc_ring = Ring(buf_len)
    gyr_ring = Ring(buf_len)

    last_acc = (0.0, 0.0, 0.0)
    last_gyr = (0.0, 0.0, 0.0)

    # drop detection + rate
    last_acc_seq: Optional[int] = None
    last_gyr_seq: Optional[int] = None
    acc_drops = 0
    gyr_drops = 0
    acc_count = 0
    gyr_count = 0
    rate_t0 = time.time()
    last_rates = (0.0, 0.0)

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

    acc_text = ax_acc_txt.text(0.01, 0.72, "", fontsize=11, family="monospace")
    acc_text2 = ax_acc_txt.text(0.01, 0.32, "", fontsize=11, family="monospace")
    acc_stats = ax_acc_txt.text(0.01, 0.02, "", fontsize=10, family="monospace")

    gyr_text = ax_gyr_txt.text(0.01, 0.55, "", fontsize=11, family="monospace")
    gyr_stats = ax_gyr_txt.text(0.01, 0.10, "", fontsize=10, family="monospace")

    def drain_queue():
        nonlocal last_acc, last_gyr
        nonlocal last_acc_seq, last_gyr_seq, acc_drops, gyr_drops
        nonlocal acc_count, gyr_count
        while True:
            try:
                item = sample_q.get_nowait()
            except Empty:
                break

            if isinstance(item, AccSample):
                # unit convert to g
                if ACC_PAYLOAD_MODE == 'mg':
                    ax_g = item.ax_mg / 1000.0
                    ay_g = item.ay_mg / 1000.0
                    az_g = item.az_mg / 1000.0
                else:
                    ax_g = (item.ax_mg / 1000.0) / G0
                    ay_g = (item.ay_mg / 1000.0) / G0
                    az_g = (item.az_mg / 1000.0) / G0

                # drop detection
                if last_acc_seq is not None:
                    gap = item.seq - last_acc_seq - 1
                    if gap > 0:
                        acc_drops += gap
                last_acc_seq = item.seq

                # plot uses wall time so it always moves
                acc_ring.push(item.t, ax_g, ay_g, az_g)
                last_acc = (ax_g, ay_g, az_g)
                acc_count += 1

            elif isinstance(item, GyrSample):
                gx_dps = item.gx_mdps / 1000.0
                gy_dps = item.gy_mdps / 1000.0
                gz_dps = item.gz_mdps / 1000.0

                if last_gyr_seq is not None:
                    gap = item.seq - last_gyr_seq - 1
                    if gap > 0:
                        gyr_drops += gap
                last_gyr_seq = item.seq

                gyr_ring.push(item.t, gx_dps, gy_dps, gz_dps)
                last_gyr = (gx_dps, gy_dps, gz_dps)
                gyr_count += 1

    def update(_frame):
        nonlocal rate_t0, acc_count, gyr_count, last_rates
        drain_queue()
        now = time.time()

        dt = now - rate_t0
        if dt >= 1.0:
            acc_hz = acc_count / dt if dt > 0 else 0.0
            gyr_hz = gyr_count / dt if dt > 0 else 0.0
            last_rates = (acc_hz, gyr_hz)
            rate_t0 = now
            acc_count = 0
            gyr_count = 0
        else:
            acc_hz, gyr_hz = last_rates

        # accel plot
        t, x, y, z = acc_ring.get_window(now, WINDOW_SEC)
        lax.set_data(t, x)
        lay.set_data(t, y)
        laz.set_data(t, z)

        ax_g, ay_g, az_g = last_acc
        ax_acc_txt.set_title("acc value")
        acc_text.set_text(f"g   : Ax={ax_g:+.3f}  Ay={ay_g:+.3f}  Az={az_g:+.3f}")
        acc_text2.set_text(f"m/s²: Ax={ax_g*G0:+.3f}  Ay={ay_g*G0:+.3f}  Az={az_g*G0:+.3f}")
        acc_stats.set_text(f"ACC rate≈{acc_hz:6.1f} Hz  drops={acc_drops}")

        # gyro plot
        t2, x2, y2, z2 = gyr_ring.get_window(now, WINDOW_SEC)
        lgx.set_data(t2, x2)
        lgy.set_data(t2, y2)
        lgz.set_data(t2, z2)

        gx, gy, gz = last_gyr
        ax_gyr_txt.set_title("gyr value")
        gyr_text.set_text(f"dps : Gx={gx:+.2f}  Gy={gy:+.2f}  Gz={gz:+.2f}")
        gyr_stats.set_text(f"GYR rate≈{gyr_hz:6.1f} Hz  drops={gyr_drops}")

        return lax, lay, laz, lgx, lgy, lgz, acc_text, acc_text2, acc_stats, gyr_text, gyr_stats

    _ = FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)

    def on_close(_evt):
        stop_evt.set()

    fig.canvas.mpl_connect("close_event", on_close)

    plt.tight_layout()
    plt.show()
    stop_evt.set()


if __name__ == "__main__":
    main()
