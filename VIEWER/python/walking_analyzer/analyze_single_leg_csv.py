#!/usr/bin/env python3
"""
Minimal single-leg walking analyzer for ILGA TAG CSV logs.

Input CSV:
- seq, ax_mg, ay_mg, az_mg, gx_mdps, gy_mdps, gz_mdps

Assumptions:
- Offline analysis
- Fixed sampling rate provided by --fs
- Single-leg periodic motion
- Minimal peak-based step detection, not a clinical-grade algorithm

Outputs:
- summary JSON
- step_events CSV
- optional PNG plot

Examples:
  python analyze_single_leg_csv.py input.csv --fs 100
  python analyze_single_leg_csv.py input.csv --fs 100 --out-dir out
  python analyze_single_leg_csv.py input.csv --fs 100 --plot out/steps.png

Notes:
- The parser skips malformed lines and non-numeric rows.
- Peak acceleration is computed from dynamic acceleration magnitude:
  abs(norm(acc_xyz) - median(norm(acc_xyz)))
- Peak angular velocity is computed from gyro norm.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean, median, pstdev
from typing import Dict, List, Sequence, Tuple


REQUIRED_COLUMNS = ("seq", "ax_mg", "ay_mg", "az_mg", "gx_mdps", "gy_mdps", "gz_mdps")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze single-leg walking from ILGA TAG CSV.")
    parser.add_argument("input_csv", help="Input CSV or UART log file")
    parser.add_argument("--fs", type=float, required=True, help="Sampling frequency in Hz, e.g. 100")
    parser.add_argument(
        "--out-dir",
        default=None,
        help="Output directory. Default: <input_stem>_analysis next to input file",
    )
    parser.add_argument(
        "--plot",
        default=None,
        help="Optional PNG output path for a simple diagnostic plot",
    )
    parser.add_argument(
        "--min-step-sec",
        type=float,
        default=0.35,
        help="Minimum allowed interval between step peaks in seconds (default: 0.35)",
    )
    parser.add_argument(
        "--threshold-scale",
        type=float,
        default=0.35,
        help="Adaptive threshold scale between median and p95 of dynamic acceleration (default: 0.35)",
    )
    return parser.parse_args()


def try_parse_floats(row: Sequence[str]) -> List[float]:
    values: List[float] = []
    for cell in row:
        values.append(float(cell.strip()))
    return values


def load_samples(path: Path) -> Dict[str, List[float]]:
    data = {name: [] for name in REQUIRED_COLUMNS}
    column_map = None

    with path.open("r", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue

            stripped = [cell.strip() for cell in row]
            lowered = [cell.lower() for cell in stripped]

            if column_map is None and all(name in lowered for name in REQUIRED_COLUMNS):
                column_map = {name: lowered.index(name) for name in REQUIRED_COLUMNS}
                continue

            try:
                if column_map is not None:
                    for name in REQUIRED_COLUMNS:
                        data[name].append(float(stripped[column_map[name]]))
                else:
                    numeric = try_parse_floats(stripped)
                    if len(numeric) < 7:
                        continue
                    for idx, name in enumerate(REQUIRED_COLUMNS):
                        data[name].append(numeric[idx])
            except (ValueError, IndexError):
                continue

    if not data["seq"]:
        raise ValueError(f"No valid samples found in {path}")

    return data


def vector_norm3(x: Sequence[float], y: Sequence[float], z: Sequence[float]) -> List[float]:
    return [math.sqrt(ax * ax + ay * ay + az * az) for ax, ay, az in zip(x, y, z)]


def moving_average(signal: Sequence[float], window: int) -> List[float]:
    if window <= 1:
        return list(signal)

    out: List[float] = []
    acc = 0.0
    q: List[float] = []
    for value in signal:
        q.append(value)
        acc += value
        if len(q) > window:
            acc -= q.pop(0)
        out.append(acc / len(q))
    return out


def percentile(values: Sequence[float], p: float) -> float:
    if not values:
        return 0.0
    if p <= 0:
        return min(values)
    if p >= 100:
        return max(values)

    sorted_values = sorted(values)
    rank = (len(sorted_values) - 1) * (p / 100.0)
    low = math.floor(rank)
    high = math.ceil(rank)
    if low == high:
        return sorted_values[low]
    frac = rank - low
    return sorted_values[low] * (1.0 - frac) + sorted_values[high] * frac


def detect_step_peaks(signal: Sequence[float], fs: float, min_step_sec: float, threshold_scale: float) -> Tuple[List[int], float]:
    smooth_window = max(1, int(fs * 0.12))
    smoothed = moving_average(signal, smooth_window)

    base = median(smoothed)
    p95 = percentile(smoothed, 95.0)
    threshold = base + threshold_scale * max(0.0, p95 - base)
    min_distance = max(1, int(fs * min_step_sec))

    peaks: List[int] = []
    last_peak = -min_distance
    for idx in range(1, len(smoothed) - 1):
        value = smoothed[idx]
        if value < threshold:
            continue
        if value < smoothed[idx - 1] or value < smoothed[idx + 1]:
            continue
        if idx - last_peak < min_distance:
            if value > smoothed[peaks[-1]]:
                peaks[-1] = idx
                last_peak = idx
            continue
        peaks.append(idx)
        last_peak = idx

    return peaks, threshold


def build_step_events(
    peaks: Sequence[int],
    fs: float,
    acc_dyn_mg: Sequence[float],
    gyro_norm_mdps: Sequence[float],
) -> List[Dict[str, float]]:
    events: List[Dict[str, float]] = []
    if not peaks:
        return events

    for step_idx, peak_idx in enumerate(peaks):
        left = 0 if step_idx == 0 else (peaks[step_idx - 1] + peak_idx) // 2
        right = len(acc_dyn_mg) if step_idx == len(peaks) - 1 else (peak_idx + peaks[step_idx + 1]) // 2

        peak_acc = max(acc_dyn_mg[left:right]) if right > left else acc_dyn_mg[peak_idx]
        peak_gyro = max(gyro_norm_mdps[left:right]) if right > left else gyro_norm_mdps[peak_idx]
        step_time_sec = peak_idx / fs

        interval_sec = None
        if step_idx > 0:
            interval_sec = (peak_idx - peaks[step_idx - 1]) / fs

        events.append(
            {
                "step_index": step_idx + 1,
                "sample_index": peak_idx,
                "time_sec": step_time_sec,
                "step_interval_sec": interval_sec,
                "peak_acc_dyn_mg": peak_acc,
                "peak_gyro_norm_mdps": peak_gyro,
            }
        )

    return events


def write_step_events_csv(path: Path, events: Sequence[Dict[str, float]]) -> None:
    fieldnames = [
        "step_index",
        "sample_index",
        "time_sec",
        "step_interval_sec",
        "peak_acc_dyn_mg",
        "peak_gyro_norm_mdps",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(events)


def save_plot(
    plot_path: Path,
    fs: float,
    acc_dyn_mg: Sequence[float],
    ax_mg: Sequence[float],
    ay_mg: Sequence[float],
    az_mg: Sequence[float],
    gyro_norm_mdps: Sequence[float],
    gx_mdps: Sequence[float],
    gy_mdps: Sequence[float],
    gz_mdps: Sequence[float],
    peaks: Sequence[int],
    threshold: float,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        raise RuntimeError("matplotlib is required for --plot") from exc

    time_axis = [idx / fs for idx in range(len(acc_dyn_mg))]
    peak_times = [idx / fs for idx in peaks]
    peak_values = [acc_dyn_mg[idx] for idx in peaks]

    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

    for axis in (ax1, ax2, ax3, ax4):
        for peak_time in peak_times:
            axis.axvline(peak_time, color="tab:red", alpha=0.12, linewidth=0.8)
        axis.grid(True, alpha=0.3)

    ax1.plot(time_axis, acc_dyn_mg, label="acc_dyn_mg")
    ax1.axhline(threshold, color="tab:red", linestyle="--", label="threshold")
    ax1.scatter(peak_times, peak_values, color="tab:red", s=18, label="step peaks")
    ax1.set_ylabel("Dynamic Acc [mg]")
    ax1.legend(loc="upper right")

    ax2.plot(time_axis, ax_mg, label="ax_mg", linewidth=0.9)
    ax2.plot(time_axis, ay_mg, label="ay_mg", linewidth=0.9)
    ax2.plot(time_axis, az_mg, label="az_mg", linewidth=0.9)
    ax2.set_ylabel("Acc Raw [mg]")
    ax2.legend(loc="upper right")

    ax3.plot(time_axis, gyro_norm_mdps, label="gyro_norm_mdps", color="tab:green")
    ax3.set_ylabel("Gyro Norm [mdps]")
    ax3.legend(loc="upper right")

    ax4.plot(time_axis, gx_mdps, label="gx_mdps", linewidth=0.9)
    ax4.plot(time_axis, gy_mdps, label="gy_mdps", linewidth=0.9)
    ax4.plot(time_axis, gz_mdps, label="gz_mdps", linewidth=0.9)
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Gyro Raw [mdps]")
    ax4.legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)


def build_summary(
    input_path: Path,
    fs: float,
    sample_count: int,
    peaks: Sequence[int],
    step_events: Sequence[Dict[str, float]],
    acc_dyn_mg: Sequence[float],
    gyro_norm_mdps: Sequence[float],
    threshold: float,
) -> Dict[str, object]:
    duration_sec = sample_count / fs if fs > 0 else 0.0
    step_intervals = [event["step_interval_sec"] for event in step_events if event["step_interval_sec"] is not None]

    cadence_spm = 0.0
    if duration_sec > 0:
        cadence_spm = len(peaks) * 60.0 / duration_sec

    interval_mean = mean(step_intervals) if step_intervals else None
    interval_std = pstdev(step_intervals) if len(step_intervals) >= 2 else None
    interval_cv = None
    if interval_mean and interval_mean > 0 and interval_std is not None:
        interval_cv = (interval_std / interval_mean) * 100.0

    periodicity_score = None
    if interval_cv is not None:
        periodicity_score = max(0.0, 100.0 - interval_cv)

    return {
        "input_file": str(input_path),
        "fs_hz": fs,
        "sample_count": sample_count,
        "duration_sec": duration_sec,
        "step_count": len(peaks),
        "cadence_spm": cadence_spm,
        "step_intervals_sec": step_intervals,
        "step_interval_mean_sec": interval_mean,
        "step_interval_std_sec": interval_std,
        "step_interval_cv_pct": interval_cv,
        "single_leg_periodicity_score": periodicity_score,
        "peak_acc_dyn_mg_global": max(acc_dyn_mg) if acc_dyn_mg else None,
        "peak_gyro_norm_mdps_global": max(gyro_norm_mdps) if gyro_norm_mdps else None,
        "detection_threshold_mg": threshold,
    }


def main() -> int:
    args = parse_args()
    input_path = Path(args.input_csv)
    out_dir = Path(args.out_dir) if args.out_dir else input_path.with_name(f"{input_path.stem}_analysis")
    out_dir.mkdir(parents=True, exist_ok=True)

    data = load_samples(input_path)
    sample_count = len(data["seq"])

    acc_norm_mg = vector_norm3(data["ax_mg"], data["ay_mg"], data["az_mg"])
    acc_norm_base = median(acc_norm_mg)
    acc_dyn_mg = [abs(value - acc_norm_base) for value in acc_norm_mg]
    gyro_norm_mdps = vector_norm3(data["gx_mdps"], data["gy_mdps"], data["gz_mdps"])

    peaks, threshold = detect_step_peaks(
        acc_dyn_mg,
        fs=args.fs,
        min_step_sec=args.min_step_sec,
        threshold_scale=args.threshold_scale,
    )
    step_events = build_step_events(peaks, args.fs, acc_dyn_mg, gyro_norm_mdps)
    summary = build_summary(
        input_path=input_path,
        fs=args.fs,
        sample_count=sample_count,
        peaks=peaks,
        step_events=step_events,
        acc_dyn_mg=acc_dyn_mg,
        gyro_norm_mdps=gyro_norm_mdps,
        threshold=threshold,
    )

    summary_path = out_dir / "summary.json"
    step_events_path = out_dir / "step_events.csv"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=True) + "\n")
    write_step_events_csv(step_events_path, step_events)

    print(f"[INFO] input_file={input_path}")
    print(f"[INFO] sample_count={sample_count}")
    print(f"[INFO] duration_sec={summary['duration_sec']:.3f}")
    print(f"[INFO] step_count={summary['step_count']}")
    print(f"[INFO] cadence_spm={summary['cadence_spm']:.3f}")
    print(f"[INFO] step_interval_mean_sec={summary['step_interval_mean_sec']}")
    print(f"[INFO] step_interval_cv_pct={summary['step_interval_cv_pct']}")
    print(f"[INFO] peak_acc_dyn_mg_global={summary['peak_acc_dyn_mg_global']}")
    print(f"[INFO] peak_gyro_norm_mdps_global={summary['peak_gyro_norm_mdps_global']}")
    print(f"[INFO] summary_saved={summary_path}")
    print(f"[INFO] step_events_saved={step_events_path}")

    if args.plot:
        plot_path = Path(args.plot)
        plot_path.parent.mkdir(parents=True, exist_ok=True)
        save_plot(
            plot_path,
            args.fs,
            acc_dyn_mg,
            data["ax_mg"],
            data["ay_mg"],
            data["az_mg"],
            gyro_norm_mdps,
            data["gx_mdps"],
            data["gy_mdps"],
            data["gz_mdps"],
            peaks,
            threshold,
        )
        print(f"[INFO] plot_saved={plot_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
