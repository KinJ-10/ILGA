#!/usr/bin/env python3
"""Compare ILRAS1 distance estimators across known static distances."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import statistics
import sys

import numpy as np


ESTIMATORS = {
    "firmware_ifft_m": "Official IFFT",
    "firmware_phase_slope_m": "Official phase slope",
    "early_peak_m": "Early IFFT peak",
    "robust_phase_slope_m": "Robust phase slope",
    "firmware_rtt_m": "RTT",
}


def endpoint_calibration(measured_low: float, true_low: float,
                         measured_high: float, true_high: float) -> tuple[float, float]:
    if measured_high == measured_low:
        raise ValueError("endpoint measurements are identical")
    gain = (true_high - true_low) / (measured_high - measured_low)
    offset = true_low - gain * measured_low
    return gain, offset


def load_trial(distance: float, path: Path) -> dict:
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"no procedure rows: {path}")
    result = {"true_distance_m": distance, "path": str(path), "frames": len(rows), "values": {}}
    for key in ESTIMATORS:
        values = [float(row[key]) for row in rows if row.get(key) not in (None, "", "nan")]
        finite = [value for value in values if math.isfinite(value)]
        if finite:
            result["values"][key] = finite
    return result


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", action="append", nargs=2, metavar=("TRUE_M", "PROCEDURES_CSV"),
                        required=True)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args(argv)
    if len(args.trial) < 3:
        parser.error("at least three --trial entries are required")
    if args.output_dir.exists() and (not args.output_dir.is_dir() or any(args.output_dir.iterdir())):
        parser.error("output directory must be empty")
    try:
        trials = sorted((load_trial(float(distance), Path(path)) for distance, path in args.trial),
                        key=lambda item: item["true_distance_m"])
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    summary_rows = []
    for trial in trials:
        for key, label in ESTIMATORS.items():
            values = trial["values"].get(key, [])
            if not values:
                continue
            summary_rows.append({
                "true_distance_m": trial["true_distance_m"],
                "estimator": key,
                "label": label,
                "frames": len(values),
                "median_m": statistics.median(values),
                "mean_m": statistics.mean(values),
                "standard_deviation_m": statistics.stdev(values) if len(values) > 1 else math.nan,
                "minimum_m": min(values),
                "maximum_m": max(values),
            })

    calibration_rows = []
    middle_trials = trials[1:-1]
    for key, label in ESTIMATORS.items():
        medians = []
        for trial in trials:
            values = trial["values"].get(key, [])
            medians.append(statistics.median(values) if values else math.nan)
        finite = all(math.isfinite(value) for value in medians)
        monotonic = finite and all(left < right for left, right in zip(medians, medians[1:]))
        if not finite:
            continue
        try:
            gain, offset = endpoint_calibration(
                medians[0], trials[0]["true_distance_m"], medians[-1], trials[-1]["true_distance_m"])
        except ValueError:
            gain, offset = math.nan, math.nan
        for trial, measured in zip(middle_trials, medians[1:-1]):
            predicted = gain * measured + offset
            calibration_rows.append({
                "estimator": key,
                "label": label,
                "monotonic_medians": monotonic,
                "calibration_low_true_m": trials[0]["true_distance_m"],
                "calibration_high_true_m": trials[-1]["true_distance_m"],
                "gain": gain,
                "offset_m": offset,
                "validation_true_m": trial["true_distance_m"],
                "validation_measured_m": measured,
                "validation_predicted_m": predicted,
                "validation_error_m": predicted - trial["true_distance_m"],
            })

    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "distance_summary.csv", summary_rows)
    write_csv(args.output_dir / "endpoint_calibration_validation.csv", calibration_rows)

    report = [
        "# IL RAS static distance comparison",
        "",
        "Endpoint calibration uses only the shortest and longest distances; intermediate distances are validation data.",
        "",
        "| Estimator | Monotonic | Gain | Offset (m) | Validation true (m) | Predicted (m) | Error (m) |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for row in calibration_rows:
        report.append(
            f"| {row['label']} | {row['monotonic_medians']} | {row['gain']:.4f} | "
            f"{row['offset_m']:.4f} | {row['validation_true_m']:.3f} | "
            f"{row['validation_predicted_m']:.3f} | {row['validation_error_m']:+.3f} |")
    (args.output_dir / "COMPARISON_REPORT.md").write_text("\n".join(report) + "\n", encoding="utf-8")

    try:
        import matplotlib.pyplot as plt

        figure, axis = plt.subplots(figsize=(9, 6), constrained_layout=True)
        true_values = np.asarray([trial["true_distance_m"] for trial in trials])
        for key, label in ESTIMATORS.items():
            medians = [statistics.median(trial["values"][key]) for trial in trials]
            axis.plot(true_values, medians, marker="o", label=label)
        axis.set(xlabel="True distance (m)", ylabel="Uncalibrated estimate (m)",
                 title="IL RAS estimator response")
        axis.grid(alpha=0.3)
        axis.legend()
        figure.savefig(args.output_dir / "distance_response.png", dpi=160)
        plt.close(figure)
    except ImportError:
        print("WARNING: matplotlib unavailable; PNG was not generated", file=sys.stderr)

    print(f"Compared {len(trials)} distances and {len(ESTIMATORS)} estimators")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
