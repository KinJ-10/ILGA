#!/usr/bin/env python3
"""Evaluate rolling-median RAS estimators across repeated static-distance series."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import statistics
import sys


ESTIMATORS = {
    "firmware_phase_slope_m": "Official phase slope",
    "firmware_ifft_m": "Official IFFT",
}


def rolling_medians(values: list[float], window_size: int) -> list[float]:
    if window_size < 1 or window_size % 2 == 0:
        raise ValueError("window size must be a positive odd integer")
    if len(values) < window_size:
        return []
    return [statistics.median(values[index - window_size + 1:index + 1])
            for index in range(window_size - 1, len(values))]


def endpoint_calibration(measured_low: float, true_low: float,
                         measured_high: float, true_high: float) -> tuple[float, float]:
    if measured_high == measured_low:
        raise ValueError("endpoint measurements are identical")
    gain = (true_high - true_low) / (measured_high - measured_low)
    return gain, true_low - gain * measured_low


def sample_sd(values: list[float]) -> float:
    return statistics.stdev(values) if len(values) > 1 else math.nan


def median_absolute_deviation(values: list[float]) -> float:
    center = statistics.median(values)
    return statistics.median(abs(value - center) for value in values)


def load_trial(series: str, distance: float, path: Path, window_size: int) -> dict:
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"no procedure rows: {path}")
    result = {
        "series": series,
        "true_distance_m": distance,
        "path": str(path),
        "frames": len(rows),
        "median_good_tones": statistics.median(float(row["good_tone_count"]) for row in rows),
        "estimators": {},
    }
    for key in ESTIMATORS:
        values = [float(row[key]) for row in rows if row.get(key) not in (None, "", "nan")]
        values = [value for value in values if math.isfinite(value)]
        windows = rolling_medians(values, window_size)
        if not windows:
            raise ValueError(f"not enough {key} rows for a {window_size}-frame window: {path}")
        result["estimators"][key] = {
            "raw": values,
            "windows": windows,
            "raw_median": statistics.median(values),
            "raw_sd": sample_sd(values),
            "window_median": statistics.median(windows),
            "window_sd": sample_sd(windows),
            "window_mad": median_absolute_deviation(windows),
        }
    return result


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", action="append", nargs=3,
                        metavar=("SERIES", "TRUE_M", "PROCEDURES_CSV"), required=True)
    parser.add_argument("--window-size", type=int, default=5)
    parser.add_argument("--max-window-sd-m", type=float, default=0.20)
    parser.add_argument("--anchor-tolerance-m", type=float, default=0.20)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.window_size < 1 or args.window_size % 2 == 0:
        parser.error("--window-size must be a positive odd integer")
    if args.max_window_sd_m <= 0 or args.anchor_tolerance_m <= 0:
        parser.error("quality thresholds must be positive")
    if args.output_dir.exists() and (not args.output_dir.is_dir() or any(args.output_dir.iterdir())):
        parser.error("output directory must be empty")

    try:
        trials = [load_trial(series, float(distance), Path(path), args.window_size)
                  for series, distance, path in args.trial]
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    by_series: dict[str, list[dict]] = {}
    for trial in trials:
        by_series.setdefault(trial["series"], []).append(trial)
    distance_sets = []
    for series_trials in by_series.values():
        series_trials.sort(key=lambda item: item["true_distance_m"])
        distances = [item["true_distance_m"] for item in series_trials]
        if len(distances) < 3 or len(set(distances)) != len(distances):
            parser.error("each series requires at least three unique distances")
        distance_sets.append(distances)
    if len(by_series) < 2 or any(distances != distance_sets[0] for distances in distance_sets[1:]):
        parser.error("at least two series with identical distance sets are required")

    trial_rows = []
    window_rows = []
    for trial in trials:
        for key, label in ESTIMATORS.items():
            metrics = trial["estimators"][key]
            stable = metrics["window_sd"] <= args.max_window_sd_m
            trial_rows.append({
                "series": trial["series"], "true_distance_m": trial["true_distance_m"],
                "estimator": key, "label": label, "frames": trial["frames"],
                "median_good_tones": trial["median_good_tones"],
                "raw_median_m": metrics["raw_median"], "raw_sd_m": metrics["raw_sd"],
                "window_size": args.window_size,
                "window_median_m": metrics["window_median"],
                "window_sd_m": metrics["window_sd"], "window_mad_m": metrics["window_mad"],
                "capture_stable": stable,
            })
            for index, value in enumerate(metrics["windows"], args.window_size - 1):
                window_rows.append({
                    "series": trial["series"], "true_distance_m": trial["true_distance_m"],
                    "estimator": key, "capture_index": index, "window_median_m": value,
                })

    calibration_rows = []
    calibrations: dict[tuple[str, str], tuple[float, float]] = {}
    for series, series_trials in by_series.items():
        low, high = series_trials[0], series_trials[-1]
        for key, label in ESTIMATORS.items():
            low_value = low["estimators"][key]["window_median"]
            high_value = high["estimators"][key]["window_median"]
            gain, offset = endpoint_calibration(
                low_value, low["true_distance_m"], high_value, high["true_distance_m"])
            calibrations[(series, key)] = gain, offset
            for trial in series_trials[1:-1]:
                measured = trial["estimators"][key]["window_median"]
                predicted = gain * measured + offset
                calibration_rows.append({
                    "series": series, "estimator": key, "label": label,
                    "gain": gain, "offset_m": offset,
                    "validation_true_m": trial["true_distance_m"],
                    "validation_measured_m": measured,
                    "validation_predicted_m": predicted,
                    "validation_error_m": predicted - trial["true_distance_m"],
                })

    cross_rows = []
    for source_series in by_series:
        for target_series, target_trials in by_series.items():
            if source_series == target_series:
                continue
            for key, label in ESTIMATORS.items():
                gain, offset = calibrations[(source_series, key)]
                for trial in target_trials:
                    measured = trial["estimators"][key]["window_median"]
                    predicted = gain * measured + offset
                    error = predicted - trial["true_distance_m"]
                    cross_rows.append({
                        "source_series": source_series, "target_series": target_series,
                        "estimator": key, "label": label,
                        "true_distance_m": trial["true_distance_m"],
                        "measured_m": measured, "predicted_m": predicted,
                        "error_m": error,
                        "within_anchor_tolerance": abs(error) <= args.anchor_tolerance_m,
                    })

    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "trial_window_summary.csv", trial_rows)
    write_csv(args.output_dir / "window_timeseries.csv", window_rows)
    write_csv(args.output_dir / "series_calibration.csv", calibration_rows)
    write_csv(args.output_dir / "cross_series_validation.csv", cross_rows)

    report = [
        "# IL RAS window and reposition evaluation", "",
        f"Rolling window: median of {args.window_size} frames.", "",
        "## Same-series endpoint calibration", "",
        "| Series | Estimator | Validation true (m) | Predicted (m) | Error (m) |",
        "|---|---|---:|---:|---:|",
    ]
    for row in calibration_rows:
        report.append(f"| {row['series']} | {row['label']} | {row['validation_true_m']:.3f} | "
                      f"{row['validation_predicted_m']:.3f} | {row['validation_error_m']:+.3f} |")
    report.extend(["", "## Cross-series calibration", "",
                   "| Source | Target | Estimator | Maximum absolute error (m) | All within tolerance |",
                   "|---|---|---|---:|---|"])
    for source_series in by_series:
        for target_series in by_series:
            if source_series == target_series:
                continue
            for key, label in ESTIMATORS.items():
                selected = [row for row in cross_rows
                            if row["source_series"] == source_series and
                            row["target_series"] == target_series and row["estimator"] == key]
                maximum = max(abs(row["error_m"]) for row in selected)
                within = all(row["within_anchor_tolerance"] for row in selected)
                report.append(f"| {source_series} | {target_series} | {label} | {maximum:.3f} | {within} |")
    unstable = [row for row in trial_rows if not row["capture_stable"]]
    report.extend(["", "## Quality interpretation", "",
                   f"Within-capture stability threshold: rolling SD <= {args.max_window_sd_m:.3f} m.",
                   f"Cross-series anchor tolerance: absolute error <= {args.anchor_tolerance_m:.3f} m.",
                   f"Unstable trial/estimator rows: {len(unstable)} of {len(trial_rows)}.", "",
                   "A stable capture only shows that the current placement is internally stable. "
                   "It cannot prove that a calibration from another placement is still valid. "
                   "Cross-placement validity requires a known-distance anchor check."])
    (args.output_dir / "REPORT.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    print(f"Evaluated {len(trials)} trials in {len(by_series)} series")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
