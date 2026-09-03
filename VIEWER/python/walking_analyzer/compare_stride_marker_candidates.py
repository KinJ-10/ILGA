#!/usr/bin/env python3
"""Compare exploratory gx stride-marker candidates without changing the baseline detector."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
from scipy.signal import butter, filtfilt, find_peaks

import evaluate_gait_events as baseline


DIAGNOSTIC_SEARCH_SEC = 0.80


@dataclass(frozen=True)
class CandidateConfig:
    name: str
    crossing_level_dps: float = 0.0
    crossing_window_sec: float = baseline.ZERO_CROSS_WINDOW_SEC
    confirmation_required: bool = True
    slow_only_window_sec: float | None = None
    slow_step_rate_threshold: float = 90.0
    description: str = ""

    def effective_window(self, observed_step_rate: float | None) -> float:
        if (
            self.slow_only_window_sec is not None
            and observed_step_rate is not None
            and observed_step_rate < self.slow_step_rate_threshold
        ):
            return self.slow_only_window_sec
        return self.crossing_window_sec


CONFIGS = (
    CandidateConfig(name="baseline_fixed", description="unchanged fixed detector"),
    CandidateConfig(name="zero_cross_window_0p35", crossing_window_sec=0.35),
    CandidateConfig(name="zero_cross_window_0p40", crossing_window_sec=0.40),
    CandidateConfig(name="zero_cross_window_0p45", crossing_window_sec=0.45),
    CandidateConfig(
        name="near_zero_plus10_window_0p30",
        crossing_level_dps=10.0,
        crossing_window_sec=0.30,
        description="exploratory +10 dps downward crossing; confirmation remains -50 dps",
    ),
    CandidateConfig(
        name="speed_adaptive_window_0p45_below90",
        slow_only_window_sec=0.45,
        description="0.45 s only when hand-count step rate is below 90 steps/min",
    ),
    CandidateConfig(
        name="negative50_cross_window_0p45",
        crossing_level_dps=-50.0,
        crossing_window_sec=0.45,
        confirmation_required=False,
        description="exploratory direct -50 dps crossing; event meaning differs from baseline",
    ),
)


def first_downward_crossing(
    values: np.ndarray,
    times: np.ndarray,
    peak_index: int,
    level_dps: float,
    window_sec: float,
) -> float | None:
    limit = min(len(values) - 1, peak_index + round(window_sec * baseline.FS_HZ))
    for index in range(peak_index, limit):
        left = float(values[index] - level_dps)
        right = float(values[index + 1] - level_dps)
        if left > 0.0 and right <= 0.0:
            return baseline.interpolate_zero_cross(
                float(times[index]), left, float(times[index + 1]), right
            )
    return None


def waveform_diagnostic(
    values: np.ndarray,
    times: np.ndarray,
    peak_index: int,
    config: CandidateConfig,
    observed_step_rate: float | None,
) -> dict[str, object]:
    peak_time = float(times[peak_index])
    effective_window = config.effective_window(observed_step_rate)
    crossing = first_downward_crossing(
        values, times, peak_index, config.crossing_level_dps, effective_window
    )
    zero_cross = first_downward_crossing(
        values, times, peak_index, 0.0, DIAGNOSTIC_SEARCH_SEC
    )
    confirm_min: float | None = None
    confirmation_pass = not config.confirmation_required
    if crossing is not None and config.confirmation_required:
        crossing_index = int(np.searchsorted(times, crossing, side="right"))
        confirm_end = min(
            len(values), crossing_index + round(baseline.CONFIRM_WINDOW_SEC * baseline.FS_HZ)
        )
        confirm_values = values[crossing_index:confirm_end]
        if len(confirm_values):
            confirm_min = float(np.min(confirm_values))
            confirmation_pass = confirm_min <= baseline.CONFIRM_GX_DPS
    elif crossing is not None:
        confirm_min = float(config.crossing_level_dps)

    def min_after(window_sec: float) -> float | None:
        end = min(len(values), peak_index + 1 + round(window_sec * baseline.FS_HZ))
        section = values[peak_index + 1 : end]
        return None if len(section) == 0 else float(np.min(section))

    zero_lag = None if zero_cross is None else zero_cross - peak_time
    if zero_lag is None:
        shape = "no_zero_cross_within_0p80s"
    elif zero_lag > baseline.ZERO_CROSS_WINDOW_SEC:
        shape = "delayed_zero_cross_after_baseline_window"
    else:
        shape = "zero_cross_within_baseline_window"
    formal = crossing is not None and confirmation_pass
    return {
        "effective_crossing_window_sec": effective_window,
        "selected_crossing_time_sec": crossing,
        "selected_crossing_lag_sec": None if crossing is None else crossing - peak_time,
        "diagnostic_zero_cross_time_sec": zero_cross,
        "diagnostic_zero_cross_lag_sec": zero_lag,
        "min_gx_peak_to_0p30_sec": min_after(0.30),
        "min_gx_peak_to_0p40_sec": min_after(0.40),
        "min_gx_peak_to_0p45_sec": min_after(0.45),
        "confirmation_window_sec": baseline.CONFIRM_WINDOW_SEC,
        "confirmation_min_gx_dps": confirm_min,
        "confirmation_pass": confirmation_pass,
        "formal": formal,
        "failure_reason": (
            ""
            if formal
            else (
                "no_zero_cross"
                if crossing is None and config.crossing_level_dps == 0.0
                else ("no_crossing" if crossing is None else "no_negative_confirmation")
            )
        ),
        "waveform_shape": shape,
    }


def detect_with_config(
    sensor: baseline.SensorSeries,
    config: CandidateConfig,
    observed_step_rate: float | None,
) -> list[dict[str, object]]:
    b, a = butter(
        baseline.LPF_ORDER,
        baseline.LPF_CUTOFF_HZ / (baseline.FS_HZ / 2.0),
        btype="low",
    )
    filtered = filtfilt(b, a, sensor.grid_gx_dps)
    peaks, properties = find_peaks(
        filtered,
        height=baseline.PEAK_HEIGHT_DPS,
        prominence=baseline.PEAK_PROMINENCE_DPS,
        distance=math.ceil(baseline.MIN_DISTANCE_SEC * baseline.FS_HZ),
    )
    rows: list[dict[str, object]] = []
    for index, (peak, height, prominence) in enumerate(
        zip(peaks, properties["peak_heights"], properties["prominences"]), start=1
    ):
        diagnostic = waveform_diagnostic(
            filtered, sensor.grid_time_sec, int(peak), config, observed_step_rate
        )
        event_time = diagnostic["selected_crossing_time_sec"]
        rows.append(
            {
                "candidate_index": index,
                "peak_seq": int(sensor.grid_seq[peak] % baseline.SEQ_MODULUS),
                "peak_time_sec": float(sensor.grid_time_sec[peak]),
                "peak_gx_dps": float(height),
                "prominence_dps": float(prominence),
                "event_time_sec": event_time,
                **diagnostic,
            }
        )
    return rows


def evaluate_trial(
    row: dict[str, str], config: CandidateConfig
) -> tuple[dict[str, object], list[dict[str, object]]]:
    sensor = baseline.load_sensor_csv(Path(row["sensor_csv"]))
    start = float(row["interval_start_sensor_sec"])
    end = float(row["interval_end_sensor_sec"])
    actual_steps = int(row["actual_steps"])
    expected = int(row["expected_sensor_foot_contacts"])
    observed_step_rate = 60.0 * actual_steps / (end - start)
    candidates = detect_with_config(sensor, config, observed_step_rate)
    detail: list[dict[str, object]] = []
    event_times: list[float] = []
    for candidate in candidates:
        peak_inside = start <= float(candidate["peak_time_sec"]) <= end
        event_time = candidate["event_time_sec"]
        event_inside = event_time is not None and start <= float(event_time) <= end
        if peak_inside:
            detail.append(
                {
                    "trial": row["trial"],
                    "dataset_group": row["dataset_group"],
                    "speed_condition": row["speed_condition"],
                    "config": config.name,
                    "peak_inside_interval": peak_inside,
                    "event_inside_interval": event_inside,
                    **candidate,
                }
            )
        if bool(candidate["formal"]) and event_inside:
            event_times.append(float(event_time))
    intervals = np.diff(event_times)
    median_stride = float(np.median(intervals)) if len(intervals) else None
    cadence = None if median_stride is None or median_stride <= 0.0 else 120.0 / median_stride
    return (
        {
            "trial": row["trial"],
            "dataset_group": row["dataset_group"],
            "speed_condition": row["speed_condition"],
            "config": config.name,
            "actual_steps": actual_steps,
            "expected_sensor_foot_contacts": expected,
            "formal_marker_count": len(event_times),
            "marker_tp_count_proxy": min(len(event_times), expected),
            "marker_fp_count_proxy": max(0, len(event_times) - expected),
            "marker_fn_count_proxy": max(0, expected - len(event_times)),
            "marker_recall_count_proxy": min(len(event_times), expected) / expected,
            "interval_missing_count_proxy": max(0, expected - len(event_times)),
            "median_stride_time_sec": median_stride,
            "cadence_steps_per_min": cadence,
            "hand_count_rate_steps_per_min": observed_step_rate,
            "cadence_abs_error_steps_per_min": (
                None if cadence is None else abs(cadence - observed_step_rate)
            ),
            "comparison_scope": "count proxy only; not event-time ground truth",
        },
        detail,
    )


def _write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
    fields = list(rows[0]) if rows else []
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        if fields:
            writer.writeheader()
            writer.writerows(rows)


def run(spec_csv: Path, out_dir: Path) -> dict[str, object]:
    if out_dir.exists():
        raise ValueError(f"output directory already exists: {out_dir}")
    with spec_csv.open(newline="", encoding="utf-8") as handle:
        trial_rows = list(csv.DictReader(handle))
    if not trial_rows:
        raise ValueError("trial specification is empty")
    metrics: list[dict[str, object]] = []
    details: list[dict[str, object]] = []
    for config in CONFIGS:
        for trial in trial_rows:
            metric, detail = evaluate_trial(trial, config)
            metrics.append(metric)
            details.extend(detail)
    baseline_counts = {
        row["trial"]: int(row["formal_marker_count"])
        for row in metrics
        if row["config"] == "baseline_fixed"
    }
    aggregates = []
    for config in CONFIGS:
        selected = [row for row in metrics if row["config"] == config.name]
        cadence_errors = [
            float(row["cadence_abs_error_steps_per_min"])
            for row in selected
            if row["cadence_abs_error_steps_per_min"] is not None
        ]
        normal_changes = sum(
            row["speed_condition"] == "normal"
            and int(row["formal_marker_count"]) != baseline_counts[row["trial"]]
            for row in selected
        )
        aggregates.append(
            {
                "config": config.name,
                "trial_count": len(selected),
                "expected_marker_total": sum(int(row["expected_sensor_foot_contacts"]) for row in selected),
                "formal_marker_total": sum(int(row["formal_marker_count"]) for row in selected),
                "marker_tp_count_proxy": sum(int(row["marker_tp_count_proxy"]) for row in selected),
                "marker_fp_count_proxy": sum(int(row["marker_fp_count_proxy"]) for row in selected),
                "marker_fn_count_proxy": sum(int(row["marker_fn_count_proxy"]) for row in selected),
                "marker_recall_count_proxy": (
                    sum(int(row["marker_tp_count_proxy"]) for row in selected)
                    / sum(int(row["expected_sensor_foot_contacts"]) for row in selected)
                ),
                "mean_cadence_abs_error_steps_per_min": (
                    statistics.mean(cadence_errors) if cadence_errors else None
                ),
                "normal_trial_marker_count_changes_from_baseline": normal_changes,
                "production_decision": "comparison_only_not_adopted",
            }
        )
    result = {
        "schema_version": 1,
        "baseline": "baseline_fixed is identical in constants to evaluate_gait_events.py",
        "trial_count": len(trial_rows),
        "configuration_count": len(CONFIGS),
        "aggregate": aggregates,
        "limitations": [
            "Recall, FP and FN are count proxies against expected attachment-foot contacts, not timestamped ground truth.",
            "Cadence reference uses hand-count steps divided by operator-marker duration.",
            "No candidate is adopted as the production detector by this comparison.",
        ],
    }
    out_dir.mkdir(parents=True, exist_ok=False)
    _write_csv(out_dir / "configuration_trial_metrics.csv", metrics)
    _write_csv(out_dir / "candidate_waveform_diagnostics.csv", details)
    _write_csv(out_dir / "configuration_aggregate.csv", aggregates)
    (out_dir / "comparison_summary.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return result


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial_spec_csv", type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    try:
        args = parse_args(argv)
        result = run(args.trial_spec_csv, args.out_dir)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
