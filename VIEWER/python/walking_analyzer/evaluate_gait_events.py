#!/usr/bin/env python3
"""Offline evaluation of the fixed ILGA right-foot gx event candidate.

This tool is deliberately separate from ``analyze_single_leg_csv.py``.  It uses
the Kin/doi GA Phase1-a parameters without tuning and maps the sensor's 100 Hz
sequence grid to video time with synchronization points before and after the
evaluation interval.

The Butterworth filter is zero-phase and therefore offline-only.  Results from
this script must not be treated as timing results for a future causal detector.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
from scipy.signal import butter, filtfilt, find_peaks


FS_HZ = 100.0
SEQ_MODULUS = 65536
LPF_ORDER = 4
LPF_CUTOFF_HZ = 5.0
PEAK_HEIGHT_DPS = 80.0
PEAK_PROMINENCE_DPS = 50.0
MIN_DISTANCE_SEC = 0.75
ZERO_CROSS_WINDOW_SEC = 0.30
CONFIRM_WINDOW_SEC = 0.40
CONFIRM_GX_DPS = -50.0
MIN_SYNC_COVERAGE_RATIO = 0.90
MAX_SYNC_RESIDUAL_SEC = 0.020

GT_COLUMNS = (
    "trial",
    "event_index",
    "foot",
    "video_frame",
    "video_time_sec",
    "event_type",
    "confidence",
    "notes",
)
SYNC_COLUMNS = ("trial", "sync_index", "video_frame", "video_time_sec", "seq", "notes")
SYNC_PHASES = ("pre", "post")
CONTACT_TYPES = ("normal_contact", "terminal_contact")
CONTEXT_TYPES = ("adjustment", "turn")
ALLOWED_EVENT_TYPES = CONTACT_TYPES + CONTEXT_TYPES


@dataclass(frozen=True)
class SensorSeries:
    raw_seq: np.ndarray
    unwrapped_seq: np.ndarray
    sensor_time_sec: np.ndarray
    gx_dps: np.ndarray
    grid_seq: np.ndarray
    grid_time_sec: np.ndarray
    grid_gx_dps: np.ndarray
    missing_seq: np.ndarray


@dataclass(frozen=True)
class SyncFit:
    slope: float
    intercept: float
    rmse_sec: float
    max_abs_residual_sec: float
    point_count: int

    def to_video_time(self, sensor_time_sec: float) -> float:
        return self.slope * sensor_time_sec + self.intercept


@dataclass(frozen=True)
class SyncCoverage:
    pre_point_count: int
    post_point_count: int
    inferred_phase_point_count: int
    standard_six_point_protocol: bool
    sync_video_start_sec: float
    sync_video_end_sec: float
    eval_video_start_sec: float
    eval_video_end_sec: float
    video_coverage_ratio: float
    sync_sensor_start_sec: float
    sync_sensor_end_sec: float
    eval_sensor_start_sec: float
    eval_sensor_end_sec: float
    sensor_coverage_ratio: float
    max_allowed_residual_sec: float
    warnings: tuple[str, ...]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate the fixed offline gx candidate against synchronized video ground truth."
    )
    parser.add_argument("sensor_csv", type=Path)
    parser.add_argument("ground_truth_csv", type=Path)
    parser.add_argument("sync_points_csv", type=Path)
    parser.add_argument("--trial", required=True, help="Trial ID present in GT and sync CSVs")
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--eval-start-video-sec", type=float, required=True)
    parser.add_argument("--terminal-start-video-sec", type=float, required=True)
    parser.add_argument("--eval-end-video-sec", type=float, required=True)
    parser.add_argument("--tolerance-sec", type=float, default=0.10)
    parser.add_argument("--missing-near-sec", type=float, default=0.10)
    return parser.parse_args()


def unwrap_seq(raw_seq: Sequence[int], modulus: int = SEQ_MODULUS) -> np.ndarray:
    if not raw_seq:
        raise ValueError("sensor CSV contains no samples")
    out = [int(raw_seq[0])]
    offset = 0
    previous = int(raw_seq[0])
    for value_in in raw_seq[1:]:
        value = int(value_in)
        delta = value - previous
        if delta < -(modulus // 2):
            offset += modulus
        elif delta < 0:
            raise ValueError(f"seq is reverse ordered: {previous} -> {value}")
        unwrapped = value + offset
        if unwrapped == out[-1]:
            raise ValueError(f"duplicate seq: {value}")
        if unwrapped < out[-1]:
            raise ValueError(f"seq is reverse ordered after unwrap: {previous} -> {value}")
        out.append(unwrapped)
        previous = value
    return np.asarray(out, dtype=np.int64)


def load_sensor_csv(path: Path) -> SensorSeries:
    raw_seq: list[int] = []
    gx_dps: list[float] = []
    with path.open(newline="") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None or not {"seq", "gx_mdps"}.issubset(reader.fieldnames):
            raise ValueError("sensor CSV must contain seq and gx_mdps")
        for line_number, row in enumerate(reader, start=2):
            try:
                raw_seq.append(int(row["seq"]))
                gx_dps.append(float(row["gx_mdps"]) / 1000.0)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"invalid sensor row at line {line_number}") from exc

    unwrapped = unwrap_seq(raw_seq)
    if len(unwrapped) < 16:
        raise ValueError("sensor CSV is too short for zero-phase filtering")
    grid_seq = np.arange(unwrapped[0], unwrapped[-1] + 1, dtype=np.int64)
    grid_time = (grid_seq - grid_seq[0]) / FS_HZ
    sensor_time = (unwrapped - grid_seq[0]) / FS_HZ
    grid_gx = np.interp(grid_seq, unwrapped, np.asarray(gx_dps, dtype=float))
    missing = np.setdiff1d(grid_seq, unwrapped, assume_unique=True)
    return SensorSeries(
        raw_seq=np.asarray(raw_seq, dtype=np.int64),
        unwrapped_seq=unwrapped,
        sensor_time_sec=sensor_time,
        gx_dps=np.asarray(gx_dps, dtype=float),
        grid_seq=grid_seq,
        grid_time_sec=grid_time,
        grid_gx_dps=grid_gx,
        missing_seq=missing,
    )


def read_rows(path: Path, required: Sequence[str]) -> list[dict[str, str]]:
    with path.open(newline="") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None:
            raise ValueError(f"{path} has no CSV header")
        missing = [column for column in required if column not in reader.fieldnames]
        if missing:
            raise ValueError(f"{path} is missing columns: {', '.join(missing)}")
        return list(reader)


def load_ground_truth(path: Path, trial: str) -> list[dict[str, object]]:
    rows = [row for row in read_rows(path, GT_COLUMNS) if row["trial"] == trial]
    if not rows:
        raise ValueError(f"no ground-truth rows for trial {trial!r}")
    parsed: list[dict[str, object]] = []
    for row in rows:
        event_type = row["event_type"].strip().lower()
        foot = row["foot"].strip().lower()
        if event_type not in ALLOWED_EVENT_TYPES:
            raise ValueError(f"unsupported event_type {event_type!r}")
        if foot not in {"left", "right", "na"}:
            raise ValueError(f"unsupported foot {foot!r}")
        parsed.append(
            {
                **row,
                "event_index": int(row["event_index"]),
                "video_frame": int(row["video_frame"]),
                "video_time_sec": float(row["video_time_sec"]),
                "event_type": event_type,
                "foot": foot,
            }
        )
    parsed.sort(key=lambda row: float(row["video_time_sec"]))
    return parsed


def sensor_time_for_raw_seq(sensor: SensorSeries, raw_value: int) -> float:
    matches = np.flatnonzero(sensor.raw_seq == raw_value)
    if len(matches) != 1:
        raise ValueError(
            f"sync seq {raw_value} must occur exactly once in sensor CSV; found {len(matches)}"
        )
    return float(sensor.sensor_time_sec[int(matches[0])])


def fit_sync(sensor: SensorSeries, path: Path, trial: str) -> tuple[SyncFit, list[dict[str, object]]]:
    rows = [row for row in read_rows(path, SYNC_COLUMNS) if row["trial"] == trial]
    if len(rows) < 3:
        raise ValueError("at least three synchronization points are required")
    parsed: list[dict[str, object]] = []
    sensor_times: list[float] = []
    video_times: list[float] = []
    for row in rows:
        seq = int(row["seq"])
        sensor_time = sensor_time_for_raw_seq(sensor, seq)
        video_time = float(row["video_time_sec"])
        phase = row.get("phase", "").strip().lower()
        if phase and phase not in SYNC_PHASES:
            raise ValueError(f"unsupported sync phase {phase!r}; expected pre or post")
        sensor_times.append(sensor_time)
        video_times.append(video_time)
        parsed.append(
            {
                **row,
                "sync_index": int(row["sync_index"]),
                "video_frame": int(row["video_frame"]),
                "video_time_sec": video_time,
                "seq": seq,
                "sensor_time_sec": sensor_time,
                "phase": phase,
            }
        )
    if len(set(sensor_times)) < 2:
        raise ValueError("synchronization points need at least two distinct seq times")
    slope, intercept = np.polyfit(sensor_times, video_times, 1)
    predicted = slope * np.asarray(sensor_times) + intercept
    residuals = predicted - np.asarray(video_times)
    for row, predicted_time, residual in zip(parsed, predicted, residuals):
        row["predicted_video_time_sec"] = float(predicted_time)
        row["residual_sec"] = float(residual)
    fit = SyncFit(
        slope=float(slope),
        intercept=float(intercept),
        rmse_sec=float(np.sqrt(np.mean(residuals**2))),
        max_abs_residual_sec=float(np.max(np.abs(residuals))),
        point_count=len(rows),
    )
    return fit, parsed


def interval_coverage_ratio(
    span_start: float, span_end: float, interval_start: float, interval_end: float
) -> float:
    interval_length = interval_end - interval_start
    if interval_length <= 0.0:
        raise ValueError("evaluation interval must have positive duration")
    overlap = max(0.0, min(span_end, interval_end) - max(span_start, interval_start))
    return overlap / interval_length


def assess_sync_coverage(
    sync: SyncFit,
    sync_rows: Sequence[dict[str, object]],
    eval_start_video_sec: float,
    eval_end_video_sec: float,
) -> SyncCoverage:
    if sync.slope <= 0.0:
        raise ValueError(f"sync scale must be positive; got {sync.slope}")
    if sync.max_abs_residual_sec > MAX_SYNC_RESIDUAL_SEC + 1e-12:
        raise ValueError(
            "sync residual exceeds limit: "
            f"max={sync.max_abs_residual_sec:.6f}s limit={MAX_SYNC_RESIDUAL_SEC:.6f}s"
        )

    inferred_count = 0
    pre_rows: list[dict[str, object]] = []
    post_rows: list[dict[str, object]] = []
    for row in sync_rows:
        phase = str(row.get("phase", ""))
        if not phase:
            video_time = float(row["video_time_sec"])
            if video_time < eval_start_video_sec:
                phase = "pre"
            elif video_time > eval_end_video_sec:
                phase = "post"
            else:
                phase = "inside"
            inferred_count += 1
        row["effective_phase"] = phase
        if phase == "pre":
            pre_rows.append(row)
        elif phase == "post":
            post_rows.append(row)

    if not pre_rows or not post_rows:
        raise ValueError(
            "sync points must include both pre and post phases; "
            f"found pre={len(pre_rows)} post={len(post_rows)}"
        )
    latest_pre = max(float(row["video_time_sec"]) for row in pre_rows)
    earliest_post = min(float(row["video_time_sec"]) for row in post_rows)
    if latest_pre >= eval_start_video_sec:
        raise ValueError(
            "pre synchronization must be outside and before the evaluation interval: "
            f"latest_pre={latest_pre:.6f}s eval_start={eval_start_video_sec:.6f}s"
        )
    if earliest_post <= eval_end_video_sec:
        raise ValueError(
            "post synchronization must be outside and after the evaluation interval: "
            f"earliest_post={earliest_post:.6f}s eval_end={eval_end_video_sec:.6f}s"
        )

    video_times = [float(row["video_time_sec"]) for row in sync_rows]
    sensor_times = [float(row["sensor_time_sec"]) for row in sync_rows]
    sync_video_start, sync_video_end = min(video_times), max(video_times)
    sync_sensor_start, sync_sensor_end = min(sensor_times), max(sensor_times)
    eval_sensor_start = (eval_start_video_sec - sync.intercept) / sync.slope
    eval_sensor_end = (eval_end_video_sec - sync.intercept) / sync.slope
    video_coverage = interval_coverage_ratio(
        sync_video_start, sync_video_end, eval_start_video_sec, eval_end_video_sec
    )
    sensor_coverage = interval_coverage_ratio(
        sync_sensor_start, sync_sensor_end, eval_sensor_start, eval_sensor_end
    )
    if min(video_coverage, sensor_coverage) < MIN_SYNC_COVERAGE_RATIO - 1e-12:
        raise ValueError(
            "sync span is insufficient for the evaluation interval: "
            f"video_coverage={video_coverage:.3f} sensor_coverage={sensor_coverage:.3f} "
            f"required={MIN_SYNC_COVERAGE_RATIO:.3f}"
        )

    warnings: list[str] = []
    if inferred_count:
        warnings.append(
            f"phase inferred from evaluation boundaries for {inferred_count} legacy sync points"
        )
    if len(pre_rows) < 3 or len(post_rows) < 3:
        warnings.append(
            "non-standard synchronization count; tomorrow's protocol requires three pre and three post points"
        )
    return SyncCoverage(
        pre_point_count=len(pre_rows),
        post_point_count=len(post_rows),
        inferred_phase_point_count=inferred_count,
        standard_six_point_protocol=(
            len(pre_rows) >= 3 and len(post_rows) >= 3 and inferred_count == 0
        ),
        sync_video_start_sec=sync_video_start,
        sync_video_end_sec=sync_video_end,
        eval_video_start_sec=eval_start_video_sec,
        eval_video_end_sec=eval_end_video_sec,
        video_coverage_ratio=video_coverage,
        sync_sensor_start_sec=sync_sensor_start,
        sync_sensor_end_sec=sync_sensor_end,
        eval_sensor_start_sec=eval_sensor_start,
        eval_sensor_end_sec=eval_sensor_end,
        sensor_coverage_ratio=sensor_coverage,
        max_allowed_residual_sec=MAX_SYNC_RESIDUAL_SEC,
        warnings=tuple(warnings),
    )


def interpolate_zero_cross(time0: float, value0: float, time1: float, value1: float) -> float:
    if value0 == value1:
        return time1
    fraction = value0 / (value0 - value1)
    return time0 + fraction * (time1 - time0)


def missing_context(
    event_sensor_time: float, sensor: SensorSeries, window_sec: float
) -> tuple[bool, float | None, int]:
    if len(sensor.missing_seq) == 0:
        return False, None, 0
    missing_time = (sensor.missing_seq - sensor.grid_seq[0]) / FS_HZ
    distances = np.abs(missing_time - event_sensor_time)
    nearest = float(np.min(distances))
    nearby_seq = sensor.missing_seq[distances <= window_sec + 1e-12]
    if len(nearby_seq) == 0:
        return False, nearest, 0
    max_run = 1
    run = 1
    for previous, current in zip(nearby_seq[:-1], nearby_seq[1:]):
        if current == previous + 1:
            run += 1
            max_run = max(max_run, run)
        else:
            run = 1
    return True, nearest, max_run


def detect_fixed_candidates(
    sensor: SensorSeries, sync: SyncFit, missing_near_sec: float
) -> list[dict[str, object]]:
    b, a = butter(LPF_ORDER, LPF_CUTOFF_HZ / (FS_HZ / 2.0), btype="low")
    filtered = filtfilt(b, a, sensor.grid_gx_dps)
    peaks, properties = find_peaks(
        filtered,
        height=PEAK_HEIGHT_DPS,
        prominence=PEAK_PROMINENCE_DPS,
        distance=math.ceil(MIN_DISTANCE_SEC * FS_HZ),
    )
    candidates: list[dict[str, object]] = []
    for candidate_index, (peak_index, peak_height, prominence) in enumerate(
        zip(peaks, properties["peak_heights"], properties["prominences"]), start=1
    ):
        cross_limit = min(len(filtered) - 1, peak_index + round(ZERO_CROSS_WINDOW_SEC * FS_HZ))
        cross_pair: int | None = None
        for index in range(int(peak_index), cross_limit):
            if filtered[index] > 0.0 and filtered[index + 1] <= 0.0:
                cross_pair = index
                break

        cross_time: float | None = None
        confirm_min: float | None = None
        confirm_pass = False
        if cross_pair is not None:
            cross_time = interpolate_zero_cross(
                float(sensor.grid_time_sec[cross_pair]),
                float(filtered[cross_pair]),
                float(sensor.grid_time_sec[cross_pair + 1]),
                float(filtered[cross_pair + 1]),
            )
            confirm_end = min(
                len(filtered) - 1, cross_pair + 1 + round(CONFIRM_WINDOW_SEC * FS_HZ)
            )
            confirm_values = filtered[cross_pair + 1 : confirm_end + 1]
            if len(confirm_values):
                confirm_min = float(np.min(confirm_values))
                confirm_pass = confirm_min <= CONFIRM_GX_DPS

        formal = cross_time is not None and confirm_pass
        event_sensor_time = cross_time if cross_time is not None else float(sensor.grid_time_sec[peak_index])
        missing_near, nearest_missing, max_missing_run = missing_context(
            event_sensor_time, sensor, missing_near_sec
        )
        candidates.append(
            {
                "candidate_index": candidate_index,
                "peak_seq": int(sensor.grid_seq[peak_index] % SEQ_MODULUS),
                "peak_sensor_time_sec": float(sensor.grid_time_sec[peak_index]),
                "peak_video_time_sec": sync.to_video_time(float(sensor.grid_time_sec[peak_index])),
                "peak_gx_dps": float(peak_height),
                "prominence_dps": float(prominence),
                "zero_cross_sensor_time_sec": cross_time,
                "zero_cross_video_time_sec": None if cross_time is None else sync.to_video_time(cross_time),
                "confirmation_min_gx_dps": confirm_min,
                "confirmation_pass": confirm_pass,
                "formal": formal,
                "failure_reason": "" if formal else ("no_zero_cross" if cross_time is None else "no_negative_confirmation"),
                "missing_near": missing_near,
                "nearest_missing_sec": nearest_missing,
                "max_missing_run_near": max_missing_run,
            }
        )
    return candidates


def ordered_match(
    detection_times: Sequence[float], gt_times: Sequence[float], tolerance_sec: float
) -> tuple[list[tuple[int, int]], list[int], list[int]]:
    """Maximum-cardinality, minimum-total-error, order-preserving matching."""

    n, m = len(detection_times), len(gt_times)
    score: list[list[tuple[int, float]]] = [[(0, 0.0) for _ in range(m + 1)] for _ in range(n + 1)]
    choice: list[list[str]] = [["" for _ in range(m + 1)] for _ in range(n + 1)]

    def better(left: tuple[int, float], right: tuple[int, float]) -> bool:
        return left[0] > right[0] or (left[0] == right[0] and left[1] < right[1] - 1e-15)

    for i in range(1, n + 1):
        choice[i][0] = "d"
    for j in range(1, m + 1):
        choice[0][j] = "g"
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            best = score[i - 1][j]
            action = "d"
            if better(score[i][j - 1], best):
                best = score[i][j - 1]
                action = "g"
            error = abs(float(detection_times[i - 1]) - float(gt_times[j - 1]))
            if error <= tolerance_sec + 1e-12:
                previous = score[i - 1][j - 1]
                matched = (previous[0] + 1, previous[1] + error)
                if better(matched, best):
                    best = matched
                    action = "m"
            score[i][j] = best
            choice[i][j] = action

    matches: list[tuple[int, int]] = []
    i, j = n, m
    while i or j:
        action = choice[i][j]
        if action == "m":
            matches.append((i - 1, j - 1))
            i -= 1
            j -= 1
        elif action == "d":
            i -= 1
        else:
            j -= 1
    matches.reverse()
    matched_d = {pair[0] for pair in matches}
    matched_g = {pair[1] for pair in matches}
    return matches, [i for i in range(n) if i not in matched_d], [j for j in range(m) if j not in matched_g]


def safe_ratio(numerator: int, denominator: int) -> float | None:
    return None if denominator == 0 else numerator / denominator


def metric_row(
    scope: str,
    detections: Sequence[dict[str, object]],
    gt: Sequence[dict[str, object]],
    matches: Sequence[tuple[int, int]],
) -> dict[str, object]:
    tp = len(matches)
    fp = len(detections) - tp
    fn = len(gt) - tp
    errors = [
        float(detections[di]["event_video_time_sec"]) - float(gt[gi]["video_time_sec"])
        for di, gi in matches
    ]
    return {
        "scope": scope,
        "gt_count": len(gt),
        "detected_count": len(detections),
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": safe_ratio(tp, tp + fp),
        "recall": safe_ratio(tp, tp + fn),
        "f1": safe_ratio(2 * tp, 2 * tp + fp + fn),
        "mean_signed_error_sec": None if not errors else float(np.mean(errors)),
        "mae_sec": None if not errors else float(np.mean(np.abs(errors))),
        "count_error": len(detections) - len(gt),
    }


def write_csv(path: Path, rows: Sequence[dict[str, object]], fieldnames: Sequence[str]) -> None:
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def json_ready(value: object) -> object:
    if isinstance(value, dict):
        return {key: json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_ready(item) for item in value]
    if isinstance(value, (np.integer, np.floating)):
        return value.item()
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def run(args: argparse.Namespace) -> dict[str, object]:
    if not (
        args.eval_start_video_sec < args.terminal_start_video_sec < args.eval_end_video_sec
    ):
        raise ValueError("require eval-start < terminal-start < eval-end")
    if args.tolerance_sec <= 0 or args.missing_near_sec < 0:
        raise ValueError("tolerance must be positive and missing-near must be non-negative")
    if args.out_dir.exists():
        raise ValueError(f"output directory already exists: {args.out_dir}")

    sensor = load_sensor_csv(args.sensor_csv)
    ground_truth = load_ground_truth(args.ground_truth_csv, args.trial)
    sync, sync_rows = fit_sync(sensor, args.sync_points_csv, args.trial)
    sync_coverage = assess_sync_coverage(
        sync,
        sync_rows,
        args.eval_start_video_sec,
        args.eval_end_video_sec,
    )
    arms = detect_fixed_candidates(sensor, sync, args.missing_near_sec)

    formal: list[dict[str, object]] = []
    for arm in arms:
        if not arm["formal"]:
            continue
        event_time = float(arm["zero_cross_video_time_sec"])
        if not (args.eval_start_video_sec <= event_time <= args.eval_end_video_sec):
            continue
        phase = "normal_contact" if event_time < args.terminal_start_video_sec else "terminal_contact"
        formal.append({**arm, "event_video_time_sec": event_time, "evaluation_scope": phase})

    positive_gt = [
        row
        for row in ground_truth
        if row["foot"] == "right"
        and row["event_type"] in CONTACT_TYPES
        and args.eval_start_video_sec <= float(row["video_time_sec"]) <= args.eval_end_video_sec
    ]
    context_gt = [
        row
        for row in ground_truth
        if row["event_type"] in CONTEXT_TYPES
        and args.eval_start_video_sec <= float(row["video_time_sec"]) <= args.eval_end_video_sec
    ]

    match_rows: list[dict[str, object]] = []
    metrics: list[dict[str, object]] = []
    all_matches: list[tuple[dict[str, object], dict[str, object]]] = []
    all_unmatched_d: list[dict[str, object]] = []
    all_unmatched_gt: list[dict[str, object]] = []
    for scope in CONTACT_TYPES:
        detections = [row for row in formal if row["evaluation_scope"] == scope]
        gt_rows = [row for row in positive_gt if row["event_type"] == scope]
        matches, unmatched_d, unmatched_gt = ordered_match(
            [float(row["event_video_time_sec"]) for row in detections],
            [float(row["video_time_sec"]) for row in gt_rows],
            args.tolerance_sec,
        )
        metrics.append(metric_row(scope, detections, gt_rows, matches))
        for detection_index, gt_index in matches:
            detection, gt_row = detections[detection_index], gt_rows[gt_index]
            all_matches.append((detection, gt_row))
            match_rows.append(
                {
                    "scope": scope,
                    "status": "TP",
                    "candidate_index": detection["candidate_index"],
                    "event_video_time_sec": detection["event_video_time_sec"],
                    "gt_event_index": gt_row["event_index"],
                    "gt_video_time_sec": gt_row["video_time_sec"],
                    "signed_error_sec": float(detection["event_video_time_sec"])
                    - float(gt_row["video_time_sec"]),
                    "gt_confidence": gt_row["confidence"],
                    "missing_near": detection["missing_near"],
                    "fp_context": "",
                }
            )
        for index in unmatched_d:
            detection = detections[index]
            all_unmatched_d.append(detection)
        for index in unmatched_gt:
            gt_row = gt_rows[index]
            all_unmatched_gt.append(gt_row)
            match_rows.append(
                {
                    "scope": scope,
                    "status": "FN",
                    "candidate_index": "",
                    "event_video_time_sec": "",
                    "gt_event_index": gt_row["event_index"],
                    "gt_video_time_sec": gt_row["video_time_sec"],
                    "signed_error_sec": "",
                    "gt_confidence": gt_row["confidence"],
                    "missing_near": "",
                    "fp_context": "",
                }
            )

    fp_context_counts = {"adjustment": 0, "turn": 0, "other": 0}
    for detection in all_unmatched_d:
        nearby = [
            row
            for row in context_gt
            if abs(float(detection["event_video_time_sec"]) - float(row["video_time_sec"]))
            <= args.tolerance_sec + 1e-12
        ]
        if nearby:
            nearest = min(
                nearby,
                key=lambda row: abs(
                    float(detection["event_video_time_sec"]) - float(row["video_time_sec"])
                ),
            )
            fp_context = str(nearest["event_type"])
        else:
            fp_context = "other"
        fp_context_counts[fp_context] += 1
        match_rows.append(
            {
                "scope": detection["evaluation_scope"],
                "status": "FP",
                "candidate_index": detection["candidate_index"],
                "event_video_time_sec": detection["event_video_time_sec"],
                "gt_event_index": "",
                "gt_video_time_sec": "",
                "signed_error_sec": "",
                "gt_confidence": "",
                "missing_near": detection["missing_near"],
                "fp_context": fp_context,
            }
        )

    overall_match_indices = [(index, index) for index in range(len(all_matches))]
    overall_detections = [pair[0] for pair in all_matches] + all_unmatched_d
    overall_gt = [pair[1] for pair in all_matches] + all_unmatched_gt
    overall_metrics = metric_row("all_right_contacts", overall_detections, overall_gt, overall_match_indices)
    metrics.insert(0, overall_metrics)

    args.out_dir.mkdir(parents=True, exist_ok=False)
    arm_fields = (
        "candidate_index",
        "peak_seq",
        "peak_sensor_time_sec",
        "peak_video_time_sec",
        "peak_gx_dps",
        "prominence_dps",
        "zero_cross_sensor_time_sec",
        "zero_cross_video_time_sec",
        "confirmation_min_gx_dps",
        "confirmation_pass",
        "formal",
        "failure_reason",
        "missing_near",
        "nearest_missing_sec",
        "max_missing_run_near",
    )
    write_csv(args.out_dir / "detector_arms.csv", arms, arm_fields)
    write_csv(
        args.out_dir / "matches.csv",
        match_rows,
        (
            "scope",
            "status",
            "candidate_index",
            "event_video_time_sec",
            "gt_event_index",
            "gt_video_time_sec",
            "signed_error_sec",
            "gt_confidence",
            "missing_near",
            "fp_context",
        ),
    )
    write_csv(args.out_dir / "metrics.csv", metrics, tuple(metrics[0].keys()))
    write_csv(
        args.out_dir / "sync_fit.csv",
        sync_rows,
        (
            "trial",
            "sync_index",
            "video_frame",
            "video_time_sec",
            "seq",
            "sensor_time_sec",
            "predicted_video_time_sec",
            "residual_sec",
            "notes",
            "phase",
            "effective_phase",
        ),
    )

    summary: dict[str, object] = {
        "trial": args.trial,
        "mode": "offline zero-phase evaluation; not a causal production detector",
        "fixed_detector": {
            "axis": "gx",
            "fs_hz": FS_HZ,
            "grid": "seq 100 Hz with linear interpolation for missing samples",
            "filter": "4th-order Butterworth LPF 5 Hz zero-phase",
            "peak_height_dps": PEAK_HEIGHT_DPS,
            "prominence_dps": PEAK_PROMINENCE_DPS,
            "minimum_distance_sec": MIN_DISTANCE_SEC,
            "zero_cross_window_sec": ZERO_CROSS_WINDOW_SEC,
            "confirmation_after_zero_cross_sec": CONFIRM_WINDOW_SEC,
            "confirmation_gx_lte_dps": CONFIRM_GX_DPS,
            "automatic_tuning": False,
        },
        "evaluation_window_video_sec": {
            "start": args.eval_start_video_sec,
            "terminal_start": args.terminal_start_video_sec,
            "end": args.eval_end_video_sec,
        },
        "tolerance_sec": args.tolerance_sec,
        "missing_near_sec": args.missing_near_sec,
        "sensor_quality": {
            "received_samples": len(sensor.raw_seq),
            "grid_samples": len(sensor.grid_seq),
            "missing_samples": len(sensor.missing_seq),
        },
        "sync_fit": sync.__dict__,
        "sync_coverage": sync_coverage.__dict__,
        "metrics": metrics,
        "false_positive_context": fp_context_counts,
        "ground_truth_all_feet_contact_count": sum(
            row["event_type"] in CONTACT_TYPES for row in ground_truth
        ),
    }
    (args.out_dir / "summary.json").write_text(
        json.dumps(json_ready(summary), ensure_ascii=False, indent=2) + "\n"
    )
    return summary


def main() -> int:
    try:
        summary = run(parse_args())
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    overall = summary["metrics"][0]
    print(
        f"{summary['trial']}: TP={overall['tp']} FP={overall['fp']} "
        f"FN={overall['fn']} F1={overall['f1']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
