#!/usr/bin/env python3
"""Compute offline functional gait metrics from ILGA sensor and operator markers.

For walking, the unchanged fixed gx detector is reused as an attachment-foot
``stride_marker``. It is not interpreted as anatomical initial contact.
START/FINISH duration uses the PC receive monotonic clock, while stride periods
use the 100 Hz seq grid. The zero-phase detector makes walking analysis
offline-only. TUG mode reports operator-marker total time only.
"""

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

import analyze_single_leg_csv as basic_analyzer
import evaluate_gait_events as fixed_detector


TOOL_SCHEMA_VERSION = 1
MARKER_COLUMNS = (
    "schema_version",
    "trial_name",
    "event",
    "event_index",
    "marker_monotonic_ns",
    "marker_elapsed_ns",
    "source",
    "notes",
)
SENSOR_COLUMNS = (
    "seq",
    "rx_monotonic_ns",
    "ax_mg",
    "ay_mg",
    "az_mg",
    "gx_mdps",
    "gy_mdps",
    "gz_mdps",
)
ALLOWED_MARKER_EVENTS = ("START", "FINISH")
MARKER_SOURCE = "operator_key"
MISSING_NEAR_SEC = 0.10
INLIER_MIN_RATIO = 0.70
INLIER_MAX_RATIO = 1.50
PAUSE_EXTRA_SEC = 0.50


@dataclass(frozen=True)
class SensorInput:
    rows: tuple[dict[str, object], ...]
    rx_monotonic_ns: np.ndarray
    zero_accel_count: int
    zero_accel_ratio: float
    rx_duplicate_count: int
    rx_reverse_count: int
    rx_elapsed_mismatch_count: int
    rail_counts: dict[str, int]
    near_rail_counts: dict[str, int]


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compute offline walking or TUG functional metrics using operator "
            "START/FINISH and the unchanged fixed gx stride marker for walking."
        )
    )
    parser.add_argument("sensor_csv", type=Path)
    parser.add_argument("markers_csv", type=Path)
    parser.add_argument("--test-type", choices=("walk10m", "walk", "tug"))
    parser.add_argument("--distance-m", type=float)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--trial-name")
    parser.add_argument("--metadata-json", type=Path)
    actual_steps_group = parser.add_mutually_exclusive_group()
    actual_steps_group.add_argument("--actual-steps", type=int)
    actual_steps_group.add_argument(
        "--actual-steps-unknown",
        action="store_true",
        help="Do not use actual_steps from metadata; preserve it as unknown.",
    )
    parser.add_argument("--start-foot", choices=("left", "right", "unknown"))
    parser.add_argument("--end-foot", choices=("left", "right", "unknown"))
    parser.add_argument("--sensor-foot", choices=("left", "right", "unknown"))
    parser.add_argument("--accel-range-g", type=float, default=4.0)
    parser.add_argument("--gyro-range-dps", type=float, default=1000.0)
    return parser.parse_args(argv)


def _required_columns(fieldnames: Sequence[str] | None, required: Sequence[str], label: str) -> None:
    if fieldnames is None:
        raise ValueError(f"{label} has no CSV header")
    missing = [name for name in required if name not in fieldnames]
    if missing:
        raise ValueError(f"{label} is missing columns: {', '.join(missing)}")


def _finite_float(value: str, label: str, line_number: int) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid {label} at sensor line {line_number}") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"non-finite {label} at sensor line {line_number}")
    return parsed


def load_sensor_input(path: Path, accel_range_g: float, gyro_range_dps: float) -> SensorInput:
    if accel_range_g <= 0.0 or gyro_range_dps <= 0.0:
        raise ValueError("sensor ranges must be positive")

    rows: list[dict[str, object]] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        _required_columns(reader.fieldnames, SENSOR_COLUMNS, "sensor CSV")
        has_rx_elapsed = "rx_elapsed_ns" in (reader.fieldnames or ())
        for line_number, row in enumerate(reader, start=2):
            try:
                parsed: dict[str, object] = {
                    "seq": int(row["seq"]),
                    "rx_monotonic_ns": int(row["rx_monotonic_ns"]),
                }
            except (TypeError, ValueError) as exc:
                raise ValueError(f"invalid seq/rx timestamp at sensor line {line_number}") from exc
            for name in SENSOR_COLUMNS[2:]:
                parsed[name] = _finite_float(row[name], name, line_number)
            if has_rx_elapsed:
                try:
                    parsed["rx_elapsed_ns"] = int(row["rx_elapsed_ns"])
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"invalid rx_elapsed_ns at sensor line {line_number}") from exc
            rows.append(parsed)

    if not rows:
        raise ValueError("sensor CSV contains no samples")

    # Reuse the existing analyzer's zero-acceleration rejection rule unchanged.
    basic_data = {
        name: [float(row[name]) for row in rows]
        for name in basic_analyzer.REQUIRED_COLUMNS
    }
    basic_analyzer.validate_sensor_samples(basic_data)

    rx = np.asarray([int(row["rx_monotonic_ns"]) for row in rows], dtype=np.int64)
    rx_deltas = np.diff(rx)
    rx_duplicate_count = int(np.sum(rx_deltas == 0))
    rx_reverse_count = int(np.sum(rx_deltas < 0))
    first_rx = int(rx[0])
    rx_elapsed_mismatch_count = sum(
        1
        for row in rows
        if "rx_elapsed_ns" in row
        and int(row["rx_elapsed_ns"]) != int(row["rx_monotonic_ns"]) - first_rx
    )
    zero_accel_count = sum(
        1
        for row in rows
        if all(float(row[name]) == 0.0 for name in ("ax_mg", "ay_mg", "az_mg"))
    )

    limits = {
        "ax_mg": accel_range_g * 1000.0,
        "ay_mg": accel_range_g * 1000.0,
        "az_mg": accel_range_g * 1000.0,
        "gx_mdps": gyro_range_dps * 1000.0,
        "gy_mdps": gyro_range_dps * 1000.0,
        "gz_mdps": gyro_range_dps * 1000.0,
    }
    rail_counts = {
        name: sum(abs(float(row[name])) >= limit for row in rows)
        for name, limit in limits.items()
    }
    near_rail_counts = {
        name: sum(abs(float(row[name])) >= 0.95 * limit for row in rows)
        for name, limit in limits.items()
    }
    return SensorInput(
        rows=tuple(rows),
        rx_monotonic_ns=rx,
        zero_accel_count=zero_accel_count,
        zero_accel_ratio=zero_accel_count / len(rows),
        rx_duplicate_count=rx_duplicate_count,
        rx_reverse_count=rx_reverse_count,
        rx_elapsed_mismatch_count=rx_elapsed_mismatch_count,
        rail_counts=rail_counts,
        near_rail_counts=near_rail_counts,
    )


def load_metadata(path: Path | None) -> dict[str, object]:
    if path is None:
        return {}
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("metadata JSON must contain an object")
    return value


def normalize_foot(value: object | None) -> str | None:
    if value is None or str(value).strip() == "":
        return None
    foot = str(value).strip().lower()
    if foot == "none":
        return "unknown"
    if foot not in {"left", "right", "unknown"}:
        raise ValueError(f"unsupported foot {value!r}")
    return foot


def load_markers(path: Path, requested_trial: str | None) -> tuple[str, list[dict[str, object]]]:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        _required_columns(reader.fieldnames, MARKER_COLUMNS, "markers CSV")
        raw_rows = list(reader)
    if not raw_rows:
        raise ValueError("markers CSV contains no rows")

    trial_names = sorted({row["trial_name"].strip() for row in raw_rows if row["trial_name"].strip()})
    if requested_trial is None:
        if len(trial_names) != 1:
            raise ValueError("--trial-name is required when markers CSV contains multiple trials")
        trial_name = trial_names[0]
    else:
        trial_name = requested_trial

    selected: list[dict[str, object]] = []
    for line_number, row in enumerate(raw_rows, start=2):
        if row["trial_name"].strip() != trial_name:
            continue
        if row["schema_version"].strip() != "1":
            raise ValueError(f"unsupported marker schema_version at line {line_number}")
        event = row["event"].strip()
        if event not in ALLOWED_MARKER_EVENTS:
            raise ValueError(f"unsupported marker event {event!r} at line {line_number}")
        if row["source"].strip() != MARKER_SOURCE:
            raise ValueError(f"unsupported marker source at line {line_number}; expected {MARKER_SOURCE}")
        try:
            event_index = int(row["event_index"])
            marker_monotonic_ns = int(row["marker_monotonic_ns"])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid marker integer at line {line_number}") from exc
        elapsed_text = row["marker_elapsed_ns"].strip()
        try:
            marker_elapsed_ns = None if elapsed_text == "" else int(elapsed_text)
        except ValueError as exc:
            raise ValueError(f"invalid marker_elapsed_ns at line {line_number}") from exc
        if event_index < 1 or marker_monotonic_ns < 0:
            raise ValueError(f"negative or zero-invalid marker value at line {line_number}")
        selected.append(
            {
                "schema_version": 1,
                "trial_name": trial_name,
                "event": event,
                "event_index": event_index,
                "marker_monotonic_ns": marker_monotonic_ns,
                "marker_elapsed_ns": marker_elapsed_ns,
                "source": MARKER_SOURCE,
                "notes": row["notes"],
            }
        )
    return trial_name, selected


def metadata_value(metadata: dict[str, object], name: str) -> object | None:
    return metadata.get(name)


def capture_metric(metadata: dict[str, object], name: str) -> int:
    metrics = metadata.get("metrics", {})
    if not isinstance(metrics, dict):
        return 0
    try:
        return int(metrics.get(name, 0) or 0)
    except (TypeError, ValueError):
        return 0


def derive_reference_step_estimate(
    marker_count: int,
    sensor_foot: str | None,
    start_foot: str | None,
    end_foot: str | None,
) -> tuple[int | None, str | None]:
    if (
        sensor_foot not in {"left", "right"}
        or start_foot not in {"left", "right"}
        or end_foot not in {"left", "right"}
    ):
        return None, None
    starts_on_sensor_foot = start_foot == sensor_foot
    ends_on_sensor_foot = end_foot == sensor_foot
    symbol = "L" if sensor_foot == "left" else "R"
    if starts_on_sensor_foot and ends_on_sensor_foot:
        return 2 * marker_count - 1, f"sensor={sensor_foot}, {start_foot}->{end_foot}: 2{symbol}-1"
    if not starts_on_sensor_foot and not ends_on_sensor_foot:
        return 2 * marker_count + 1, f"sensor={sensor_foot}, {start_foot}->{end_foot}: 2{symbol}+1"
    return 2 * marker_count, f"sensor={sensor_foot}, {start_foot}->{end_foot}: 2{symbol}"


def expected_sensor_foot_contacts(
    actual_steps: int | None,
    sensor_foot: str | None,
    start_foot: str | None,
    end_foot: str | None,
) -> int | None:
    """Return a count reference from manually observed steps and foot ordering."""

    if (
        actual_steps is None
        or sensor_foot not in {"left", "right"}
        or start_foot not in {"left", "right"}
        or end_foot not in {"left", "right"}
    ):
        return None
    sensor_count = (actual_steps + (1 if start_foot == sensor_foot else 0)) // 2
    expected_end = sensor_foot if sensor_count * 2 - (1 if start_foot == sensor_foot else 0) == actual_steps else (
        "left" if sensor_foot == "right" else "right"
    )
    return sensor_count if expected_end == end_foot else None


def summarize_range_by_interval(
    sensor: SensorInput,
    start: dict[str, object] | None,
    finish: dict[str, object] | None,
    interval_valid: bool,
    accel_range_g: float,
    gyro_range_dps: float,
) -> dict[str, object]:
    limits = {
        "ax_mg": accel_range_g * 1000.0,
        "ay_mg": accel_range_g * 1000.0,
        "az_mg": accel_range_g * 1000.0,
        "gx_mdps": gyro_range_dps * 1000.0,
        "gy_mdps": gyro_range_dps * 1000.0,
        "gz_mdps": gyro_range_dps * 1000.0,
    }

    def summarize(rows: Sequence[dict[str, object]]) -> dict[str, object]:
        count = len(rows)
        rails = {
            name: sum(abs(float(row[name])) >= limit for row in rows)
            for name, limit in limits.items()
        }
        near = {
            name: sum(abs(float(row[name])) >= 0.95 * limit for row in rows)
            for name, limit in limits.items()
        }
        return {
            "sample_count": count,
            "rail_counts": rails,
            "rail_ratios": {name: (value / count if count else None) for name, value in rails.items()},
            "near_rail_95pct_counts": near,
            "near_rail_95pct_ratios": {
                name: (value / count if count else None) for name, value in near.items()
            },
        }

    full = summarize(sensor.rows)
    if not interval_valid or start is None or finish is None:
        return {"full": full, "marked_interval": None, "outside_marked_interval": None}
    start_ns = int(start["marker_monotonic_ns"])
    finish_ns = int(finish["marker_monotonic_ns"])
    inside = [row for row in sensor.rows if start_ns <= int(row["rx_monotonic_ns"]) <= finish_ns]
    outside = [row for row in sensor.rows if not (start_ns <= int(row["rx_monotonic_ns"]) <= finish_ns)]
    return {
        "full": full,
        "marked_interval": summarize(inside),
        "outside_marked_interval": summarize(outside),
    }


def summarize_intervals(event_times: Sequence[float]) -> tuple[dict[str, object], list[dict[str, object]]]:
    marker_count = len(event_times)
    if marker_count < 2:
        return (
            {
                "stride_marker_count": marker_count,
                "interval_count": 0,
                "median_stride_time_sec": None,
                "mean_stride_time_sec": None,
                "sd_stride_time_sec": None,
                "robust_cadence_steps_per_min": None,
                "raw_cv_pct": None,
                "inlier_cv_pct": None,
                "inlier_interval_count": 0,
                "interval_outlier_count": 0,
                "pause_or_missed_marker_count": 0,
            },
            [],
        )

    intervals = [float(right - left) for left, right in zip(event_times[:-1], event_times[1:])]
    median_value = float(statistics.median(intervals))
    mean_value = float(statistics.mean(intervals))
    interval_rows: list[dict[str, object]] = []
    inliers: list[float] = []
    pause_count = 0
    for index, (left, right, value) in enumerate(
        zip(event_times[:-1], event_times[1:], intervals), start=1
    ):
        ratio = value / median_value if median_value > 0.0 else math.inf
        inlier = INLIER_MIN_RATIO <= ratio <= INLIER_MAX_RATIO
        pause = ratio > INLIER_MAX_RATIO and value - median_value >= PAUSE_EXTRA_SEC
        if inlier:
            inliers.append(value)
        if pause:
            pause_count += 1
        interval_rows.append(
            {
                "interval_index": index,
                "start_stride_marker_sec": float(left),
                "end_stride_marker_sec": float(right),
                "stride_time_sec": value,
                "ratio_to_trial_median": ratio,
                "inlier": inlier,
                "interval_outlier": not inlier,
                "pause_or_missed_marker": pause,
            }
        )

    sd_value = float(statistics.stdev(intervals)) if marker_count >= 3 else None
    raw_cv = None if sd_value is None or mean_value == 0.0 else 100.0 * sd_value / mean_value
    inlier_cv = None
    if len(inliers) >= 2:
        inlier_mean = statistics.mean(inliers)
        if inlier_mean != 0.0:
            inlier_cv = 100.0 * statistics.stdev(inliers) / inlier_mean
    return (
        {
            "stride_marker_count": marker_count,
            "interval_count": len(intervals),
            "median_stride_time_sec": median_value,
            "mean_stride_time_sec": mean_value,
            "sd_stride_time_sec": sd_value,
            "robust_cadence_steps_per_min": None if median_value <= 0.0 else 120.0 / median_value,
            "raw_cv_pct": raw_cv,
            "inlier_cv_pct": inlier_cv,
            "inlier_interval_count": len(inliers),
            "interval_outlier_count": sum(not bool(row["inlier"]) for row in interval_rows),
            "pause_or_missed_marker_count": pause_count,
        },
        interval_rows,
    )


def _marker_validation(
    marker_rows: Sequence[dict[str, object]], sensor: SensorInput
) -> tuple[dict[str, object] | None, dict[str, object] | None, list[str]]:
    errors: list[str] = []
    starts = [row for row in marker_rows if row["event"] == "START"]
    finishes = [row for row in marker_rows if row["event"] == "FINISH"]
    if not starts:
        errors.append("marker_missing_start")
    elif len(starts) > 1:
        errors.append("marker_duplicate_start")
    if not finishes:
        errors.append("marker_missing_finish")
    elif len(finishes) > 1:
        errors.append("marker_duplicate_finish")
    start = starts[0] if len(starts) == 1 else None
    finish = finishes[0] if len(finishes) == 1 else None
    first_rx = int(sensor.rx_monotonic_ns[0])
    for row in marker_rows:
        note_tokens = {
            token.strip().lower()
            for token in str(row.get("notes", "")).split(";")
            if token.strip()
        }
        note_error_map = {
            "missing_start": "marker_missing_start",
            "missing_finish": "marker_missing_finish",
            "finish_not_after_start": "marker_order_invalid",
            "missing_rx_reference": "marker_elapsed_missing",
            "duplicate_start_rejected": "marker_duplicate_start_rejected",
            "duplicate_finish_rejected": "marker_duplicate_finish_rejected",
        }
        errors.extend(note_error_map[token] for token in note_tokens if token in note_error_map)
        elapsed = row["marker_elapsed_ns"]
        if elapsed is None:
            errors.append("marker_elapsed_missing")
            continue
        if int(elapsed) < 0:
            errors.append("marker_elapsed_negative")
        if int(elapsed) != int(row["marker_monotonic_ns"]) - first_rx:
            errors.append("marker_elapsed_clock_mismatch")
    if start is not None and finish is not None:
        if int(finish["marker_monotonic_ns"]) <= int(start["marker_monotonic_ns"]):
            errors.append("marker_order_invalid")
        sensor_start = int(sensor.rx_monotonic_ns[0])
        sensor_end = int(sensor.rx_monotonic_ns[-1])
        if not (
            sensor_start <= int(start["marker_monotonic_ns"]) <= sensor_end
            and sensor_start <= int(finish["marker_monotonic_ns"]) <= sensor_end
        ):
            errors.append("marker_outside_sensor_rx_span")
    return start, finish, sorted(set(errors))


def _event_rx_ns(event_sensor_time: float, sensor_series: fixed_detector.SensorSeries, sensor: SensorInput) -> int:
    event_unwrapped_seq = sensor_series.grid_seq[0] + event_sensor_time * fixed_detector.FS_HZ
    return int(round(float(np.interp(event_unwrapped_seq, sensor_series.unwrapped_seq, sensor.rx_monotonic_ns))))


def _write_csv(path: Path, rows: Sequence[dict[str, object]], fieldnames: Sequence[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _json_text(value: object) -> str:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def build_boundary_diagnostics(
    candidate_rows: Sequence[dict[str, object]],
    start: dict[str, object] | None,
    finish: dict[str, object] | None,
) -> list[dict[str, object]]:
    """Return nearest armed candidate and formal marker on both sides of each boundary."""

    diagnostics: list[dict[str, object]] = []
    for boundary_name, boundary in (("START", start), ("FINISH", finish)):
        if boundary is None:
            continue
        boundary_ns = int(boundary["marker_monotonic_ns"])
        for event_kind, rows in (
            ("stride_candidate", list(candidate_rows)),
            (
                "formal_stride_marker",
                [row for row in candidate_rows if bool(row["candidate_formal"])],
            ),
        ):
            for side in ("before", "after"):
                eligible = [
                    row
                    for row in rows
                    if (
                        int(row["event_rx_monotonic_ns_estimate"]) - boundary_ns < 0
                        if side == "before"
                        else int(row["event_rx_monotonic_ns_estimate"]) - boundary_ns >= 0
                    )
                ]
                nearest = min(
                    eligible,
                    key=lambda row: abs(
                        int(row["event_rx_monotonic_ns_estimate"]) - boundary_ns
                    ),
                    default=None,
                )
                diagnostics.append(
                    {
                        "boundary": boundary_name,
                        "side": side,
                        "event_kind": event_kind,
                        "boundary_marker_monotonic_ns": boundary_ns,
                        "found": nearest is not None,
                        "candidate_index": None if nearest is None else nearest["candidate_index"],
                        "candidate_formal": None if nearest is None else nearest["candidate_formal"],
                        "event_time_source": None if nearest is None else nearest["event_time_source"],
                        "event_seq_time_sec": None if nearest is None else nearest["event_seq_time_sec"],
                        "event_rx_monotonic_ns_estimate": (
                            None if nearest is None else nearest["event_rx_monotonic_ns_estimate"]
                        ),
                        "signed_time_diff_sec": (
                            None
                            if nearest is None
                            else (
                                int(nearest["event_rx_monotonic_ns_estimate"]) - boundary_ns
                            )
                            / 1_000_000_000.0
                        ),
                        "within_marked_interval": (
                            None if nearest is None else nearest["within_marked_interval"]
                        ),
                        "adopted_as_stride_marker": (
                            None if nearest is None else nearest["adopted_as_stride_marker"]
                        ),
                        "exclusion_reason": None if nearest is None else nearest["exclusion_reason"],
                    }
                )
    return diagnostics


def format_boundary_diagnostic(row: dict[str, object]) -> str:
    label = f"boundary_{str(row['boundary']).lower()}_{row['side']}_{row['event_kind']}"
    if not row["found"]:
        return f"{label}: unavailable"
    return (
        f"{label}: candidate_index={row['candidate_index']} "
        f"signed_time_diff_sec={row['signed_time_diff_sec']} "
        f"formal={row['candidate_formal']} "
        f"within={row['within_marked_interval']} "
        f"adopted={row['adopted_as_stride_marker']} "
        f"exclusion_reason={row['exclusion_reason']}"
    )


def build_text_summary(summary: dict[str, object]) -> str:
    timing = summary["timing"]
    stride = summary["stride_metrics"]
    quality = summary["quality"]
    reference = summary["reference_step_evaluation"]
    range_by_interval = summary["sensor_quality"]["range_by_interval"]
    marked_range = range_by_interval["marked_interval"]
    lines = [
        f"trial: {summary['trial_name']}",
        f"test_type: {summary['test_type']}",
        f"mode: {summary['mode']}",
        f"quality_status: {quality['status']}",
        f"quality_errors: {', '.join(quality['errors']) if quality['errors'] else 'none'}",
        f"quality_warnings: {', '.join(quality['warnings']) if quality['warnings'] else 'none'}",
        f"marked_duration_sec: {timing['marked_duration_sec']}",
        f"distance_m: {timing['distance_m']}",
        f"speed_mps: {timing['speed_mps']}",
        f"stride_marker_count: {stride['stride_marker_count']}",
        f"median_stride_time_sec: {stride['median_stride_time_sec']}",
        f"mean_stride_time_sec: {stride['mean_stride_time_sec']}",
        f"sd_stride_time_sec: {stride['sd_stride_time_sec']}",
        f"robust_cadence_steps_per_min: {stride['robust_cadence_steps_per_min']}",
        f"raw_cv_pct: {stride['raw_cv_pct']}",
        f"inlier_cv_pct: {stride['inlier_cv_pct']}",
        f"actual_steps: {reference['actual_steps']}",
        f"sensor_foot: {reference['sensor_foot']}",
        f"expected_sensor_foot_contacts_reference: {reference['expected_sensor_foot_contacts_reference']}",
        f"estimated_total_steps_reference_only: {reference['estimated_total_steps_reference_only']}",
        f"estimated_minus_actual_steps_reference_only: {reference['estimated_minus_actual_steps']}",
        "reference_step_note: estimated_total_steps_reference_only is a start/end-foot "
        "conversion from attachment-foot stride markers, not a detected or confirmed total step count.",
        "marked_interval_rail_counts: "
        + ("unavailable" if marked_range is None else _json_text(marked_range["rail_counts"])),
        "outside_marked_interval_rail_counts: "
        + (
            "unavailable"
            if range_by_interval["outside_marked_interval"] is None
            else _json_text(range_by_interval["outside_marked_interval"]["rail_counts"])
        ),
        *[format_boundary_diagnostic(row) for row in summary["boundary_diagnostics"]],
        "note: stride_marker is not anatomical initial contact.",
        "note: marker count alone is not a confirmed total step count.",
        "note: START/FINISH duration uses rx monotonic time; stride period uses seq/100 Hz.",
        "note: walking stride output is offline zero-phase and is not a causal production detector.",
        "note: TUG mode reports operator START/FINISH total time only; phase detection is not implemented.",
    ]
    return "\n".join(lines) + "\n"


def run(args: argparse.Namespace) -> dict[str, object]:
    if args.out_dir.exists():
        raise ValueError(f"output directory already exists: {args.out_dir}")

    metadata = load_metadata(args.metadata_json)
    requested_trial = args.trial_name or metadata_value(metadata, "trial_name")
    trial_name, marker_rows = load_markers(
        args.markers_csv, None if requested_trial is None else str(requested_trial)
    )
    sensor = load_sensor_input(args.sensor_csv, args.accel_range_g, args.gyro_range_dps)
    sensor_series = fixed_detector.load_sensor_csv(args.sensor_csv)

    raw_test_type = getattr(args, "test_type", None) or metadata_value(metadata, "test_type") or "Walk10m"
    test_type = str(raw_test_type).strip().lower()
    if test_type not in {"walk10m", "walk", "tug"}:
        raise ValueError(f"unsupported test_type {raw_test_type!r}")
    is_tug = test_type == "tug"
    distance_m = getattr(args, "distance_m", None)
    if distance_m is None and metadata_value(metadata, "distance_m") is not None:
        distance_m = float(metadata_value(metadata, "distance_m"))

    actual_steps_unknown = bool(getattr(args, "actual_steps_unknown", False))
    actual_steps_value = None if actual_steps_unknown else args.actual_steps
    if (
        not actual_steps_unknown
        and actual_steps_value is None
        and metadata_value(metadata, "actual_steps") is not None
    ):
        actual_steps_value = int(metadata_value(metadata, "actual_steps"))
    if actual_steps_value is not None and actual_steps_value < 0:
        raise ValueError("actual_steps must be non-negative")
    actual_steps_source = (
        "explicit_unknown"
        if actual_steps_unknown
        else ("cli" if args.actual_steps is not None else ("metadata" if actual_steps_value is not None else "unavailable"))
    )
    start_foot = normalize_foot(args.start_foot or metadata_value(metadata, "start_foot"))
    end_foot = normalize_foot(args.end_foot or metadata_value(metadata, "end_foot"))
    requested_sensor_foot = getattr(args, "sensor_foot", None)
    sensor_foot = normalize_foot(requested_sensor_foot or metadata_value(metadata, "sensor_foot"))
    if sensor_foot is None:
        sensor_foot = "unknown"
    start_foot_source = "cli" if args.start_foot is not None else ("metadata" if start_foot is not None else "unavailable")
    end_foot_source = "cli" if args.end_foot is not None else ("metadata" if end_foot is not None else "unavailable")
    sensor_foot_source = (
        "cli"
        if requested_sensor_foot is not None
        else ("metadata" if metadata_value(metadata, "sensor_foot") is not None else "unavailable")
    )

    errors: list[str] = []
    warnings: list[str] = []
    if not is_tug and (
        distance_m is None or distance_m <= 0.0 or not math.isfinite(distance_m)
    ):
        errors.append("distance_nonpositive_or_nonfinite")
    if sensor_foot == "unknown" and not is_tug:
        warnings.append("sensor_foot_unknown_reference_steps_disabled")
    start, finish, marker_errors = _marker_validation(marker_rows, sensor)
    errors.extend(marker_errors)
    if sensor.rx_duplicate_count:
        warnings.append("rx_timestamp_duplicate_or_batched")
    if sensor.rx_reverse_count:
        errors.append("rx_timestamp_reverse")
    if sensor.rx_elapsed_mismatch_count:
        warnings.append("rx_elapsed_mismatch")
    if len(sensor_series.missing_seq):
        warnings.append("missing_seq")
    if any(sensor.rail_counts.values()):
        warnings.append("sensor_rail")
    if any(sensor.near_rail_counts.values()):
        warnings.append("sensor_near_rail")
    invalid_sensor_samples = capture_metric(metadata, "invalid_sensor_samples")
    sensor_fault = capture_metric(metadata, "sensor_fault")
    metadata_missing_seq = capture_metric(metadata, "missing_seq")
    metadata_metrics = metadata.get("metrics", {})
    if isinstance(metadata_metrics, dict) and "marker_valid" in metadata_metrics:
        try:
            if int(metadata_metrics["marker_valid"]) != 1:
                errors.append("metadata_marker_invalid")
        except (TypeError, ValueError):
            errors.append("metadata_marker_valid_invalid")
    if invalid_sensor_samples:
        warnings.append("capture_invalid_sensor_samples")
    if sensor_fault:
        warnings.append("capture_sensor_fault")
    if metadata_missing_seq and not len(sensor_series.missing_seq):
        warnings.append("metadata_missing_seq_mismatch")

    metadata_marker_error = any(error.startswith("metadata_marker_") for error in errors)
    marker_interval_valid = (
        start is not None
        and finish is not None
        and not marker_errors
        and not metadata_marker_error
    )
    stride_gate_valid = marker_interval_valid and sensor.rx_reverse_count == 0 and not is_tug
    duration_sec: float | None = None
    speed_mps: float | None = None
    if marker_interval_valid:
        duration_sec = (
            int(finish["marker_monotonic_ns"]) - int(start["marker_monotonic_ns"])
        ) / 1_000_000_000.0
        if not is_tug and not any(error.startswith("distance_") for error in errors):
            speed_mps = float(distance_m) / duration_sec

    identity_sync = fixed_detector.SyncFit(1.0, 0.0, 0.0, 0.0, 0)
    candidates = (
        []
        if is_tug
        else fixed_detector.detect_fixed_candidates(
            sensor_series, identity_sync, MISSING_NEAR_SEC
        )
    )
    candidate_output: list[dict[str, object]] = []
    marker_output: list[dict[str, object]] = []
    selected_event_times: list[float] = []
    selected_missing_near = 0
    for candidate in candidates:
        candidate_formal = bool(candidate["formal"])
        event_time_source = (
            "zero_cross" if candidate["zero_cross_sensor_time_sec"] is not None else "peak"
        )
        event_sensor_time = float(
            candidate["zero_cross_sensor_time_sec"]
            if candidate["zero_cross_sensor_time_sec"] is not None
            else candidate["peak_sensor_time_sec"]
        )
        event_rx_ns = _event_rx_ns(event_sensor_time, sensor_series, sensor)
        within: bool | None = None
        if stride_gate_valid:
            within = (
                int(start["marker_monotonic_ns"])
                <= event_rx_ns
                <= int(finish["marker_monotonic_ns"])
            )
        adopted = candidate_formal and within is True
        if not candidate_formal:
            exclusion_reason = str(candidate["failure_reason"])
        elif within is None:
            exclusion_reason = "marked_interval_invalid"
        elif event_rx_ns < int(start["marker_monotonic_ns"]):
            exclusion_reason = "before_start"
        elif event_rx_ns > int(finish["marker_monotonic_ns"]):
            exclusion_reason = "after_finish"
        else:
            exclusion_reason = ""
        candidate_row = {
            "event_name": (
                f"{sensor_foot}_foot_stride_marker"
                if candidate_formal
                else f"{sensor_foot}_foot_stride_candidate"
            ),
            "candidate_index": candidate["candidate_index"],
            "candidate_formal": candidate_formal,
            "candidate_failure_reason": candidate["failure_reason"],
            "event_time_source": event_time_source,
            "within_marked_interval": within,
            "adopted_as_stride_marker": adopted,
            "exclusion_reason": exclusion_reason,
            "event_seq_time_sec": event_sensor_time,
            "event_rx_monotonic_ns_estimate": event_rx_ns,
            "event_marker_elapsed_sec_estimate": (
                event_rx_ns - int(sensor.rx_monotonic_ns[0])
            )
            / 1_000_000_000.0,
            "signed_time_from_start_sec": (
                None
                if start is None
                else (event_rx_ns - int(start["marker_monotonic_ns"])) / 1_000_000_000.0
            ),
            "signed_time_from_finish_sec": (
                None
                if finish is None
                else (event_rx_ns - int(finish["marker_monotonic_ns"])) / 1_000_000_000.0
            ),
            "peak_seq": candidate["peak_seq"],
            "peak_gx_dps": candidate["peak_gx_dps"],
            "prominence_dps": candidate["prominence_dps"],
            "confirmation_min_gx_dps": candidate["confirmation_min_gx_dps"],
            "missing_seq_near": candidate["missing_near"],
            "nearest_missing_sec": candidate["nearest_missing_sec"],
            "max_missing_run_near": candidate["max_missing_run_near"],
        }
        candidate_output.append(candidate_row)
        if adopted:
            selected_event_times.append(event_sensor_time)
            if candidate["missing_near"]:
                selected_missing_near += 1
        if candidate_formal:
            marker_output.append(dict(candidate_row))

    boundary_diagnostics = (
        [] if is_tug else build_boundary_diagnostics(candidate_output, start, finish)
    )

    stride_metrics, interval_rows = summarize_intervals(selected_event_times)
    if stride_gate_valid:
        marker_count = int(stride_metrics["stride_marker_count"])
        if marker_count < 2:
            warnings.append("stride_marker_count_lt_2_period_unavailable")
        if marker_count < 3:
            warnings.append("stride_marker_count_lt_3_sd_cv_unavailable")
        if int(stride_metrics["interval_outlier_count"]):
            warnings.append("stride_interval_outlier")
        if int(stride_metrics["pause_or_missed_marker_count"]):
            warnings.append("pause_or_missed_marker")
        if selected_missing_near:
            warnings.append("stride_marker_near_missing_seq")
    else:
        stride_metrics = {
            **stride_metrics,
            "stride_marker_count": None,
        }

    reference_estimate: int | None = None
    reference_formula: str | None = None
    if not is_tug and stride_metrics["stride_marker_count"] is not None:
        reference_estimate, reference_formula = derive_reference_step_estimate(
            int(stride_metrics["stride_marker_count"]), sensor_foot, start_foot, end_foot
        )
    expected_sensor_contacts = expected_sensor_foot_contacts(
        actual_steps_value, sensor_foot, start_foot, end_foot
    )
    step_evaluated = actual_steps_value is not None and reference_estimate is not None
    step_error = reference_estimate - actual_steps_value if step_evaluated else None

    errors = sorted(set(errors))
    warnings = sorted(set(warnings))
    quality_status = "error" if errors else ("warning" if warnings else "ok")
    summary: dict[str, object] = {
        "schema_version": TOOL_SCHEMA_VERSION,
        "trial_name": trial_name,
        "test_type": test_type,
        "mode": (
            "operator-marker TUG total time"
            if is_tug
            else "offline zero-phase functional gait metrics"
        ),
        "inputs": {
            "sensor_csv": str(args.sensor_csv),
            "markers_csv": str(args.markers_csv),
            "metadata_json": None if args.metadata_json is None else str(args.metadata_json),
        },
        "clock_roles": {
            "marked_duration_and_speed": "marker_monotonic_ns on PC receive monotonic clock",
            "stride_period_and_cadence": "seq 100 Hz grid",
            "event_interval_gating": "seq event mapped by interpolation onto rx_monotonic_ns",
        },
        "fixed_detector": {
            "applied": not is_tug,
            "event_name": f"{sensor_foot}_foot_stride_marker",
            "sensor_foot": sensor_foot,
            "anatomical_initial_contact_claim": False,
            "fs_hz": fixed_detector.FS_HZ,
            "lpf_order": fixed_detector.LPF_ORDER,
            "lpf_cutoff_hz": fixed_detector.LPF_CUTOFF_HZ,
            "zero_phase": True,
            "peak_height_dps": fixed_detector.PEAK_HEIGHT_DPS,
            "peak_prominence_dps": fixed_detector.PEAK_PROMINENCE_DPS,
            "minimum_distance_sec": fixed_detector.MIN_DISTANCE_SEC,
            "zero_cross_window_sec": fixed_detector.ZERO_CROSS_WINDOW_SEC,
            "negative_confirmation_window_sec": fixed_detector.CONFIRM_WINDOW_SEC,
            "negative_confirmation_dps": fixed_detector.CONFIRM_GX_DPS,
        },
        "timing": {
            "start_marker_monotonic_ns": None if start is None else start["marker_monotonic_ns"],
            "finish_marker_monotonic_ns": None if finish is None else finish["marker_monotonic_ns"],
            "marked_duration_sec": duration_sec,
            "distance_m": distance_m,
            "speed_mps": speed_mps,
        },
        "boundary_diagnostics": boundary_diagnostics,
        "stride_metrics": stride_metrics,
        "interval_rule": {
            "inlier_ratio_min_inclusive": INLIER_MIN_RATIO,
            "inlier_ratio_max_inclusive": INLIER_MAX_RATIO,
            "pause_or_missed_marker": (
                "interval > 1.50 * trial median and interval - median >= 0.50 sec; "
                "does not distinguish a true pause from a missed marker"
            ),
            "sd_cv_minimum_stride_markers": 3,
            "period_cadence_minimum_stride_markers": 2,
        },
        "reference_step_evaluation": {
            "is_primary_metric": False,
            "actual_steps": actual_steps_value,
            "actual_steps_source": actual_steps_source,
            "sensor_foot": sensor_foot,
            "sensor_foot_source": sensor_foot_source,
            "expected_sensor_foot_contacts_reference": expected_sensor_contacts,
            "start_foot": start_foot,
            "start_foot_source": start_foot_source,
            "end_foot": end_foot,
            "end_foot_source": end_foot_source,
            "estimated_total_steps_reference_only": reference_estimate,
            "estimation_formula": reference_formula,
            "evaluated": step_evaluated,
            "estimated_minus_actual_steps": step_error,
            "warning": (
                "estimated_total_steps_reference_only is a start/end-foot conversion "
                "from attachment-foot stride markers; it is not a detected or confirmed total step count"
            ),
        },
        "sensor_quality": {
            "sample_count": len(sensor.rows),
            "seq_first": int(sensor_series.unwrapped_seq[0]),
            "seq_last": int(sensor_series.unwrapped_seq[-1]),
            "missing_seq_count": int(len(sensor_series.missing_seq)),
            "invalid_sensor_samples_metadata": invalid_sensor_samples,
            "sensor_fault_metadata": sensor_fault,
            "zero_accel_count": sensor.zero_accel_count,
            "zero_accel_ratio": sensor.zero_accel_ratio,
            "rx_duplicate_count": sensor.rx_duplicate_count,
            "rx_reverse_count": sensor.rx_reverse_count,
            "rx_elapsed_mismatch_count": sensor.rx_elapsed_mismatch_count,
            "accel_range_g_for_diagnostics": args.accel_range_g,
            "gyro_range_dps_for_diagnostics": args.gyro_range_dps,
            "rail_counts": sensor.rail_counts,
            "near_rail_95pct_counts": sensor.near_rail_counts,
            "range_by_interval": summarize_range_by_interval(
                sensor,
                start,
                finish,
                marker_interval_valid,
                args.accel_range_g,
                args.gyro_range_dps,
            ),
            "stride_markers_near_missing_seq": selected_missing_near,
        },
        "trial_metadata": metadata,
        "quality": {
            "status": quality_status,
            "errors": errors,
            "warnings": warnings,
            "flags": errors + warnings,
        },
        "tug_scope": {
            "total_time_implemented": is_tug,
            "total_time_sec": duration_sec if is_tug else None,
            "phase_detection_implemented": False,
            "phase_detection_validated": False,
            "foot_tag_does_not_define_seat_off_or_sit_contact": True,
            "warning": "Foot TAG alone does not establish seat-off or seat-contact.",
        },
    }

    args.out_dir.mkdir(parents=True, exist_ok=False)
    (args.out_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    flat = {
        "schema_version": summary["schema_version"],
        "trial_name": trial_name,
        "test_type": test_type,
        "quality_status": quality_status,
        "marked_duration_sec": duration_sec,
        "distance_m": distance_m,
        "speed_mps": speed_mps,
        **stride_metrics,
        "actual_steps": actual_steps_value,
        "actual_steps_source": actual_steps_source,
        "sensor_foot": sensor_foot,
        "sensor_foot_source": sensor_foot_source,
        "expected_sensor_foot_contacts_reference": expected_sensor_contacts,
        "start_foot": start_foot,
        "end_foot": end_foot,
        "estimated_total_steps_reference_only": reference_estimate,
        "estimated_minus_actual_steps": step_error,
        "marked_interval_rail_counts_json": _json_text(
            summary["sensor_quality"]["range_by_interval"]["marked_interval"]
        ),
        "outside_marked_interval_rail_counts_json": _json_text(
            summary["sensor_quality"]["range_by_interval"]["outside_marked_interval"]
        ),
        "quality_errors_json": _json_text(errors),
        "quality_warnings_json": _json_text(warnings),
    }
    _write_csv(args.out_dir / "summary.csv", [flat], tuple(flat.keys()))
    candidate_fields = (
        "event_name",
        "candidate_index",
        "candidate_formal",
        "candidate_failure_reason",
        "event_time_source",
        "within_marked_interval",
        "adopted_as_stride_marker",
        "exclusion_reason",
        "event_seq_time_sec",
        "event_rx_monotonic_ns_estimate",
        "event_marker_elapsed_sec_estimate",
        "signed_time_from_start_sec",
        "signed_time_from_finish_sec",
        "peak_seq",
        "peak_gx_dps",
        "prominence_dps",
        "confirmation_min_gx_dps",
        "missing_seq_near",
        "nearest_missing_sec",
        "max_missing_run_near",
    )
    _write_csv(args.out_dir / "stride_candidates.csv", candidate_output, candidate_fields)
    _write_csv(
        args.out_dir / "stride_markers.csv",
        marker_output,
        candidate_fields,
    )
    _write_csv(
        args.out_dir / "boundary_diagnostics.csv",
        boundary_diagnostics,
        (
            "boundary",
            "side",
            "event_kind",
            "boundary_marker_monotonic_ns",
            "found",
            "candidate_index",
            "candidate_formal",
            "event_time_source",
            "event_seq_time_sec",
            "event_rx_monotonic_ns_estimate",
            "signed_time_diff_sec",
            "within_marked_interval",
            "adopted_as_stride_marker",
            "exclusion_reason",
        ),
    )
    _write_csv(
        args.out_dir / "stride_intervals.csv",
        interval_rows,
        (
            "interval_index",
            "start_stride_marker_sec",
            "end_stride_marker_sec",
            "stride_time_sec",
            "ratio_to_trial_median",
            "inlier",
            "interval_outlier",
            "pause_or_missed_marker",
        ),
    )
    (args.out_dir / "summary.txt").write_text(build_text_summary(summary), encoding="utf-8")
    return summary


def main(argv: Sequence[str] | None = None) -> int:
    try:
        args = parse_args(argv)
        summary = run(args)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(build_text_summary(summary), end="")
    return 2 if summary["quality"]["status"] == "error" else 0


if __name__ == "__main__":
    raise SystemExit(main())
