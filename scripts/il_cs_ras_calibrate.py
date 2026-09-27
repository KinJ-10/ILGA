#!/usr/bin/env python3
"""Create, verify, and apply an installation-specific IL RAS calibration."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import sys

from il_cs_ras_window_evaluate import endpoint_calibration, rolling_medians, sample_sd


SCHEMA_VERSION = 1
ESTIMATOR = "firmware_phase_slope_m"


def load_capture(path: Path, window_size: int) -> dict:
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    values = [float(row[ESTIMATOR]) for row in rows if row.get(ESTIMATOR) not in (None, "", "nan")]
    values = [value for value in values if math.isfinite(value)]
    windows = rolling_medians(values, window_size)
    if not windows:
        raise ValueError(f"not enough valid phase rows for a {window_size}-frame window: {path}")
    good_tones = [float(row["good_tone_count"]) for row in rows if row.get("good_tone_count")]
    return {
        "path": str(path), "frames": len(rows), "valid_values": len(values),
        "stabilized_m": statistics.median(windows),
        "window_output_sd_m": sample_sd(windows),
        "median_good_tones": statistics.median(good_tones),
    }


def validate_config(config: dict) -> None:
    if config.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported calibration schema version")
    if config.get("estimator") != ESTIMATOR:
        raise ValueError("unsupported estimator")
    window = config["window"]
    if window.get("kind") != "rolling_median" or window["frames"] < 1 or window["frames"] % 2 == 0:
        raise ValueError("invalid rolling-median window")
    calibration = config["calibration"]
    low, high = calibration["low_anchor"], calibration["high_anchor"]
    if not low["true_m"] < high["true_m"] or calibration["gain"] <= 0:
        raise ValueError("invalid calibration anchors or gain")
    zone = config["zone"]
    if not low["true_m"] <= zone["enter_m"] < zone["exit_m"] <= high["true_m"]:
        raise ValueError("zone thresholds must be ordered inside the calibrated range")
    quality = config["quality"]
    if min(quality["max_window_output_sd_m"], quality["anchor_tolerance_m"],
           quality["min_median_good_tones"]) <= 0:
        raise ValueError("quality thresholds must be positive")


def corrected_distance(config: dict, measured_m: float) -> float:
    calibration = config["calibration"]
    return calibration["gain"] * measured_m + calibration["offset_m"]


def classify_distances(distances: list[float], enter_m: float, exit_m: float,
                       initial_state: str = "outside") -> list[str]:
    if initial_state not in ("inside", "outside"):
        raise ValueError("initial state must be inside or outside")
    state = initial_state
    states = []
    for distance in distances:
        if state == "outside" and distance <= enter_m:
            state = "inside"
        elif state == "inside" and distance >= exit_m:
            state = "outside"
        states.append(state)
    return states


def load_config(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        config = json.load(stream)
    validate_config(config)
    return config


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def create_config(args: argparse.Namespace) -> int:
    if not 0 < args.low_distance_m < args.high_distance_m:
        raise ValueError("anchor distances must be positive and ordered")
    if not args.low_distance_m <= args.enter_m < args.exit_m <= args.high_distance_m:
        raise ValueError("zone thresholds must be inside the calibrated range")
    low = load_capture(args.low_csv, args.window_size)
    high = load_capture(args.high_csv, args.window_size)
    for name, capture in (("low", low), ("high", high)):
        if capture["window_output_sd_m"] > args.max_window_sd_m:
            raise ValueError(f"{name} anchor capture is unstable")
        if capture["median_good_tones"] < args.min_good_tones:
            raise ValueError(f"{name} anchor has too few good tones")
    gain, offset = endpoint_calibration(
        low["stabilized_m"], args.low_distance_m,
        high["stabilized_m"], args.high_distance_m)
    if gain <= 0:
        raise ValueError("measured phase does not increase with anchor distance")
    config = {
        "schema_version": SCHEMA_VERSION,
        "created_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "site_id": args.site_id,
        "locator_id": args.locator_id,
        "tag_id": args.tag_id,
        "estimator": ESTIMATOR,
        "window": {"kind": "rolling_median", "frames": args.window_size},
        "calibration": {
            "low_anchor": {"true_m": args.low_distance_m,
                           "measured_m": low["stabilized_m"], "source": low},
            "high_anchor": {"true_m": args.high_distance_m,
                            "measured_m": high["stabilized_m"], "source": high},
            "gain": gain, "offset_m": offset,
        },
        "quality": {
            "max_window_output_sd_m": args.max_window_sd_m,
            "anchor_tolerance_m": args.anchor_tolerance_m,
            "min_median_good_tones": args.min_good_tones,
        },
        "zone": {"enter_m": args.enter_m, "exit_m": args.exit_m,
                 "initial_state": "outside", "scope": "technical_demo_0p5_to_1p5m"},
    }
    validate_config(config)
    write_json(args.output, config)
    print(f"Created calibration: {args.output}")
    return 0


def verify_anchor(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    capture = load_capture(args.anchor_csv, config["window"]["frames"])
    predicted = corrected_distance(config, capture["stabilized_m"])
    error = predicted - args.known_distance_m
    quality = config["quality"]
    result = {
        "known_distance_m": args.known_distance_m,
        "measured_phase_m": capture["stabilized_m"],
        "corrected_distance_m": predicted,
        "error_m": error,
        "window_output_sd_m": capture["window_output_sd_m"],
        "median_good_tones": capture["median_good_tones"],
        "stable": capture["window_output_sd_m"] <= quality["max_window_output_sd_m"],
        "tones_ok": capture["median_good_tones"] >= quality["min_median_good_tones"],
        "anchor_ok": abs(error) <= quality["anchor_tolerance_m"],
    }
    result["valid"] = result["stable"] and result["tones_ok"] and result["anchor_ok"]
    write_json(args.output, result)
    print(f"Anchor verification: {'PASS' if result['valid'] else 'FAIL'}")
    return 0 if result["valid"] else 1


def classify_capture(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    window_size = config["window"]["frames"]
    with args.input_csv.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    values = [float(row[ESTIMATOR]) for row in rows]
    windows = rolling_medians(values, window_size)
    corrected = [corrected_distance(config, value) for value in windows]
    zone = config["zone"]
    states = classify_distances(corrected, zone["enter_m"], zone["exit_m"], zone["initial_state"])
    output_rows = []
    for window_index, (measured, distance, state) in enumerate(zip(windows, corrected, states),
                                                                window_size - 1):
        output_rows.append({
            "capture_index": rows[window_index].get("capture_index", window_index),
            "phase_window_median_m": measured,
            "corrected_distance_m": distance,
            "zone_state": state,
        })
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output_rows[0]))
        writer.writeheader()
        writer.writerows(output_rows)
    print(f"Classified {len(output_rows)} filtered frames: {args.output_csv}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    create = commands.add_parser("create", help="create a two-anchor calibration")
    create.add_argument("--low-csv", required=True, type=Path)
    create.add_argument("--low-distance-m", required=True, type=float)
    create.add_argument("--high-csv", required=True, type=Path)
    create.add_argument("--high-distance-m", required=True, type=float)
    create.add_argument("--window-size", type=int, default=5)
    create.add_argument("--max-window-sd-m", type=float, default=0.20)
    create.add_argument("--anchor-tolerance-m", type=float, default=0.20)
    create.add_argument("--min-good-tones", type=float, default=60)
    create.add_argument("--enter-m", type=float, default=0.75)
    create.add_argument("--exit-m", type=float, default=1.25)
    create.add_argument("--site-id", default="UNSET")
    create.add_argument("--locator-id", default="UNSET")
    create.add_argument("--tag-id", default="UNSET")
    create.add_argument("--output", required=True, type=Path)
    create.set_defaults(handler=create_config)

    verify = commands.add_parser("verify", help="verify a known-distance anchor")
    verify.add_argument("--config", required=True, type=Path)
    verify.add_argument("--anchor-csv", required=True, type=Path)
    verify.add_argument("--known-distance-m", required=True, type=float)
    verify.add_argument("--output", required=True, type=Path)
    verify.set_defaults(handler=verify_anchor)

    classify = commands.add_parser("classify", help="apply calibration and zone hysteresis")
    classify.add_argument("--config", required=True, type=Path)
    classify.add_argument("--input-csv", required=True, type=Path)
    classify.add_argument("--output-csv", required=True, type=Path)
    classify.set_defaults(handler=classify_capture)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.handler(args)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
