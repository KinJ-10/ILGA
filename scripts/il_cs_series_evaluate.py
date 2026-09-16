#!/usr/bin/env python3
"""Evaluate labelled IL Channel Sounding trial directories without changing raw analysis."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path


SPEED_OF_LIGHT_M_PER_S = 299_792_458.0


def wrap_phase(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def circular_fit_cost(samples: list[tuple[float, float]], distance_m: float) -> float:
    slope = -4.0 * math.pi * distance_m * 1_000_000.0 / SPEED_OF_LIGHT_M_PER_S
    shifted = [phase - slope * frequency for frequency, phase in samples]
    intercept = math.atan2(
        sum(math.sin(value) for value in shifted),
        sum(math.cos(value) for value in shifted),
    )
    residuals = [wrap_phase(phase - (intercept + slope * frequency)) for frequency, phase in samples]
    return math.sqrt(sum(value * value for value in residuals) / len(residuals))


def search_distance(
    samples: list[tuple[float, float]], max_distance_m: float, grid_step_m: float
) -> tuple[float, float]:
    if len(samples) < 2:
        raise ValueError("distance search requires at least two phase samples")
    if max_distance_m <= 0 or grid_step_m <= 0:
        raise ValueError("distance range and grid step must be positive")

    step_count = int(math.floor(max_distance_m / grid_step_m))
    best_distance = 0.0
    best_cost = math.inf
    for step in range(step_count + 1):
        distance = step * grid_step_m
        cost = circular_fit_cost(samples, distance)
        if cost < best_cost:
            best_distance = distance
            best_cost = cost
    return best_distance, best_cost


def parse_series(value: str) -> tuple[float, Path]:
    try:
        distance_text, path_text = value.split("=", 1)
        distance = float(distance_text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("series must be ACTUAL_DISTANCE_M=TRIAL_DIR") from error
    if distance < 0 or not path_text:
        raise argparse.ArgumentTypeError("series distance and path must be valid")
    return distance, Path(path_text)


def procedure_key(row: dict[str, str]) -> tuple[str, str]:
    return row["boot_index"], row["procedure_counter"]


def load_trial(
    actual_distance_m: float,
    trial_dir: Path,
    skip_procedures: int,
    max_distance_m: float,
    grid_step_m: float,
) -> list[dict[str, object]]:
    summary_path = trial_dir / "procedure_summary.csv"
    pbr_path = trial_dir / "pbr_samples.csv"
    if not summary_path.is_file() or not pbr_path.is_file():
        raise FileNotFoundError(f"missing raw analysis CSV in {trial_dir}")

    with summary_path.open(newline="", encoding="utf-8") as handle:
        summaries = list(csv.DictReader(handle))
    accepted_rows = [
        row
        for row in summaries[skip_procedures:]
        if row["ended"] == "True" and row["pbr_valid"] != "0" and row["pbr_distance_m"]
    ]
    accepted = {procedure_key(row): row for row in accepted_rows}

    samples_by_procedure: dict[tuple[str, str], list[tuple[float, float]]] = defaultdict(list)
    with pbr_path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            key = procedure_key(row)
            if key not in accepted or row["valid"] != "True":
                continue
            samples_by_procedure[key].append(
                (float(row["frequency_mhz"]), float(row["wrapped_phase_rad"]))
            )

    results = []
    for (boot_index, procedure_counter), summary in accepted.items():
        samples = samples_by_procedure.get((boot_index, procedure_counter), [])
        if len(samples) < 2:
            continue
        candidate, cost = search_distance(samples, max_distance_m, grid_step_m)
        results.append(
            {
                "actual_distance_m": actual_distance_m,
                "trial_dir": str(trial_dir),
                "boot_index": int(boot_index),
                "procedure_counter": int(procedure_counter),
                "pbr_valid": int(summary["pbr_valid"]),
                "basic_pbr_distance_m": float(summary["pbr_distance_m"]),
                "basic_pbr_residual_rms_rad": float(summary["pbr_residual_rms_rad"]),
                "circular_candidate_distance_m": candidate,
                "circular_candidate_cost_rad": cost,
            }
        )
    return results


def sample_stdev(values: list[float]) -> float:
    return statistics.stdev(values) if len(values) > 1 else 0.0


def rank_values(values: list[float]) -> list[float]:
    order = sorted(range(len(values)), key=values.__getitem__)
    ranks = [0.0] * len(values)
    position = 0
    while position < len(order):
        end = position + 1
        while end < len(order) and values[order[end]] == values[order[position]]:
            end += 1
        rank = (position + 1 + end) / 2.0
        for index in order[position:end]:
            ranks[index] = rank
        position = end
    return ranks


def correlation(left: list[float], right: list[float]) -> float | None:
    if len(left) != len(right) or len(left) < 2:
        return None
    left_mean = statistics.mean(left)
    right_mean = statistics.mean(right)
    numerator = sum((x - left_mean) * (y - right_mean) for x, y in zip(left, right))
    denominator = math.sqrt(
        sum((x - left_mean) ** 2 for x in left)
        * sum((y - right_mean) ** 2 for y in right)
    )
    return numerator / denominator if denominator else None


def summarize(rows: list[dict[str, object]]) -> tuple[list[dict[str, object]], dict[str, object]]:
    grouped: dict[float, list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[float(row["actual_distance_m"])].append(row)

    summaries = []
    for actual_distance in sorted(grouped):
        distance_rows = grouped[actual_distance]
        basic = [float(row["basic_pbr_distance_m"]) for row in distance_rows]
        circular = [float(row["circular_candidate_distance_m"]) for row in distance_rows]
        residual = [float(row["basic_pbr_residual_rms_rad"]) for row in distance_rows]
        cost = [float(row["circular_candidate_cost_rad"]) for row in distance_rows]
        summaries.append(
            {
                "actual_distance_m": actual_distance,
                "valid_procedures": len(distance_rows),
                "basic_pbr_median_m": statistics.median(basic),
                "basic_pbr_stdev_m": sample_stdev(basic),
                "basic_residual_median_rad": statistics.median(residual),
                "circular_candidate_median_m": statistics.median(circular),
                "circular_candidate_stdev_m": sample_stdev(circular),
                "circular_cost_median_rad": statistics.median(cost),
            }
        )

    actual = [float(row["actual_distance_m"]) for row in summaries]
    basic = [float(row["basic_pbr_median_m"]) for row in summaries]
    circular = [float(row["circular_candidate_median_m"]) for row in summaries]
    metrics = {
        "distance_count": len(summaries),
        "procedure_count": len(rows),
        "basic_spearman": correlation(rank_values(actual), rank_values(basic)),
        "circular_spearman": correlation(rank_values(actual), rank_values(circular)),
    }
    return summaries, metrics


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        raise ValueError("no rows to write")
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--series",
        type=parse_series,
        action="append",
        required=True,
        metavar="ACTUAL_DISTANCE_M=TRIAL_DIR",
    )
    parser.add_argument("--skip-procedures", type=int, default=10)
    parser.add_argument("--max-distance-m", type=float, default=10.0)
    parser.add_argument("--grid-step-m", type=float, default=0.01)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    if args.skip_procedures < 0:
        parser.error("--skip-procedures must be non-negative")
    if args.output_dir.exists():
        parser.error(f"output directory already exists: {args.output_dir}")

    rows = []
    for actual_distance, trial_dir in args.series:
        rows.extend(
            load_trial(
                actual_distance,
                trial_dir,
                args.skip_procedures,
                args.max_distance_m,
                args.grid_step_m,
            )
        )
    if not rows:
        parser.error("no valid PBR procedures found")

    summaries, metrics = summarize(rows)
    args.output_dir.mkdir(parents=True)
    write_csv(args.output_dir / "procedure_candidates.csv", rows)
    write_csv(args.output_dir / "distance_summary.csv", summaries)
    with (args.output_dir / "metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(metrics, handle, ensure_ascii=False, indent=2)
        handle.write("\n")

    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
