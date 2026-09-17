#!/usr/bin/env python3
"""Extract Nordic RAS + cs_de distance estimates from a timestamped UART log."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
from pathlib import Path
from typing import Any


FLOAT_PATTERN = r"(?:[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?|[-+]?inf|nan)"
DISTANCE_PATTERN = re.compile(
    rf"Latest distance estimates on antenna path\s+(?P<antenna_path>\d+):\s*"
    rf"ifft:\s*(?P<ifft>{FLOAT_PATTERN}),\s*"
    rf"phase_slope:\s*(?P<phase_slope>{FLOAT_PATTERN}),\s*"
    rf"rtt:\s*(?P<rtt>{FLOAT_PATTERN})\s+meters",
    re.IGNORECASE,
)
METHODS = ("ifft", "phase_slope", "rtt")


def parse_log(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line_number, raw_line in enumerate(handle, start=1):
            line = raw_line.rstrip("\r\n")
            match = DISTANCE_PATTERN.search(line)
            if not match:
                continue

            host_timestamp = line.split("\t", 1)[0] if "\t" in line else ""
            rows.append(
                {
                    "line_number": line_number,
                    "host_timestamp": host_timestamp,
                    "antenna_path": int(match.group("antenna_path")),
                    "ifft_m": float(match.group("ifft")),
                    "phase_slope_m": float(match.group("phase_slope")),
                    "rtt_m": float(match.group("rtt")),
                }
            )
    return rows


def _method_stats(values: list[float], true_distance_m: float | None) -> dict[str, Any]:
    finite_values = [value for value in values if math.isfinite(value)]
    result: dict[str, Any] = {
        "count": len(finite_values),
        "rejected_non_finite": len(values) - len(finite_values),
        "mean_m": None,
        "median_m": None,
        "stdev_m": None,
        "min_m": None,
        "max_m": None,
        "mae_m": None,
        "median_error_m": None,
    }
    if not finite_values:
        return result

    result.update(
        {
            "mean_m": statistics.fmean(finite_values),
            "median_m": statistics.median(finite_values),
            "stdev_m": statistics.pstdev(finite_values),
            "min_m": min(finite_values),
            "max_m": max(finite_values),
        }
    )
    if true_distance_m is not None:
        result["mae_m"] = statistics.fmean(
            abs(value - true_distance_m) for value in finite_values
        )
        result["median_error_m"] = result["median_m"] - true_distance_m
    return result


def summarize(
    rows: list[dict[str, Any]], true_distance_m: float | None
) -> dict[str, Any]:
    antenna_paths = sorted({row["antenna_path"] for row in rows})
    by_path: dict[str, Any] = {}
    for antenna_path in antenna_paths:
        path_rows = [row for row in rows if row["antenna_path"] == antenna_path]
        by_path[str(antenna_path)] = {
            method: _method_stats(
                [row[f"{method}_m"] for row in path_rows], true_distance_m
            )
            for method in METHODS
        }

    return {
        "estimate_rows": len(rows),
        "antenna_paths": antenna_paths,
        "true_distance_m": true_distance_m,
        "by_antenna_path": by_path,
    }


def write_outputs(
    output_dir: Path,
    source_log: Path,
    rows: list[dict[str, Any]],
    summary: dict[str, Any],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "distance_estimates.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=(
                "line_number",
                "host_timestamp",
                "antenna_path",
                "ifft_m",
                "phase_slope_m",
                "rtt_m",
            ),
        )
        writer.writeheader()
        writer.writerows(rows)

    payload = {"source_log": str(source_log.resolve()), **summary}
    (output_dir / "summary.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    lines = [
        "# Nordic RAS + cs_de distance summary",
        "",
        f"- Source log: `{source_log.resolve()}`",
        f"- Estimate rows: {summary['estimate_rows']}",
        f"- Antenna paths: {summary['antenna_paths']}",
        f"- True distance: {summary['true_distance_m']}",
        "",
        "| antenna path | method | count | median (m) | mean (m) | stdev (m) | MAE (m) |",
        "|---:|---|---:|---:|---:|---:|---:|",
    ]
    for antenna_path, methods in summary["by_antenna_path"].items():
        for method, stats in methods.items():
            def display(value: Any) -> str:
                return "" if value is None else f"{value:.6f}"

            lines.append(
                f"| {antenna_path} | {method} | {stats['count']} | "
                f"{display(stats['median_m'])} | {display(stats['mean_m'])} | "
                f"{display(stats['stdev_m'])} | {display(stats['mae_m'])} |"
            )
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze Nordic RAS + cs_de distance estimates from an ILGA UART log."
    )
    parser.add_argument("log", type=Path, help="Timestamped LOCATOR UART log")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--true-distance-m", type=float)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.log.is_file():
        raise SystemExit(f"ERROR: log file not found: {args.log}")
    if args.true_distance_m is not None and args.true_distance_m < 0:
        raise SystemExit("ERROR: --true-distance-m must be non-negative")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise SystemExit(f"ERROR: output directory is not empty: {args.output_dir}")

    rows = parse_log(args.log)
    if not rows:
        raise SystemExit("ERROR: no Nordic RAS + cs_de distance estimates found")
    summary = summarize(rows, args.true_distance_m)
    write_outputs(args.output_dir, args.log, rows, summary)
    print(f"Parsed {len(rows)} distance estimates into {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
