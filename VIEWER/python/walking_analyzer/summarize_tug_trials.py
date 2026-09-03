#!/usr/bin/env python3
"""Aggregate operator-marker TUG total times without inferring TUG phases."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Sequence


def summarize(records: Sequence[dict[str, object]]) -> dict[str, object]:
    if not records:
        raise ValueError("at least one TUG summary is required")
    durations: list[float] = []
    rows: list[dict[str, object]] = []
    for record in records:
        if str(record.get("test_type", "")).lower() != "tug":
            raise ValueError(f"non-TUG summary supplied: {record.get('trial_name')}")
        quality = record.get("quality", {})
        if not isinstance(quality, dict) or quality.get("status") == "error":
            raise ValueError(f"TUG summary has quality error: {record.get('trial_name')}")
        timing = record.get("timing", {})
        duration = timing.get("marked_duration_sec") if isinstance(timing, dict) else None
        if duration is None or not math.isfinite(float(duration)) or float(duration) <= 0.0:
            raise ValueError(f"invalid TUG duration: {record.get('trial_name')}")
        durations.append(float(duration))
        rows.append(
            {
                "trial_name": record.get("trial_name"),
                "tug_total_time_sec": float(duration),
                "quality_status": quality.get("status"),
                "phase_detection_implemented": False,
            }
        )
    mean_value = float(statistics.mean(durations))
    sd_value = float(statistics.stdev(durations)) if len(durations) >= 2 else None
    return {
        "schema_version": 1,
        "metric": "operator_marker_tug_total_time",
        "trial_count": len(durations),
        "mean_sec": mean_value,
        "sd_sec": sd_value,
        "cv_pct": None if sd_value is None else 100.0 * sd_value / mean_value,
        "min_sec": min(durations),
        "max_sec": max(durations),
        "range_sec": max(durations) - min(durations),
        "trials": rows,
        "phase_detection": {
            "implemented": False,
            "validated": False,
            "unreported_phases": ["sit_to_stand", "walk_out", "turn", "walk_back", "sit_down"],
        },
        "limitations": [
            "Foot TAG alone does not establish seat-off or seat-contact.",
            "No automatic TUG phase boundaries are produced.",
            "This output is not a medical or fall-risk diagnosis.",
        ],
    }


def run(summary_paths: Sequence[Path], out_dir: Path) -> dict[str, object]:
    if out_dir.exists():
        raise ValueError(f"output directory already exists: {out_dir}")
    records = [json.loads(path.read_text(encoding="utf-8")) for path in summary_paths]
    result = summarize(records)
    out_dir.mkdir(parents=True, exist_ok=False)
    (out_dir / "tug_aggregate.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    fields = ("trial_name", "tug_total_time_sec", "quality_status", "phase_detection_implemented")
    with (out_dir / "tug_trials.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(result["trials"])
    text = "\n".join(
        [
            "metric: operator_marker_tug_total_time",
            f"trial_count: {result['trial_count']}",
            f"mean_sec: {result['mean_sec']}",
            f"sd_sec: {result['sd_sec']}",
            f"cv_pct: {result['cv_pct']}",
            f"min_sec: {result['min_sec']}",
            f"max_sec: {result['max_sec']}",
            f"range_sec: {result['range_sec']}",
            "phase_detection: not implemented or validated",
            "seat_event_note: foot TAG alone does not establish seat-off or seat-contact",
        ]
    )
    (out_dir / "summary.txt").write_text(text + "\n", encoding="utf-8")
    return result


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary_json", nargs="+", type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    try:
        args = parse_args(argv)
        result = run(args.summary_json, args.out_dir)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
