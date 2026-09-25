#!/usr/bin/env python3
"""Recalculate NCS v3.2.3 RAS distances and experimental candidates from ILRAS1 UART frames."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np


NFFT = 512
CHANNELS = 75
C_M_PER_S = 299_792_458.0
INVALID = -(2**31)
BIN_M = C_M_PER_S / (2 * NFFT * 1_000_000)


def fnv1a_lines(lines: list[str]) -> int:
    value = 2166136261
    for line in lines:
        for byte in (line + "\n").encode("ascii"):
            value = ((value ^ byte) * 16777619) & 0xFFFFFFFF
    return value


def parse_frames(path: Path) -> tuple[list[dict], list[dict]]:
    complete: list[dict] = []
    errors: list[dict] = []
    active: dict[int, dict] = {}
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line_no, source_line in enumerate(stream, 1):
            offset = source_line.find("ILRAS1,")
            if offset < 0:
                continue
            wire = source_line[offset:].strip()
            fields = wire.split(",")
            try:
                kind = fields[1]
                sample_id = int(fields[2])
                if kind == "S" and len(fields) == 15:
                    if sample_id in active:
                        errors.append({"line": line_no, "sample_id": sample_id,
                                       "reason": "previous frame interrupted by new start"})
                    values = [int(value) for value in fields[2:]]
                    if values[2] != 1:
                        raise ValueError("unsupported antenna path count")
                    active[sample_id] = {"header": values, "tones": {}, "wire": [wire]}
                elif kind == "T" and len(fields) == 10:
                    frame = active[sample_id]
                    values = [int(value) for value in fields[3:]]
                    channel = values[0]
                    if not 2 <= channel <= 76 or channel in frame["tones"]:
                        raise ValueError("invalid or duplicate channel")
                    frame["tones"][channel] = values[1:]
                    frame["wire"].append(wire)
                elif kind == "E" and len(fields) == 5:
                    frame = active.pop(sample_id)
                    if int(fields[3]) != CHANNELS or len(frame["tones"]) != CHANNELS:
                        raise ValueError("incomplete tone set")
                    if int(fields[4], 16) != fnv1a_lines(frame["wire"]):
                        raise ValueError("frame checksum mismatch")
                    frame["capture_index"] = len(complete)
                    complete.append(frame)
                else:
                    raise ValueError("unsupported record or field count")
            except (KeyError, IndexError, ValueError) as exc:
                errors.append({"line": line_no, "sample_id": fields[2] if len(fields) > 2 else "", "reason": str(exc)})
                if len(fields) > 2 and fields[2].isdigit():
                    active.pop(int(fields[2]), None)
    for sample_id in active:
        errors.append({"line": "", "sample_id": sample_id, "reason": "incomplete frame at end of capture"})
    return complete, errors


def _distance_for_peak(magnitude: np.ndarray, peak: int) -> float:
    prompt = magnitude[peak]
    early = magnitude[(peak - 1) % NFFT]
    late = magnitude[(peak + 1) % NFFT]
    denominator = 4 * prompt - 2 * (early + late)
    interpolation = (late - early) / denominator if prompt >= early and prompt >= late and denominator else 0.0
    distance = (peak + interpolation) * BIN_M
    return float(distance) if peak < NFFT - 2 and distance >= 0 else math.nan


def nordic_ifft_peak(magnitude: np.ndarray) -> int:
    """Port of cs_de.c find_ifft_peak_index, including its left-null compensation."""
    maximum_index = int(np.argmax(magnitude))
    maximum = float(magnitude[maximum_index])
    nw, next_index = NFFT - 2, NFFT - 1
    first_rise_found = False
    chosen = maximum_index
    while nw != maximum_index:
        if magnitude[next_index] < magnitude[nw]:
            if 2.5 * magnitude[nw] > maximum and first_rise_found:
                chosen = nw
                break
        else:
            first_rise_found = True
        nw = next_index
        next_index = (next_index + 1) % NFFT

    left = chosen
    while True:
        next_left = (left - 1) % NFFT
        if ((magnitude[left] * 2 > magnitude[chosen] or
             magnitude[left] > 1.10 * magnitude[next_left]) and
                magnitude[left] * 10 > magnitude[chosen] and next_left != chosen):
            left = next_left
        else:
            break
    distance_to_null = (NFFT + chosen - left) % NFFT
    normal_peak_to_null = (NFFT + CHANNELS - 1) // CHANNELS
    if distance_to_null > normal_peak_to_null:
        if left > chosen:
            candidate = left + normal_peak_to_null - NFFT
            if candidate > 0:
                chosen = candidate
        else:
            chosen = left + normal_peak_to_null
    return chosen


def early_peak(magnitude: np.ndarray, relative: float = 0.15,
               noise_sigma: float = 5.0) -> int | None:
    """Earliest local maximum above a robust noise floor and a fraction of the strongest path."""
    floor_region = magnitude[64:256]
    median = float(np.median(floor_region))
    mad = float(np.median(np.abs(floor_region - median))) * 1.4826
    threshold = max(median + noise_sigma * mad, float(np.max(magnitude)) * relative)
    for index in range(1, NFFT // 2):
        if (magnitude[index] >= threshold and
                magnitude[index] >= magnitude[index - 1] and
                magnitude[index] > magnitude[index + 1]):
            return index
    return None


def nordic_phase_slope(z: np.ndarray) -> float:
    correlation = np.sum(z[1:] * np.conj(z[:-1]))
    distance = -C_M_PER_S * np.angle(correlation) / (4 * math.pi * 1_000_000)
    return float(distance) if distance >= 0 else math.nan


def robust_phase_slope(z: np.ndarray, good: np.ndarray, huber_k: float = 1.5) -> tuple[float, int]:
    indices = np.flatnonzero(good & (np.abs(z) > 1e-8))
    if len(indices) < 15:
        return math.nan, len(indices)
    phases = np.unwrap(np.angle(z[indices]))
    amplitude = np.abs(z[indices])
    scale = float(np.median(amplitude))
    base_weights = np.clip(np.sqrt(amplitude / max(scale, 1e-12)), 0.25, 4.0)
    x = indices.astype(float)
    x -= x.mean()
    weights = base_weights.copy()
    slope = math.nan
    for _ in range(8):
        design = np.column_stack((np.ones(len(x)), x))
        coefficients = np.linalg.lstsq(design * np.sqrt(weights[:, None]),
                                        phases * np.sqrt(weights), rcond=None)[0]
        residual = phases - design @ coefficients
        sigma = max(1.4826 * np.median(np.abs(residual - np.median(residual))), 1e-3)
        limit = huber_k * sigma
        weights = base_weights * np.minimum(1.0, limit / np.maximum(np.abs(residual), 1e-12))
        slope = float(coefficients[1])
    distance = -slope * C_M_PER_S / (4 * math.pi * 1_000_000)
    return (distance if distance >= 0 else math.nan), len(indices)


def analyze_frame(frame: dict, early_relative: float = 0.15,
                  early_noise_sigma: float = 5.0, huber_k: float = 1.5) -> tuple[dict, list[dict], list[dict]]:
    header = frame["header"]
    sample_id, ranging_counter, n_ap, quality, tone_quality, rtt_accum, rtt_count, *tail = header
    fw_ifft_mm, fw_phase_mm, fw_rtt_mm, fw_best_mm, missed, high_quality_value = tail
    samples = np.array([frame["tones"][channel] for channel in range(2, 77)], dtype=np.int64)
    local_qi, peer_qi = samples[:, 0], samples[:, 1]
    iq = samples[:, 2:].astype(float) / 1_000_000
    z = (iq[:, 2] + 1j * iq[:, 3]) * (iq[:, 0] + 1j * iq[:, 1])
    good = (local_qi == high_quality_value) & (peer_qi == high_quality_value)
    magnitude = np.abs(np.fft.ifft(z, n=NFFT))
    baseline_peak = nordic_ifft_peak(magnitude)
    candidate_peak = early_peak(magnitude, early_relative, early_noise_sigma)
    robust_distance, robust_count = robust_phase_slope(z, good, huber_k)

    def from_mm(value: int) -> float:
        return math.nan if value == INVALID else value / 1000

    summary = {
        "capture_index": frame.get("capture_index", 0),
        "sample_id": sample_id, "ranging_counter": ranging_counter,
        "quality": quality, "tone_quality": tone_quality,
        "good_tone_count": int(np.count_nonzero(good)), "robust_tone_count": robust_count,
        "rtt_accumulated_half_ns": rtt_accum, "rtt_count": rtt_count,
        "missed_capture_slots": missed, "firmware_ifft_m": from_mm(fw_ifft_mm),
        "firmware_phase_slope_m": from_mm(fw_phase_mm),
        "firmware_rtt_m": from_mm(fw_rtt_mm), "firmware_best_m": from_mm(fw_best_mm),
        "nordic_ifft_peak": baseline_peak,
        "nordic_ifft_recalc_m": _distance_for_peak(magnitude, baseline_peak),
        "nordic_phase_slope_recalc_m": nordic_phase_slope(z),
        "nordic_rtt_recalc_m": (max(rtt_accum * C_M_PER_S / (4 * rtt_count * 1e9), 0.0)
                                if rtt_count else math.nan),
        "early_peak": candidate_peak,
        "early_peak_m": _distance_for_peak(magnitude, candidate_peak) if candidate_peak is not None else math.nan,
        "robust_phase_slope_m": robust_distance,
    }
    summary["ifft_recalc_minus_firmware_m"] = (summary["nordic_ifft_recalc_m"] -
                                               summary["firmware_ifft_m"])
    tone_rows = []
    for index, values in enumerate(samples):
        tone_rows.append({
            "capture_index": frame.get("capture_index", 0),
            "sample_id": sample_id, "channel": index + 2,
            "local_quality": int(values[0]), "peer_quality": int(values[1]),
            "local_i": float(iq[index, 0]), "local_q": float(iq[index, 1]),
            "peer_i": float(iq[index, 2]), "peer_q": float(iq[index, 3]),
            "combined_amplitude": float(abs(z[index])),
            "combined_phase_rad": float(np.angle(z[index])),
            "valid": bool(good[index]),
        })
    local_maxima = [index for index in range(1, NFFT // 2)
                    if magnitude[index] >= magnitude[index - 1] and magnitude[index] > magnitude[index + 1]]
    selected_peaks = sorted(local_maxima, key=lambda n: magnitude[n], reverse=True)[:8]
    if baseline_peak not in selected_peaks:
        selected_peaks.append(baseline_peak)
    if candidate_peak is not None and candidate_peak not in selected_peaks:
        selected_peaks.append(candidate_peak)
    peak_rows = [{"capture_index": frame.get("capture_index", 0),
                  "sample_id": sample_id, "rank": rank, "bin": index,
                  "magnitude": float(magnitude[index]), "distance_m": _distance_for_peak(magnitude, index),
                  "nordic_selected": index == baseline_peak, "early_selected": index == candidate_peak}
                 for rank, index in enumerate(selected_peaks, 1)]
    return summary, tone_rows, peak_rows


def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--true-distance-m", type=float)
    parser.add_argument("--early-relative", type=float, default=0.15)
    parser.add_argument("--early-noise-sigma", type=float, default=5.0)
    parser.add_argument("--huber-k", type=float, default=1.5)
    args = parser.parse_args(argv)
    if args.output_dir.exists() and (not args.output_dir.is_dir() or
                                     any(args.output_dir.iterdir())):
        parser.error("output directory must be empty")
    if not 0 < args.early_relative <= 1 or args.early_noise_sigma < 0 or args.huber_k <= 0:
        parser.error("invalid candidate algorithm parameter")
    try:
        frames, errors = parse_frames(args.log)
    except OSError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    if not frames:
        print("ERROR: no complete ILRAS1 capture frames", file=sys.stderr)
        return 1
    summaries, tones, peaks = [], [], []
    for frame in frames:
        summary, tone_rows, peak_rows = analyze_frame(
            frame, args.early_relative, args.early_noise_sigma, args.huber_k)
        if args.true_distance_m is not None:
            for name in ("firmware_ifft_m", "nordic_ifft_recalc_m", "early_peak_m",
                         "robust_phase_slope_m"):
                summary[name + "_error_m"] = summary[name] - args.true_distance_m
        summaries.append(summary)
        tones.extend(tone_rows)
        peaks.extend(peak_rows)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    _write_csv(args.output_dir / "procedures.csv", summaries)
    _write_csv(args.output_dir / "tones.csv", tones)
    _write_csv(args.output_dir / "ifft_peaks.csv", peaks)
    _write_csv(args.output_dir / "parse_errors.csv", errors)
    (args.output_dir / "summary.json").write_text(json.dumps({
        "frames": len(frames), "parse_errors": len(errors), "true_distance_m": args.true_distance_m,
        "candidate_parameters": {"early_relative": args.early_relative,
                                 "early_noise_sigma": args.early_noise_sigma, "huber_k": args.huber_k},
    }, indent=2) + "\n", encoding="utf-8")
    print(f"Analyzed {len(frames)} complete frames; {len(errors)} framing errors")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
