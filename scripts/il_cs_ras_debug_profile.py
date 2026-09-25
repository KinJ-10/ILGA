#!/usr/bin/env python3
"""Inspect the full IFFT profile of complete ILRAS1 captures."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np

from il_cs_ras_debug_analyze import (
    BIN_M,
    CHANNELS,
    C_M_PER_S,
    NFFT,
    _distance_for_peak,
    analyze_frame,
    early_peak,
    nordic_ifft_peak,
    parse_frames,
)


def frame_profile(frame: dict) -> tuple[np.ndarray, int]:
    samples = np.array([frame["tones"][channel] for channel in range(2, 77)], dtype=np.int64)
    iq = samples[:, 2:].astype(float) / 1_000_000
    z = (iq[:, 2] + 1j * iq[:, 3]) * (iq[:, 0] + 1j * iq[:, 1])
    magnitude = np.abs(np.fft.ifft(z, n=NFFT))
    maximum = float(np.max(magnitude))
    normalized = magnitude / maximum if maximum else magnitude
    return normalized, nordic_ifft_peak(magnitude)


def local_maxima(values: np.ndarray) -> list[int]:
    return [index for index in range(1, len(values) - 1)
            if values[index] >= values[index - 1] and values[index] > values[index + 1]]


def write_csv(path: Path, rows: list[dict]) -> None:
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
    parser.add_argument("--true-distance-m", required=True, type=float)
    parser.add_argument("--max-distance-m", type=float, default=10.0)
    args = parser.parse_args(argv)
    if args.true_distance_m < 0 or args.max_distance_m <= 0:
        parser.error("distances must be positive")
    if args.output_dir.exists() and (not args.output_dir.is_dir() or any(args.output_dir.iterdir())):
        parser.error("output directory must be empty")
    try:
        frames, parse_errors = parse_frames(args.log)
    except OSError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    if not frames:
        print("ERROR: no complete ILRAS1 capture frames", file=sys.stderr)
        return 1

    max_bin = min(NFFT // 2, int(math.ceil(args.max_distance_m / BIN_M)) + 1)
    full_profiles = []
    profiles = []
    official_bins = []
    early_bins = []
    dominant_bins = []
    frame_summaries = []
    for frame in frames:
        profile, official_bin = frame_profile(frame)
        full_profiles.append(profile)
        profiles.append(profile[:max_bin])
        official_bins.append(official_bin)
        dominant_bins.append(int(np.argmax(profile[:NFFT // 2])))
        summary, _, _ = analyze_frame(frame)
        frame_summaries.append(summary)
        early_bins.append(summary["early_peak"])
    matrix = np.asarray(profiles)
    median = np.median(matrix, axis=0)
    q1 = np.quantile(matrix, 0.25, axis=0)
    q3 = np.quantile(matrix, 0.75, axis=0)
    aggregate_peaks = local_maxima(median)

    peak_presence = np.zeros(max_bin, dtype=int)
    for profile in matrix:
        for index in local_maxima(profile):
            peak_presence[index] += 1

    profile_rows = []
    for capture_index, profile in enumerate(matrix):
        for index, value in enumerate(profile):
            profile_rows.append({
                "capture_index": capture_index,
                "bin": index,
                "nominal_distance_m": index * BIN_M,
                "normalized_magnitude": float(value),
                "local_maximum": index in local_maxima(profile),
                "official_selected": index == official_bins[capture_index],
                "early_selected": index == early_bins[capture_index],
                "dominant_selected": index == dominant_bins[capture_index],
            })

    aggregate_rows = []
    for index in range(max_bin):
        aggregate_rows.append({
            "bin": index,
            "nominal_distance_m": index * BIN_M,
            "median_normalized_magnitude": float(median[index]),
            "q1_normalized_magnitude": float(q1[index]),
            "q3_normalized_magnitude": float(q3[index]),
            "local_peak_frame_count": int(peak_presence[index]),
            "local_peak_frame_fraction": float(peak_presence[index] / len(frames)),
            "aggregate_local_maximum": index in aggregate_peaks,
            "official_selected_count": official_bins.count(index),
            "early_selected_count": early_bins.count(index),
            "dominant_selected_count": dominant_bins.count(index),
        })

    true_bin = args.true_distance_m / BIN_M
    direct_low = max(1, int(math.floor(true_bin)) - 1)
    direct_high = min(max_bin - 1, int(math.ceil(true_bin)) + 1)
    direct_indices = list(range(direct_low, direct_high + 1))
    direct_strength = np.max(matrix[:, direct_indices], axis=1)
    direct_peak_frames = sum(any(index in local_maxima(profile) for index in direct_indices)
                             for profile in matrix)
    dominant = sorted(aggregate_peaks, key=lambda index: median[index], reverse=True)[:8]
    if len(matrix) > 1:
        correlations = np.corrcoef(matrix)
        pairwise = correlations[np.triu_indices(len(matrix), 1)]
        median_correlation = float(np.median(pairwise))
    else:
        median_correlation = math.nan

    dominant_distances = np.asarray([
        _distance_for_peak(full_profiles[index], dominant_bins[index])
        for index in range(len(frames))
    ], dtype=float)
    early_distances = np.asarray([row["early_peak_m"] for row in frame_summaries], dtype=float)
    robust_distances = np.asarray([row["robust_phase_slope_m"] for row in frame_summaries], dtype=float)
    cross_method_valid = np.isfinite(dominant_distances) & np.isfinite(robust_distances)
    if np.count_nonzero(cross_method_valid) > 1:
        cross_method_correlation = float(np.corrcoef(
            dominant_distances[cross_method_valid], robust_distances[cross_method_valid])[0, 1])
        cross_method_difference = dominant_distances[cross_method_valid] - robust_distances[cross_method_valid]
        cross_method_median_difference = float(np.median(cross_method_difference))
    else:
        cross_method_correlation = math.nan
        cross_method_median_difference = math.nan
    early_method_valid = np.isfinite(early_distances) & np.isfinite(robust_distances)
    early_method_correlation = (float(np.corrcoef(
        early_distances[early_method_valid], robust_distances[early_method_valid])[0, 1])
        if np.count_nonzero(early_method_valid) > 1 else math.nan)

    candidate_rows = []
    for relative in (0.02, 0.05, 0.10, 0.15, 0.25):
        for noise_sigma in (0.0, 2.0, 5.0):
            distances = []
            bins = []
            for profile in full_profiles:
                peak = early_peak(profile, relative, noise_sigma)
                if peak is not None:
                    bins.append(peak)
                    distances.append(_distance_for_peak(profile, peak))
            finite = [value for value in distances if math.isfinite(value)]
            candidate_rows.append({
                "candidate": "early_peak",
                "relative": relative,
                "noise_sigma": noise_sigma,
                "huber_k": "",
                "valid_frames": len(finite),
                "median_distance_m": float(np.median(finite)) if finite else math.nan,
                "standard_deviation_m": float(np.std(finite, ddof=1)) if len(finite) > 1 else math.nan,
                "mean_absolute_error_m": (float(np.mean(np.abs(np.asarray(finite) - args.true_distance_m)))
                                          if finite else math.nan),
                "selected_bins": json.dumps({str(index): bins.count(index)
                                             for index in sorted(set(bins))}),
            })
    for huber_k in (0.5, 1.0, 1.5, 2.0, 3.0):
        distances = [analyze_frame(frame, huber_k=huber_k)[0]["robust_phase_slope_m"]
                     for frame in frames]
        finite = [value for value in distances if math.isfinite(value)]
        candidate_rows.append({
            "candidate": "robust_phase",
            "relative": "",
            "noise_sigma": "",
            "huber_k": huber_k,
            "valid_frames": len(finite),
            "median_distance_m": float(np.median(finite)) if finite else math.nan,
            "standard_deviation_m": float(np.std(finite, ddof=1)) if len(finite) > 1 else math.nan,
            "mean_absolute_error_m": (float(np.mean(np.abs(np.asarray(finite) - args.true_distance_m)))
                                      if finite else math.nan),
            "selected_bins": "",
        })

    result = {
        "frames": len(frames),
        "parse_errors": len(parse_errors),
        "bin_spacing_m": BIN_M,
        "true_distance_m": args.true_distance_m,
        "true_fractional_bin": true_bin,
        "direct_window_bins": direct_indices,
        "direct_window_median_relative_strength": float(np.median(direct_strength)),
        "direct_window_q1_relative_strength": float(np.quantile(direct_strength, 0.25)),
        "direct_window_q3_relative_strength": float(np.quantile(direct_strength, 0.75)),
        "direct_window_local_peak_frames": direct_peak_frames,
        "direct_window_local_peak_fraction": direct_peak_frames / len(frames),
        "median_profile_pairwise_correlation": median_correlation,
        "approximate_ifft_range_resolution_m": C_M_PER_S / (2 * CHANNELS * 1_000_000),
        "dominant_ifft_vs_robust_phase_correlation": cross_method_correlation,
        "dominant_ifft_minus_robust_phase_median_m": cross_method_median_difference,
        "early_ifft_vs_robust_phase_correlation": early_method_correlation,
        "official_selected_bins": {str(index): official_bins.count(index)
                                   for index in sorted(set(official_bins))},
        "early_selected_bins": {str(index): early_bins.count(index)
                                for index in sorted(set(early_bins))},
        "dominant_selected_bins": {str(index): dominant_bins.count(index)
                                   for index in sorted(set(dominant_bins))},
        "dominant_aggregate_peaks": [
            {"bin": index, "nominal_distance_m": index * BIN_M,
             "median_relative_strength": float(median[index]),
             "local_peak_frame_fraction": float(peak_presence[index] / len(frames))}
            for index in dominant
        ],
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "ifft_profiles.csv", profile_rows)
    write_csv(args.output_dir / "ifft_profile_summary.csv", aggregate_rows)
    write_csv(args.output_dir / "candidate_sweep.csv", candidate_rows)
    write_csv(args.output_dir / "frame_metrics.csv", [
        {
            "capture_index": index,
            "official_bin": official_bins[index],
            "official_ifft_m": frame_summaries[index]["nordic_ifft_recalc_m"],
            "dominant_bin": dominant_bins[index],
            "dominant_ifft_m": dominant_distances[index],
            "early_bin": early_bins[index],
            "early_ifft_m": early_distances[index],
            "robust_phase_m": robust_distances[index],
        }
        for index in range(len(frames))
    ])
    (args.output_dir / "profile_metrics.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8")

    report = [
        "# IL RAS IFFT profile analysis",
        "",
        f"- Complete frames: {len(frames)}",
        f"- True distance: {args.true_distance_m:.3f} m (fractional bin {true_bin:.2f})",
        f"- Direct-window bins: {direct_low}–{direct_high}",
        ("- Direct-window relative strength: median "
         f"{np.median(direct_strength):.3f}, IQR "
         f"{np.quantile(direct_strength, 0.25):.3f}–{np.quantile(direct_strength, 0.75):.3f}"),
        ("- Direct-window local peak presence: "
         f"{direct_peak_frames}/{len(frames)} frames"),
        f"- Median pairwise profile correlation: {median_correlation:.3f}",
        f"- Approximate IFFT range resolution: {C_M_PER_S / (2 * CHANNELS * 1_000_000):.3f} m",
        f"- Dominant IFFT / robust-phase correlation: {cross_method_correlation:.3f}",
        ("- Dominant IFFT minus robust-phase median: "
         f"{cross_method_median_difference:.3f} m"),
        "",
        "## Dominant aggregate peaks",
        "",
        "| Rank | Bin | Nominal distance (m) | Median relative strength | Peak frame fraction |",
        "|---:|---:|---:|---:|---:|",
    ]
    for rank, index in enumerate(dominant, 1):
        report.append(
            f"| {rank} | {index} | {index * BIN_M:.3f} | {median[index]:.3f} | "
            f"{peak_presence[index] / len(frames):.2f} |")
    (args.output_dir / "PROFILE_REPORT.md").write_text("\n".join(report) + "\n", encoding="utf-8")

    try:
        import matplotlib.pyplot as plt

        distances = np.arange(max_bin) * BIN_M
        figure, axes = plt.subplots(2, 1, figsize=(11, 8), constrained_layout=True)
        axes[0].fill_between(distances, q1, q3, color="#8fb9dd", alpha=0.35, label="IQR")
        axes[0].plot(distances, median, color="#1f5f99", linewidth=2, label="median")
        axes[0].axvline(args.true_distance_m, color="#2a9d55", linestyle="--", label="true distance")
        axes[0].set(xlabel="Nominal IFFT distance (m)", ylabel="Relative magnitude",
                    title="Median normalized IFFT profile")
        axes[0].grid(alpha=0.25)
        axes[0].legend()
        image = axes[1].imshow(matrix, aspect="auto", origin="lower", interpolation="nearest",
                               extent=(0, max_bin * BIN_M, -0.5, len(frames) - 0.5),
                               cmap="viridis", vmin=0, vmax=1)
        axes[1].axvline(args.true_distance_m, color="white", linestyle="--", linewidth=1.5)
        axes[1].scatter(np.asarray(official_bins) * BIN_M, np.arange(len(frames)),
                        marker="x", color="#ffb000", s=28, label="official selection")
        axes[1].scatter(np.asarray(dominant_bins) * BIN_M, np.arange(len(frames)),
                        marker="+", color="#e63946", s=24, label="dominant peak")
        axes[1].set(xlabel="Nominal IFFT distance (m)", ylabel="Capture index",
                    title="Per-frame normalized IFFT profile")
        axes[1].legend(loc="upper right")
        figure.colorbar(image, ax=axes[1], label="Relative magnitude")
        figure.savefig(args.output_dir / "ifft_profile.png", dpi=160)
        plt.close(figure)
    except ImportError:
        print("WARNING: matplotlib unavailable; PNG was not generated", file=sys.stderr)

    print(f"Analyzed full IFFT profiles for {len(frames)} frames")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
