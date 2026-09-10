"""Reporting adapter for the unchanged ILGA functional gait analyzer."""
from __future__ import annotations

import csv
import html
import json
import shutil
import sys
import uuid
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "walking_analyzer"))
import compute_functional_gait_metrics as functional

NOTICE = (
    "全歩を直接検出しているわけではなく、右足イベントから総歩数を参考換算しています。"
    "検出点は解剖学的な厳密な接地時刻ではありません。"
    "個々の歩幅は推定しません。医療診断には使用しません。"
)
PROFILE = "right_dorsum_current_orientation"
MESSAGES = {
    "missing_seq": "通信中にデータの抜けがあります。",
    "rx_timestamp_duplicate_or_batched": "複数データがまとめて届いた時刻があります。",
    "rx_timestamp_reverse": "受信時刻の順序に問題があり、結果を確認できません。",
    "rx_elapsed_mismatch": "受信時刻の基準にずれがあります。",
    "sensor_rail": "センサーの測定範囲を超えた可能性があります。",
    "sensor_near_rail": "測定範囲の上限に近い動きがあります。",
    "capture_invalid_sensor_samples": "正常でないセンサーデータを除外しました。",
    "capture_sensor_fault": "センサーが正常に動作していない可能性があります。",
    "stride_marker_count_lt_2_period_unavailable": "右足イベントが少なく、周期と歩行テンポを計算できません。",
    "stride_marker_count_lt_3_sd_cv_unavailable": "右足イベントが少なく、ばらつきを計算できません。",
    "stride_interval_outlier": "右足イベント間隔に大きく異なる区間があります。",
    "pause_or_missed_marker": "停止またはイベントの見逃しと思われる区間があります。",
    "stride_marker_near_missing_seq": "イベントの近くにデータの抜けがあります。",
    "marker_missing_start": "STARTが記録されていません。",
    "marker_missing_finish": "FINISHが記録されていません。",
    "marker_outside_sensor_rx_span": "STARTまたはFINISHの時刻が受信データの範囲外です。",
    "marker_order_invalid": "STARTとFINISHの順番を確認してください。",
    "marker_elapsed_clock_mismatch": "マーカーと受信データの時刻基準が一致しません。",
    "metadata_marker_invalid": "開始・終了マーカーが揃っていないため歩行区間を確認できません。",
}


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def new_run(root: Path, label: str = "walk") -> Path:
    run = root / (datetime.now().strftime("%Y%m%d_%H%M%S_") + label + "_" + uuid.uuid4().hex[:8])
    run.mkdir(parents=True, exist_ok=False)
    return run


def view_model(summary: dict, metadata: dict) -> dict:
    timing = summary.get("timing", {})
    stride = summary.get("stride_metrics", {})
    reference = summary.get("reference_step_evaluation", {})
    count = stride.get("stride_marker_count")
    valid = summary.get("quality", {}).get("status") != "error"
    actual = reference.get("actual_steps")
    estimated = reference.get("estimated_total_steps_reference_only")
    method = reference.get("estimation_formula")
    warnings = [MESSAGES.get(code, "データに確認が必要な項目があります。詳細は解析JSONを参照してください。")
                for code in summary.get("quality", {}).get("flags", [])]
    # Unknown boundary feet are deliberately a separate, explicit demo approximation.
    if valid and count and estimated is None:
        estimated = 2 * count
        method = "右足候補数×2（開始足・終了足が不明のため概算）"
        warnings.append("開始足・終了足が不明です。総歩数は右足候補数×2の概算で、境界の1歩などは補正していません。")
    if not valid or not count:
        estimated = None
    sync = metadata.get("sync_observed", {})
    if any(sync.get(side) is None or sync.get(side, 0) < 3 for side in ("pre", "post")):
        warnings.append("同期用足踏みが前後各3回に不足、または未確認です。解析は継続しました。足踏み回数は自己申告で、自動検出ではありません。")
    if metadata.get("capture_error"):
        warnings.append(metadata["capture_error"])
    if metadata.get("actual_steps_source") == "ui_observation":
        actual = metadata.get("actual_steps")
    if actual and estimated is not None and actual != estimated:
        warnings.append(f"実測歩数{actual}歩と参考換算{estimated}歩に差があります。見逃し等を含む参考値として扱ってください。")
    return {
        "duration_sec": timing.get("marked_duration_sec") if valid else None,
        "speed_mps": timing.get("speed_mps") if valid else None,
        "right_stride_candidates": count if valid else None,
        "estimated_steps": estimated,
        "estimate_method": method,
        "actual_steps": actual,
        "step_length_measured_m": 10 / actual if valid and actual else None,
        "step_length_reference_m": 10 / estimated if valid and estimated and estimated > 0 else None,
        # Mean stride length derived from speed * mean right-foot cycle, not 10/count.
        "stride_length_reference_m": timing["speed_mps"] * stride["mean_stride_time_sec"]
            if valid and timing.get("speed_mps") is not None and stride.get("mean_stride_time_sec") else None,
        "cadence": stride.get("robust_cadence_steps_per_min") if valid else None,
        "cv_pct": stride.get("inlier_cv_pct") if valid else None,
        "raw_cv_pct": stride.get("raw_cv_pct") if valid else None,
        "mean_stride_time_sec": stride.get("mean_stride_time_sec") if valid else None,
        "sample_count": summary.get("sensor_quality", {}).get("sample_count", 0),
        "missing_seq_count": summary.get("sensor_quality", {}).get("missing_seq_count", 0),
        "quality": "計算不可" if not valid else ("注意あり" if warnings else "確認項目なし"),
        "warnings": list(dict.fromkeys(warnings)),
    }


def render_report(run: Path, summary: dict, metadata: dict) -> None:
    view = summary["demo"]
    fields = [
        ("歩行時間", "duration_sec", "秒"), ("歩行速度", "speed_mps", "m/s"),
        ("右足の歩行周期検出数", "right_stride_candidates", "件"),
        ("参考換算の総歩数", "estimated_steps", "歩"),
        ("実測歩数（任意入力）", "actual_steps", "歩"),
        ("平均歩幅・実測歩数由来（10m÷実測歩数）", "step_length_measured_m", "m"),
        ("平均歩幅・参考（10m÷参考総歩数）", "step_length_reference_m", "m"),
        ("平均歩行周期距離・参考（速度×平均歩行周期時間）", "stride_length_reference_m", "m"),
        ("歩行テンポ・右足周期由来", "cadence", "歩/分"),
        ("歩行周期のばらつき", "cv_pct", "%"),
        ("歩行周期のばらつき・全区間", "raw_cv_pct", "%"),
        ("平均歩行周期時間", "mean_stride_time_sec", "秒"),
        ("受信サンプル", "sample_count", "件"), ("データ欠損", "missing_seq_count", "件"),
    ]
    cards = []
    for label, key, unit in fields:
        value = view[key]
        text = "未入力" if value is None and key == "actual_steps" else ("解析できません" if value is None else (str(value) if isinstance(value, int) else f"{value:.3f}") + " " + unit)
        cards.append(f"<li><small>{label}</small><strong>{text}</strong></li>")
    warnings = "".join("<li>" + html.escape(w) + "</li>" for w in view["warnings"])
    title = html.escape(str(metadata.get("trial_name", run.name)))
    state_label = {"計算不可": "解析できません", "注意あり": "確認事項あり", "確認項目なし": "良好"}[view["quality"]]
    report = f"""<!doctype html><html lang="ja"><meta charset="utf-8">
<meta name="viewport" content="width=device-width"><title>{title} | ILGA</title>
<style>body{{font-family:system-ui,sans-serif;color:#183449;background:#f5f8fa;max-width:1120px;margin:32px auto;padding:16px}}
h1{{font-size:26px}}.cards{{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px;padding:0;list-style:none}}
.cards li{{background:white;padding:20px;border:1px solid #dbe5ea;border-radius:12px}}strong{{display:block;margin-top:12px;font-size:24px}}
.notice{{padding:18px;background:#fff4dc;border-radius:8px}}img{{width:100%;background:white}}a{{color:#006581}}</style>
<h1>{title}</h1><p>装着：右足の足甲</p>
<h2>波形と区間</h2><p>横軸は最初の受信からの経過秒。START・FINISHと採用された右足候補を表示します。</p>
<img src="waveform.png" alt="加速度・角速度、歩行区間と右足イベントの波形">
<h2>レポート指標</h2><ul class="cards">{''.join(cards)}</ul>
<p>周期の揃い具合を割合で示します。小さいほど一定です。</p>
<p>総歩数の換算方法：{html.escape(view.get('estimate_method') or '解析できません')}</p>
<h2>測定状態：{state_label}</h2><h3>確認事項</h3><ul>{warnings}</ul><p class="notice">{NOTICE}</p>
<p><a href="sensor.csv">測定CSV</a> · <a href="markers.csv">マーカー</a> ·
<a href="metadata.json">測定条件</a> · <a href="analysis.json">解析JSON</a> · <a href="waveform.png">PNG保存</a></p></html>"""
    (run / "report.html").write_text(report, encoding="utf-8")


def render_waveform(run: Path, summary: dict) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    with (run / "sensor.csv").open(encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    if rows:
        first = int(rows[0]["rx_monotonic_ns"])
        times = [(int(r["rx_monotonic_ns"]) - first) / 1e9 for r in rows]
        for axis, names, scale, label in (
            (axes[0], ("ax_mg", "ay_mg", "az_mg"), 1, "Acceleration [mg]"),
            (axes[1], ("gx_mdps", "gy_mdps", "gz_mdps"), 1000, "Angular velocity [dps]"),
        ):
            for name in names:
                axis.plot(times, [float(r[name]) / scale for r in rows], label=name, linewidth=.65)
            for key, color in (("start_marker_monotonic_ns", "green"), ("finish_marker_monotonic_ns", "red")):
                mark = summary.get("timing", {}).get(key)
                if mark is not None:
                    axis.axvline((mark - first) / 1e9, color=color, label=key.split("_")[0].upper())
            axis.set_ylabel(label)
            axis.grid(alpha=.2)
        events_path = run / summary.get("analysis_directory", "") / "stride_markers.csv"
        if events_path.is_file():
            with events_path.open(newline="", encoding="utf-8") as handle:
                for event in csv.DictReader(handle):
                    if event["adopted_as_stride_marker"] == "True":
                        axes[1].axvline(float(event["event_marker_elapsed_sec_estimate"]), color="#111", alpha=.35, linestyle=":")
    else:
        axes[0].text(.5, .5, "No received samples", transform=axes[0].transAxes, ha="center")
    for axis in axes:
        if axis.lines:
            axis.legend(loc="upper right", ncol=3, fontsize=8)
    axes[1].set_xlabel("PC receive elapsed time [s] — dotted: adopted right-foot candidates")
    fig.tight_layout()
    fig.savefig(run / "waveform.png", dpi=140)
    plt.close(fig)


def analyze_run(run: Path, metadata: dict) -> dict:
    directory = "analysis_" + uuid.uuid4().hex[:8]
    try:
        args = functional.parse_args([
            str(run / "sensor.csv"), str(run / "markers.csv"), "--out-dir", str(run / directory),
            "--metadata-json", str(run / "metadata.json"), "--test-type", "walk10m",
            "--distance-m", "10", "--sensor-foot", "right",
        ])
        summary = functional.run(args)
        summary["analysis_directory"] = directory
    except (ValueError, OSError, KeyError) as exc:
        summary = {"quality": {"status": "error", "flags": []}, "diagnostic_error": str(exc)}
        metadata["capture_error"] = metadata.get("capture_error") or "解析に必要なデータまたはマーカーが不足・不正です。保存データを確認し、測り直してください。"
    summary["demo"] = view_model(summary, metadata)
    write_json(run / "analysis.json", summary)
    render_waveform(run, summary)
    render_report(run, summary, metadata)
    return summary


def offline_run(root: Path, sensor: Path, markers: Path, metadata_path: Path) -> Path:
    metadata = json.loads(metadata_path.read_text(encoding="utf-8-sig"))
    if str(metadata.get("test_type", "")).lower() != "walk10m" or float(metadata.get("distance_m", 0)) != 10:
        raise ValueError("このアプリは10m歩行専用です。")
    if str(metadata.get("sensor_foot", "right")).lower() != "right":
        raise ValueError("右足以外の装着データはv0.1の対象外です。")
    run = new_run(root, "offline")
    for source, name in ((sensor, "sensor.csv"), (markers, "markers.csv"), (metadata_path, "source_metadata.json")):
        shutil.copy2(source, run / name)
    metadata["demo_profile"] = PROFILE
    metadata["offline_import"] = True
    metadata["profile_assumption"] = "右足の足甲・現在の向きは利用者が確認（自動判定なし）"
    write_json(run / "metadata.json", metadata)
    analyze_run(run, metadata)
    return run
