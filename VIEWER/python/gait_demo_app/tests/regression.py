"""Replay actual 2026-09-03 right-foot 10m logs, compare the unchanged analyzer."""
import hashlib
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import reporting

APP = Path(__file__).resolve().parents[1]
REPO = APP.parents[2]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    root = reporting.new_run(APP / "runs", "regression")
    results = []
    for name in ("marker_pilot_03", "slow_right_01", "fast_right_01"):
        stem = "20260903_ga_phase1a_S01_10m_" + name
        sources = [REPO / "logs" / "ble" / (stem + suffix)
                   for suffix in (".csv", "_markers.csv", "_metadata.json")]
        before = [digest(p) for p in sources]
        run = reporting.offline_run(root, *sources)
        actual = json.loads((run / "analysis.json").read_text(encoding="utf-8"))
        baseline = reporting.functional.run(reporting.functional.parse_args([
            str(sources[0]), str(sources[1]), "--metadata-json", str(sources[2]),
            "--out-dir", str(root / ("baseline_" + name)), "--sensor-foot", "right",
        ]))
        for key in ("timing", "stride_metrics", "reference_step_evaluation", "quality"):
            assert actual[key] == baseline[key], (name, key)
        assert actual["demo"]["quality"] != "計算不可", actual
        assert actual["demo"]["actual_steps"] == baseline["reference_step_evaluation"]["actual_steps"]
        for filename in ("sensor.csv", "markers.csv", "metadata.json", "analysis.json", "waveform.png", "report.html"):
            assert (run / filename).stat().st_size > 0, filename
        assert (run / "waveform.png").read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
        assert "右足" in (run / "report.html").read_text(encoding="utf-8")
        assert before == [digest(p) for p in sources], "Source data changed"
        results.append({"trial": name, "duration_sec": actual["demo"]["duration_sec"],
                        "stride_candidates": actual["demo"]["right_stride_candidates"],
                        "actual_steps": actual["demo"]["actual_steps"],
                        "reference_steps": actual["demo"]["estimated_steps"], "output": str(run),
                        "source_hashes": before})
    reporting.write_json(root / "regression.json", {"passed": True, "trials": results})
    print(json.dumps({"passed": True, "results": results, "output": str(root)}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
