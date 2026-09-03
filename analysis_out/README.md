# analysis_out 案内

このディレクトリには、歩行・BLE・動画解析の生成成果を保存する。原本CSVや動画、既存成果は参照関係を保つため、整理目的で移動・削除しない。

## 現在の正式参照先

- `ga_functional_metrics_improvement_20260903_v2/`
  - 2026-09-03取得データの最終解析
  - 低速候補比較、左右装着評価、高速rail評価、TUG総時間集計を含む
- `ga_functional_metrics_pilot_20260902/`
  - 10 m marker pilot 01/02の境界診断と比較
- `ga_functional_metrics_phase1_20260901/`
  - GA機能指標Phase 1の全体再評価

## 途中版・個別確認

- `ga_functional_metrics_improvement_20260903/`
  - `_v2`作成前の途中版。最終判断には使用しない
- `ga_functional_metrics_20260903/`
  - 2026-09-03各試行の初回個別解析
- `S01_10m_marker_pilot_01_functional/`
- `S01_10m_marker_pilot_02_functional/`
- `S01_marker_pilot_01_functional/`

## GA Phase 1履歴

- `ga_phase1a_initial_20260825/`: Phase 1-a初期解析
- `ga_phase1a_20260827_Kin/`: Kin試行の統合解析
- `ga_phase1a_20260827_doi/`: doi試行の統合解析
- `ga_phase1a_sync_20260828_S01/`: 動画同期解析
- `ga_phase1a_video_only_20260828_S01/`: 動画のみの確認成果
- `ga_phase1a_short_20260901_S01/`: 短距離・高fps確認成果
- `20260827_ga_phase1a_*`: 2026-08-27の個別試行解析
- `ble_gap_investigation_20260827/`: BLE欠落調査

## Legacy

2026年3月作成の次の成果は過去検討用として保持する。

- `run01/`
- `ble_walk_00/`, `ble_walk_01/`, `ble_walk_01_retry/`, `ble_walk_02/`
- `0312_12m_r_kaneda00/`, `0312_12m_r_kaneda01/`, `12m_単脚/`

## 運用ルール

- 最終版はフォルダ名へ日付と、必要に応じて`v2`などの版を含める。
- 途中版は削除せず、最終版をこのREADMEへ明記する。
- 生データは`logs/ble/`を正本とする。
- このREADMEを除き、`analysis_out/`の生成成果はGitへコミットしない。
- 再現スクリプトやmanifestが参照する可能性があるため、既存フォルダ名を変更しない。
