# ILGA 現在状態

更新日: 2026-08-07

## 基準

- リポジトリ: `KinJ-10/ILGA`
- 基準ブランチ: `main`
- 再構築時に確認した基準コミット: `289e7fc4191912691ebd610eb259bc22f991cf32`
- システム仕様の上位基準: `ILGA 共通基準文書 2026-03-16`
- 歩行解析の参考資料: `加速度・角速度を用いた歩行解析手法とILGA評価指標の体系的レビュー`

## 確認済みの成果

### TAG

- 現行作業対象は `TAG/zephyr_apps/current/nrf54l15_port`。
- ボードは `nrf54l15dk/nrf54l15/cpuapp`。
- nRF54L15 DKでbuild、flash、起動ログ取得まで確認済み。
- BMI270の初期化、加速度・角速度取得、UART CSV出力を確認済み。
- BMI270配線は `P1.11 = SCL`、`P1.12 = SDA`。
- I2Cは `i2c21` を使用。
- BMI270初期化時のbuffer不足は以下で解消済み。
  - `zephyr,concat-buf-size = <260>`
  - `zephyr,flash-buf-max-size = <260>`
- BLE advertising、ACC/GYR notify、WindowsでのCSV保存まで確認済み。
- BLE切断後のadvertising再開と再接続を確認済み。
- BLE payloadとCSVの単位は加速度 `mg`、角速度 `mdps`。

### VIEWER / ログ取得

- Windows側のBLE受信は `recv_bmi270_ble_notify_cli.py` を正式なログ取得経路とする。
- 保存CSV列は以下。
  - `seq,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps`
- `realtime_plot_ble_v7.py` では実受信レートが100 Hz未満でも表示窓が空にならないよう、到着時刻を描画時刻に使用する。
- 実歩行ログの保存先は `logs/ble/` を推奨する。

### GA

- `analyze_single_leg_csv.py` で片足歩行CSVのオフライン解析が可能。
- 現在の出力は `summary.json`、`step_events.csv`、任意の `diagnostic.png`。
- 現在の基本指標は歩数、ケイデンス、平均ステップ間隔、ステップ間隔CV、周期性スコア。
- diagnostic plotには未加工のX/Y/Z加速度・角速度を表示する。

### IL / LOCATOR

- Phase1はCS主体、単一LOCATOR、約1秒周期の検知エリア侵入・離脱通知を成立条件とする。
- AoA統合と1 m級の連続位置表示はPhase1の成立条件に含めない。
- 現リポジトリのLOCATORはREADMEのみで、実装は未着手。

## 開発環境

### WSL側

- Windows + WSL2 Ubuntu。
- nRF Connect SDK: v3.2.3。
- NCS workspace既定位置: `~/ncs/ncs-v3.2.3`。
- NCS Python venv既定位置: `~/ncs/.venv`。
- west: 1.5.0を使用した実績あり。
- build / flash / UARTログ取得はWSLで行う。

### Windows側

- Python 3.12。
- Bleak 2.1.1での動作実績あり。
- BLE central、notify受信、CSV保存はWindows PowerShellで行う。
- リアルタイム表示にはBleak、Matplotlib、NumPyを使用する。

## 未完了・未確認

確認済み成果と混在させないため、以下は完了扱いにしない。

1. BMI270は100 Hzに設定し10 ms周期でpollしているが、BLE実受信レートは約20 Hzとなるケースがある。実効100 Hzの安定化と欠損率評価が必要。
2. 長時間連続動作時のTAG、BLE再接続、CSV保存の安定性は未確認。
3. BMI270の厳密な較正評価は未実施。
4. 右足装着での実歩行試験と、手カウントとの歩数整合確認が必要。
5. `step_events.csv` へのイベント時X/Y/Z加速度・角速度の追加は未実装。diagnostic plotへの表示のみ完了。
6. TAG `main.c` のdeprecated macro warning 1件は未対応。
7. GA-Phase1Bの5 m / 10 m試験、歩行速度出力、GA-Phase1CのTUG総時間は未実装。
8. IL-Phase1のCS測距、ゾーン判定、LOCATOR、Viewerエリア設定は未実装。

## 次の実装順

1. Desktop / CLI共通の作業コピーと環境を確認する。
2. 30秒BLE取得で実効サンプリングレート、欠損seq、再接続を測定する。
3. 実効100 Hz化または低レートプロファイル化の方針を決める。
4. 右足直線歩行ログを取得し、手カウントと解析結果を比較する。
5. `step_events.csv` にイベント時の未加工6軸値を追加する。
6. Viewer連続保存と長時間試験を行う。
7. GA-Phase1Bへ進む。

## 完了判定の注意

- 実機が必要な項目は、Desktop上でコード確認できただけでは完了にしない。
- build成功、flash成功、起動ログ、BLE受信、解析結果を分けて記録する。
- サンプリング周波数は設定値ではなく、CSVのseq数と実時間から求めた実効値も記録する。
