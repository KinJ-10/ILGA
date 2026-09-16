# ILGA 現在状態

更新日: 2026-09-16

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
- IL-Phase 0として、nRF54L15 DK 2台用のLOCATOR/InitiatorとTAG/ReflectorをNCS v3.2.3 `connected_cs`基準で追加した。
- Phase 0の両アプリは`build/il_cs_initiator`と`build/il_cs_reflector`でbuild成功済み。2026-09-11にnRF54L15 DK 2台へflashし、BLE接続、CS設定、約0.99秒周期の反復距離出力を確認した。
- 先頭10秒を除く評価で、0.5 m正対・見通しの60秒×3試行はPBR中央値2.393 m、事象率2.65%、接続断0だった。同一机上のおおよそ1 m・60秒×3試行はPBR中央値2.255 m、事象率3.40%、接続断0で、距離順序が逆転した。
- 約1 m測定後に0.5 mへ戻すA-B-A試験では、戻りAのPBR中央値が2.372 mとなり、初回Aとの差は-0.021 mだった。配置変更に反応する可能性はあるが、反応方向が逆で分布も重なるため、現条件での0.5 m／約1 m距離識別は未成立。現ケーブルでは1 m超の試験が難しく、次回は給電・配置方法と生データ取得を見直す。
- Initiator側USBがWSLから外れた際、Initiatorだけの再接続ではCS距離出力が復旧せず、両DKの同時リセットで復旧した。自動再接続は未実装。
- ILCS2 raw診断版は230400 baud、32 channels、6 ms pacingで120秒smokeを通過し、reset後124 proceduresでchecksum、連番、parse、firmware、512 byte overflowの各エラー0件を確認した。
- 2026-09-14の旧机raw A-B-Aは0.5 mが5.194 / 0.412 m、1.0 mが4.003 mで再現性と距離順序が成立しなかった。机変更後のA-B-Aは0.5 mが1.712 / 1.699 m、1.0 mが2.337 mとなり、環境変更後に改善した。
- 新しい机で0.50 / 0.75 / 1.00 mを各60秒×3試行した結果、PBR合算中央値は1.389 / 1.929 / 2.220 mで単調増加し、457/458 proceduresが有効だった。固定配置の相対距離識別と試行間再現性は合格、絶対距離精度は不合格、補正と再設置再現性は未成立・未評価とする。
- 近距離サンプルベース技術検証は条件付き合格。IL Phase 0全体は遠距離、遮蔽、向き、動的、境界、復帰試験が未実施のため継続する。成果は`analysis_out/il_cs_raw/20260914_env2_distance_series_summary/`に保存した。
- 2026-09-16にTAGをモバイルバッテリー給電とした見通し環境で0.5 / 1.0 / 2.0 / 3.0 mを各60秒×3試行した。採用12試行のPBRは609/612 procedures（99.5%）が有効で、firmware errorは0件だった。
- 基本PBR中央値は0.383 / 7.991 / 6.441 / 6.662 mとなり、真距離とのSpearman順位相関は0.40で基準0.90を満たさなかった。別方式の円周位相探索、外れchannel除去、0.5 m基準のchannel補正でも改善せず、0.5～3.0 m距離順序と絶対距離精度は不合格、ゾーン判定への移行は保留とした。
- 採用外の2.0 m 2試行と3.0 m 1試行でUART record破損を確認した。距離推定と分離し、raw pacing等のsmoke試験を行う。正式結果は`docs/il/results/20260916_open_space_static/`へ保存する。
- 実機試験手順と合否基準は`docs/il/IL_PHASE0_CHANNEL_SOUNDING.md`を参照する。

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
8. IL-Phase 0はCS通信・raw観測まで成立したが、0.5～3.0 m静止距離の順位評価は不合格。推定方式、UART完全性、再設置、遮蔽、向き、動的、境界、復帰試験は未完了。
9. IL-Phase1のゾーン判定、LOCATOR統合、Viewerエリア設定は未実装。静止距離順位が成立するまで実装を保留する。

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
