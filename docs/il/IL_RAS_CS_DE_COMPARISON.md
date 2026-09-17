# IL Nordic RAS + cs_de比較評価

## 1. 目的

現行ILGAのZephyr `connected_cs`+位相傾き方式と、NCS v3.2.3のNordic公式`channel_sounding_ras_*`+`cs_de`を同じDK・同じ設置条件で比較する。

比較版は現行ファームを修正しない。NCS v3.2.3内の公式RAS Initiator / Reflectorを別buildディレクトリへビルドし、必要時だけ両DKへ書き込む。

## 2. 比較方式

Nordic `cs_de`が出力する次の3方式を同時取得する。

- `ifft`: PBR混合信号のIFFT最大振幅から最短伝搬経路を推定
- `phase_slope`: 周波数に対する位相傾きから推定
- `rtt`: RTT packetの往復時間から推定

公式Initiatorは`cs_de`の全体品質とtone品質が合格した値だけを保存し、直近9 proceduresの中央値をシリアル出力する。IFFTは512-point構成である。

## 3. ビルド

```bash
cd /home/in/work/ILGA
./scripts/il_cs_ras_build.sh
```

生成物:

```text
build/il_cs_ras_initiator/merged.hex
build/il_cs_ras_reflector/merged.hex
```

## 4. 書き込み

最初はdry runでroleとserialを確認する。

```bash
./scripts/il_cs_ras_flash_pair.sh <LOCATOR_SERIAL> <TAG_SERIAL>
```

両DKをUSB接続し、表示されたrole mapと実機ラベルが一致した場合だけ実行する。

```bash
./scripts/il_cs_ras_flash_pair.sh <LOCATOR_SERIAL> <TAG_SERIAL> --execute
```

## 5. ログ取得

RAS比較版は115200 baudを使う。LOCATORのVCOM1を指定する。

```bash
./scripts/il_cs_log_capture.sh initiator \
  /dev/serial/by-id/<LOCATOR_VCOM1> 120 --normal
```

## 6. 解析

```bash
python3 scripts/il_cs_ras_analyze.py \
  logs/il_cs/<LOCATOR_LOG>.log \
  --output-dir analysis_out/il_cs_ras/<TRIAL_NAME> \
  --true-distance-m 0.5
```

生成物:

- `distance_estimates.csv`: IFFT・位相傾き・RTTの時系列
- `summary.json`: 機械処理用集計
- `summary.md`: 確認用集計表

## 7. 初回試験

1. 同じ高さ・同じ向き・見通し条件で0.5 mを120秒取得する。
2. IFFT、位相傾き、RTTの3方式が連続出力されることを確認する。
3. 問題がなければ0.5 / 1.0 / 2.0 / 3.0 mを各3試行取得する。
4. 各方式について距離順序、試行間再現性、中央値誤差、MAEを比較する。

初回smokeの合格条件は、BLE / CS接続が維持され、IFFT / phase slope / RTTがすべて取得でき、fatal errorや再起動がないこととする。距離精度の合否はこのsmokeでは決めない。

## 8. 初回実機smoke結果（2026-09-17）

次の役割で両DKへ比較版を書き込み、45秒間のUARTログを取得した。

- LOCATOR / Initiator: `1057707951`
- TAG / Reflector: `1057727822`
- LOCATORログ: `logs/il_cs/initiator_20260917_112713.log`
- TAGログ: `logs/il_cs/reflector_20260917_112713.log`

LOCATORログから449件の距離推定を解析でき、IFFT・位相傾き・RTTの3方式がすべて連続出力された。fatal error、assert、fault、再起動は検出されなかった。TAG側は接続確立後の取得区間にUART出力がなかったが、LOCATOR側で距離推定が継続したためReflector動作は確認できた。

| 方式 | 件数 | 中央値 (m) | 平均 (m) | 標準偏差 (m) |
|---|---:|---:|---:|---:|
| IFFT | 449 | 1.350 | 1.351 | 0.010 |
| 位相傾き | 449 | 1.380 | 1.370 | 0.028 |
| RTT | 449 | 4.640 | 4.611 | 0.197 |

以上から、初回smokeの合格条件を満たした。今回の設置距離は試験条件として記録していないため、表示値の絶対距離精度は評価対象外とする。次は同じ高さ・向き・見通し条件で0.5 mを120秒取得し、真値付き比較を開始する。
