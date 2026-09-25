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

## 9. 1 m固定のアルゴリズム検証（2026-09-25着手）

既存の0.5 m / 1.0 mの最終推定ログは比較基準として保存する。IQを含まないため、アルゴリズムの再計算には使えない。追加の実測は1.0 m固定で、最初に生データを1回取得する。

診断版は公式RAS Initiatorの作業コピーへ小さなパッチを当てて別buildにする。TAG / Reflectorは公式RAS版を継続使用する。既存の`il_cs_ras_build.sh`、公式build、GA/currentを変更しない。

```bash
./scripts/il_cs_ras_debug_build.sh
# TAGに公式RAS Reflectorが既に書き込まれていることを確認する。
# LOCATORだけ診断版をflashする。シリアル番号は実機ラベルと照合する。
west flash --skip-rebuild -d build/il_cs_ras_debug_initiator --dev-id <LOCATOR_SERIAL>
./scripts/il_cs_log_capture.sh initiator /dev/serial/by-id/<LOCATOR_VCOM1> 120 --normal
python3 scripts/il_cs_ras_debug_analyze.py logs/il_cs/<CAPTURE>.log \
  --output-dir analysis_out/il_cs_ras_debug/one_meter_v1 --true-distance-m 1.0
```

診断版は最大2秒に1回、`cs_de`が使用したlocal/peer IQ、両側tone品質、RTT累積値と件数、公式の未平滑化3方式の推定値を`ILRAS1`形式で出力する。取得したIQは高品質toneの平均値であり、個々のRF測定値ではない。専用の低優先度キューからUARTへ出すため、測定コールバックで全行の出力待ちはしない。UARTは115200 baud、既存ログ取得スクリプトの`--normal`を使う。フレーム末尾には75 tone件数とFNV-1aチェックサムを付け、欠落・途中切断したprocedureは解析対象から除く。取りこぼした抽出スロット数もフレームヘッダへ記録する。

解析器は同一フレームから次を計算し、`procedures.csv`、`tones.csv`、`ifft_peaks.csv`、`summary.json`を出力する。
PC側解析にはPython 3とNumPyが必要である。

- NCS v3.2.3 `cs_de.c`のIFFTピーク選択・位相傾き・RTTの再計算。まずファームの生の推定値との一致を確認する。
- IFFTの上位ピーク、Nordic選択ピーク、ノイズ床と最大ピーク比で判定する最初の有効ピーク。
- 高品質toneと合成振幅で重み付けし、外れ位相の影響を下げる位相傾き。

候補パラメータは`--early-relative`、`--early-noise-sigma`、`--huber-k`で同じログに繰り返し適用できる。1 mの結果だけで採用せず、既存0.5 m / 1.0 m結果との整合と、後日の0.5 m / 2 m各1回で過学習を確認する。診断版のビルド、実機書き込み、UART記録、解析一致はそれぞれ別の検証項目とする。

### 9.1 1 m初回実測結果

2026-09-25にLOCATORへ診断版、TAGへ公式RAS Reflectorを書き込み、同じ高さ・同じ向き・見通しの1.0 m固定配置でUARTを取得した。公式INFOログが`ILRAS1`のtone行へ割り込むことを初回取得で確認したため、診断buildだけ`CONFIG_LOG_OVERRIDE_LEVEL=1`としてINFO表示を抑制した。公式RAS buildとTAGは変更していない。

- ログ: `logs/il_cs/locator_uart_clean_1m.log`
- 解析先: `analysis_out/il_cs_ras_debug/uart_clean_1m/`
- 完全フレーム: 20件
- 完全フレーム内の`missed_capture_slots`: 全件0
- 解析器が報告した24件のframing error: 取得開始時点ですでに送信中だったsample 446の先頭欠落行。以後の完全フレームとは分離して除外した。
- good tone数: 中央値70、範囲61～72

| 方式 | 中央値 (m) | 範囲 (m) | 真値1.0 mとの差 |
|---|---:|---:|---:|
| ファームIFFT | 3.220 | 2.635～3.513 | +2.220 m |
| Nordic相当IFFT再計算 | 3.220 | 2.635～3.513 | +2.220 m |
| 早期IFFTピーク候補 | 4.466 | 4.215～4.871 | +3.466 m |
| ファーム位相傾き | 4.699 | 4.345～5.936 | +3.699 m |
| ロバスト位相傾き候補 | 4.514 | 4.243～5.064 | +3.514 m |
| ファームRTT | 9.941 | 8.229～15.523 | +8.941 m |

ファームIFFTとオフライン再計算の差は各フレームで1 mm未満であり、IQ抽出、固定小数点変換、tone品質除外、Nordic相当IFFT再現は成立した。一方、最初の有効ピーク候補はbin 15～17付近の強いピークを選び、公式IFFTが選ぶbin 9～12よりさらに遠距離となった。ロバスト位相傾きも誤差を小さくできなかった。したがって、早期ピーク候補とロバスト位相候補は現設定のまま採用しない。

今回のIFFT形状だけでは真値1 m付近に安定した独立ピークを確認できず、「公式推定が単純に後方の反射ピークだけを選んでいる」とは断定できない。既取得の開放空間0.5～3.0 m rawでも距離順位が成立していないため、1 mだけのoffset補正や、この1ログへ合わせた閾値調整は行わない。次は保存IQ上でピーク全体と品質指標を比較し、採用可能な候補が得られた場合だけ0.5 m / 2 mを各1回追加取得する。

### 9.2 全IFFTプロファイルとパラメータ掃引

`scripts/il_cs_ras_debug_profile.py`を追加し、同じ20完全フレームについて0～10 m相当の全IFFT bin、フレーム別正規化強度、局所ピーク頻度、公式選択位置、候補パラメータ掃引を出力した。

```bash
python3 scripts/il_cs_ras_debug_profile.py \
  logs/il_cs/locator_uart_clean_1m.log \
  --output-dir analysis_out/il_cs_ras_debug/uart_clean_1m_profile \
  --true-distance-m 1.0 --max-distance-m 10.0
```

主な結果は次のとおり。

- 1.0 mはfractional bin 3.42に相当する。周辺bin 2～5の最大強度は全体最大に対して中央値0.174、IQR 0.153～0.196だった。
- bin 2～5に局所ピークが現れたフレームは0/20件だった。真値付近から選択可能な独立ピークは観測されていない。
- 集約プロファイルの最大はbin 15、nominal 4.391 mだった。幅広い主ローブの頂点はフレームごとにbin 14～17へ移る。
- フレーム間IFFTプロファイル相関の中央値は0.964であり、形状は高い再現性を持つ。
- 75 tones、1 MHz間隔から見た概算距離分解能は約2.0 mである。0.293 m/binは512点ゼロ詰め後の表示間隔であり、独立経路を0.293 m間隔で分離できることを意味しない。
- 支配IFFTピークとロバスト位相距離のフレーム間相関は0.905、両者の中央値差は-0.034 mだった。独立した2方式が約4.5 mの同じ実効遅延を観測している可能性がある。

早期ピークは相対閾値0.02 / 0.05 / 0.10 / 0.15 / 0.25、noise sigma 0 / 2 / 5の15通りを評価した。低閾値では1フレームだけbin 1を拾って標準偏差0.98 mとなり、残りはbin 14～17を選んだ。安定設定の中央値は4.466 mであり、真値付近へ改善する設定はなかった。ロバスト位相はHuber係数0.5 / 1.0 / 1.5 / 2.0 / 3.0を評価したが、中央値は4.512～4.569 mで、閾値調整では系統差を除けなかった。

以上から、真値1 m付近の弱い肩をピークとみなす方式、早期ピークの閾値調整、位相外れ値係数の調整は候補から外す。次の検証仮説は、支配IFFTとロバスト位相が共有する実効遅延を機器・配置校正値として扱い、複数距離で補正可能性を確認する方法とする。ただし1 m一点だけで補正式は採用しない。

### 9.3 0.5 / 1.0 / 1.5 mの校正可能性確認

同じ向き・高さ・見通しを維持し、設置可能範囲の上限を1.5 mとして0.5 / 1.0 / 1.5 mを各1回取得した。解析対象となった完全フレームは順に19 / 20 / 18件だった。1.5 mログには取得開始途中のsampleと別の不完全sampleが含まれたため除外し、完全フレームだけを採用した。

- 0.5 m: `logs/il_cs/locator_uart_clean_0p5m.log`
- 1.0 m: `logs/il_cs/locator_uart_clean_1m.log`
- 1.5 m: `logs/il_cs/locator_uart_clean_1p5m.log`
- 比較結果: `analysis_out/il_cs_ras_debug/static_0p5_1p0_1p5_comparison/`

| 方式 | 0.5 m中央値 | 1.0 m中央値 | 1.5 m中央値 | 距離順序 |
|---|---:|---:|---:|---|
| 公式IFFT | 1.464 m | 3.220 m | 5.114 m | 成立 |
| 公式位相傾き | 3.301 m | 4.699 m | 6.525 m | 成立 |
| 早期IFFTピーク | 2.124 m | 4.466 m | 2.131 m | 不成立 |
| ロバスト位相傾き | 1.721 m | 4.514 m | 5.138 m | 成立 |
| RTT | 5.478 m | 9.941 m | 8.966 m | 不成立 |

0.5 mと1.5 mだけから一次補正を作り、補正に使用していない1.0 mを予測した。公式IFFTは0.981 mで誤差-0.019 m、公式位相傾きは0.934 mで誤差-0.066 mだった。ロバスト位相傾きは1.317 m、RTTは1.779 mであり、早期IFFTピークとRTTは距離順序自体が成立しなかった。

この結果から、現環境では複数フレームの中央値を用いた公式IFFTのgain / offset補正が最有力であり、公式位相傾きは補助指標の候補とする。1.5 mの公式IFFTには経路切替とみられる低値1件があり、単発値ではなく時間窓中央値を使う必要がある。今回の結果は各距離1設置だけなので、製品用の絶対距離精度が成立したとは判定しない。次は同じ3距離の再設置試験で補正係数と1.0 m予測の再現性を確認し、成立した場合に診断用オフライン処理へ暫定校正式と品質判定を実装する。
