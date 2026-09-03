# GA 10 m機能指標オフライン試作

## 目的と責務

`VIEWER/python/walking_analyzer/compute_functional_gait_metrics.py`は、操作員が記録したSTART/FINISHと右足TAGのCSVから、10 m歩行時間・速度・歩行周期・ケイデンス・品質情報を算出するオフラインCLIである。

固定gx候補は解剖学的なinitial contactではない。名称を`stride_marker`（右足歩行周期マーカー）とし、既存`evaluate_gait_events.py`の固定条件を変更せず再利用する。4次Butterworth 5 Hz zero-phase filterを使うため、本番causal detectorではない。

既存`analyze_single_leg_csv.py`、`evaluate_gait_events.py`、Viewer、capture wrapperの既存APIと出力は変更しない。

## 時計の役割分担

3種類を混同しない。

1. `marker_monotonic_ns`
   - PC側BLE受信callbackと同じmonotonic時計で記録された操作キー時刻。
   - STARTからFINISHまでの`marked_duration_sec`と`speed_mps`に使う。
2. `marker_elapsed_ns`
   - センサCSV最初の`rx_monotonic_ns`を0とした操作マーカー相対時刻。
   - CLIは`marker_monotonic_ns - first_rx_monotonic_ns`との一致を検証する。
3. seq 100 Hz grid
   - stride marker間隔、median/mean/SD stride time、cadence、CVに使う。
   - 欠番は既存固定検出器と同じ線形補間を使う。

stride markerをSTART/FINISH内へ絞るときだけ、seq位置をCSVの`rx_monotonic_ns`へ線形補間して同じ時計上で比較する。この推定受信時刻をstride periodの計算には使わない。

## markers CSV

共通schemaは次の通りである。

```text
schema_version,trial_name,event,event_index,marker_monotonic_ns,marker_elapsed_ns,source,notes
```

- `schema_version`: `1`
- `event`: `START`または`FINISH`
- `source`: `operator_key`
- 1 trialにつきSTART/FINISH各1件
- STARTとFINISHはセンサCSVの受信時刻span内にあり、START < FINISHであること

複数trialを1ファイルへ保存した場合は`--trial-name`で対象を選ぶ。1 trialだけなら自動選択する。

START/FINISHの欠落・重複・逆順・時計基準不一致では`quality.status=error`とし、`marked_duration_sec`と`speed_mps`を出さない。センサ閾値から歩行区間を代用しない。

capture側は重複キーを2行目として保存せず、元の行の`notes`へ`duplicate_start_rejected`または`duplicate_finish_rejected`を残す。CLIは行数だけでなくこのnotesとmetadataの`metrics.marker_valid`も検証する。`marker_elapsed_ns`が空欄になる`missing_rx_reference`も入力例外へ潰さず、明示的なquality errorとして出力する。

## 入力と実行例

```bash
python3 VIEWER/python/walking_analyzer/compute_functional_gait_metrics.py \
  logs/ble/S01_10m_normal_01.csv \
  logs/ble/S01_10m_normal_01_markers.csv \
  --trial-name S01_10m_normal_01 \
  --distance-m 10 \
  --metadata-json logs/ble/S01_10m_normal_01_metadata.json \
  --out-dir analysis_out/S01_10m_normal_01_functional
```

metadata JSONは任意である。既存capture metadataの`trial_name`、`actual_steps`、`start_foot`、`end_foot`、`metrics.missing_seq`、`metrics.invalid_sensor_samples`、`metrics.sensor_fault`を利用できる。CLIの`--actual-steps`、`--start-foot`、`--end-foot`はmetadata値より優先する。

metadataに歩数があっても正解値として採用できない試行は`--actual-steps-unknown`を指定する。最終足が不明なら`--end-foot unknown`を指定する。これらはmetadata値を0や推測値へ置換せず、参考歩数評価を無効にする。

装着足は`--sensor-foot right|left|unknown`、またはmetadataの`sensor_foot`で明示する。既存metadataに項目がない場合はRightと仮定せず`unknown`としてwarningを出し、総歩数の参考換算を無効にする。出力名は`right_foot_stride_marker`または`left_foot_stride_marker`のように装着足依存とするが、いずれも解剖学的initial contactではない。

診断レンジの既定値はBMI270の±4 g、±1000 dpsである。別設定のCSVでは`--accel-range-g`と`--gyro-range-dps`を明示する。データからレンジを推測しない。

出力ディレクトリが既に存在する場合は上書きしない。

## 固定stride marker条件

- gx
- seq 100 Hz grid、欠番線形補間
- 4th-order Butterworth LPF 5 Hz、zero-phase
- positive peak 80 dps以上
- prominence 50 dps以上
- minimum distance 0.75秒
- peak後0.30秒以内のpositive-to-negative zero crossing
- zero crossing後0.40秒以内にgx -50 dps以下を確認

閾値はCLI引数にせず、`evaluate_gait_events.py`の定数と`detect_fixed_candidates()`を直接再利用する。

## 指標

### 主要指標

- `marked_duration_sec = (FINISH.marker_monotonic_ns - START.marker_monotonic_ns) / 1e9`
- `speed_mps = distance_m / marked_duration_sec`
- `stride_marker_count`
- median / mean / SD stride time
- `robust_cadence_steps_per_min = 120 / median stride time`
- raw CV
- inlier CV

markerが2個未満ならperiodとcadenceはunavailable、3個未満ならSDとCVはunavailableとする。

inlierはtrial medianの`0.70～1.50倍`を両端含みで残す。raw CVとinlier CVを併記し、都合のよい方だけを表示しない。

`pause_or_missed_marker`は次を両方満たすintervalである。

- interval > 1.50 × trial median
- interval - trial median >= 0.50秒

これは「真の途中停止」と「stride marker欠落」を区別しない品質flagであり、停止判定として公開しない。

### 参考歩数推定

stride marker数だけから総歩数を確定しない。装着足・開始足・終了足がすべて既知の場合だけ、装着足と同じ足で開始・終了なら`2M-1`、どちらも反対足なら`2M+1`、片側だけ同じなら`2M`を`estimated_total_steps_reference_only`として別区画へ出す。`M`は装着足stride marker数である。

- right → right: `2R - 1`
- right → left / left → right: `2R`
- left → left: `2R + 1`

`actual_steps`が入力された場合だけ`estimated_minus_actual_steps`を算出する。これは主要指標ではなく、境界marker欠落を評価する参考値である。

`summary.txt`と`summary.csv`では`stride_marker_count`、`estimated_total_steps_reference_only`、`actual_steps`を別項目として表示する。例えばstride marker 7個から開始・終了足で14歩相当に換算できても、「14歩を検出した」とは表示しない。

### START/FINISH区間端診断

固定検出器がarmedした全候補を`stride_candidates.csv`へ保存し、次を区別する。

- 正式stride markerか、不採用candidateか
- START/FINISH内外
- interval指標へ採用したか
- `before_start`、`after_finish`、`no_zero_cross`、`no_negative_confirmation`等の除外理由
- START/FINISHからの符号付き時差（candidate時刻 - boundary時刻）

`boundary_diagnostics.csv`と`summary.json`にはSTART直前・直後、FINISH直前・直後について、最寄りarmed candidateと最寄りformal stride markerを別々に出す。正は境界後、負は境界前である。この診断は区間端の見落としを可視化するが、区間外markerを自動で歩数へ追加しない。

## Railの区間分離

`sensor_quality.range_by_interval`は`full`、`marked_interval`、`outside_marked_interval`ごとにsample数、rail/near-rail件数、割合を出す。周期・時間指標への影響と衝撃振幅への影響は分けて判断し、この診断だけでTAGレンジを自動変更しない。

## 低速候補の比較

`compare_stride_marker_candidates.py`は固定ベースラインを変更せず、zero-cross確認窓0.35/0.40/0.45秒、+10 dps crossing、手カウントrateが90 steps/min未満の場合だけ0.45秒にする案、-50 dps直接crossingを横並び比較する。出力するrecall/FP/FNは期待装着足接触数に対する件数proxyであり、時刻同期Ground Truthによるprecision/recallではない。比較結果が良くても、本番条件への採用には独立被験者・速度別・時刻GT・causal条件での再評価が必要である。

## TUG

`--test-type tug`ではoperator START/FINISH間の総時間だけを正式出力し、距離・速度・stride marker・立ち上がり・旋回・着座phaseを算出しない。複数試行は`summarize_tug_trials.py`で平均、標本SD、CV、最小、最大、範囲を集計する。足部TAGだけでseat-offまたはseat-contactを確定しない。

## 品質情報

`summary.json`の`quality`へerrorとwarningを分けて保存する。

- marker欠落、重複、逆順、時計不一致、センサ受信span外
- capture notesの重複拒否・受信時計基準欠落とmetadataの`marker_valid=0`
- distanceが0以下または非有限
- seq欠番
- 同一受信timestamp（束ね受信候補）と受信timestamp逆転
- stride marker近傍の欠番
- rail / 95% near-rail
- CSV内の全軸加速度ゼロ割合
- metadataの`invalid_sensor_samples`と`sensor_fault`
- interval外れ
- `pause_or_missed_marker`
- marker不足によるperiod/SD/CV unavailable

非数値・非有限センサ行、必要列欠落、95%以上の全軸加速度ゼロ、不正marker schemaは入力エラーとしてexit code 2で拒否し、出力ディレクトリを生成しない。品質errorをsummaryへ保存できるmarker欠落等もexit code 2とする。warningだけなら指標とともにexit code 0で完了する。

同一`rx_monotonic_ns`は束ね受信候補としてwarningにする。逆転はseq eventを受信時計へ写像できないためquality errorとし、START/FINISHによる時間・速度は保持するがstride metricsはunavailableとする。

## 出力

- `summary.json`: 全設定、時計の役割、指標、品質、metadata
- `summary.csv`: 主要値を1行へ平坦化した機械可読summary
- `summary.txt`: 人が確認する短いsummary
- `stride_candidates.csv`: 不採用を含むarmed candidate、境界時差、採否、除外理由
- `stride_markers.csv`: 全正式stride markerとSTART/FINISH内外、欠番近傍
- `stride_intervals.csv`: interval、median比、inlier、外れ、pause/missed flag
- `boundary_diagnostics.csv`: START/FINISH前後の最寄りcandidate/formal marker

## TUGとの境界

本CLIはTUGを実装しない。同じSTART/FINISH schemaは将来、外部操作マーカー間のTUG総時間に再利用できる。ただし足部TAGだけで臀部離床、臀部接触、立ち上がり完了、着座完了を確定しない。TUG状態推定は実測データと外部Ground Truth取得後の別作業とする。
