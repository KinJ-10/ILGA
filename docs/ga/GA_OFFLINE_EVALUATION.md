# GA 同期Ground Truthオフライン評価

## 用途

`evaluate_gait_events.py`は、右足TAGのセンサCSV、動画Ground Truth CSV、同期対応点CSVを使い、Kin/doi解析で固定したgx候補を評価する。入力データを見た自動閾値調整は行わない。

本ツールは4次Butterworth 5 Hz zero-phase filterを使うoffline評価専用ツールであり、本番event detectorではない。未来サンプルを使うため、結果にはcausal実装の遅延が含まれない。

## 入力

### センサCSV

最低限`seq`と`gx_mdps`が必要。seqを100 Hz sample gridとして使い、欠番だけを線形補間する。`rx_monotonic_ns`はBLE到着品質の診断列で、センサイベント時刻への変換には使わない。

### Ground Truth CSV

`templates/ground_truth_template.csv`をコピーしてtrialごとに記入する。

- 列: `trial,event_index,foot,video_frame,video_time_sec,event_type,confidence,notes`
- `event_type`: `normal_contact`, `terminal_contact`, `adjustment`, `turn`
- 左右の接地をすべて記録する。主評価のpositive GTは右足の`normal_contact`と`terminal_contact`。
- `adjustment`と`turn`はpositive GTではなく、その近傍の未対応候補を種別別false positiveとして数えるための注釈。
- `video_time_sec`は動画の実時間軸を使う。固定fps動画なら`video_frame / video_fps`と整合することを確認する。

### 同期対応点CSV

`templates/sync_points_template.csv`へ、開始時と終了時の同期踏み込み各3回について動画フレーム、動画時刻、対応seqを記入する。末尾の`phase`列は開始側を`pre`、終了側を`post`とし、標準手順は計6点とする。

`phase`を持たない旧CSVは列構成として読み込み、評価区間より前の点をpre、後の点をpostとして推定する。ただし開始3点しかない旧データはpost不在およびspan不足でエラーとなる。

ツールは`video_time_sec = slope * sensor_seq_time_sec + intercept`を最小二乗fitし、`sync_fit.csv`へ各点の残差を出す。次を同期品質の必須条件とする。

- preとpostが各1点以上ある。
- pre同期動作は評価開始前、post同期動作は評価終了後にある。
- 同期点の最小〜最大時間spanが、動画時間・sensor時間の両方で評価区間の90%以上を覆う。
- 最大fit残差が20 ms以下。120 fps以上では「2フレームまたは20 msの大きい方」が20 msとなる。

いずれかを満たさない場合は、正式な指標出力前に明確なエラーとする。成功時はcoverage比、pre/post点数、残差、旧phase推定の有無、標準6点を満たしたかを`summary.json`へ記録する。pre/postが各3点未満でも最低条件を満たす旧データは評価できるが、summaryへ警告を残し、明日の標準手順には使わない。

### 評価境界

CLIで次の3時刻を動画時間として明示する。

- `--eval-start-video-sec`: 同期踏み込みと開始前静止を除いた歩行評価開始。
- `--terminal-start-video-sec`: 通常歩行から終端状態へ移る境界。この時刻以降の正式候補をterminal集計へ入れる。
- `--eval-end-video-sec`: 終了後静止や戻り歩行を除く評価終了。

境界はGT接地に都合よく候補ごとに動かさず、動画上の10 m区間と終端条件からtrial単位で決め、記録へ残す。

## 実行例

```bash
python3 VIEWER/python/walking_analyzer/evaluate_gait_events.py \
  logs/ble/20260827_example_right_walk1.csv \
  annotations/20260827_example_walk1_ground_truth.csv \
  annotations/20260827_example_walk1_sync_points.csv \
  --trial 20260827_example_normal_left_adjustment_1 \
  --eval-start-video-sec 6.20 \
  --terminal-start-video-sec 15.10 \
  --eval-end-video-sec 17.00 \
  --tolerance-sec 0.10 \
  --out-dir analysis_out/ga_eval_20260827_example_walk1
```

出力先が既に存在する場合は上書きせず終了する。既存評価結果を消さず、trialごとに新しい出力先を使う。

## 固定検出条件

- gx
- seq 100 Hz grid、欠番線形補間
- 4th-order Butterworth LPF 5 Hz、zero-phase
- positive peak 80 dps以上
- prominence 50 dps以上
- minimum distance 0.75秒
- peak後0.30秒以内の最初のpositive-to-negative zero crossingをイベント時刻とする
- zero crossing後0.40秒以内にgx -50 dps以下を確認

これらの値はCLI引数にせず固定している。夕方データを見て変更しない。

## 対応付けと指標

normalとterminalを別々に、時系列順を保った最大対応数・最小合計絶対誤差の一対一対応を行う。初期許容窓は±100 ms。

- TP: 許容窓内で一対一対応した正式候補と右足GT。
- FP: 対応しなかった正式候補。
- FN: 対応しなかった右足GT。
- signed error: `検出動画時刻 - GT動画時刻`。正は遅れ、負は先行。
- MAE: 対応イベントの絶対誤差平均。
- count error: `正式候補数 - 右足GT数`。
- precision / recall / F1: overall、normal、terminalで出力。
- adjustment / turn FP: 未対応候補が同種注釈の許容窓内にある場合に別集計。
- 欠番近傍: event時刻の既定±100 msに欠番gridがあればフラグ化し、近傍最大連続欠番長も残す。

## 出力

- `summary.json`: 固定条件、同期fit、データ品質、指標、動作別FP。
- `sync_fit.csv`: pre/post同期点のphase、fit時刻、残差。
- `detector_arms.csv`: 採用・不採用を含むarmed peak、prominence、zero crossing、確認値、欠番近傍。
- `matches.csv`: TP / FP / FN、signed error、GT confidence、FP動作種別。
- `metrics.csv`: overall / normal / terminalのTP、FP、FN、precision、recall、F1、signed error、MAE、count error。

同期残差、低confidence GT、欠番近傍イベントは数値だけで自動除外せず、trialの評価表で併記する。
