# GA event detector 状態設計案

## 前提

これは評価用の設計案であり、本番event detectorの実装仕様ではない。現在の固定候補方式はgx、seq 100 Hz grid、欠番線形補間、4次Butterworth 5 Hz low-pass zero-phase、正ピーク80 dps以上、prominence 50 dps以上、最小間隔0.75秒、peak後0.30秒以内の正→負zero crossing、その後0.40秒以内のgx -50 dps以下確認である。

zero-phase処理は未来サンプルを使うためoffline評価専用。本番のcausal実装と遅延は別途評価する。

## 状態遷移

`static → gait_start → steady_gait → terminal → static`

### static

- 静止品質とbiasを監視する。
- 単発ピークだけで接地を出力しない。
- 一定期間の周期活動が成立した場合に`gait_start`へ進む。

### gait_start

- 最初の左右開始差と、同期踏み込み・偶発動作を区別する。
- 固定gx条件を満たした候補を保持するが、steady gait成立前の確定規則は同期GTで決める。
- 周期性、候補間隔、活動継続が成立した場合に`steady_gait`へ進む。

### steady_gait

- 通常接地には現在の厳密条件を維持する。
- 被験者や速度に応じて全体閾値を自動的に緩めない。
- dropout補間の有無と欠番近傍フラグをイベント品質へ残す。

### terminal

- 歩行周期の終了、速度低下、停止姿勢への遷移を状態として扱う。
- strict条件を満たす通常候補と、peak・zero crossingまでは成立するが確認条件が異なる終端候補を別クラスで出す。
- 終端候補は`terminal_contact`として独立評価し、通常接地の閾値や確認条件を緩めない。
- adjustmentとturnは接地イベントではなく、誤検出評価用の終端動作クラスとして扱う。

### staticへの復帰

- 一定期間の低活動と姿勢安定を必要とする。
- 短い足位置調整や旋回の谷間を静止と誤判定しない。

## 本番causal実装前に確定する項目

1. 同期GTに対するstrict通常接地とterminal候補の時刻誤差分布。
2. gait_start、terminal、static復帰の特徴量、窓長、遷移条件。
3. slow / normal / fast、左右開始、通常停止 / adjustment / turnでの固定条件の汎化。
4. 再装着、別被験者、別日での軸向き・振幅差への対応。
5. 欠番の許容run長、補間イベントのconfidence低下規則、再同期方法。
6. causal low-passの方式、群遅延、イベント時刻補正と処理バッファ長。
7. event確定までの最大許容遅延と、後から取り消せない出力APIの扱い。
8. terminal候補を正式接地へ昇格する条件。通常条件を緩めるのではなく別判定器として評価する。
9. adjustment / turn / 同期踏み込み / 戻り歩行のfalse positive上限。
10. GT許容窓、低confidence注釈の集計方針、被験者単位の合格基準。

## 評価ゲート案

- まずoffline zero-phase固定条件で、normalとterminalを分離した被験者別結果を確定する。
- 次に同じtrialへcausal版を適用し、検出数だけでなくsigned errorと遅延増分を比較する。
- terminal候補の仕様が確定するまで、doi walk1/3のような未確認armを正式イベントへ含めない。
- 独立検証データを見た後の閾値変更は新しい版として記録し、そのデータを再び独立検証には使わない。
