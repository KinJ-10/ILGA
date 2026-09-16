# IL-Phase 0 2026-09-16 UART record pacing比較

## 1. 目的

距離評価で散発したILCS2 UART record破損を、RF距離条件と分離して切り分ける。TAG / Reflectorは同じraw診断ファームでモバイルバッテリー給電し、LOCATOR / Initiatorのrecord間待機だけを6 / 8 / 10 msへ変更した。

## 2. 条件

- UART: 230400 baud、8-N-1
- CS raw channels: 32
- TAG / Reflector: 同一ファーム、BLE / CS peerとして動作
- LOCATOR / Initiator: pacing条件ごとに専用buildを使用
- 取得: 各条件120秒×2試行
- 評価除外: capture開始時に途中から入ったprocedure、終了時の最終未完行
- 評価対象: checksum、record sequence、local / peer header、行結合、semantic parse error、firmware error、procedure周期

## 3. 使用ログ

| pacing | 試行 | ログ |
|---:|---:|---|
| 6 ms | 1 | `initiator_raw_230400_20260916_121040.log` |
| 6 ms | 2 | `initiator_raw_230400_20260916_122227.log` |
| 8 ms | 1 | `initiator_raw_230400_20260916_143609.log` |
| 8 ms | 2 | `initiator_raw_230400_20260916_144003.log` |
| 10 ms | 1 | `initiator_raw_230400_20260916_150758.log` |
| 10 ms | 2 | `initiator_raw_230400_20260916_151358.log` |

## 4. 結果

| pacing | 試行 | 評価対象完結procedure | local / peer header | 中間parse / framing error | firmware error | 周期中央値 | 95%周期 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6 ms | 1 | 121 | 121 / 121 | 1 | 0 | 0.990秒 | 1.009秒 |
| 6 ms | 2 | 120 | 120 / 120 | 0 | 0 | 0.992秒 | 1.001秒 |
| 8 ms | 1 | 121 | 121 / 121 | 0 | 0 | 0.990秒 | 1.017秒 |
| 8 ms | 2 | 120 | 120 / 120 | 0 | 0 | 0.996秒 | 1.006秒 |
| 10 ms | 1 | 118 | 118 / 118 | 6 | 7 | 1.005秒 | 1.041秒 |
| 10 ms | 2 | 120 | 120 / 120 | 14 | 15 | 0.988秒 | 1.037秒 |

6 ms試行1では、通常表示の`Phase-Based Ranging method`行の末尾へ、checksumが正しい次procedureのILCS2 headerが改行なしで結合した。record sequenceは連続しておりILCS2 payload自体は回収できたが、UART framingとしては不合格である。

8 msは2試行とも評価対象区間のchecksum mismatch、record sequence gap、非raw文字列結合、semantic parse error、firmware errorが0件だった。procedure周期も公称約0.99秒を維持した。

10 msでは、checksumが正しいにもかかわらずpermutationまたはqualityがAPI値域を外れるrecordと、firmwareの`UNSUPPORTED_MODE`が両試行で再現した。単純なUART byte欠落ではなく、待機延長がpeer step dataの処理タイミングへ干渉した可能性がある。

## 5. parserの見逃し

従来parserは、行の途中に`ILCS2,`があればその位置から後ろのchecksumを検証して受理した。そのため、6 ms試行1のようにhost timestampとILCS2の間へ通常表示が結合してもparse errorを出さなかった。

非raw文字列を`NON_RAW_PREFIX_BEFORE_RECORD`として記録し、checksumが正しいILCS2 record自体は解析へ残す。これにより、データを捨てずにUART framing異常をgateへ反映できる。

## 6. 判定

- 6 ms: 条件付き不合格。2試行中1試行で行結合1件
- 8 ms: 合格。2試行の評価対象区間で破損0件
- 10 ms: 不合格。semantic / firmware errorが両試行で再現
- 採用値: 8 ms

LOCATOR / Initiator raw診断の`CONFIG_IL_CS_RAW_RECORD_PACING_MS`を8へ変更する。TAG / ReflectorのBLE / CS設定、channel数、UART baudは変更しない。

## 7. 8 ms復帰後の最終確認

2026-09-17にLOCATORを8 ms構成で再起動し、TAGと再接続後に45秒間取得した。使用ログは`initiator_raw_230400_20260917_084420.log`である。

- 取得procedure: 46件
- 取得開始時の途中procedure: 1件（評価対象外）
- 評価対象の完結procedure: 45件
- local / peer header: 45 / 45件
- parse / framing error: 0件
- firmware error: 0件

8 msへ復帰した実機でBLE / CS再接続とraw取得が正常に動作することを確認した。
