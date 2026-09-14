# IL-Phase 0: nRF54L15 DK Channel Sounding単体技術検証

更新日: 2026-09-14

## 1. 目的と成立範囲

2台のnRF54L15 DKだけを使い、Bluetooth Channel Sounding（CS）による距離変化の検出可能性を確認する。

- 固定側: LOCATOR = Initiator
- 装着側: TAG = Reflector
- 主評価: 約1秒周期、距離順序、欠測率、境界チャタリング
- Phase 0の対象外: 単一LOCATORによる2D座標、絶対距離1 m精度の保証、AoA統合
- nRF5340 DKは標準CSノードとして使用しない

本試験の距離値は標準サンプル付属の簡易推定器による技術検証値であり、製品精度を表さない。

## 2. 基準サンプルと妥当性

基準はNCS v3.2.3に含まれる次のZephyrサンプルとする。

```text
~/ncs/ncs-v3.2.3/zephyr/samples/bluetooth/channel_sounding/connected_cs/initiator
~/ncs/ncs-v3.2.3/zephyr/samples/bluetooth/channel_sounding/connected_cs/reflector
```

採用理由:

- 2台のBLE接続上でInitiator/ReflectorのCS procedureを反復する標準サンプルである。
- 要件に「Bluetooth LEとChannel Soundingを備えた2台（例: nRF54L15 DK）」が明記される。
- RTTとPhase-Based Ranging（PBR）の簡易距離推定結果をInitiator UARTへ出力する。
- `CONFIG_BT_CHANNEL_SOUNDING=y`からnRF54L15のcontroller側`CONFIG_BT_CTLR_CHANNEL_SOUNDING=y`が選択される。

注意点:

- NCS v3.2.3ではCS機能がExperimental扱いである。
- 標準サンプルはprocedure間隔を100接続イベントとしており、接続間隔次第で実時間が変わる。
- 付属推定器は基本アルゴリズムであり、絶対距離精度は本Phaseの合否にしない。
- 単一subevent、約512 byteのGATT交換というサンプル制約を維持する。

## 3. リポジトリ内の最小構成

```text
LOCATOR/zephyr_apps/current/il_cs_initiator/
  CMakeLists.txt
  Kconfig
  prj.conf
  raw_diagnostics.conf
  src/main.c
TAG/zephyr_apps/current/il_cs_reflector/
  CMakeLists.txt
  Kconfig
  prj.conf
  raw_diagnostics.conf
  src/main.c
scripts/
  il_cs_build.sh
  il_cs_flash_pair.sh
  il_cs_log_capture.sh
  il_cs_raw_analyze.py
docs/il/
  IL_PHASE0_CHANNEL_SOUNDING.md
tests/il/
  fixtures/il_cs_raw_synthetic.log
  test_il_cs_raw_analyze.py
```

`main.c`はNCS v3.2.3のconnected_csを派生元とし、Phase 0で必要な差分だけを持つ。共通ヘッダと`distance_estimation.c`は`${ZEPHYR_BASE}/samples/bluetooth/channel_sounding`を参照し、NCSコードの重複管理を避ける。

ILGA差分は次の4点である。

1. LOCATOR/InitiatorとTAG/Reflectorを起動ログに明示する。
2. Reflectorの広告名を`IL-CS-TAG`とし、Initiatorは同名だけへ接続する。
3. Initiatorが30 ms固定の接続間隔を要求する。
4. CS procedure間隔を33接続イベントとし、公称周期を0.99秒にする。

周期はcontrollerのスケジューリング、GATT交換、UART処理にも影響されるため、実ログの到着時刻で判定する。

## 4. build

NCS v3.2.3環境のWSLで実行する。

```bash
cd /home/in/work/ILGA
chmod +x scripts/il_cs_build.sh scripts/il_cs_flash_pair.sh scripts/il_cs_log_capture.sh
./scripts/il_cs_build.sh
```

専用出力先:

```text
build/il_cs_initiator/merged.hex
build/il_cs_reflector/merged.hex
```

`build/tag_nrf54l15_port`は参照もcleanもせず、Phase 0のbuildと完全に分離する。

build後の設定確認:

```bash
grep -E '^CONFIG_(BT_CHANNEL_SOUNDING|BT_CTLR_CHANNEL_SOUNDING|BT_CENTRAL)=' \
  build/il_cs_initiator/il_cs_initiator/zephyr/.config
grep -E '^CONFIG_(BT_CHANNEL_SOUNDING|BT_CTLR_CHANNEL_SOUNDING|BT_PERIPHERAL)=' \
  build/il_cs_reflector/il_cs_reflector/zephyr/.config
```

すべて該当する役割が`=y`であることを確認する。

## 5. 2台の識別と安全なflash

### 5.1 flash前の必須記録

2台を同時接続し、まだflashせずに次を実行する。

```bash
nrfutil device list
ls -l /dev/serial/by-id/
```

筐体へ物理ラベルを貼り、次を試験記録へ転記する。

| 物理ラベル | 固定役割 | DKシリアル | UARTのby-idパス | 試験前ファームウェア | 復元build |
|---|---|---:|---|---|---|
| LOCATOR-CS | Initiator | 記入 | 記入 | 記入 | 記入 |
| TAG-CS | Reflector | 記入 | 記入 | 記入 | 記入 |

シリアル番号と役割の対応が未記入、重複、または不確実ならflashしない。

### 5.2 dry-runと実行

最初は`--execute`なしで、対象とコマンドだけを表示する。

```bash
./scripts/il_cs_flash_pair.sh <INITIATOR_SERIAL> <REFLECTOR_SERIAL>
```

物理ラベルと表示を別の人または指差し確認で照合後、実行する。

```bash
./scripts/il_cs_flash_pair.sh <INITIATOR_SERIAL> <REFLECTOR_SERIAL> --execute
```

実行モードでも、スクリプトは次を満たさない限りflashしない。

- 2つのシリアルが数字で、互いに異なる
- 両方が`nrfutil device list`に存在する
- 対応する専用buildが存在する
- 対話入力で`FLASH <INITIATOR_SERIAL> <REFLECTOR_SERIAL>`を完全一致で再入力する

`west flash`や`nrfutil`をシリアル省略で直接実行しない。

## 6. UARTログ保存

UARTは`/dev/ttyACM*`ではなく、シリアルを含む安定した`/dev/serial/by-id/...`を優先する。2端末、または1つのbashで次のように同時取得する。

```bash
./scripts/il_cs_log_capture.sh initiator /dev/serial/by-id/<LOCATOR_UART> 180 --normal &
init_log_pid=$!
./scripts/il_cs_log_capture.sh reflector /dev/serial/by-id/<TAG_UART> 180 --normal &
refl_log_pid=$!
wait "${init_log_pid}" "${refl_log_pid}"
```

保存先は`logs/il_cs/`で、各行にホスト時刻を付加する。Initiatorログでは少なくとも次を確認する。

```text
Starting ILGA Channel Sounding Phase 0 (LOCATOR/Initiator)
Found device with name IL-CS-TAG, connecting...
CS capability exchange completed.
CS config creation complete. ID: 0
CS security enabled.
CS procedures enabled.
Estimated distance to reflector:
```

Reflectorログでは`Starting ILGA Channel Sounding Phase 0 (TAG/Reflector)`と接続・CS有効化を確認する。

### 6.1 2026-09-11接続実績

| 役割 | DKシリアル | WSL UART | 備考 |
|---|---:|---|---|
| LOCATOR / Initiator | 1057707951 | `/dev/ttyACM3`（vcom 1） | CS距離結果の出力側 |
| TAG / Reflector | 1057727822 | `/dev/ttyACM1`（vcom 1） | `IL-CS-TAG`として広告 |

両DKへのflash、BLE接続、CS capability交換、config作成、CS security、有効化、反復距離出力まで確認した。初回確認ではPBR/RTTとも距離値が出力され、Channel Soundingの送受信成立を確認できた。一方、Nordic標準サンプル由来の512 byte制約により、一部procedureで`Not enough memory to store step data`が発生した。Phase 0の成立確認を妨げるものではないが、欠測として評価時に数える。

### 6.2 S01（0.5 m静止）初回結果

2026-09-11に同じ机上でアンテナ間距離を0.5 m、両DKの向きを合わせた見通し条件として、60秒を3試行取得した。

| 試行 | Initiatorログ | PBR中央値 | PBR範囲 | RTT中央値 | 算出不能 | 512 byte警告 | 周期中央値 | 切断 |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| S01-1 | `initiator_20260911_090958.log` | 2.397 m | 2.212～2.653 m | 5.883 m | 0 | 1 | 0.990秒 | 0 |
| S01-2 | `initiator_20260911_091205.log` | 2.472 m | 2.068～2.832 m | 5.712 m | 3 | 0 | 0.990秒 | 0 |
| S01-3 | `initiator_20260911_091345.log` | 2.360 m | 2.077～2.656 m | 5.971 m | 0 | 0 | 0.990秒 | 0 |

各試行の先頭10秒を除いた評価区間を合算すると、PBR有効値147件、中央値2.393 m、範囲2.068～2.832 m、RTT中央値5.876 mだった。算出不能3件と512 byte警告1件を合わせた事象率は4/151（2.65%）、接続断は0件だった。出力周期と欠測率はPhase 0の暫定基準を満たすが、実距離0.5 mに対する大きな正方向バイアスがある。机上反射、アンテナ条件、標準サンプル付属推定器の特性を含む可能性があるため、絶対距離の補正は行わず、1 m以降の距離順序を確認してから判断する。

### 6.3 S02（約1 m静止）初回結果

2026-09-11に同じ机上・同じ向きで、アンテナ間をケーブル長の範囲でおおよそ1 mとして60秒を3試行取得した。測定前にInitiator側のWSL USB接続が外れ、Initiatorだけを再接続しても距離出力は再開しなかった。両DKの同時リセットによりBLE接続とCS procedureが復旧したため、復旧後のログだけを評価対象とした。

| 試行 | Initiatorログ | PBR中央値 | PBR範囲 | RTT中央値 | 算出不能 | 512 byte警告 | 周期中央値 | 切断 |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| S02-1 | `initiator_20260911_093412.log` | 2.353 m | 1.232～3.828 m | 6.670 m | 2 | 2 | 0.990秒 | 0 |
| S02-2 | `initiator_20260911_093550.log` | 2.245 m | 1.277～2.368 m | 6.408 m | 0 | 0 | 0.990秒 | 0 |
| S02-3 | `initiator_20260911_093725.log` | 2.223 m | 1.477～2.417 m | 6.795 m | 0 | 1 | 0.990秒 | 0 |

各試行の先頭10秒を除いた評価区間を合算すると、PBR有効値142件、中央値2.255 m、範囲1.232～3.828 m、RTT中央値6.670 mだった。算出不能2件と512 byte警告3件を合わせた事象率は5/147（3.40%）、接続断は0件だった。

0.5 m条件のPBR中央値2.393 mに対し、約1 m条件は2.255 mで0.138 m低く、距離順序が逆転した。RTT中央値は5.876 mから6.670 mへ増えたが、絶対誤差と試行内変動が大きい。したがって、この机上配置と標準推定器のままでは0.5 mと約1 mの距離識別は未成立と判定する。CS通信・約1秒周期・接続維持は成立している。

### 6.4 A-B-A切り分け結果

S02終了後に両DKを約0.5 mへ戻し、60秒を1試行追加した（`initiator_20260911_102619.log`）。先頭10秒を除いた評価結果は次のとおり。

| 区間 | 実距離 | PBR中央値 | PBR範囲 | RTT中央値 | 算出不能 | 512 byte警告 |
|---|---:|---:|---:|---:|---:|---:|
| A（初回3試行合算） | 0.5 m | 2.393 m | 2.068～2.832 m | 5.876 m | 3 | 1 |
| B（3試行合算） | 約1 m | 2.255 m | 1.232～3.828 m | 6.670 m | 2 | 3 |
| A（戻り1試行） | 0.5 m | 2.372 m | 2.060～2.740 m | 6.096 m | 2 | 1 |

戻りAのPBR中央値は初回Aとの差が-0.021 mで、初回水準へほぼ戻った。これにより、AからBへの変化を時間経過による一方向ドリフトだけで説明する可能性は低下した。一方、PBRは距離増加に対して逆方向へ変化しており、範囲も大きく重なるため、距離尺度や閾値としては使用できない。現時点では、配置変更に反応する可能性はあるものの、机上反射、位相の多値性、アンテナ・ケーブル条件、標準推定器の影響を分離できていない。

### 6.5 raw診断版による環境変更後の近距離評価

2026-09-14にILCS2 raw診断版（UART 230400 baud、32 channels、record pacing 6 ms）を使用し、机を変更して見通し・同じ高さ・同じ向きの条件で再測定した。旧机のraw A-B-Aでは0.5 mが5.194 mと0.412 m、1.0 mが4.003 mとなり、同距離再現性と距離順序が成立しなかった。新しい机のraw A-B-Aでは0.5 mが1.712 mと1.699 m、1.0 mが2.337 mとなり、机変更後に戻り再現性と距離順序が改善した。この差から、旧机では反射・マルチパスを含む環境要因の影響が大きかった可能性が高い。

新しい机で0.50、0.75、1.00 mをそれぞれ60秒×3試行、距離ごとに配置を固定したまま連続取得した。各試行の先頭10 proceduresを除外し、完結procedureだけを評価した。

| 実距離 | PBR有効/評価 | 合算中央値 | 合算標準偏差 | 試行中央値幅 | 中央値誤差 |
|---:|---:|---:|---:|---:|---:|
| 0.50 m | 153/153 | 1.389 m | 0.058 m | 0.024 m | +0.889 m |
| 0.75 m | 152/153 | 1.929 m | 0.176 m | 0.045 m | +1.179 m |
| 1.00 m | 152/152 | 2.220 m | 0.095 m | 0.080 m | +1.220 m |

全体で457/458 proceduresのPBR値が有効だった。評価区間内のchecksum mismatch、record sequence gap、local/peer counter mismatch、firmware error、非境界parse errorは0件だった。距離別中央値は0.50 < 0.75 < 1.00 mの順に単調増加し、固定配置での試行中央値幅は2.4～8.0 cmだった。一方、推定値の増分は0.50→0.75 mで+0.540 m、0.75→1.00 mで+0.290 mとなり、真値増分0.25 mに対して一定ではない。絶対誤差も+0.889～+1.220 mであり、固定offsetまたは3点だけの線形補正は採用しない。

同じ新環境でも、置き直し前の0.5 m約1.70 mに対し、後続の固定配置3試行は約1.39 mだった。固定配置中の時間再現性は良好だが、再設置時の位置、角度、ケーブル条件への感度は残る。場所の制約により再設置反復試験は実施せず、未評価項目として残す。

以上から、近距離のサンプルベース技術検証は次の条件付き合格とする。

- CS送受信、約1秒周期、raw取得・解析: 合格
- 新環境での0.50 / 0.75 / 1.00 m相対距離順序: 合格
- 固定配置の試行間再現性: 合格
- 絶対距離精度: 不合格
- 線形性・一般化可能な補正: 未成立
- 再設置再現性: 未評価
- IL Phase 0全体: 継続（遠距離、遮蔽、向き、動的、境界、復帰試験は未実施）

集計、図、元ログ索引は次に保存する。

```text
analysis_out/il_cs_raw/20260914_env2_distance_series_summary/
```

Git管理する要約レポートと代表グラフは次に保存する。

```text
docs/il/results/20260914_env2_near_range/
```

## 7. 測定共通条件

- 屋内の見通し直線を床へマーキングし、アンテナ間距離を測る。
- LOCATORは三脚等で固定し、アンテナ高さを記録する。
- TAGの装着位置・高さ・向き、周囲の壁・金属・人の位置を記録する。
- 各静止条件は60秒、最初の10秒を安定待ちとして除外し、残り50秒を評価する。
- PBR値を主系列、RTT値を参考系列として別々に集計し、混ぜて平均しない。
- 明示された移動試験以外は人と物を動かさない。
- 1試行ごとにログファイル名、開始・終了時刻、異常を試験表へ記録する。

## 8. 試験表

| ID | 条件 | 距離・動作 | 向き/遮蔽 | 時間・反復 | 主確認 |
|---|---|---|---|---|---|
| S01 | 最短静止 | 0.5 m | 正対、見通し | 60秒×3 | 周期、欠測、安定性 |
| S02 | 静止 | 1 m | 正対、見通し | 60秒×3 | 同上 |
| S03 | 静止 | 2 m | 正対、見通し | 60秒×3 | 同上 |
| S04 | 静止 | 3 m | 正対、見通し | 60秒×3 | 同上 |
| S05 | 静止 | 5 m | 正対、見通し | 60秒×3 | 同上 |
| S06 | 最長静止 | 10 m | 正対、見通し | 60秒×3 | 到達性、欠測、順序 |
| O01 | 人体遮蔽 | 1 m | TAGを胸/腰装着、人体が間に入る | 60秒×3 | 欠測増加、復帰 |
| O02 | 人体遮蔽 | 3 m | 同上 | 60秒×3 | 同上 |
| O03 | 人体遮蔽 | 5 m | 同上 | 60秒×3 | 同上 |
| R01 | 向き変更 | 1 m | TAG 0/90/180/270度 | 各60秒×1 | 向き依存、切断なし |
| R02 | 向き変更 | 3 m | TAG 0/90/180/270度 | 各60秒×1 | 同上 |
| R03 | 向き変更 | 5 m | TAG 0/90/180/270度 | 各60秒×1 | 同上 |
| D01 | 接近 | 10→0.5 m | 見通し、通常歩行 | 3往路 | 距離トレンド、欠測 |
| D02 | 離脱 | 0.5→10 m | 見通し、通常歩行 | 3復路 | 距離トレンド、欠測 |
| C01 | 境界静止 | 2.5 m | 正対、見通し | 120秒×3 | ノイズ起因の状態反転 |
| C02 | 境界横断 | 5→1→5 m | 2 mをenter、3 mをexitの仮境界 | 5往復 | チャタリング、検知遅延 |
| REC01 | 遮蔽復帰 | 3 m | 金属以外の人体遮蔽を10秒ON/OFF | 5回 | 再出力までの時間 |

試験順はS01→S06を基本とする。ランダム化した再試行も1系列追加できれば、温度・時間ドリフトと距離順序を切り分けやすい。

## 9. 指標と合否基準

### 9.1 出力周期

Initiatorログの連続する`Estimated distance to reflector:`のホスト時刻差を周期とする。

- 合格: S01〜S06を合算した中央値が0.8〜1.2秒、95パーセンタイルが1.5秒以下
- 不合格: 1.5秒超の周期が継続、または周期が距離に応じて系統的に悪化

公称0.99秒は設定確認にすぎず、実測を優先する。

### 9.2 欠測率

評価区間長を`T`秒、実測周期中央値を`P`秒、距離結果数を`N`とし、`max(0, 1 - N/(T/P))`を欠測率とする。`A reliable distance estimate could not be computed.`も欠測へ数える。

- 合格: 見通しS01〜S05は各5%以下、S06は10%以下
- 条件付き合格: 人体遮蔽・向き変更・動的試験は20%以下で、切断せず遮蔽解除後3周期以内に復帰
- 不合格: 接続断、手動resetが必要、または上記を超える

### 9.3 距離順序

各見通し静止距離のPBR中央値を使う。絶対誤差は判定しない。

- 合格: 0.5/1/2/3/5/10 mの真値順位と推定中央値のSpearman順位相関が0.9以上、かつ隣接距離の逆転が1組以下
- 不合格: 近距離と遠距離の区別が繰り返し逆転、または再試行で順位関係が再現しない

RTTは同じ集計を参考値として残すが、PBRと平均して見かけの精度を作らない。

### 9.4 接近・離脱

- 合格: 各往路のPBR系列と移動方向のSpearman順位相関の絶対値が0.8以上、3往路中2往路以上で正しい符号
- 合格: 欠測率10%以下、接続断なし
- 記録: 方向転換前後の遅延、外れ値、人体や腕振りの影響

### 9.5 境界チャタリング

Phase 0評価用の仮ルールとして、平滑化後PBRが2 m以下でenter、3 m以上でexitとする。2〜3 mは直前状態を保持する。これはPhase 1仕様の確定値ではない。

- C01合格: 2.5 m静止中に状態反転なし
- C02合格: 1往復につきenter 1回・exit 1回で、追加反転なし
- C02合格: 物理的に各境界を越えてから3周期以内に期待状態へ変化
- 不合格: 同一横断で2回以上の追加反転、または状態が戻らない

## 10. 失敗時の切り分け

1. 起動role文字列が物理ラベルと一致するか確認する。
2. `CS capability exchange completed.`、config、security、procedure enabledの順に到達したか確認する。
3. ReflectorからGATT step dataが届かない、512 byte制約、multiple subevent警告がないか確認する。
4. 同距離で向きだけを変え、RF/アンテナ要因とsoftware欠測を分ける。
5. 30 ms接続間隔または33イベントがcontrollerで受理されない場合はログを保存し、推測で値を変えず別試験条件として変更する。

## 11. 既存GAファームウェアへの復元

Phase 0前に「試験前ファームウェア」と「復元build」を必ず記録する。既存GA TAGへ戻す場合、Phase 0専用buildではなく既存のGA buildをシリアル指定でflashする。

```bash
cd /home/in/work/ILGA
./scripts/tag_flash.sh --dev-id <GA_TAG_SERIAL>
```

これは既存`build/tag_nrf54l15_port`が正しいGA成果物として残っている場合の手順である。存在しない、内容が不明、またはbuild基準が違う場合はflashせず、先に`./scripts/tag_build.sh`でGAを再buildするか、記録済みの正しいGA buildを指定する。再buildは`build/tag_nrf54l15_port`を更新するため、Phase 0測定とは別作業として実施する。

復元確認:

1. 起動ログでGA TAGファームウェアであることを確認する。
2. BMI270初期化、ACC/GYR UART出力、BLE advertisingを確認する。
3. Windowsの正式経路で30秒CSVを保存し、packet数・欠測seq・再接続を確認する。
4. Phase 0の`IL-CS-TAG` advertisingが残っていないことを確認する。

LOCATOR側に試験前ファームウェアがあった場合も、記録した専用buildとシリアルで同様に復元する。試験前ファームウェアが不明なら上書きしない。

## 12. Phase 0完了条件

- 両役割のbuild成功
- 2台のシリアル/役割/安定UARTパスを記録してから安全にflash成功
- 接続、CS設定、反復距離出力を両UARTログで確認
- S01〜S06、O01〜O03、R01〜R03、D01〜D02、C01〜C02、REC01を記録
- 周期、距離順序、欠測率、境界チャタリングを上記基準で判定
- GA復元とGAの基本動作確認

2026-09-14時点では、6.5節の近距離サンプルベース技術検証だけを条件付き合格とする。本節に列挙したPhase 0全体の完了条件は満たしていないため、Phase 0全体を完了扱いにしない。

buildだけでは実機項目を完了扱いにしない。

## 13. channel-level raw診断モード

最終距離値だけではPBR逆転の原因を分離できないため、通常モードから分離したオプトイン診断モードを用意する。通常buildは従来どおり次で作る。

```bash
./scripts/il_cs_build.sh
```

raw診断ペアは次で作り、通常buildとは別の出力先へ保存する。

```bash
./scripts/il_cs_build.sh --raw-diagnostics
```

```text
build/il_cs_initiator_raw/merged.hex
build/il_cs_reflector_raw/merged.hex
```

raw診断はInitiatorの`CONFIG_IL_CS_RAW_DIAGNOSTICS=y`とReflectorの同設定を組にして使う。診断buildと通常buildを混在させない。`CONFIG_IL_CS_RAW_EVERY_N=1`は全procedureを出す。`CONFIG_IL_CS_RAW_RECORD_PACING_MS=6`は完全なILCS2 recordごとにmain threadをyieldし、`CONFIG_PRINTK_SYNC=y`はstatus用の1回の`printk()`が他contextの出力と混在しないようserializeする。ILCS2 record自体はchecksumまで完成バッファ化し、chosen console UARTへmutex下で直接poll送信する。`CONFIG_IL_CS_RAW_CHANNEL_COUNT=32`はraw時だけCS channel mapを32 channelsへ制限する。

診断モードではReflectorが512 byteのstep data writeに先立ち、同じGATT characteristicへ短いmetadata writeを送る。metadataにはprotocol magic/version、procedure counter、実step長、valid/overflow/no-data、CS result headerを含む。Initiatorは次を確認してから距離計算とraw出力を行う。

- local/peer procedure counterが一致する
- 両側のstep dataがvalidである
- 実step長が512 byte以下である
- 当該procedureのmetadataを受信済みである

各procedure開始時にlocal bufferをゼロクリアし、peer payload受信時にもpeer bufferをゼロクリアする。不一致・overflow・no-dataでは前回値を流用せず、`ILCS2`の`E` recordへ理由を出す。step payload自体は従来と同じ固定512 byte writeを維持するため、metadata追加によってstep data上限を削らない。

## 14. ILCS2 UART schema

UART raw recordはCSV互換の1行1recordで、すべて`ILCS2`から始まる。`record_seq`はboot後のraw record連番、`checksum`は最後のcommaより前のASCII文字列全体に対する32-bit FNV-1aを8桁16進で表した値である。`L`はInitiator local result、`P`はReflectorからGATTで受信したpeer resultを表す。

```text
ILCS2,record_seq,H,procedure,side,config_id,start_acl_event,freq_comp,ref_power,
  procedure_status,subevent_status,procedure_abort,subevent_abort,
  n_ap,num_steps,abort_step,step_len,transport_flags,checksum

ILCS2,record_seq,0,procedure,side,step,channel,aa_quality,bit_errors,rssi,antenna,
  measured_freq_offset_or_NA,checksum

ILCS2,record_seq,1,procedure,side,step,channel,aa_quality,bit_errors,nadm,rssi,
  timing,antenna,checksum

ILCS2,record_seq,2,procedure,side,step,channel,tone,permutation,i,q,quality,
  extension,checksum

ILCS2,record_seq,E,procedure,side,step,reason,checksum
ILCS2,record_seq,Z,procedure,checksum
```

`H`はCS subevent result header、`0`/`1`/`2`は各step mode、`E`は欠測・malformed・counter不一致等、`Z`はprocedure raw record終端である。`transport_flags`はbit 0=`valid`、bit 1=`overflow`、bit 2=`no data`。Mode 1の`timing`は`L`で`toa_tod_initiator`、`P`で`tod_toa_reflector`である。Mode 2の`i,q`はZephyrの`bt_le_cs_parse_pct()`で24-bit PCTを12-bit signed IQへ展開した値である。UART帯域削減のため、Nordic推定器と同じく`extension_indicator != BT_HCI_LE_CS_NOT_TONE_EXT_SLOT`のtoneは出力しない。従ってILCS2のMode 2は距離推定へ投入可能なtoneだけで、`extension`は0となる。

host parserは旧smokeログ再解析のためILCS1も受理するが、ILCS1にはchecksumと連番がない。formal試験の完全性判定にはILCS2だけを使う。checksum不一致はrecordを受理せず、連番飛びは`RECORD_SEQUENCE_GAP`、capture終了で改行なしの最終recordは`TRUNCATED_AT_CAPTURE_END`として`parse_errors.csv`と該当procedure summaryへ残す。field数が正しくてもchannel、quality、IQ等のAPI値域外なら受理しない。checksumが正しい`record_seq=0`はfirmware再起動境界でありgapとはしない。CSVの`boot_index`で再起動前後を分離し、同じprocedure counterを混在させない。

APIから取得できるため保存する項目は次のとおり。

- procedure: config ID、開始ACL event、procedure counter、frequency compensation、reference power、procedure/subevent status、abort理由、antenna path数、step数、abort step、step byte長
- Mode 0: channel、AA quality、bit error数、RSSI、antenna、Initiator measured frequency offset
- Mode 1: channel、AA quality、bit error数、NADM、RSSI、role別timing、antenna
- Mode 2: channel、tone index、antenna permutation、local/peer PCT IQ、quality、extension indicator

controller/APIが提供しない環境補正値、multipath分類、真距離、校正済み位相は生成しない。`frequency_compensation`はAPIの16-bit raw表現をそのまま保存する。

## 15. UART帯域と既知制約

通常buildのconsole UARTはboard既定の115200 baudを維持する。raw診断buildだけは`raw_uart_230400.overlay`でchosen consoleの`uart20`を230400 baudへ上書きする。`il_cs_build.sh --raw-diagnostics`がInitiator/Reflector双方へoverlayを明示するため、通常CSとGAには波及しない。

`il_cs_log_capture.sh`はcapture profileを必須とし、`--normal`を115200、`--raw-diagnostics`を230400へ固定する。profileと`--baud`が一致しなければserial portを開く前に停止する。host側はpyserialでSEGGER VCOMを指定baud、8 data bit、no parity、1 stop bit、flow controlなしで開き、行ごとの外部`date`起動をせずhost timestampを付ける。

2026-09-11の25秒smoke captureでは、旧ILCS1のprocedure 36がhost timestamp込み10,355 byte/157行を約0.49秒で出し、timestampを除くUART payloadは約5.0 kbyteだった。同区間の115200 baud上限約5.6 kbyteへ近く、1行でfield fragmentが混在した。ファイル上でも改行byteが失われており、host loggerによる正常2行の結合ではない。旧emitterはextension toneも1行ずつ出していたため、ILCS2では推定器が使用しないextension toneを除外し、A1/B1時のMode 2行数を概ね半減させる。checksumと連番により、残るbyte化け・完全なrecord欠落も検出する。

2026-09-11のILCS2 120秒smokeを115200 baud、`CONFIG_IL_CS_RAW_EVERY_N=1`で実施した結果、122 procedures、firmware error 0、512 byte overflow 0だったが、parse errorは19件だった。末尾の`TRUNCATED_AT_CAPTURE_END` 1件を除いても、procedure 22、46、56、68、73、84、109等で`CHECKSUM_FIELD_INVALID`、`CHECKSUM_MISMATCH`、`RECORD_SEQUENCE_GAP`が発生した。Reflectorログは0 byteであり、起動・接続状態の独立確認にも不足する。この結果からILCS2検出機構は機能したが、115200 baud raw captureはformal試験に使用しない。

2026-09-11の次の125秒smokeでは、raw firmwareとhostを230400 baudへ揃え、両capture開始後に両DKをresetした。Initiatorログ890,129 byte、Reflectorログ1,047 byteで、Reflectorの起動、接続、MTU 247、CS config/security/procedure開始を確認できた。しかし旧parserではparse error 59件、boot-aware再解析でも全体58件、reset後区間55件だった。`CHECKSUM_FIELD_INVALID`、`CHECKSUM_MISMATCH`、`RECORD_SEQUENCE_GAP`、field fragment混在が多数あり、baudを2倍にするだけでは解消しなかった。Reflectorではstep data 519、519、528 byteの3件が512-byte上限を超え、procedure 57、90がlocal/peer invalidになった。従ってこの結果もformal試験には使用しない。

Zephyr v4.2.99のUART consoleは`printk()`を1 byteずつ`uart_poll_out()`へ渡すpoll方式であり、非同期software ringのoverflowではない。一方、`CONFIG_PRINTK_SYNC`が無効だとpreempting interrupt等の出力が1 record内へinterleaveでき、`uart_poll_out()`の完了はSEGGER VCOMからhost applicationまでのdrain完了を保証しない。rawだけ`CONFIG_PRINTK_SYNC=y`をstatus出力へ使用し、ILCS2は完成行をchosen UARTへ直接poll送信する。record間pacingは6 msとする。

230400 baudの8-N-1 payload上限は約23.0 kbyte/sである。既知の約5.0 kbyte/procedureはwire上約217 msで、1 procedureを最大100 recordsと保守的に見積もった6 ms pacingを加えても約817 msである。公称0.99秒procedure周期内に収まり、record間にVCOM/host drain時間を与える。

NCS v3.2.3標準connected CS sampleは、512-byte単一GATT writeへ収める方法として`channel_map_repetition=1`と限定channel mapを採用し、コメントでは32 consecutive channelsとしている。ただし実装loopの`26 <= channel < 62`は36 channelsである。raw時だけ同じ公開API方式で32 channelsへ修正し、通常構成の36 channelsは維持する。Bluetooth CS APIは最低15 channelsを要求するため32は有効範囲内である。GATT/ATTの512-byte上限自体は変更しない。

2026-09-11の2 ms pacing＋32 channels版120秒smokeでは、128 procedures、firmware error 0、Reflectorの512-byte overflow 0、接続成立まで改善した。しかしparse errorは30件残り、ILCS2行の途中欠落・別recordとの連結と、それに対応する`RECORD_SEQUENCE_GAP`を確認した。host captureは受信byteをbytearrayへ追加してLF分割するだけであり、生成configもUART console polling、`CONFIG_PRINTK_SYNC=y`、logging無効だった。従ってこの結果もgate不合格とし、ILCS2だけを完成行のdirect UART poll送信へ変更し、pacingを4 msへ増やした次版で再smokeする。

4 ms pacing版120秒smokeでは、125 procedures、firmware error 0、512-byte overflow 0まで維持し、parse errorは8件へ減少した。ただしprocedure 60、63でILCS2行の混在と連番飛びが残ったためgate不合格とした。

6 ms pacing版では、両capture開始後のreset前に送信中だった旧procedure 1行だけへboot bannerが連結した。reset後の`boot_index=1`では124 proceduresを121.769秒で取得し、平均周期は約0.990秒だった。reset後区間のchecksum error、record sequence gap、parse error、firmware error、512-byte overflowはすべて0件であり、formal A-B-Aへ進むsmoke gateを満たした。

次の制約は残る。

- Nordic sample由来の単一subevent設計であり、同じprocedure counterの2つ目のsubeventは扱わない。
- Reflector step dataは1回の最大512 byte GATT writeであり、rawの32-channel制限後も超過した場合はgate不合格とする。512を超えるwriteは行わず、必要ならchannel数の追加削減またはchunked protocolを別変更として検討する。
- raw UART自体がtimingへ影響する可能性があるため、通常モードの周期・距離値と別に評価する。
- PBRは一次元unwrap＋通常の線形回帰、RTTはNCS v3.2.3実装の単純処理を再計算するだけで、multipath除去、外れ値ロバスト化、環境・個体校正ではない。
- 固定offset補正は行わない。
- 既存のS01/S02/A-B-Aログは最終PBR/RTT値しか含まないため、channel-level raw解析には使用できない。既存ログは改変しない。

## 16. host解析

raw capture後、Initiatorログを新しい出力ディレクトリへ解析する。

```bash
python3 scripts/il_cs_raw_analyze.py \
  logs/il_cs/<initiator_raw_log>.log \
  --output-dir analysis_out/il_cs_raw/<trial_id>
```

出力ディレクトリは既存成果の上書きを避けるため、空または未作成でなければならない。

| 出力 | 内容 |
|---|---|
| `procedures.csv` | boot index、local/peer header、status、abort、step長、transport flags |
| `mode0_samples.csv` | Mode 0 quality/RSSI/frequency offset |
| `pbr_samples.csv` | channel/tone別IQ、local/peer record連番、wrapped/unwrapped phase、回帰残差、欠測理由 |
| `rtt_samples.csv` | paired timing、local/peer record連番、quality/RSSI/NADM、RTT中間値、欠測理由 |
| `procedure_summary.csv` | procedure別PBR傾き・残差RMS・再計算距離、RTT再計算値、件数 |
| `firmware_errors.csv` | firmwareが出したoverflow/mismatch/malformed等 |
| `parse_errors.csv` | boot index、schema、record連番、procedure、checksum/連番/field数/値域/EOF partialエラー |

PBRはlocal IQとpeer IQの複素積から`atan2`でwrapped phaseを求め、周波数`2402 + channel` MHzでsortし、隣接差が±πを越えた位置を2π補正する。その後、通常の最小二乗直線`phase = intercept + slope * frequency`を当て、次で距離を再計算する。

```text
distance_m = -slope_rad_per_MHz * c / (4 * pi) / 1e6
```

`pbr_samples.csv`の`residual_rad`と`procedure_summary.csv`の`pbr_residual_rms_rad`を、距離条件間の傾き変化、channel局所的な外れ、非線形性の観察に使う。quality low/unavailable、extension slot、local/peer欠落、zero complex productは回帰から除外し理由を保存する。

RTTはNCS v3.2.3 `distance_estimation.c`のcumulative moving averageを再現した`rtt_ncs_v3_2_3_distance_m`と、有効sampleだけの算術平均`rtt_valid_mean_distance_m`を別列へ出す。この2値を混同せず、AA quality失敗、RSSI/timing unavailable、step/channel不一致を保存する。

## 17. parser単体試験

実機rawがまだない段階では、保存済み合成fixtureでparser、π境界のunwrap、回帰符号、PBR/RTT再計算、全CSV生成、旧形式ログ拒否を確認する。

```bash
python3 -m unittest discover -s tests/il -p 'test_*.py' -v
```

fixtureはテスト専用であり、既存A-B-Aログを変換または上書きしない。

## 18. raw診断のflash後最小試験

本節のコマンドはbuild完了後の次作業であり、シリアルと物理ラベルを再確認してから実行する。最初はdry-runだけを行う。

```bash
./scripts/il_cs_flash_pair.sh 1057707951 1057727822 --raw-diagnostics
```

表示対象が次と一致することを確認する。

- LOCATOR / Initiator `1057707951`: `build/il_cs_initiator_raw/merged.hex`
- TAG / Reflector `1057727822`: `build/il_cs_reflector_raw/merged.hex`

実機flashを許可する作業でのみ`--execute`を追加する。flash後は現在確認済みのvcom1を使い、まず両ログを同時取得する。

```bash
./scripts/il_cs_log_capture.sh initiator /dev/ttyACM3 120 --raw-diagnostics --baud 230400 &
./scripts/il_cs_log_capture.sh reflector /dev/ttyACM1 120 --raw-diagnostics --baud 230400 &
wait
```

230400 baud＋direct UART poll＋6 ms pacing＋32 channels変更後は最初に同じ向き・高さ・見通しのまま2分smokeを行う。両captureを開始してから両DKをresetし、Reflector側にも起動・接続ログを残す。Initiator起動行の`ILCS2 raw diagnostics: 32 channels, 6 ms record pacing, direct UART`も確認する。解析ではreset後の`boot_index`だけをgate対象にする。次をすべて満たした場合だけ、0.5 mを60秒、約1 mを60秒、0.5 mへ戻して60秒のformal A-B-Aへ進む。

1. Initiator/Reflectorログがともに非0 byteで、各roleの起動・接続状態を確認できる。
2. `ILCS2`の`H`と`Z`が同じprocedure counterで対になり、欠落procedureがない。
3. 末尾の意図したcapture境界を除き、checksum mismatch、checksum field invalid、record sequence gap、local/peer counter mismatch、512 byte overflow、firmware error、parse errorがすべて0件である。
4. procedure周期が通常構成に対して大きく悪化していない。

smoke合格後のformal A-B-Aでは各区間の最初10秒を従来どおり除外し、移動時刻を別記録する。各区間で次を確認する。

1. checksum mismatch、record sequence gap、local/peer counter mismatch、512 byte overflow、parse errorの件数を数える。
2. channel別wrapped/unwrapped phase、傾き、残差RMSがA-B-Aでどう変化するか比較する。
3. PBR再計算値がfirmware最終値と丸め誤差範囲で一致するか確認する。
4. RTTのNCS再現値と有効sample平均の差を確認する。
5. raw UART有効時もprocedure周期が大きく崩れないか確認する。

この試験の目的は逆転原因の観測可能性を得ることであり、0.5 m/約1 mの識別成立や製品向け推定器完成をこの1回で主張しない。
