# TAG nrf54l15 Porting Notes

## 現在の構成概要

- 対象アプリ: `TAG/zephyr_apps/current/nrf54l15_port`
- 参照用 snapshot: `TAG/zephyr_apps/snapshots/nrf5340dk-bmi270_ble_marge`
- ボード: `nrf54l15dk/nrf54l15/cpuapp`
- 現在の状態:
  - build 成功
  - flash 成功
  - 起動ログ取得可能
  - BLE advertising まで確認済み
  - BMI270 初期化成功、UART CSV 出力まで確認済み
  - BLE notify を VIEWER 側で CSV 保存可能
  - BLE 切断後の再 advertising / 再接続成功を確認済み

## 実配線

TAG 側の BMI270 I2C 配線は以下を正とする。

- `P1.11 = I2C SCL`
- `P1.12 = I2C SDA`

現行 overlay ではこの配線に合わせて `TWIM_SCL/TWIM_SDA` を設定している。

## 使用中の I2C instance

- 使用中の I2C instance は `i2c21`
- 現行設定ファイル: `TAG/zephyr_apps/current/nrf54l15_port/app.overlay`
- BMI270 node:
  - `bmi270@68`
  - `compatible = "bosch,bmi270"`
  - `status = "okay"`
- 現行 buffer 設定:
  - `zephyr,concat-buf-size = <260>`
  - `zephyr,flash-buf-max-size = <260>`

## build / flash / log capture 手順

リポジトリルートで実行する。

### build

```bash
./scripts/tag_build.sh
```

### flash

```bash
./scripts/tag_flash.sh
```

補足:

- `tag_flash.sh` は `build/tag_nrf54l15_port` を使う
- 必要に応じて `west flash` の追加引数をそのまま渡せる

### log capture

```bash
./scripts/tag_log_capture.sh
./scripts/tag_log_capture.sh 10
```

補足:

- 既定ポートは `/dev/ttyACM1`
- 既定ボーレートは `115200`
- ログは `logs/` に `tag_log_YYYYMMDD_HHMMSS.log` 形式で保存される
- 画面にも出しつつ保存する
- 必要なら `PORT=/dev/ttyACM0 ./scripts/tag_log_capture.sh 5` のように上書き可能

## 運用方針

- 実歩行ログ取得の主経路は BLE notify + CSV 保存とする
- UART は補助用途とし、常時記録の主経路にはしない
- UART の用途は以下に限定する
  - 起動確認
  - advertising / notify 状態確認
  - 異常時切り分け

## 実歩行ログ取得の正式手順

BLE 主運用の実行場所を以下のように分ける。

- Windows PowerShell:
  - BLE central として TAG に接続
  - notify を subscribe
  - 実歩行 CSV を保存
- WSL:
  - build / flash
  - UART 補助確認
  - walking_analyzer 実行

### 0. flash 前の前提

- TAG は `nrf54l15dk/nrf54l15/cpuapp` 向け current ソースを使う
- build / flash はリポジトリルートで WSL から実行する
- BLE ログ保存先は WSL から参照できる場所に置く
  - 推奨: リポジトリ配下の `logs/ble/`
- Windows 側では `VIEWER/python/bmi270_BLE_viewer/requirements_ble_cli.txt` の依存を入れる

WSL:

```bash
cd ~/work/ILGA
./scripts/tag_build.sh
./scripts/tag_flash.sh
```

Windows PowerShell:

```powershell
cd <ILGA working copy>\VIEWER\python\bmi270_BLE_viewer
py -3 -m pip install -r .\requirements_ble_cli.txt
```

### 1. 起動と補助確認

TAG 起動直後の最小確認は UART で行う。BLE 本取得の前に以下だけ見ればよい。

WSL:

```bash
cd ~/work/ILGA
./scripts/tag_log_capture.sh 10
```

確認観点:

- `Bluetooth initialized`
- `Advertising start requested (startup)`
- `Advertising started (startup)`
- 異常時は `Advertising failed ...` や BMI init failure の有無を見る

### 2. BLE 受信開始と CSV 保存

Windows PowerShell で BLE notify を受け、CSV を保存する。保存 CSV は `walking_analyzer` がそのまま読める列名で出力される。

推奨保存先:

- `logs/ble/`
- 例: `logs/ble/walk_trial01.csv`

Windows PowerShell:

```powershell
cd <ILGA working copy>\VIEWER\python\bmi270_BLE_viewer
py -3 .\recv_bmi270_ble_notify_cli.py `
  --name BMI270_BLE_SAMPLE `
  --save-csv ..\..\..\logs\ble\walk_trial01.csv `
  --duration-sec 30 `
  --disconnect-on-finish
```

期待する PowerShell ログ:

- `[BLE] found ...`
- `[BLE] connected`
- `[BLE] subscribe ACC ...`
- `[BLE] subscribe GYR ...`
- `[CSV] saving merged samples to ...`
- `[BLE] notifications started; press Ctrl+C to stop`

CSV 保存先の扱い:

- 保存先は `--save-csv` で指定したパス
- 親ディレクトリは存在しなければ自動作成される
- 保存列は `seq,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps`

### 3. 再接続確認

実歩行前に 1 回は切断後再接続を確認する。

確認観点:

- 1 回目の取得終了後、2 回目の scan で `BMI270_BLE_SAMPLE` が再発見できる
- 2 回目の接続で再び CSV 保存を開始できる
- UART 補助ログで以下の流れが見える
  - `Disconnected: ...`
  - `BMI ACC notify DISABLED (disconnect)`
  - `BMI GYR notify DISABLED (disconnect)`
  - `Advertising restart scheduled (disconnect, 200 ms)`
  - `Advertising start requested (disconnect)`
  - `Advertising started (disconnect)`

### 4. 解析コマンド

BLE 保存 CSV の解析は WSL で行う。`walking_analyzer` は BLE 保存 CSV をそのまま入力にできる。

WSL:

```bash
cd ~/work/ILGA
python3 VIEWER/python/walking_analyzer/analyze_single_leg_csv.py \
  logs/ble/walk_trial01.csv \
  --fs 100 \
  --out-dir logs/ble/walk_trial01_analysis \
  --plot logs/ble/walk_trial01_analysis/diagnostic.png
```

解析結果:

- `summary.json`
- `step_events.csv`
- `diagnostic.png`

`diagnostic.png` は歩行ピークと閾値のざっくり確認用で、取得直後の簡易診断に使う。

### 5. 異常時の切り分け

BLE 取得が不安定なときだけ UART を併用する。

WSL:

```bash
cd ~/work/ILGA
./scripts/tag_log_capture.sh 15
```

主な確認点:

- advertising が再開しているか
- notify enable / disable が想定どおりか
- `Advertising failed ...` や notify error が出ていないか
- BLE central 側で別端末が掴んでいないか

## ログポート

- UART ログ取得ポートは `/dev/ttyACM1`
- ボーレートは `115200`

## 過去の BUS FAULT と解消経緯

過去に起動直後の BUS FAULT が発生していた。

- 観測された fault 情報:
  - `BUS FAULT`
  - `BFAR Address: 0x4`
  - `PC: 0x0002a812`
  - `LR: 0x0002a80d`
- 調査時の `addr2line`:
  - `0x0002a812 -> nrf_gpio_port_out_set`
  - `0x0002a80d -> nrf_gpio_pin_set`

推定を含む整理:

- fault は device init 中の GPIO 操作と整合していた
- 調査時点では BMI270 初期化経路が強い候補だった
- 一時的に BMI270 を disabled にすると BUS FAULT は消え、BLE advertising まで進んだ

その後の解消経緯:

1. `app.overlay` の I2C pinctrl を実配線に合わせて修正
   - `P1.11 = SCL`
   - `P1.12 = SDA`
2. BMI270 を再有効化
3. `i2c21` の concat buffer 不足を解消
   - `zephyr,concat-buf-size = <260>`
   - `zephyr,flash-buf-max-size = <260>`

現在は BUS FAULT は再現していない。

## BMI270 の現在状態

- BMI270 は現在 `status = "okay"`
- init 成功を確認済み
- `device_is_ready()` 通過済み
- accel / gyro の `sensor_attr_set()` 通過済み
- サンプル取得と UART CSV 出力を確認済み

## UART CSV の単位

現在の UART CSV は raw ではない。現在の単位は以下。

- accel: `mg`
- gyro: `mdps`

現在の出力形式:

```text
seq,ax,ay,az,gx,gy,gz
```

補足:

- 加速度は `sensor_ms2_to_mg()` で `mg` に変換
- 角速度は `sensor_rad_to_10udegrees() / 100` で `mdps` に変換
- 以前は `milli-m/s^2` と `milli-rad/s` 相当だったが、現在は `mg / mdps` に統一済み

## BLE notify の現状

- BMI270 の値は UART CSV だけでなく、BLE notify 経路にも格納される実装になっている
- Accel notify payload:
  - `seq(u32 LE) + ax(i16 LE) + ay(i16 LE) + az(i16 LE)`
  - 合計 `10 bytes`
- Gyro notify payload:
  - `seq(u32 LE) + gx(i32 LE) + gy(i32 LE) + gz(i32 LE)`
  - 合計 `16 bytes`
- 単位は UART と同じ
  - accel = `mg`
  - gyro = `mdps`

起動ログ上の UUID:

- device name: `BMI270_BLE_SAMPLE`
- ACC characteristic UUID: `12345678-1234-5678-1234-6789abcdef11`
- GYR characteristic UUID: `12345678-1234-5678-1234-6789abcdef12`

評価手順:

1. 中央機器から `BMI270_BLE_SAMPLE` に接続
2. ACC / GYR characteristic の notify を有効化
3. UART 側で以下を確認
   - `BMI ACC notify ENABLED`
   - `BMI GYR notify ENABLED`
   - 成功時: `BMI ACC notify active (10 bytes)`, `BMI GYR notify active (16 bytes)`
   - 失敗時: `BMI ACC notify err=...`, `BMI GYR notify err=...`

補足:

- このリポジトリの実装では、中央機器未接続または未 subscribe 状態では notify は送られない
- これまでの作業ではコード経路と payload 整合までは確認済み
- over-the-air notify 受信、CSV 保存、切断後の再接続は確認済み

## 未解決事項 / 今後の確認項目

- 長時間連続動作時の安定性は未確認
- UART CSV の数値妥当性は概ね整合しているが、厳密な較正評価は未実施
- BLE notify 側の受信側実装が `mg / mdps` 前提であることを再確認すると安全
- `main.c` 内の既存 warning 1 件は未対応
  - build 時に deprecated macro warning が出る
- `i2c21` 以外への切り替えは現時点では不要
  - 実配線と現行動作は `i2c21` で整合している
