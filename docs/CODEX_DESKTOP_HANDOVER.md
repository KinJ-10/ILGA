# Codex Desktop 導入・引継ぎ手順

## 採用構成

ILGAでは以下を標準構成とする。

- 正本リポジトリ: WSL2の `~/work/ILGA`
- Codex Desktop agent: WSL
- Codex Desktop integrated terminal: PowerShell
- TAG build / flash / UART: WSL
- BLE受信 / CSV保存 / リアルタイム表示: Windows PowerShell
- Codex CLI: WSLに残す

Windows側とWSL側に別々のILGA cloneを作らない。コード、ログ、解析結果はWSL側の1つの作業コピーを共有する。

## 1. 既存作業コピーの保護

既存のWSL作業コピーがある場合、最初に以下を実行する。

```bash
cd ~/work/ILGA
git status --short --branch
git remote -v
git rev-parse HEAD
```

未commitの変更が表示された場合は、その出力を保存し、内容を確認するまでpullやcloneのやり直しを行わない。

WSLに作業コピーがない場合のみ、以下を実行する。

```bash
mkdir -p ~/work
cd ~/work
git clone https://github.com/KinJ-10/ILGA.git
cd ILGA
git switch main
```

## 2. Codex Desktopの導入

Windows PowerShellで以下を実行する。

```powershell
winget install --id 9PLM9XGG6VKS -s msstore
```

インストール後、ChatGPT Desktopを起動し、Codexを選択する。

## 3. DesktopをWSL agentへ変更

1. DesktopのSettingsを開く。
2. Codex agentをWindows nativeからWSLへ変更する。
3. integrated terminalはPowerShellを選択する。
4. Desktopを完全に再起動する。

agentとintegrated terminalは別設定である。WSL agentのままPowerShell terminalを利用できる。

## 4. ILGAをLocal Projectとして開く

1. DesktopでAdd new projectを選択する。
2. ファイル選択欄へ `\\wsl$\` を入力する。
3. Ubuntuの `home/<WSL-user>/work/ILGA` を選択する。
4. ILGAフォルダをPrimary folderにする。
5. Projectをtrustedとして開く。

Desktopはリポジトリルートの `AGENTS.md` を自動読込する。新しいチャットで以下を確認する。

```text
このプロジェクトで読み込んだ指示ファイルと、現在のILGA到達点を列挙して。
```

期待結果:

- `AGENTS.md` が読み込まれる。
- `docs/CURRENT_STATUS.md` が現在状態として参照される。
- TAGのcurrentとsnapshotsを区別する。

## 5. WSL環境の確認

WSLで以下を実行する。

```bash
cd ~/work/ILGA
chmod +x scripts/desktop_preflight_wsl.sh
./scripts/desktop_preflight_wsl.sh
```

確認後、TAG buildを行う。

```bash
./scripts/tag_build.sh
```

実機接続時のみflashとUART確認を行う。

```bash
./scripts/tag_flash.sh
./scripts/tag_log_capture.sh 10
```

## 6. Windows BLE環境の確認

Windows PowerShellで、WSL上のリポジトリをUNCパスで指定する。

```powershell
$IlgaRepo = "\\wsl.localhost\Ubuntu\home\<WSL-user>\work\ILGA"
py -3.12 -m pip install -r "$IlgaRepo\VIEWER\python\bmi270_BLE_viewer\requirements_desktop.txt"
powershell -ExecutionPolicy Bypass -File "$IlgaRepo\scripts\desktop_preflight_windows.ps1" -RepoPath $IlgaRepo
```

`Ubuntu`は実際のdistribution名、`<WSL-user>`はWSLのユーザー名に置き換える。

## 7. 同じ進捗状態までの動作確認

### TAG

- `./scripts/tag_build.sh` が成功する。
- 実機接続時に `./scripts/tag_flash.sh` が成功する。
- UARTに `Bluetooth initialized`、`Advertising started` が出る。
- BMI270 init failureやBUS FAULTが出ない。

### BLE / CSV

PowerShellで以下を実行する。

```powershell
$IlgaRepo = "\\wsl.localhost\Ubuntu\home\<WSL-user>\work\ILGA"
py -3.12 "$IlgaRepo\VIEWER\python\bmi270_BLE_viewer\recv_bmi270_ble_notify_cli.py" `
  --name BMI270_BLE_SAMPLE `
  --save-csv "$IlgaRepo\logs\ble\desktop_handover_30s.csv" `
  --duration-sec 30 `
  --disconnect-on-finish
```

期待結果:

- TAGを発見して接続できる。
- ACC/GYR notifyをsubscribeできる。
- CSVが作成される。
- summaryに受信packet数、completed samples、missing seqが表示される。
- 取得終了後に再実行して再接続できる。

### GA解析

WSLで以下を実行する。

```bash
cd ~/work/ILGA
python3 VIEWER/python/walking_analyzer/analyze_single_leg_csv.py \
  logs/ble/desktop_handover_30s.csv \
  --fs 100 \
  --out-dir logs/ble/desktop_handover_30s_analysis \
  --plot logs/ble/desktop_handover_30s_analysis/diagnostic.png
```

注意: `--fs 100`は現在の設定値である。実効受信レートが100 Hzでない場合、時間指標の評価には実効レートを使用して再解析する。

## 8. CLIとの切り分け

### Desktopを主に使う作業

- 複数ファイルにまたがる実装とレビュー
- 設計方針、実験計画、長時間作業
- diff確認、worktreeを使う独立作業
- 添付資料、画像、PDFを含む調査と成果物作成
- `docs/CURRENT_STATUS.md` の更新

### CLIを残す作業

- WSL端末での短いbuild / flash / UART反復
- `/dev/ttyACM*` を直接確認する作業
- Desktopを開かずに行う緊急のログ確認
- shell pipelineを目で追いながら行う切り分け
- Desktop側の不具合時の代替経路

同じファイルをDesktopとCLIで同時編集しない。実機作業中はCLI、設計・実装作業中はDesktopという単位で切り替える。

## 9. DesktopとCLIの設定共有

プロジェクト固有の指示はリポジトリの `AGENTS.md` と `docs/` で共有するため、最初はCodex homeの移動を行わなくても開発状態を引き継げる。

Windows Desktopは `%USERPROFILE%\.codex`、WSL CLIは通常 `~/.codex` を使用する。認証、個人設定、session historyまで共有したい場合は、動作確認後にWSLの `CODEX_HOME` をWindows側へ向ける方法を検討する。初回導入時は変更しない。

## 10. 完了条件

以下がすべて成立すればDesktop移行完了とする。

1. DesktopがWSL上のILGAをPrimary folderとして開ける。
2. `AGENTS.md` と `docs/CURRENT_STATUS.md` を読み込める。
3. WSL preflightが重大エラーなしで完了する。
4. Windows preflightが重大エラーなしで完了する。
5. TAG buildが成功する。
6. 実機へflashでき、起動ログを取得できる。
7. Windowsで30秒BLE CSVを保存できる。
8. 切断後に再接続できる。
9. WSLでwalking analyzerを実行し、3種類の結果ファイルを生成できる。
