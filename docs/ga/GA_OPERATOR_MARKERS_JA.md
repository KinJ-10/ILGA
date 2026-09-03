# GA 10 m歩行・TUG オペレーターマーカー手順

## 目的と時刻の定義

BLEセンサCSVとは別に、測定者がキーを押したSTART/FINISHを専用CSVへ記録します。10 m歩行では測定区間の開始・終了、TUGでは合図から規定終了までを示すためのオペレーター時刻です。足接地イベントやTAG側のセンサ取得時刻ではありません。

キー取得直後にPythonの`time.monotonic_ns()`を読みます。`marker_elapsed_ns`は最初に受理したBLE notificationの`rx_monotonic_ns`を0とした同一PCクロック上の相対時刻です。`marked_duration_sec`はFINISHとSTARTの差です。

## 最短コマンド

PowerShell 7でILGAリポジトリを開いて実行します。

10 m歩行:

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\run_ga_phase1a_capture.ps1 -TestType Walk10m -TrialName Kin_walk1 -StartFoot Right -EndCondition "stop after line"
```

TUG（30秒で不足する場合を考慮した45秒例）:

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\run_ga_phase1a_capture.ps1 -TestType TUG -TrialName Kin_tug1 -StartFoot Right -EndCondition "seated after TUG" -DurationSec 45
```

10 m以外の歩行は`-TestType Walk -DistanceM 4`のように距離を明示します。`Walk10m`は10 m固定です。Staticではマーカー機能を起動せず、従来どおり完全静止だけを記録します。

## 測定時の操作

1. BLE scan・接続・notification開始を画面で確認します。
2. 標準手順のpre同期踏み込み3回と静止を行います。同期踏み込みはSTART/FINISHの対象外です。
3. 10 m歩行では歩行開始の瞬間、TUGでは開始合図の瞬間に`s`キーを1回押します。
4. 10 m歩行では規定の終了線・終了条件の瞬間、TUGでは規定の終了状態になった瞬間に`f`キーを1回押します。
5. 終端後の静止3〜5秒とpost同期踏み込み3回を行い、記録終了を待ちます。
6. 取得成功後にActualStepsとEndFootを入力し、センサ品質とマーカー品質を確認します。

大文字・小文字のどちらでも受け付けます。画面には`[MARKER] recorded START ...`または`[MARKER] recorded FINISH ...`が即時表示されます。

同じ種類のキーを2回以上押した場合、最初の値を保持して重複を拒否します。START/FINISH不足、FINISHがSTART以前、重複、BLE受信基準時刻不足は`marker_valid=0`となりWARNINGを表示します。センサCSV、マーカーCSV、summary logは削除しません。マーカー品質だけではViewerの成功exit codeを失敗へ変更しません。

## 出力ファイル

Walk10m・Walk・TUGでは同じstemで次の4ファイルを作成します。

- `YYYYMMDD_ga_phase1a_<TrialName>.csv`: BLEセンサ値
- `YYYYMMDD_ga_phase1a_<TrialName>_markers.csv`: START/FINISH
- `YYYYMMDD_ga_phase1a_<TrialName>_summary.log`: 画面と同じ完全ログ
- `YYYYMMDD_ga_phase1a_<TrialName>_metadata.json`: 試行条件・品質値・各ファイルパス

いずれかの対象ファイルが既に存在する場合、取得開始前に上書きを拒否します。StaticはマーカーCSVを作成しません。

マーカーCSVのschema version 1は次の列順で固定します。

```text
schema_version,trial_name,event,event_index,marker_monotonic_ns,marker_elapsed_ns,source,notes
```

- `event`: `START`または`FINISH`
- `event_index`: 受理したマーカーの試行内順序
- `marker_monotonic_ns`: キー取得直後のPC monotonic時刻
- `marker_elapsed_ns`: 最初の受理BLE notificationからの相対時刻
- `source`: `operator_key`
- `notes`: 不完全・逆順・重複拒否などの品質注記

summaryとmetadataには`markers_file`、`start_count`、`finish_count`、`marker_valid`、`marked_duration_sec`を保存します。マーカー無効時はdurationを`NA`として扱います。

## 実機なしの確認

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\run_ga_phase1a_capture.ps1 -SelfTest
pwsh -ExecutionPolicy Bypass -File .\scripts\run_ga_phase1a_capture.ps1 -TestType Walk10m -TrialName dry_walk -StartFoot Right -EndCondition "stop after line" -DryRun
pwsh -ExecutionPolicy Bypass -File .\scripts\run_ga_phase1a_capture.ps1 -TestType TUG -TrialName dry_tug -StartFoot Right -EndCondition "seated after TUG" -DurationSec 45 -DryRun
```

dry-runはBLE接続、キー待ち、ファイル作成を行いません。Windowsコンソールでの実キー入力と実際の操作タイミングは、次回のpilot試験で確認します。
