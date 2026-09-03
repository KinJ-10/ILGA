# GA-Phase1A BLE capture helper

This helper records one BLE trial through the existing Viewer CLI, using 30 seconds by default, and writes the raw CSV, complete console/summary log and structured metadata. Walk10m, distance Walk and TUG also write a separate operator START/FINISH marker CSV. Scan, connection, progress, marker and summary lines are displayed live while the same lines are saved to the log. It does not run the generic Analyzer automatically. Japanese marker operation details are in [GA_OPERATOR_MARKERS_JA.md](GA_OPERATOR_MARKERS_JA.md).

## Walk capture — shortest command

Walk is the default TrialType. Open PowerShell 7 in the ILGA repository and run:

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\run_ga_phase1a_capture.ps1 -TestType Walk10m -TrialName Kin_walk1 -SensorFoot Right -StartFoot Right -EndCondition "stop after line"
```

Walk10m is the default TestType and is fixed at 10 m. Set TrialName, SensorFoot, StartFoot and EndCondition. `SensorFoot` accepts `Right`, `Left` or `Unknown`; it means the foot carrying TAG, not the first foot to step. Omission is stored as `Unknown` with source `default_unknown`, never silently assumed to be Right. `StartFoot` accepts `Right`, `Left` or `None`. After a successful Walk capture summary, the script asks for ActualSteps and EndFoot. If either value is uncertain, enter blank or `Unknown`; EndFoot also accepts `None`. Metadata stores an unknown value as JSON null, records a `post_capture_unknown` source, and marks the prompt as completed rather than pending. It never substitutes zero or a guessed value. If already known, both may be supplied with `-ActualSteps 16 -EndFoot Left`. `-EndFoot None` is stored as unknown/null with source `command_line_unknown`. For a slow Walk that cannot complete the full pre-sync through post-sync sequence in 30 seconds, add a longer value such as `-DurationSec 45`.

## TUG capture — shortest command

TUG uses the same operator markers without assigning a 10 m distance. Extend DurationSec when needed.

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\run_ga_phase1a_capture.ps1 -TestType TUG -TrialName Kin_tug1 -SensorFoot Right -StartFoot Right -EndCondition "seated after TUG" -DurationSec 45
```

For another known walking distance, use `-TestType Walk -DistanceM <meters>`; this keeps the distance explicit instead of labeling it as 10 m.

## Static capture — 30-second command

Static performs complete stillness only. It automatically uses ActualSteps=0, StartFoot=None and EndFoot=None, with no synchronization stomps, 10 m walk or terminal action.

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\run_ga_phase1a_capture.ps1 -TrialType Static -TrialName Kin_stop30s -DurationSec 30
```

The default output stem is `logs\ble\YYYYMMDD_ga_phase1a_<TrialName>`. Walk10m, distance Walk and TUG create a sensor CSV plus `_markers.csv`, `_summary.log` and `_metadata.json`. The script refuses to start if any matching output already exists. Static creates no marker CSV.

## Walk trial procedure

1. Confirm only one BLE Viewer/client is running.
2. Record the PC-TAG distance, TAG orientation/body side, PC AC/battery and power mode, and noteworthy background applications in the operator notes.
3. Start the command.
4. Perform three clear pre-sync synchronization stomps.
5. Stand still for the protocol-defined pre-walk interval.
6. At the defined interval START, press `s` once and begin the 10 m walk (or selected test), using the stated starting foot.
7. At the defined interval FINISH, press `f` once, apply the stated end condition and count the actual steps.
8. Stand still for 3-5 seconds after the terminal condition.
9. Perform three clear post-sync synchronization stomps before capture stops.
10. Let capture finish and wait for the Viewer summary.
11. When prompted, enter the hand-counted ActualSteps and EndFoot. If uncertain, enter blank/Unknown instead of zero or a guess. Then verify the sensor and marker quality metrics.

## Static trial procedure

1. Start the Static command and remain completely still for the specified DurationSec.
2. Do not perform pre/post synchronization stomps, walking or a terminal action.
3. Let capture stop at the configured duration, then verify the four quality metrics. Static does not prompt for ActualSteps or EndFoot.

Metadata records TrialType and uses a separate protocol structure. Walk records pre-sync x3, pre-walk stillness, 10 m, terminal condition, 3-5 seconds stillness and post-sync x3. Static records complete stillness for DurationSec, zero synchronization stomps, zero walk distance and no terminal action, with ActualSteps=0, StartFoot=None and EndFoot=None. It also records the receive timestamp definition, paths and these summary metrics:

- `missing_seq`
- `invalid_sensor_samples`
- `sensor_fault`
- `rx_timestamp_effective_hz`
- `markers_file`
- `start_count`
- `finish_count`
- `marker_valid`
- `marked_duration_sec`

The receive timestamp is the PC-side time at BLE notification callback entry. It is not the TAG sensor acquisition time. If missing_seq, invalid_sensor_samples or sensor_fault is nonzero, the script prints a CAPTURE QUALITY WARNING and preserves the CSV. For marker-enabled trials, missing, reversed or duplicate markers produce marker_valid=0 and a separate quality warning while preserving both CSV files. Quality warnings do not turn a successful Viewer exit into a failure; a Viewer failure keeps its nonzero exit code. Marker time is PC operator input time, not TAG acquisition or foot-contact time.

## Safe checks without hardware

Summary extraction, quality-warning, ActualSteps/EndFoot validation, Walk/Static validation and overwrite-refusal self-test:

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\run_ga_phase1a_capture.ps1 -SelfTest
```

Dry-run does not prompt, connect or create files. Check both protocol branches with:

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\run_ga_phase1a_capture.ps1 -TestType Walk10m -TrialName Kin_walk1 -SensorFoot Right -StartFoot Right -EndCondition "stop after line" -DryRun
pwsh -ExecutionPolicy Bypass -File .\scripts\run_ga_phase1a_capture.ps1 -TestType TUG -TrialName Kin_tug1 -SensorFoot Right -StartFoot Right -EndCondition "seated after TUG" -DurationSec 45 -DryRun
pwsh -ExecutionPolicy Bypass -File .\scripts\run_ga_phase1a_capture.ps1 -TrialType Static -TrialName Kin_stop30s -DurationSec 30 -DryRun
```

## Optional analysis

Do not use the current generic Analyzer for automatic step acceptance, especially for stop30s, because stillness may still produce peaks. If an operator explicitly wants exploratory analysis after reviewing capture quality, run the Analyzer manually against the new CSV and write to a new, non-existing output location.
