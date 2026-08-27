#requires -Version 7.0
[CmdletBinding()]
param(
    [string]$TrialName,
    [ValidateSet("Walk", "Static")]
    [string]$TrialType = "Walk",
    [Nullable[int]]$ActualSteps,
    [string]$EndFoot,
    [ValidateSet("Right", "Left", "None")]
    [string]$StartFoot,
    [string]$EndCondition,
    [ValidateRange(1, 3600)]
    [int]$DurationSec = 30,
    [string]$RepoPath = (Split-Path -Parent $PSScriptRoot),
    [string]$OutputDirectory,
    [switch]$DryRun,
    [switch]$SelfTest
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function ConvertFrom-IlgaSummaryLine {
    param([Parameter(Mandatory)][string]$Line)

    $summaryIndex = $Line.IndexOf("[SUMMARY]", [StringComparison]::Ordinal)
    if ($summaryIndex -lt 0) {
        throw "The input does not contain a [SUMMARY] record."
    }

    $values = [ordered]@{}
    $summaryText = $Line.Substring($summaryIndex + "[SUMMARY]".Length)
    $matches = [regex]::Matches(
        $summaryText,
        "(?<key>[A-Za-z_][A-Za-z0-9_]*)=(?<value>'[^']*'|\S+)"
    )
    foreach ($match in $matches) {
        $value = $match.Groups["value"].Value
        if ($value.Length -ge 2 -and $value.StartsWith("'") -and $value.EndsWith("'")) {
            $value = $value.Substring(1, $value.Length - 2)
        }
        $values[$match.Groups["key"].Value] = $value
    }
    return [pscustomobject]$values
}

function Get-IlgaOutputPaths {
    param(
        [Parameter(Mandatory)][string]$Directory,
        [Parameter(Mandatory)][string]$Name,
        [Parameter(Mandatory)][string]$DateStamp
    )
    $stem = "{0}_ga_phase1a_{1}" -f $DateStamp, $Name
    return [pscustomobject]@{
        Csv = Join-Path $Directory ($stem + ".csv")
        SummaryLog = Join-Path $Directory ($stem + "_summary.log")
        Metadata = Join-Path $Directory ($stem + "_metadata.json")
    }
}

function Assert-IlgaOutputsAvailable {
    param([Parameter(Mandatory)]$Paths)
    foreach ($path in @($Paths.Csv, $Paths.SummaryLog, $Paths.Metadata)) {
        if (Test-Path -LiteralPath $path) {
            throw "Refusing to overwrite existing output: $path"
        }
    }
}

function ConvertTo-NullableNumber {
    param($Summary, [string]$Name, [ValidateSet("int", "double")][string]$Type)
    if ($null -eq $Summary -or $null -eq $Summary.PSObject.Properties[$Name]) {
        return $null
    }
    if ($Type -eq "int") {
        return [int64]$Summary.$Name
    }
    return [double]::Parse(
        [string]$Summary.$Name,
        [Globalization.CultureInfo]::InvariantCulture
    )
}

function Get-IlgaSummaryLine {
    param([Parameter(Mandatory)][AllowEmptyCollection()][object[]]$Lines)
    return $Lines |
        ForEach-Object { [string]$_ } |
        Where-Object { $_ -match "\[SUMMARY\]" } |
        Select-Object -Last 1
}

function Get-IlgaQualityWarnings {
    param([Parameter(Mandatory)]$Metrics)

    $warnings = [Collections.Generic.List[string]]::new()
    foreach ($name in @("missing_seq", "invalid_sensor_samples", "sensor_fault")) {
        $value = if ($Metrics -is [Collections.IDictionary]) {
            $Metrics[$name]
        }
        else {
            $property = $Metrics.PSObject.Properties[$name]
            if ($null -ne $property) { $property.Value } else { $null }
        }
        if ($null -ne $value -and [int64]$value -ne 0) {
            $warnings.Add("$name=$value")
        }
    }
    return $warnings.ToArray()
}

function Format-IlgaMetricValue {
    param($Value)
    if ($null -eq $Value) {
        return "unavailable"
    }
    if ($Value -is [double]) {
        return $Value.ToString("0.000000", [Globalization.CultureInfo]::InvariantCulture)
    }
    return [string]$Value
}

function ConvertTo-IlgaActualSteps {
    param([AllowNull()]$Value)
    $parsed = 0
    if ($null -eq $Value -or -not [int]::TryParse([string]$Value, [ref]$parsed) -or $parsed -lt 0) {
        throw "ActualSteps must be an integer of zero or greater."
    }
    return $parsed
}

function ConvertTo-IlgaFoot {
    param([AllowNull()]$Value, [string]$Name = "Foot")
    if ($null -eq $Value) {
        throw "$Name must be Right, Left, or None."
    }
    switch ([string]$Value.Trim().ToLowerInvariant()) {
        "right" { return "Right" }
        "left" { return "Left" }
        "none" { return "None" }
        default { throw "$Name must be Right, Left, or None." }
    }
}

function Read-IlgaActualSteps {
    while ($true) {
        $value = Read-Host "Enter ActualSteps (integer >= 0)"
        try { return ConvertTo-IlgaActualSteps -Value $value }
        catch { Write-Warning $_.Exception.Message }
    }
}

function Read-IlgaEndFoot {
    while ($true) {
        $value = Read-Host "Enter EndFoot (Right/Left/None)"
        try { return ConvertTo-IlgaFoot -Value $value -Name "EndFoot" }
        catch { Write-Warning $_.Exception.Message }
    }
}

function Resolve-IlgaTrialInputs {
    param(
        [Parameter(Mandatory)][ValidateSet("Walk", "Static")][string]$Type,
        [AllowNull()]$ActualStepsValue,
        [AllowNull()]$StartFootValue,
        [AllowNull()]$EndFootValue,
        [AllowNull()]$EndConditionValue
    )

    if ($Type -eq "Static") {
        if ($null -ne $ActualStepsValue -and (ConvertTo-IlgaActualSteps -Value $ActualStepsValue) -ne 0) {
            throw "Static trials require ActualSteps=0."
        }
        if (-not [string]::IsNullOrWhiteSpace([string]$StartFootValue) -and
            (ConvertTo-IlgaFoot -Value $StartFootValue -Name "StartFoot") -ne "None") {
            throw "Static trials require StartFoot=None."
        }
        if (-not [string]::IsNullOrWhiteSpace([string]$EndFootValue) -and
            (ConvertTo-IlgaFoot -Value $EndFootValue -Name "EndFoot") -ne "None") {
            throw "Static trials require EndFoot=None."
        }
        if (-not [string]::IsNullOrWhiteSpace([string]$EndConditionValue) -and
            [string]$EndConditionValue -notin @("stop30s", "duration_complete", "none")) {
            throw "Static trials do not use an end action; omit EndCondition or use stop30s/duration_complete/none."
        }
        return [pscustomobject]@{
            ActualSteps = 0
            StartFoot = "None"
            EndFoot = "None"
            EndCondition = "duration_complete"
            ActualStepsInputPending = $false
            EndFootInputPending = $false
            ActualStepsSource = "trial_type_static"
            EndFootSource = "trial_type_static"
        }
    }

    if ([string]::IsNullOrWhiteSpace([string]$StartFootValue) -or
        [string]::IsNullOrWhiteSpace([string]$EndConditionValue)) {
        throw "Walk trials require StartFoot and EndCondition."
    }
    $resolvedStartFoot = ConvertTo-IlgaFoot -Value $StartFootValue -Name "StartFoot"
    $actualPending = $null -eq $ActualStepsValue
    $endPending = [string]::IsNullOrWhiteSpace([string]$EndFootValue)
    return [pscustomobject]@{
        ActualSteps = if ($actualPending) { $null } else { ConvertTo-IlgaActualSteps -Value $ActualStepsValue }
        StartFoot = $resolvedStartFoot
        EndFoot = if ($endPending) { $null } else { ConvertTo-IlgaFoot -Value $EndFootValue -Name "EndFoot" }
        EndCondition = [string]$EndConditionValue
        ActualStepsInputPending = $actualPending
        EndFootInputPending = $endPending
        ActualStepsSource = if ($actualPending) { "post_capture_prompt" } else { "command_line" }
        EndFootSource = if ($endPending) { "post_capture_prompt" } else { "command_line" }
    }
}

function Invoke-IlgaSelfTest {
    $sample = "[SUMMARY] received=2994 missing_seq=7 invalid_sensor_samples=0 sensor_fault=0 rx_timestamp_effective_hz=99.800 csv='C:\data files\trial.csv'"
    $summaryLine = Get-IlgaSummaryLine -Lines @(
        "[SCAN] device discovery started",
        "[PROGRESS] received=1000",
        $sample,
        "[INFO] disconnected"
    )
    $parsed = ConvertFrom-IlgaSummaryLine -Line $summaryLine
    if (
        $summaryLine -ne $sample -or
        $parsed.missing_seq -ne "7" -or
        $parsed.invalid_sensor_samples -ne "0" -or
        $parsed.sensor_fault -ne "0" -or
        $parsed.rx_timestamp_effective_hz -ne "99.800" -or
        $parsed.csv -ne "C:\data files\trial.csv"
    ) {
        throw "Summary extraction self-test failed."
    }

    $cleanMetrics = [ordered]@{
        missing_seq = 0
        invalid_sensor_samples = 0
        sensor_fault = 0
        rx_timestamp_effective_hz = 99.8
    }
    if (@(Get-IlgaQualityWarnings -Metrics $cleanMetrics).Count -ne 0) {
        throw "Clean quality warning self-test failed."
    }
    $warningMetrics = [ordered]@{
        missing_seq = 7
        invalid_sensor_samples = 2
        sensor_fault = 1
        rx_timestamp_effective_hz = 95.0
    }
    $warnings = @(Get-IlgaQualityWarnings -Metrics $warningMetrics)
    if (
        $warnings.Count -ne 3 -or
        $warnings -notcontains "missing_seq=7" -or
        $warnings -notcontains "invalid_sensor_samples=2" -or
        $warnings -notcontains "sensor_fault=1"
    ) {
        throw "Quality warning self-test failed."
    }

    if (
        (ConvertTo-IlgaActualSteps -Value "0") -ne 0 -or
        (ConvertTo-IlgaActualSteps -Value "17") -ne 17 -or
        (ConvertTo-IlgaFoot -Value "right" -Name "EndFoot") -ne "Right" -or
        (ConvertTo-IlgaFoot -Value "Left" -Name "EndFoot") -ne "Left" -or
        (ConvertTo-IlgaFoot -Value "none" -Name "EndFoot") -ne "None"
    ) {
        throw "Post-capture input normalization self-test failed."
    }
    foreach ($invalidSteps in @("", "-1", "1.5", "abc")) {
        $rejected = $false
        try { ConvertTo-IlgaActualSteps -Value $invalidSteps | Out-Null }
        catch { $rejected = $true }
        if (-not $rejected) { throw "ActualSteps rejection self-test failed for '$invalidSteps'." }
    }
    foreach ($invalidFoot in @("", "Both", "R")) {
        $rejected = $false
        try { ConvertTo-IlgaFoot -Value $invalidFoot -Name "EndFoot" | Out-Null }
        catch { $rejected = $true }
        if (-not $rejected) { throw "EndFoot rejection self-test failed for '$invalidFoot'." }
    }

    $walkTrial = Resolve-IlgaTrialInputs -Type Walk -ActualStepsValue $null -StartFootValue Right -EndFootValue $null -EndConditionValue "stop after line"
    if (-not $walkTrial.ActualStepsInputPending -or -not $walkTrial.EndFootInputPending -or $walkTrial.StartFoot -ne "Right") {
        throw "Walk trial pending-input self-test failed."
    }
    $staticTrial = Resolve-IlgaTrialInputs -Type Static -ActualStepsValue $null -StartFootValue $null -EndFootValue $null -EndConditionValue "stop30s"
    if (
        $staticTrial.ActualSteps -ne 0 -or
        $staticTrial.StartFoot -ne "None" -or
        $staticTrial.EndFoot -ne "None" -or
        $staticTrial.EndCondition -ne "duration_complete" -or
        $staticTrial.ActualStepsInputPending -or
        $staticTrial.EndFootInputPending
    ) {
        throw "Static trial automatic-input self-test failed."
    }
    foreach ($invalidStatic in @(
        @{ ActualSteps = 1; StartFoot = "None"; EndFoot = "None" },
        @{ ActualSteps = 0; StartFoot = "Right"; EndFoot = "None" },
        @{ ActualSteps = 0; StartFoot = "None"; EndFoot = "Left" }
    )) {
        $rejected = $false
        try {
            Resolve-IlgaTrialInputs -Type Static -ActualStepsValue $invalidStatic.ActualSteps -StartFootValue $invalidStatic.StartFoot -EndFootValue $invalidStatic.EndFoot -EndConditionValue "stop30s" | Out-Null
        }
        catch { $rejected = $true }
        if (-not $rejected) { throw "Static trial contradiction self-test failed." }
    }
    $invalidWalkRejected = $false
    try { Resolve-IlgaTrialInputs -Type Walk -ActualStepsValue $null -StartFootValue $null -EndFootValue $null -EndConditionValue $null | Out-Null }
    catch { $invalidWalkRejected = $true }
    if (-not $invalidWalkRejected) { throw "Walk required-input self-test failed." }

    $testDirectory = Join-Path ([IO.Path]::GetTempPath()) ("ilga_capture_selftest_" + [guid]::NewGuid())
    [IO.Directory]::CreateDirectory($testDirectory) | Out-Null
    try {
        $paths = Get-IlgaOutputPaths -Directory $testDirectory -Name "collision" -DateStamp "20260827"
        [IO.File]::WriteAllText($paths.Csv, "test")
        $collisionRejected = $false
        try {
            Assert-IlgaOutputsAvailable -Paths $paths
        }
        catch {
            $collisionRejected = $_.Exception.Message.StartsWith("Refusing to overwrite")
        }
        if (-not $collisionRejected) {
            throw "Overwrite refusal self-test failed."
        }
    }
    finally {
        if ($testDirectory.StartsWith([IO.Path]::GetTempPath(), [StringComparison]::OrdinalIgnoreCase)) {
            Remove-Item -LiteralPath $testDirectory -Recurse -Force
        }
    }
    Write-Host "SELFTEST PASS: summary extraction, quality warning detection, ActualSteps/EndFoot validation, Walk/Static validation, and overwrite refusal"
}

if ($SelfTest) {
    Invoke-IlgaSelfTest
    return
}

if ([string]::IsNullOrWhiteSpace($TrialName)) {
    throw "TrialName is required."
}
if ($TrialName -notmatch "^[A-Za-z0-9][A-Za-z0-9_-]*$") {
    throw "TrialName may contain only letters, digits, underscore and hyphen."
}
$resolvedInputs = Resolve-IlgaTrialInputs -Type $TrialType -ActualStepsValue $ActualSteps -StartFootValue $StartFoot -EndFootValue $EndFoot -EndConditionValue $EndCondition
$ActualSteps = $resolvedInputs.ActualSteps
$StartFoot = $resolvedInputs.StartFoot
$EndFoot = $resolvedInputs.EndFoot
$EndCondition = $resolvedInputs.EndCondition
$actualStepsInputPending = $resolvedInputs.ActualStepsInputPending
$endFootInputPending = $resolvedInputs.EndFootInputPending

if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $OutputDirectory = Join-Path $RepoPath "logs\ble"
}
$viewerPath = Join-Path $RepoPath "VIEWER\python\bmi270_BLE_viewer\recv_bmi270_ble_notify_cli.py"
if (-not (Test-Path -LiteralPath $viewerPath -PathType Leaf)) {
    throw "Viewer CLI not found: $viewerPath"
}

$dateStamp = Get-Date -Format "yyyyMMdd"
$paths = Get-IlgaOutputPaths -Directory $OutputDirectory -Name $TrialName -DateStamp $dateStamp
Assert-IlgaOutputsAvailable -Paths $paths

$viewerArguments = @(
    "-3.12",
    "-u",
    $viewerPath,
    "--name", "BMI270_BLE_SAMPLE",
    "--save-csv", $paths.Csv,
    "--duration-sec", $DurationSec,
    "--disconnect-on-finish"
)

$protocol = if ($TrialType -eq "Static") {
    [ordered]@{
        mode = "static"
        complete_stillness = $true
        duration_sec = $DurationSec
        synchronization = [ordered]@{
            performed = $false
            pre = [ordered]@{ action = "none"; count = 0 }
            post = [ordered]@{ action = "none"; count = 0 }
            total_count = 0
        }
        walk_distance_m = 0
        terminal_action = "none"
        sequence = @("static_stillness_for_duration", "capture_stop")
    }
}
else {
    [ordered]@{
        mode = "walk"
        synchronization = [ordered]@{
            pre = [ordered]@{ action = "stomp"; count = 3; timing = "before pre-walk stillness" }
            post = [ordered]@{ action = "stomp"; count = 3; timing = "after terminal stillness, before capture stop" }
            total_count = 6
        }
        pre_walk_stillness = "Stand still before the 10 m walk as directed by the protocol."
        walk_distance_m = 10
        terminal_condition = $EndCondition
        post_terminal_stillness_sec = [ordered]@{ minimum = 3; maximum = 5 }
        sequence = @(
            "pre_sync_3_stomps",
            "pre_walk_stillness",
            "walk_10m",
            "terminal_condition",
            "post_terminal_stillness_3_to_5_sec",
            "post_sync_3_stomps",
            "capture_stop"
        )
    }
}

$metadataBase = [ordered]@{
    schema_version = 1
    trial_type = $TrialType
    trial_name = $TrialName
    actual_steps = if ($actualStepsInputPending) { $null } else { [int]$ActualSteps }
    actual_steps_source = $resolvedInputs.ActualStepsSource
    start_foot = $StartFoot
    end_foot = if ($endFootInputPending) { $null } else { $EndFoot }
    end_foot_source = $resolvedInputs.EndFootSource
    end_condition = $EndCondition
    post_capture_input_pending = [ordered]@{
        actual_steps = $actualStepsInputPending
        end_foot = $endFootInputPending
    }
    protocol = $protocol
    receive_timestamp_definition = "PC-side BLE notification callback entry; not sensor acquisition time"
    capture_duration_sec = $DurationSec
    csv_path = $paths.Csv
    summary_log_path = $paths.SummaryLog
    analyzer_automatic = $false
}

if ($DryRun) {
    Write-Host "DRY RUN: no files created and no BLE connection attempted."
    if ($TrialType -eq "Static") {
        Write-Host "Protocol: complete stillness for $DurationSec sec -> capture stop (no synchronization stomps, no walk, no terminal action)."
    }
    else {
        Write-Host "Protocol: pre sync stomps x3 -> stillness -> 10 m walk -> terminal condition -> stillness 3-5 sec -> post sync stomps x3 -> capture stop (6 sync stomps total)."
    }
    Write-Host ("ActualSteps: " + $(if ($actualStepsInputPending) { "post-capture input planned" } else { "specified: $ActualSteps" }))
    Write-Host ("EndFoot: " + $(if ($endFootInputPending) { "post-capture input planned" } else { "specified: $EndFoot" }))
    Write-Host ("Command: py " + ($viewerArguments -join " "))
    Write-Host ("Planned metadata: " + (($metadataBase | ConvertTo-Json -Depth 6) -replace "\r?\n", " "))
    return
}

New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
$startedAt = Get-Date
$captureLines = [Collections.Generic.List[string]]::new()
& py @viewerArguments 2>&1 |
    ForEach-Object {
        $line = [string]$_
        [void]$captureLines.Add($line)
        $line
    } |
    Tee-Object -FilePath $paths.SummaryLog |
    Out-Host
$viewerExitCode = $LASTEXITCODE
$finishedAt = Get-Date

$summaryLine = Get-IlgaSummaryLine -Lines ($captureLines.ToArray())
$summary = $null
if ($summaryLine) {
    $summary = ConvertFrom-IlgaSummaryLine -Line $summaryLine
}

if ($viewerExitCode -eq 0 -and $null -ne $summary) {
    if ($actualStepsInputPending) {
        $ActualSteps = Read-IlgaActualSteps
        $metadataBase["actual_steps"] = [int]$ActualSteps
        $metadataBase["post_capture_input_pending"]["actual_steps"] = $false
    }
    if ($endFootInputPending) {
        $EndFoot = Read-IlgaEndFoot
        $metadataBase["end_foot"] = $EndFoot
        $metadataBase["post_capture_input_pending"]["end_foot"] = $false
    }
}

$metadata = [ordered]@{}
foreach ($entry in $metadataBase.GetEnumerator()) {
    $metadata[$entry.Key] = $entry.Value
}
$metadata.started_at = $startedAt.ToString("o")
$metadata.finished_at = $finishedAt.ToString("o")
$metadata.viewer_exit_code = $viewerExitCode
$metadata.summary_found = ($null -ne $summary)
$metadata.metrics = [ordered]@{
    missing_seq = ConvertTo-NullableNumber -Summary $summary -Name "missing_seq" -Type "int"
    invalid_sensor_samples = ConvertTo-NullableNumber -Summary $summary -Name "invalid_sensor_samples" -Type "int"
    sensor_fault = ConvertTo-NullableNumber -Summary $summary -Name "sensor_fault" -Type "int"
    rx_timestamp_effective_hz = ConvertTo-NullableNumber -Summary $summary -Name "rx_timestamp_effective_hz" -Type "double"
}
$qualityWarnings = @(Get-IlgaQualityWarnings -Metrics $metadata.metrics)
$metadata.quality_warning = ($qualityWarnings.Count -gt 0)
$metadata.quality_warning_reasons = $qualityWarnings
$metadata | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $paths.Metadata -Encoding utf8NoBOM

Write-Host ""
Write-Host "Capture quality metrics:"
Write-Host ("  missing_seq: {0}" -f (Format-IlgaMetricValue $metadata.metrics.missing_seq))
Write-Host ("  invalid_sensor_samples: {0}" -f (Format-IlgaMetricValue $metadata.metrics.invalid_sensor_samples))
Write-Host ("  sensor_fault: {0}" -f (Format-IlgaMetricValue $metadata.metrics.sensor_fault))
Write-Host ("  rx_timestamp_effective_hz: {0}" -f (Format-IlgaMetricValue $metadata.metrics.rx_timestamp_effective_hz))
foreach ($warning in $qualityWarnings) {
    Write-Warning "CAPTURE QUALITY WARNING: $warning. Raw CSV was preserved: $($paths.Csv)"
}

if ($viewerExitCode -ne 0) {
    Write-Host "ERROR: Viewer CLI failed with exit code $viewerExitCode. CSV, metadata, and log were preserved." -ForegroundColor Red
    exit $viewerExitCode
}
if ($null -eq $summary) {
    throw "Viewer CLI completed without a [SUMMARY] record. Metadata and log were preserved."
}
Write-Host "Capture complete: $($paths.Csv)"
Write-Host "Metadata: $($paths.Metadata)"
