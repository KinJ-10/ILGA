#requires -Version 7.0
[CmdletBinding()]
param(
    [string]$TrialName,
    [ValidateSet("Walk", "Static")]
    [string]$TrialType = "Walk",
    [ValidateSet("Right", "Left", "Unknown")]
    [string]$SensorFoot = "Unknown",
    [ValidateSet("Walk10m", "Walk", "TUG", "Static")]
    [string]$TestType,
    [Nullable[double]]$DistanceM,
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
        MarkersCsv = Join-Path $Directory ($stem + "_markers.csv")
    }
}

function Assert-IlgaOutputsAvailable {
    param([Parameter(Mandatory)]$Paths, [bool]$IncludeMarkers = $false)
    $outputs = @($Paths.Csv, $Paths.SummaryLog, $Paths.Metadata)
    if ($IncludeMarkers) {
        $outputs += $Paths.MarkersCsv
    }
    foreach ($path in $outputs) {
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

function ConvertTo-IlgaOptionalActualSteps {
    param([AllowNull()]$Value)
    $text = if ($null -eq $Value) { "" } else { [string]$Value.Trim() }
    if ([string]::IsNullOrWhiteSpace($text) -or $text.ToLowerInvariant() -in @("unknown", "none")) {
        return [pscustomobject]@{
            Value = $null
            Source = "post_capture_unknown"
        }
    }
    return [pscustomobject]@{
        Value = ConvertTo-IlgaActualSteps -Value $text
        Source = "post_capture_prompt"
    }
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

function ConvertTo-IlgaOptionalEndFoot {
    param([AllowNull()]$Value)
    $text = if ($null -eq $Value) { "" } else { [string]$Value.Trim() }
    if ([string]::IsNullOrWhiteSpace($text) -or $text.ToLowerInvariant() -in @("unknown", "none")) {
        return [pscustomobject]@{
            Value = $null
            Source = "post_capture_unknown"
        }
    }
    return [pscustomobject]@{
        Value = ConvertTo-IlgaFoot -Value $text -Name "EndFoot"
        Source = "post_capture_prompt"
    }
}

function Read-IlgaActualSteps {
    while ($true) {
        $value = Read-Host "Enter ActualSteps (integer >= 0, or blank/Unknown)"
        try { return ConvertTo-IlgaOptionalActualSteps -Value $value }
        catch { Write-Warning $_.Exception.Message }
    }
}

function Read-IlgaEndFoot {
    while ($true) {
        $value = Read-Host "Enter EndFoot (Right/Left, or blank/None/Unknown)"
        try { return ConvertTo-IlgaOptionalEndFoot -Value $value }
        catch { Write-Warning $_.Exception.Message }
    }
}

function ConvertTo-IlgaDistanceM {
    param([AllowNull()]$Value)
    $parsed = 0.0
    if ($null -eq $Value -or
        -not [double]::TryParse(
            [string]$Value,
            [Globalization.NumberStyles]::Float,
            [Globalization.CultureInfo]::InvariantCulture,
            [ref]$parsed
        ) -or
        $parsed -le 0) {
        throw "DistanceM must be a number greater than zero."
    }
    return $parsed
}

function Resolve-IlgaTestDefinition {
    param(
        [Parameter(Mandatory)][ValidateSet("Walk", "Static")][string]$TrialTypeValue,
        [AllowNull()]$TestTypeValue,
        [AllowNull()]$DistanceMValue
    )

    if ($TrialTypeValue -eq "Static") {
        if (-not [string]::IsNullOrWhiteSpace([string]$TestTypeValue) -and $TestTypeValue -ne "Static") {
            throw "Static TrialType requires TestType=Static or omission."
        }
        if ($null -ne $DistanceMValue -and [double]$DistanceMValue -ne 0.0) {
            throw "Static trials require DistanceM=0 or omission."
        }
        return [pscustomobject]@{
            TestType = "Static"
            DistanceM = 0.0
            MarkersEnabled = $false
            ProtocolMode = "static"
            MeasurementEvent = "static_stillness_for_duration"
            SequenceEvent = "static_stillness_for_duration"
        }
    }

    $resolvedType = if ([string]::IsNullOrWhiteSpace([string]$TestTypeValue)) { "Walk10m" } else { [string]$TestTypeValue }
    if ($resolvedType -eq "Static") {
        throw "Walk TrialType cannot use TestType=Static."
    }
    if ($resolvedType -eq "Walk10m") {
        $distance = if ($null -eq $DistanceMValue) { 10.0 } else { ConvertTo-IlgaDistanceM -Value $DistanceMValue }
        if ($distance -ne 10.0) {
            throw "TestType=Walk10m requires DistanceM=10 or omission; use TestType=Walk for another distance."
        }
        $mode = "walk"
        $event = "marked_10m_walk"
        $sequenceEvent = "walk_10m"
    }
    elseif ($resolvedType -eq "Walk") {
        $distance = ConvertTo-IlgaDistanceM -Value $DistanceMValue
        $mode = "walk"
        $event = "marked_walk_distance"
        $sequenceEvent = "walk_distance"
    }
    elseif ($resolvedType -eq "TUG") {
        $distance = if ($null -eq $DistanceMValue) { $null } else { ConvertTo-IlgaDistanceM -Value $DistanceMValue }
        $mode = "tug"
        $event = "marked_tug_total"
        $sequenceEvent = "tug"
    }
    else {
        throw "Unsupported TestType: $resolvedType"
    }
    return [pscustomobject]@{
        TestType = $resolvedType
        DistanceM = $distance
        MarkersEnabled = $true
        ProtocolMode = $mode
        MeasurementEvent = $event
        SequenceEvent = $sequenceEvent
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
    $resolvedEndFoot = if ($endPending) { $null } else { ConvertTo-IlgaFoot -Value $EndFootValue -Name "EndFoot" }
    $endUnknown = -not $endPending -and $resolvedEndFoot -eq "None"
    return [pscustomobject]@{
        ActualSteps = if ($actualPending) { $null } else { ConvertTo-IlgaActualSteps -Value $ActualStepsValue }
        StartFoot = $resolvedStartFoot
        EndFoot = if ($endPending -or $endUnknown) { $null } else { $resolvedEndFoot }
        EndCondition = [string]$EndConditionValue
        ActualStepsInputPending = $actualPending
        EndFootInputPending = $endPending
        ActualStepsSource = if ($actualPending) { "post_capture_prompt" } else { "command_line" }
        EndFootSource = if ($endPending) { "post_capture_prompt" } elseif ($endUnknown) { "command_line_unknown" } else { "command_line" }
    }
}

function Invoke-IlgaSelfTest {
    $sample = "[SUMMARY] received=2994 missing_seq=7 invalid_sensor_samples=0 sensor_fault=0 rx_timestamp_effective_hz=99.800 csv='C:\data files\trial.csv' markers_file='C:\data files\trial_markers.csv' start_count=1 finish_count=1 marker_valid=1 marked_duration_sec=5.250000"
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
        $parsed.csv -ne "C:\data files\trial.csv" -or
        $parsed.markers_file -ne "C:\data files\trial_markers.csv" -or
        $parsed.start_count -ne "1" -or
        $parsed.finish_count -ne "1" -or
        $parsed.marker_valid -ne "1" -or
        $parsed.marked_duration_sec -ne "5.250000"
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
    $knownSteps = ConvertTo-IlgaOptionalActualSteps -Value "16"
    $unknownStepsBlank = ConvertTo-IlgaOptionalActualSteps -Value ""
    $unknownStepsText = ConvertTo-IlgaOptionalActualSteps -Value "Unknown"
    $knownEndFoot = ConvertTo-IlgaOptionalEndFoot -Value "left"
    $unknownEndFootNone = ConvertTo-IlgaOptionalEndFoot -Value "None"
    $unknownEndFootBlank = ConvertTo-IlgaOptionalEndFoot -Value ""
    if (
        $knownSteps.Value -ne 16 -or
        $knownSteps.Source -ne "post_capture_prompt" -or
        $null -ne $unknownStepsBlank.Value -or
        $unknownStepsBlank.Source -ne "post_capture_unknown" -or
        $null -ne $unknownStepsText.Value -or
        $knownEndFoot.Value -ne "Left" -or
        $knownEndFoot.Source -ne "post_capture_prompt" -or
        $null -ne $unknownEndFootNone.Value -or
        $unknownEndFootNone.Source -ne "post_capture_unknown" -or
        $null -ne $unknownEndFootBlank.Value
    ) {
        throw "Optional post-capture input self-test failed."
    }
    $unknownMetadata = [ordered]@{
        sensor_foot = "Unknown"
        sensor_foot_source = "default_unknown"
        actual_steps = $unknownStepsText.Value
        actual_steps_source = $unknownStepsText.Source
        end_foot = $unknownEndFootNone.Value
        end_foot_source = $unknownEndFootNone.Source
        post_capture_input_pending = [ordered]@{ actual_steps = $false; end_foot = $false }
    }
    $unknownRoundTrip = $unknownMetadata | ConvertTo-Json -Depth 3 | ConvertFrom-Json
    $knownMetadata = [ordered]@{
        sensor_foot = "Left"
        sensor_foot_source = "command_line"
        actual_steps = $knownSteps.Value
        actual_steps_source = $knownSteps.Source
        end_foot = $knownEndFoot.Value
        end_foot_source = $knownEndFoot.Source
        post_capture_input_pending = [ordered]@{ actual_steps = $false; end_foot = $false }
    }
    $knownRoundTrip = $knownMetadata | ConvertTo-Json -Depth 3 | ConvertFrom-Json
    if (
        $null -ne $unknownRoundTrip.actual_steps -or
        $unknownRoundTrip.sensor_foot -ne "Unknown" -or
        $unknownRoundTrip.sensor_foot_source -ne "default_unknown" -or
        $unknownRoundTrip.actual_steps_source -ne "post_capture_unknown" -or
        $null -ne $unknownRoundTrip.end_foot -or
        $unknownRoundTrip.end_foot_source -ne "post_capture_unknown" -or
        $unknownRoundTrip.post_capture_input_pending.actual_steps -or
        $unknownRoundTrip.post_capture_input_pending.end_foot -or
        $knownRoundTrip.actual_steps -ne 16 -or
        $knownRoundTrip.sensor_foot -ne "Left" -or
        $knownRoundTrip.end_foot -ne "Left"
    ) {
        throw "Known/unknown metadata JSON self-test failed."
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

    $walk10mDefinition = Resolve-IlgaTestDefinition -TrialTypeValue Walk -TestTypeValue $null -DistanceMValue $null
    $walkDistanceDefinition = Resolve-IlgaTestDefinition -TrialTypeValue Walk -TestTypeValue Walk -DistanceMValue 4
    $tugDefinition = Resolve-IlgaTestDefinition -TrialTypeValue Walk -TestTypeValue TUG -DistanceMValue $null
    $staticDefinition = Resolve-IlgaTestDefinition -TrialTypeValue Static -TestTypeValue $null -DistanceMValue $null
    $invalidWalk10mRejected = $false
    try { Resolve-IlgaTestDefinition -TrialTypeValue Walk -TestTypeValue Walk10m -DistanceMValue 4 | Out-Null }
    catch { $invalidWalk10mRejected = $true }
    if (
        $walk10mDefinition.DistanceM -ne 10 -or
        $walkDistanceDefinition.DistanceM -ne 4 -or
        $null -ne $tugDefinition.DistanceM -or
        -not $tugDefinition.MarkersEnabled -or
        $staticDefinition.DistanceM -ne 0 -or
        $staticDefinition.MarkersEnabled -or
        -not $invalidWalk10mRejected
    ) {
        throw "TestType/DistanceM self-test failed."
    }

    $walkTrial = Resolve-IlgaTrialInputs -Type Walk -ActualStepsValue $null -StartFootValue Right -EndFootValue $null -EndConditionValue "stop after line"
    if (-not $walkTrial.ActualStepsInputPending -or -not $walkTrial.EndFootInputPending -or $walkTrial.StartFoot -ne "Right") {
        throw "Walk trial pending-input self-test failed."
    }
    $walkUnknownEnd = Resolve-IlgaTrialInputs -Type Walk -ActualStepsValue 16 -StartFootValue Right -EndFootValue None -EndConditionValue "stop after line"
    if (
        $walkUnknownEnd.ActualSteps -ne 16 -or
        $null -ne $walkUnknownEnd.EndFoot -or
        $walkUnknownEnd.EndFootInputPending -or
        $walkUnknownEnd.EndFootSource -ne "command_line_unknown"
    ) {
        throw "Walk command-line unknown EndFoot self-test failed."
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
        Remove-Item -LiteralPath $paths.Csv -Force
        [IO.File]::WriteAllText($paths.MarkersCsv, "test")
        $markerCollisionRejected = $false
        try {
            Assert-IlgaOutputsAvailable -Paths $paths -IncludeMarkers $true
        }
        catch {
            $markerCollisionRejected = $_.Exception.Message -eq "Refusing to overwrite existing output: $($paths.MarkersCsv)"
        }
        if (-not $markerCollisionRejected) {
            throw "Markers CSV overwrite refusal self-test failed."
        }
    }
    finally {
        if ($testDirectory.StartsWith([IO.Path]::GetTempPath(), [StringComparison]::OrdinalIgnoreCase)) {
            Remove-Item -LiteralPath $testDirectory -Recurse -Force
        }
    }
    Write-Host "SELFTEST PASS: summary extraction, quality warning detection, SensorFoot and known/unknown ActualSteps/EndFoot validation, Walk/Static/TestType validation, and CSV/log/metadata/markers overwrite refusal"
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
$testDefinition = Resolve-IlgaTestDefinition -TrialTypeValue $TrialType -TestTypeValue $TestType -DistanceMValue $DistanceM
$TestType = $testDefinition.TestType
$DistanceM = $testDefinition.DistanceM
$markersEnabled = $testDefinition.MarkersEnabled
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
Assert-IlgaOutputsAvailable -Paths $paths -IncludeMarkers $markersEnabled

$viewerArguments = @(
    "-3.12",
    "-u",
    $viewerPath,
    "--name", "BMI270_BLE_SAMPLE",
    "--save-csv", $paths.Csv,
    "--duration-sec", $DurationSec,
    "--disconnect-on-finish"
)
if ($markersEnabled) {
    $viewerArguments += @(
        "--markers-csv", $paths.MarkersCsv,
        "--trial-name", $TrialName
    )
}

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
        mode = $testDefinition.ProtocolMode
        test_type = $TestType
        measurement_interval = $testDefinition.MeasurementEvent
        synchronization = [ordered]@{
            pre = [ordered]@{ action = "stomp"; count = 3; timing = "before pre-walk stillness" }
            post = [ordered]@{ action = "stomp"; count = 3; timing = "after terminal stillness, before capture stop" }
            total_count = 6
        }
        pre_walk_stillness = "Stand still before the marked interval as directed by the protocol."
        walk_distance_m = $DistanceM
        terminal_condition = $EndCondition
        post_terminal_stillness_sec = [ordered]@{ minimum = 3; maximum = 5 }
        sequence = @(
            "pre_sync_3_stomps",
            "pre_walk_stillness",
            $testDefinition.SequenceEvent,
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
    test_type = $TestType
    distance_m = $DistanceM
    trial_name = $TrialName
    sensor_foot = $SensorFoot
    sensor_foot_source = if ($PSBoundParameters.ContainsKey("SensorFoot")) { "command_line" } else { "default_unknown" }
    actual_steps = if ($actualStepsInputPending) { $null } else { [int]$ActualSteps }
    actual_steps_source = $resolvedInputs.ActualStepsSource
    start_foot = $StartFoot
    end_foot = if ($endFootInputPending) { $null } else { $resolvedInputs.EndFoot }
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
    markers_path = if ($markersEnabled) { $paths.MarkersCsv } else { $null }
    markers_required = $markersEnabled
    analyzer_automatic = $false
}

if ($DryRun) {
    Write-Host "DRY RUN: no files created and no BLE connection attempted."
    if ($TrialType -eq "Static") {
        Write-Host "Protocol: complete stillness for $DurationSec sec -> capture stop (no synchronization stomps, no walk, no terminal action)."
    }
    else {
        $measurementLabel = switch ($TestType) {
            "Walk10m" { "marked 10 m walk" }
            "Walk" { "marked $DistanceM m walk" }
            "TUG" { "marked TUG interval" }
        }
        Write-Host "Protocol: pre sync stomps x3 -> stillness -> $measurementLabel -> terminal condition -> stillness 3-5 sec -> post sync stomps x3 -> capture stop (6 sync stomps total)."
        Write-Host "Operator markers: press s at START and f at FINISH during the marked interval."
    }
    Write-Host ("ActualSteps: " + $(if ($actualStepsInputPending) { "post-capture input planned" } else { "specified: $ActualSteps" }))
    Write-Host "SensorFoot: $SensorFoot"
    Write-Host ("EndFoot: " + $(
        if ($endFootInputPending) { "post-capture input planned" }
        elseif ($null -eq $resolvedInputs.EndFoot) { "specified unknown" }
        else { "specified: $($resolvedInputs.EndFoot)" }
    ))
    Write-Host ("Markers CSV: " + $(if ($markersEnabled) { $paths.MarkersCsv } else { "disabled" }))
    Write-Host ("Command: py " + ($viewerArguments -join " "))
    Write-Host ("Planned metadata: " + (($metadataBase | ConvertTo-Json -Depth 6) -replace "\r?\n", " "))
    return
}

if ($TrialType -eq "Walk" -and $SensorFoot -eq "Unknown") {
    Write-Warning "SensorFoot is Unknown. Attachment-foot reference step conversion will be disabled during analysis."
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
        $actualStepsResult = Read-IlgaActualSteps
        $ActualSteps = $actualStepsResult.Value
        $metadataBase["actual_steps"] = if ($null -eq $ActualSteps) { $null } else { [int]$ActualSteps }
        $metadataBase["actual_steps_source"] = $actualStepsResult.Source
        $metadataBase["post_capture_input_pending"]["actual_steps"] = $false
    }
    if ($endFootInputPending) {
        $endFootResult = Read-IlgaEndFoot
        $EndFoot = $endFootResult.Value
        $metadataBase["end_foot"] = $endFootResult.Value
        $metadataBase["end_foot_source"] = $endFootResult.Source
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
    markers_file = if ($null -ne $summary -and $null -ne $summary.PSObject.Properties["markers_file"]) { $summary.markers_file } else { $null }
    start_count = ConvertTo-NullableNumber -Summary $summary -Name "start_count" -Type "int"
    finish_count = ConvertTo-NullableNumber -Summary $summary -Name "finish_count" -Type "int"
    marker_valid = if (
        $null -ne $summary -and
        $null -ne $summary.PSObject.Properties["marker_valid"] -and
        $summary.marker_valid -ne "NA"
    ) { ConvertTo-NullableNumber -Summary $summary -Name "marker_valid" -Type "int" } else { $null }
    marked_duration_sec = if (
        $null -ne $summary -and
        $null -ne $summary.PSObject.Properties["marked_duration_sec"] -and
        $summary.marked_duration_sec -ne "NA"
    ) { ConvertTo-NullableNumber -Summary $summary -Name "marked_duration_sec" -Type "double" } else { $null }
}
$qualityWarnings = @(Get-IlgaQualityWarnings -Metrics $metadata.metrics)
if ($markersEnabled -and $metadata.metrics.marker_valid -ne 1) {
    $qualityWarnings += "marker_valid=$($metadata.metrics.marker_valid)"
}
$metadata.quality_warning = ($qualityWarnings.Count -gt 0)
$metadata.quality_warning_reasons = $qualityWarnings
$metadata | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $paths.Metadata -Encoding utf8NoBOM

Write-Host ""
Write-Host "Capture quality metrics:"
Write-Host ("  missing_seq: {0}" -f (Format-IlgaMetricValue $metadata.metrics.missing_seq))
Write-Host ("  invalid_sensor_samples: {0}" -f (Format-IlgaMetricValue $metadata.metrics.invalid_sensor_samples))
Write-Host ("  sensor_fault: {0}" -f (Format-IlgaMetricValue $metadata.metrics.sensor_fault))
Write-Host ("  rx_timestamp_effective_hz: {0}" -f (Format-IlgaMetricValue $metadata.metrics.rx_timestamp_effective_hz))
if ($markersEnabled) {
    Write-Host ("  start_count: {0}" -f (Format-IlgaMetricValue $metadata.metrics.start_count))
    Write-Host ("  finish_count: {0}" -f (Format-IlgaMetricValue $metadata.metrics.finish_count))
    Write-Host ("  marker_valid: {0}" -f (Format-IlgaMetricValue $metadata.metrics.marker_valid))
    Write-Host ("  marked_duration_sec: {0}" -f (Format-IlgaMetricValue $metadata.metrics.marked_duration_sec))
}
foreach ($warning in $qualityWarnings) {
    Write-Warning "CAPTURE QUALITY WARNING: $warning. Raw CSV was preserved: $($paths.Csv)"
}

if ($viewerExitCode -ne 0) {
    Write-Host "ERROR: Viewer CLI failed with exit code $viewerExitCode. Sensor CSV, marker CSV (if created), metadata, and log were preserved." -ForegroundColor Red
    exit $viewerExitCode
}
if ($null -eq $summary) {
    throw "Viewer CLI completed without a [SUMMARY] record. Metadata and log were preserved."
}
Write-Host "Capture complete: $($paths.Csv)"
if ($markersEnabled) {
    Write-Host "Markers: $($paths.MarkersCsv)"
}
Write-Host "Metadata: $($paths.Metadata)"
