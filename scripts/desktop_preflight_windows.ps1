param(
    [string]$RepoPath = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = "Stop"
$Failures = 0
$Warnings = 0
$ExpectedHead = "289e7fc4191912691ebd610eb259bc22f991cf32"

function Write-Pass([string]$Message) {
    Write-Host "[PASS] $Message" -ForegroundColor Green
}

function Write-Warn([string]$Message) {
    $script:Warnings += 1
    Write-Host "[WARN] $Message" -ForegroundColor Yellow
}

function Write-Fail([string]$Message) {
    $script:Failures += 1
    Write-Host "[FAIL] $Message" -ForegroundColor Red
}

Write-Host "ILGA Codex Desktop preflight (Windows)"
Write-Host "Repo: $RepoPath"
Write-Host ""

if (-not (Test-Path -LiteralPath $RepoPath -PathType Container)) {
    Write-Fail "Repository path does not exist: $RepoPath"
} else {
    Write-Pass "Repository path is accessible"
}

if (Get-Command git -ErrorAction SilentlyContinue) {
    Write-Pass (& git --version)
    try {
        $HeadSha = (& git -C $RepoPath rev-parse HEAD).Trim()
        if ($LASTEXITCODE -ne 0 -or -not $HeadSha) {
            throw "git rev-parse failed"
        }
        $Branch = (& git -C $RepoPath branch --show-current).Trim()
        Write-Pass "repository detected: branch=$Branch head=$HeadSha"
        & git -C $RepoPath merge-base --is-ancestor $ExpectedHead $HeadSha
        if ($LASTEXITCODE -eq 0) {
            Write-Pass "handover baseline is contained in the current history"
        } else {
            Write-Warn "handover baseline $ExpectedHead is not an ancestor of HEAD; review git log before continuing"
        }
        $Dirty = & git -C $RepoPath status --porcelain
        if ($Dirty) {
            Write-Warn "working tree has uncommitted changes"
            & git -C $RepoPath status --short
        } else {
            Write-Pass "working tree is clean"
        }
    } catch {
        Write-Fail "Git repository check failed: $($_.Exception.Message)"
    }
} else {
    Write-Fail "Git for Windows is not installed"
}

if (Get-Command py -ErrorAction SilentlyContinue) {
    try {
        $PythonVersion = (& py -3.12 -c "import platform; print(platform.python_version())" 2>$null).Trim()
        if ($LASTEXITCODE -ne 0 -or -not $PythonVersion) {
            throw "Python 3.12 check failed"
        }
        Write-Pass "Python $PythonVersion"
    } catch {
        Write-Fail "Python 3.12 is not available through the py launcher"
    }
} else {
    Write-Fail "Windows Python launcher 'py' is not installed"
}

$ViewerDir = Join-Path $RepoPath "VIEWER\python\bmi270_BLE_viewer"
$AnalyzerPath = Join-Path $RepoPath "VIEWER\python\walking_analyzer\analyze_single_leg_csv.py"
$ReceiverPath = Join-Path $ViewerDir "recv_bmi270_ble_notify_cli.py"
$PlotterPath = Join-Path $ViewerDir "realtime_plot_ble_v7.py"

foreach ($RequiredPath in @($ReceiverPath, $PlotterPath, $AnalyzerPath)) {
    if (Test-Path -LiteralPath $RequiredPath -PathType Leaf) {
        Write-Pass "file: $RequiredPath"
    } else {
        Write-Fail "missing file: $RequiredPath"
    }
}

try {
    $PackageVersions = (& py -3.12 -c "import importlib.metadata as m; import bleak, matplotlib, numpy; print('bleak=' + m.version('bleak')); print('matplotlib=' + m.version('matplotlib')); print('numpy=' + m.version('numpy'))" 2>$null)
    if ($LASTEXITCODE -ne 0 -or -not $PackageVersions) {
        throw "dependency import failed"
    }
    foreach ($Line in $PackageVersions) {
        Write-Pass $Line
    }
    $BleakVersion = ($PackageVersions | Where-Object { $_ -like "bleak=*" }) -replace "bleak=", ""
    if ($BleakVersion -ne "2.1.1") {
        Write-Warn "Bleak 2.1.1 was used in the verified environment"
    }
} catch {
    Write-Fail "Viewer Python dependencies are incomplete; install requirements_desktop.txt"
}

try {
    & py -3.12 -c "import ast, pathlib, sys; [ast.parse(pathlib.Path(p).read_text(encoding='utf-8'), filename=p) for p in sys.argv[1:]]" $ReceiverPath $PlotterPath $AnalyzerPath 2>$null
    if ($LASTEXITCODE -ne 0) {
        throw "AST parse failed"
    }
    Write-Pass "Python source syntax"
} catch {
    Write-Fail "Python source syntax check failed: $($_.Exception.Message)"
}

try {
    $BluetoothDevices = Get-PnpDevice -Class Bluetooth -Status OK -ErrorAction SilentlyContinue
    if ($BluetoothDevices) {
        Write-Pass "Windows Bluetooth device is enabled"
    } else {
        Write-Warn "No enabled Windows Bluetooth device was found"
    }
} catch {
    Write-Warn "Bluetooth device status could not be queried"
}

Write-Host ""
Write-Host "Summary: failures=$Failures warnings=$Warnings"
if ($Failures -ne 0) {
    exit 1
}
exit 0
