param(
    [int]$Port = 8765,
    [switch]$NoBrowser
)
$ErrorActionPreference = "Stop"
$env:PYTHONUTF8 = "1"
if (-not (Get-Command py -ErrorAction SilentlyContinue)) {
    throw "Python 3.12 was not found. Install Python with the py launcher."
}
# Keep the working directory on Windows; the script path may be UNC.
& py -3.12 -c "import bleak, numpy, scipy, matplotlib"
if ($LASTEXITCODE -ne 0) {
    Write-Host "Install dependencies with:"
    Write-Host "py -3.12 -m pip install -r '$PSScriptRoot\requirements.txt'"
    exit 1
}
$DemoArgs = @("$PSScriptRoot\gait_demo_server.py", "--port", "$Port")
if ($NoBrowser) { $DemoArgs += "--no-browser" }
& py -3.12 @DemoArgs
exit $LASTEXITCODE
