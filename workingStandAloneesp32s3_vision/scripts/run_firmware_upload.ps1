param(
    [string]$ProjectRoot = "c:\Users\Altyseer\esp32s3_vision"
)

$ErrorActionPreference = "Stop"

$fwPath = Join-Path $ProjectRoot "firmware\cam_stream"
if (-not (Test-Path $fwPath)) {
    throw "Firmware path not found: $fwPath"
}

Write-Host "[1/2] Uploading firmware..." -ForegroundColor Cyan
Set-Location $fwPath
pio run --target upload

Write-Host "[2/2] Done. You can monitor serial with:" -ForegroundColor Green
Write-Host "pio device monitor --port COM3 --baud 115200" -ForegroundColor Yellow
