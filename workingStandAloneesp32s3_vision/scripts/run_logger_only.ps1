param(
    [string]$EspUrl = "http://192.168.4.1",
    [double]$LogInterval = 0.25,
    [string]$ProjectRoot = "c:\Users\Altyseer\esp32s3_vision"
)

$ErrorActionPreference = "Stop"

$pcPath = Join-Path $ProjectRoot "pc_client"
if (-not (Test-Path $pcPath)) {
    throw "PC client path not found: $pcPath"
}

Set-Location $pcPath

Write-Host "Running terminal logger only..." -ForegroundColor Cyan
Write-Host "URL: $EspUrl" -ForegroundColor Cyan

py -3.12 vision_processor.py --url $EspUrl --no-display --log-interval $LogInterval
