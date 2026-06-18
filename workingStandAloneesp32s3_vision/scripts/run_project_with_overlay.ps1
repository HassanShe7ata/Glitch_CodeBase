param(
    [string]$EspUrl = "http://192.168.4.1",
    [double]$LogInterval = 0.25,
    [switch]$UseStream,
    [string]$ProjectRoot = "c:\Users\Altyseer\esp32s3_vision"
)

$ErrorActionPreference = "Stop"

$fwPath = Join-Path $ProjectRoot "firmware\cam_stream"
$pcPath = Join-Path $ProjectRoot "pc_client"

if (-not (Test-Path $fwPath)) {
    throw "Firmware path not found: $fwPath"
}
if (-not (Test-Path $pcPath)) {
    throw "PC client path not found: $pcPath"
}

Write-Host "[1/3] Uploading firmware..." -ForegroundColor Cyan
Set-Location $fwPath
pio run --target upload

Write-Host "[2/3] Waiting for ESP32 to boot..." -ForegroundColor Cyan
Start-Sleep -Seconds 3

Write-Host "[3/3] Starting overlay viewer + terminal logs..." -ForegroundColor Cyan
Set-Location $pcPath

$args = @("-3.12", "vision_processor.py", "--url", $EspUrl, "--log-interval", "$LogInterval")
if ($UseStream) {
    $args += "--stream"
}

py @args
