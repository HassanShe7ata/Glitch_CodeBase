param(
    [string]$EspUrl = "http://192.168.4.1",
    [double]$LogInterval = 0.25,
    [switch]$UseStream,
    [string]$ProjectRoot = "c:\Users\Altyseer\esp32s3_vision"
)

$ErrorActionPreference = "Stop"

$pcPath = Join-Path $ProjectRoot "pc_client"
if (-not (Test-Path $pcPath)) {
    throw "PC client path not found: $pcPath"
}

Set-Location $pcPath

$args = @("-3.12", "vision_processor.py", "--url", $EspUrl, "--log-interval", "$LogInterval")
if ($UseStream) {
    $args += "--stream"
}

Write-Host "Running overlay viewer..." -ForegroundColor Cyan
Write-Host "URL: $EspUrl" -ForegroundColor Cyan
Write-Host "Press q in the OpenCV window to quit." -ForegroundColor Yellow

py @args
