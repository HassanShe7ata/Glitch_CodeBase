param(
    [string]$EspUrl = "http://192.168.4.1",
    [string]$Scenario = "",
    [string]$XOffsetMm = "",
    [string]$YOffsetMm = "",
    [string]$ZOffsetMm = "",
    [string]$YawOffsetDeg = "",
    [string]$Notes = "",
    [switch]$UseStream,
    [switch]$NoPrompt,
    [string]$ProjectRoot = "c:\Users\Altyseer\esp32s3_vision"
)

$ErrorActionPreference = "Stop"

$pcPath = Join-Path $ProjectRoot "pc_client"
if (-not (Test-Path $pcPath)) {
    throw "PC client path not found: $pcPath"
}

Write-Host "=== Scenario Test Runner ===" -ForegroundColor Cyan
if (-not $NoPrompt) {
    if ([string]::IsNullOrWhiteSpace($Scenario)) { $Scenario = Read-Host "Scenario name (e.g. S1_baseline, S2_low_light)" }
    if ([string]::IsNullOrWhiteSpace($XOffsetMm)) { $XOffsetMm = Read-Host "Measured X offset mm" }
    if ([string]::IsNullOrWhiteSpace($YOffsetMm)) { $YOffsetMm = Read-Host "Measured Y offset mm" }
    if ([string]::IsNullOrWhiteSpace($ZOffsetMm)) { $ZOffsetMm = Read-Host "Measured Z offset mm" }
    if ([string]::IsNullOrWhiteSpace($YawOffsetDeg)) { $YawOffsetDeg = Read-Host "Measured yaw offset deg" }
    if ([string]::IsNullOrWhiteSpace($Notes)) { $Notes = Read-Host "Notes (light/angle/etc.)" }
}

if ([string]::IsNullOrWhiteSpace($Scenario)) { $Scenario = "scenario_$(Get-Date -Format yyyyMMdd_HHmmss)" }
if ([string]::IsNullOrWhiteSpace($XOffsetMm)) { $XOffsetMm = "na" }
if ([string]::IsNullOrWhiteSpace($YOffsetMm)) { $YOffsetMm = "na" }
if ([string]::IsNullOrWhiteSpace($ZOffsetMm)) { $ZOffsetMm = "na" }
if ([string]::IsNullOrWhiteSpace($YawOffsetDeg)) { $YawOffsetDeg = "na" }
if ([string]::IsNullOrWhiteSpace($Notes)) { $Notes = "na" }

$logPath = Join-Path $ProjectRoot "scenario_log.csv"
if (-not (Test-Path $logPath)) {
    "timestamp,scenario,x_offset_mm,y_offset_mm,z_offset_mm,yaw_offset_deg,notes" | Out-File -FilePath $logPath -Encoding utf8
}
$line = "{0},{1},{2},{3},{4},{5},{6}" -f (Get-Date -Format s), $Scenario, $XOffsetMm, $YOffsetMm, $ZOffsetMm, $YawOffsetDeg, $Notes.Replace(',', ';')
Add-Content -Path $logPath -Value $line

Write-Host "Scenario: $Scenario" -ForegroundColor Yellow
Write-Host "Measured offsets => X:$XOffsetMm Y:$YOffsetMm Z:$ZOffsetMm Yaw:$YawOffsetDeg" -ForegroundColor Yellow
Write-Host "Notes: $Notes" -ForegroundColor Yellow
Write-Host "Saved scenario metadata to: $logPath" -ForegroundColor DarkCyan
if ($UseStream) {
    Write-Host "Running overlay in MJPEG stream mode..." -ForegroundColor Cyan
} else {
    Write-Host "Running overlay in capture mode (recommended for decode robustness)..." -ForegroundColor Cyan
}

Set-Location $pcPath
$args = @("-3.12", "vision_processor.py", "--url", $EspUrl, "--log-interval", "0.25", "--data-interval", "0.08", "--frame-interval", "0.20")
if ($UseStream) {
    $args += "--stream"
}
py @args
