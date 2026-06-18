param(
    [string]$EspUrl = "http://192.168.4.1",
    [string]$Scenario = "",
    [string]$XOffsetMm = "na",
    [string]$YOffsetMm = "na",
    [string]$ZOffsetMm = "na",
    [string]$YawOffsetDeg = "na",
    [string]$Notes = "na",
    [double]$EvalDuration = 20.0,
    [double]$EvalInterval = 0.25,
    [switch]$UseStream,
    [string]$ProjectRoot = "c:\Users\Altyseer\esp32s3_vision"
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($Scenario)) {
    $Scenario = Read-Host "Scenario name"
}
if ([string]::IsNullOrWhiteSpace($Scenario)) {
    $Scenario = "scenario_$(Get-Date -Format yyyyMMdd_HHmmss)"
}

Write-Host "=== FAST ROUND ===" -ForegroundColor Cyan
Write-Host "Scenario: $Scenario" -ForegroundColor Yellow
Write-Host "Offsets: X=$XOffsetMm Y=$YOffsetMm Z=$ZOffsetMm Yaw=$YawOffsetDeg" -ForegroundColor Yellow
Write-Host "Notes: $Notes" -ForegroundColor Yellow
Write-Host "Set physical setup now, then press ENTER to start overlay run." -ForegroundColor Cyan
[void](Read-Host "Press ENTER to begin")

$fullScript = Join-Path $ProjectRoot "scripts\run_scenario_full.ps1"
if ($UseStream) {
    & $fullScript `
        -EspUrl $EspUrl `
        -Scenario $Scenario `
        -XOffsetMm $XOffsetMm `
        -YOffsetMm $YOffsetMm `
        -ZOffsetMm $ZOffsetMm `
        -YawOffsetDeg $YawOffsetDeg `
        -Notes $Notes `
        -EvalDuration $EvalDuration `
        -EvalInterval $EvalInterval `
        -UseStream `
        -ProjectRoot $ProjectRoot
} else {
    & $fullScript `
        -EspUrl $EspUrl `
        -Scenario $Scenario `
        -XOffsetMm $XOffsetMm `
        -YOffsetMm $YOffsetMm `
        -ZOffsetMm $ZOffsetMm `
        -YawOffsetDeg $YawOffsetDeg `
        -Notes $Notes `
        -EvalDuration $EvalDuration `
        -EvalInterval $EvalInterval `
        -ProjectRoot $ProjectRoot
}
