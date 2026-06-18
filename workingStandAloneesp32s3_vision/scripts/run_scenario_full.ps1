param(
    [string]$EspUrl = "http://192.168.4.1",
    [string]$Scenario = "",
    [string]$XOffsetMm = "",
    [string]$YOffsetMm = "",
    [string]$ZOffsetMm = "",
    [string]$YawOffsetDeg = "",
    [string]$Notes = "",
    [switch]$UseStream,
    [double]$EvalDuration = 20.0,
    [double]$EvalInterval = 0.25,
    [string]$ProjectRoot = "c:\Users\Altyseer\esp32s3_vision"
)

$ErrorActionPreference = "Stop"

$scenarioScript = Join-Path $ProjectRoot "scripts\run_scenario_overlay.ps1"
$pcPath = Join-Path $ProjectRoot "pc_client"
$metricsPath = Join-Path $ProjectRoot "scenario_metrics.csv"

if (-not (Test-Path $scenarioScript)) {
    throw "Scenario runner not found: $scenarioScript"
}
if (-not (Test-Path $pcPath)) {
    throw "PC client path not found: $pcPath"
}

if ([string]::IsNullOrWhiteSpace($Scenario)) {
    $Scenario = Read-Host "Scenario name (e.g. S1_baseline, S2_low_light)"
}
if ([string]::IsNullOrWhiteSpace($XOffsetMm)) {
    $XOffsetMm = Read-Host "Measured X offset mm"
}
if ([string]::IsNullOrWhiteSpace($YOffsetMm)) {
    $YOffsetMm = Read-Host "Measured Y offset mm"
}
if ([string]::IsNullOrWhiteSpace($ZOffsetMm)) {
    $ZOffsetMm = Read-Host "Measured Z offset mm"
}
if ([string]::IsNullOrWhiteSpace($YawOffsetDeg)) {
    $YawOffsetDeg = Read-Host "Measured yaw offset deg"
}
if ([string]::IsNullOrWhiteSpace($Notes)) {
    $Notes = Read-Host "Notes (light/angle/etc.)"
}

if ([string]::IsNullOrWhiteSpace($Scenario)) { $Scenario = "scenario_$(Get-Date -Format yyyyMMdd_HHmmss)" }
if ([string]::IsNullOrWhiteSpace($XOffsetMm)) { $XOffsetMm = "na" }
if ([string]::IsNullOrWhiteSpace($YOffsetMm)) { $YOffsetMm = "na" }
if ([string]::IsNullOrWhiteSpace($ZOffsetMm)) { $ZOffsetMm = "na" }
if ([string]::IsNullOrWhiteSpace($YawOffsetDeg)) { $YawOffsetDeg = "na" }
if ([string]::IsNullOrWhiteSpace($Notes)) { $Notes = "na" }

Write-Host "=== Step 1/2: Overlay run + scenario metadata ===" -ForegroundColor Cyan
$overlayExitCode = 0
try {
    if ($UseStream) {
        & $scenarioScript `
            -EspUrl $EspUrl `
            -Scenario $Scenario `
            -XOffsetMm $XOffsetMm `
            -YOffsetMm $YOffsetMm `
            -ZOffsetMm $ZOffsetMm `
            -YawOffsetDeg $YawOffsetDeg `
            -Notes $Notes `
            -NoPrompt `
            -UseStream `
            -ProjectRoot $ProjectRoot
    } else {
        & $scenarioScript `
            -EspUrl $EspUrl `
            -Scenario $Scenario `
            -XOffsetMm $XOffsetMm `
            -YOffsetMm $YOffsetMm `
            -ZOffsetMm $ZOffsetMm `
            -YawOffsetDeg $YawOffsetDeg `
            -Notes $Notes `
            -NoPrompt `
            -ProjectRoot $ProjectRoot
    }
    if ($LASTEXITCODE) {
        $overlayExitCode = [int]$LASTEXITCODE
    }
} catch {
    $overlayExitCode = 1
    Write-Warning "Overlay phase ended with error: $($_.Exception.Message)"
}

if ($overlayExitCode -ne 0) {
    Write-Warning "Overlay exited with code $overlayExitCode; continuing to evaluator for metrics capture."
}

Write-Host "=== Step 2/2: Evaluate session and append metrics CSV ===" -ForegroundColor Cyan
Set-Location $pcPath

$evalJson = py -3.12 evaluate_session.py --url $EspUrl --duration $EvalDuration --interval $EvalInterval --json
if (-not $evalJson) {
    throw "No metrics returned from evaluator."
}

$metrics = $evalJson | ConvertFrom-Json

if (-not (Test-Path $metricsPath)) {
    "timestamp,scenario,x_offset_mm,y_offset_mm,z_offset_mm,yaw_offset_deg,notes,mode,samples,detection_ratio,decode_ratio_when_detected,camera_status_ok_ratio,confidence_mean,tz_mean_mm,tz_std_mm,proc_ms_mean,proc_ms_p95" | Out-File -FilePath $metricsPath -Encoding utf8
}

$mode = if ($UseStream) { "stream" } else { "capture" }
$line = "{0},{1},{2},{3},{4},{5},{6},{7},{8},{9:F3},{10:F3},{11:F3},{12},{13},{14},{15},{16}" -f
    (Get-Date -Format s),
    $Scenario,
    $XOffsetMm,
    $YOffsetMm,
    $ZOffsetMm,
    $YawOffsetDeg,
    $Notes.Replace(',', ';'),
    $mode,
    $metrics.samples,
    $metrics.detection_ratio,
    $metrics.decode_ratio_when_detected,
    $metrics.camera_status_ok_ratio,
    $metrics.confidence_mean,
    $metrics.tz_mean_mm,
    $metrics.tz_std_mm,
    $metrics.proc_ms_mean,
    $metrics.proc_ms_p95

Add-Content -Path $metricsPath -Value $line
Write-Host "Saved scenario metrics to: $metricsPath" -ForegroundColor Green
Write-Host "Summary => det=$([double]$metrics.detection_ratio) dec=$([double]$metrics.decode_ratio_when_detected) proc_mean_ms=$([double]$metrics.proc_ms_mean)" -ForegroundColor Yellow
