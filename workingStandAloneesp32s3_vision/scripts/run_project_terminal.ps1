param(
    [string]$ProjectRoot = "c:\Users\Altyseer\esp32s3_vision",
    [string]$EspUrl = "http://192.168.4.1",
    [switch]$UploadFirmware,
    [int]$PollMs = 350,
    [int]$SummaryEvery = 30
)

$ErrorActionPreference = "Stop"

$fwPath = Join-Path $ProjectRoot "firmware\cam_stream"
if (-not (Test-Path $fwPath)) {
    throw "Firmware path not found: $fwPath"
}

if ($UploadFirmware) {
    Write-Host "[1/2] Uploading firmware..." -ForegroundColor Cyan
    Set-Location $fwPath
    pio run --target upload
    Write-Host "Firmware upload done." -ForegroundColor Green
}

Write-Host "[2/2] Starting terminal-only live stream..." -ForegroundColor Cyan
Write-Host "URL: $EspUrl/data" -ForegroundColor Cyan
Write-Host "Press Ctrl+C to stop." -ForegroundColor Yellow

$samples = 0
$detected = 0
$decoded = 0
$estimated = 0
$procSum = 0.0
$procCount = 0

while ($true) {
    try {
        $r = Invoke-RestMethod "$EspUrl/data" -TimeoutSec 3
        $q = $null
        if ($r.qr_codes -and $r.qr_codes.Count -gt 0) {
            $q = $r.qr_codes[0]
        }

        $isDet = ($q -ne $null)
        $isDec = $false
        $isEst = $false
        $tx = "-"
        $ty = "-"
        $tz = "-"
        $roll = "-"
        $pitch = "-"
        $yaw = "-"
        $msg = "-"

        if ($isDet) {
            $isDec = [bool]$q.decoded
            $isEst = [bool]$q.estimated
            $tx = [Math]::Round([double]$q.tx, 1)
            $ty = [Math]::Round([double]$q.ty, 1)
            $tz = [Math]::Round([double]$q.tz, 1)
            $roll = [Math]::Round([double]$q.roll, 1)
            $pitch = [Math]::Round([double]$q.pitch, 1)
            $yaw = [Math]::Round([double]$q.yaw, 1)
            if ($q.text) {
                $msg = [string]$q.text
                if ($msg.Length -gt 80) {
                    $msg = $msg.Substring(0, 80) + "..."
                }
            }
        }

        $procMs = [double]$r.processing_ms
        $raw = [int]$r.raw_count
        $dcd = [int]$r.decoded_count
        $ts = (Get-Date).ToString("HH:mm:ss.fff")

        Write-Host "[$ts] det=$isDet dec=$isDec est=$isEst raw=$raw dcd=$dcd proc_ms=$procMs x=$tx y=$ty z=$tz roll=$roll pitch=$pitch yaw=$yaw msg='$msg'"

        $samples++
        if ($isDet) { $detected++ }
        if ($isDec) { $decoded++ }
        if ($isEst) { $estimated++ }
        $procSum += $procMs
        $procCount++

        if ($SummaryEvery -gt 0 -and ($samples % $SummaryEvery) -eq 0) {
            $detRatio = if ($samples -gt 0) { [Math]::Round($detected / $samples, 3) } else { 0.0 }
            $decRatio = if ($detected -gt 0) { [Math]::Round($decoded / $detected, 3) } else { 0.0 }
            $estRatio = if ($detected -gt 0) { [Math]::Round($estimated / $detected, 3) } else { 0.0 }
            $procMean = if ($procCount -gt 0) { [Math]::Round($procSum / $procCount, 2) } else { 0.0 }

            Write-Host "--- SUMMARY ($samples samples) ---" -ForegroundColor Green
            Write-Host "det_ratio=$detRatio dec_ratio_when_detected=$decRatio est_ratio_when_detected=$estRatio proc_ms_mean=$procMean"
            Write-Host "----------------------------------" -ForegroundColor Green
        }
    }
    catch {
        $ts = (Get-Date).ToString("HH:mm:ss.fff")
        Write-Host "[$ts] fetch_error"
    }

    Start-Sleep -Milliseconds $PollMs
}
