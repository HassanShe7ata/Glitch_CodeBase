# grab-mac.ps1 — Read MAC address from an ESP32 over USB serial
# Usage: .\grab-mac.ps1              (auto-detect port)
#        .\grab-mac.ps1 -Port COM7   (specify port)

param(
    [string]$Port = "",
    [int]$Baud = 115200,
    [int]$TimeoutSec = 15
)

function Find-ESP32Port {
    $esp = Get-CimInstance Win32_PnPEntity |
        Where-Object { $_.Name -match 'USB-SERIAL|CH340|CP210|Silicon Labs|FTDI|USB JTAG' -and $_.Name -match 'COM(\d+)' } |
        ForEach-Object { if ($_.Name -match 'COM(\d+)') { [pscustomobject]@{ Port = "COM$($Matches[1])"; Name = $_.Name } } }
    return $esp
}

if (-not $Port) {
    Write-Host "Detecting ESP32 serial port..." -ForegroundColor Cyan
    $ports = Find-ESP32Port
    if (-not $ports) {
        Write-Host "No ESP32-like COM port found. Pass -Port COMx explicitly." -ForegroundColor Red
        exit 1
    }
    if ($ports.Count -gt 1) {
        Write-Host "Multiple candidates:" -ForegroundColor Yellow
        $ports | Format-Table -AutoSize
        $Port = Read-Host "Enter COM port (e.g. COM7)"
    } else {
        $Port = $ports[0].Port
        Write-Host "Found: $($ports[0].Name) on $Port" -ForegroundColor Green
    }
}

Write-Host "Opening $Port @ $Baud baud..." -ForegroundColor Cyan
try {
    $serial = New-Object System.IO.Ports.SerialPort $Port, $Baud
    $serial.NewLine = "`n"
    $serial.ReadTimeout = 500
    $serial.Open()
} catch {
    Write-Host "Failed to open $Port`: $_" -ForegroundColor Red
    exit 1
}

# Most ESP32 boards need DTR toggled to reset and enter bootloader/run mode
try {
    $serial.DtrEnable = $true
    Start-Sleep -Milliseconds 100
    $serial.DtrEnable = $false
} catch {}

$start = Get-Date
$macPattern = '([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}'
$found = $null

Write-Host "Listening (timeout $TimeoutSec s). Press the ESP32 RESET button if needed..." -ForegroundColor Cyan

while ((New-TimeSpan -Start $start -End (Get-Date)).TotalSeconds -lt $TimeoutSec) {
    try {
        $line = $serial.ReadLine()
        Write-Host $line
        if ($line -match $macPattern) {
            $found = $Matches[0]
            break
        }
    } catch [TimeoutException] {
        continue
    } catch {
        break
    }
}

$serial.Close()

if ($found) {
    Write-Host ""
    Write-Host "============================================" -ForegroundColor Green
    Write-Host "  MAC ADDRESS : $found" -ForegroundColor Green
    Write-Host "============================================" -ForegroundColor Green
    Write-Host ""
    Write-Host "ESP-NOW stores MACs LITTLE-ENDIAN (LSB first)." -ForegroundColor Yellow
    $bytes = $found -split ':'
    [array]::Reverse($bytes)
    $lendian = $bytes -join ','
    Write-Host "For an ESP-NOW uint8_t[] array, use the reversed byte order:" -ForegroundColor Yellow
    Write-Host "  uint8_t xxxAddress[] = {0x$($bytes[0]), 0x$($bytes[1]), 0x$($bytes[2]), 0x$($bytes[3]), 0x$($bytes[4]), 0x$($bytes[5])};" -ForegroundColor White
} else {
    Write-Host "No MAC found in $TimeoutSec s. Try pressing RESET on the board." -ForegroundColor Red
    exit 1
}
