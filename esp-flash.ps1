# esp-flash.ps1 — flash and/or read MAC on any ESP32 board
# Usage:
#   .\esp-flash.ps1 -Action read-mac -Port COM3 -Chip esp32s3
#   .\esp-flash.ps1 -Action flash    -Port COM3 -Chip esp32s3 -Bin path\to\firmware.bin -FlashMode dio -FlashSize 8MB -FlashFreq 80m
#   .\esp-flash.ps1 -Action monitor  -Port COM3 -Baud 115200
#   .\esp-flash.ps1 -Action list-ports
param(
    [ValidateSet('read-mac','flash','monitor','list-ports','test')]
    [string]$Action = 'list-ports',
    [string]$Port = 'COM3',
    [ValidateSet('esp32','esp32s2','esp32s3','esp32c3','auto')]
    [string]$Chip = 'auto',
    [string]$Bin = '',
    [string]$FlashMode = 'dio',
    [string]$FlashSize = '8MB',
    [string]$FlashFreq = '80m',
    [int]$Baud = 115200
)

$esp = "C:\Users\Altyseer\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\5.2.0\esptool.exe"

if (-not (Test-Path $esp)) { throw "esptool not found at $esp" }

if ($Action -eq 'list-ports') {
    Get-CimInstance Win32_PnPEntity |
        Where-Object { $_.Name -match 'COM' -and $_.Name -match 'USB|CP210|CH340|Silicon|JTAG|FTDI' } |
        ForEach-Object { if ($_.Name -match 'COM(\d+)') { [pscustomobject]@{ Port = "COM$($Matches[1])"; Name = $_.Name } } } |
        Format-Table -AutoSize -Wrap
    return
}

if ($Action -eq 'test') {
    Write-Host "Probing $Port with $Chip..." -ForegroundColor Cyan
    & $esp --chip $Chip --port $Port chip_id 2>&1
    return
}

if ($Action -eq 'read-mac') {
    if ($Chip -eq 'auto') { $Chip = 'esp32s3' }   # default for AI-Thinker cam; override as needed
    Write-Host "Reading MAC from $Port (chip=$Chip)..." -ForegroundColor Cyan
    $out = & $esp --chip $Chip --port $Port read_mac 2>&1
    $out | ForEach-Object { Write-Host $_ }
    $mac = ($out | Select-String -Pattern 'MAC:\s+([0-9a-fA-F:]{17})').Matches.Groups[1].Value
    if ($mac) {
        $bytes = $mac -split ':'
        [array]::Reverse($bytes)
        $arr = ($bytes | ForEach-Object { '0x' + $_ }) -join ', '
        Write-Host ""
        Write-Host "ESP-NOW byte order (LSB first):" -ForegroundColor Green
        Write-Host "  uint8_t xxxAddress[] = {$arr};" -ForegroundColor White
    }
    return
}

if ($Action -eq 'flash') {
    if (-not $Bin) { throw "Pass -Bin path\to\firmware.bin" }
    if (-not (Test-Path $Bin)) { throw "Binary not found: $Bin" }
    if ($Chip -eq 'auto') { $Chip = 'esp32s3' }
    Write-Host "Flashing $Bin to $Port (chip=$Chip)..." -ForegroundColor Cyan
    & $esp --chip $Chip --port $Port --baud 460800 `
        --before default_reset --after hard_reset `
        write_flash -z --flash_mode $FlashMode --flash_size $FlashSize --flash_freq $FlashFreq `
        0x0 $Bin 2>&1
    return
}

if ($Action -eq 'monitor') {
    Write-Host "Opening $Port @ $Baud. Ctrl+C to exit." -ForegroundColor Cyan
    & $esp --port $Port --chip auto monitor 2>&1
    return
}
