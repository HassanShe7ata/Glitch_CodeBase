param([string]$Port = "COM15", [int]$Baud = 115200, [int]$Seconds = 15)
$portObj = New-Object System.IO.Ports.SerialPort $Port,$Baud,None,8,One
$portObj.ReadTimeout = 500
$portObj.Open()
Start-Sleep -Seconds 1
$deadline = (Get-Date).AddSeconds($Seconds)
while ((Get-Date) -lt $deadline) {
    if ($portObj.BytesAvailable -gt 0) {
        Write-Host -NoNewline $portObj.ReadExisting()
    }
    Start-Sleep -Milliseconds 200
}
$portObj.Close()
