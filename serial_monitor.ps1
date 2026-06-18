$portName = $args[0]
if (-not $portName) { $portName = "COM15" }

$p = New-Object System.IO.Ports.SerialPort
$p.PortName = $portName
$p.BaudRate = 115200
$p.Parity = "None"
$p.DataBits = 8
$p.StopBits = "One"
$p.DtrEnable = $true
$p.RtsEnable = $true
$p.ReadTimeout = 500
$p.Open()

Start-Sleep -Seconds 1
$deadline = (Get-Date).AddSeconds(12)
while ((Get-Date) -lt $deadline) {
    if ($p.BytesAvailable -gt 0) {
        $buf = New-Object byte[] 4096
        $n = $p.Read($buf, 0, $buf.Length)
        $text = [System.Text.Encoding]::ASCII.GetString($buf, 0, $n)
        Write-Host $text -NoNewline
    }
    Start-Sleep -Milliseconds 250
}
$p.Close()
