param(
    [string]$Port = "COM15",
    [int]$Baud = 115200,
    [int]$Duration = 20
)

try {
    $port = new-Object System.IO.Ports.SerialPort($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
    $port.ReadTimeout = 1000
    $port.Open()
    Write-Host "Listening on $Port for $Duration seconds. Send a command now!"
    $start = Get-Date
    while ((Get-Date) - $start -lt (New-TimeSpan -Seconds $Duration)) {
        try {
            $line = $port.ReadLine()
            Write-Host $line
        } catch [TimeoutException] {}
    }
    $port.Close()
    Write-Host "`nCapture complete."
} catch {
    Write-Host "Error: $_"
}
