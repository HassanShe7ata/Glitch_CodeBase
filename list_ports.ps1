[System.IO.Ports.SerialPort]::GetPortNames() | ForEach-Object { Write-Host "Port: $_" }
