Get-Process | Where-Object {$_.ProcessName -like '*platformio*' -or $_.ProcessName -like '*esptool*' -or $_.ProcessName -like '*python*'} | ForEach-Object { Write-Host "PID: $($_.ProcessId) Name: $($_.ProcessName)" }
Write-Host "---"
Get-CimInstance Win32_Process | Where-Object {$_.CommandLine -like '*COM15*'} | ForEach-Object { Write-Host "PID: $($_.ProcessId) Cmd: $($_.CommandLine)" }
