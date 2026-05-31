@echo off
setlocal

powershell -NoProfile -ExecutionPolicy Bypass -Command "Get-CimInstance Win32_Process -Filter \"Name='java.exe'\" | Where-Object { $_.CommandLine -like '*blynk-server*server.jar*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force }"

echo Blynk server stopped (if it was running).
pause
