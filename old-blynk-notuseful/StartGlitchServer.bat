@echo off
REM Start the Glitch Flask controller server.
REM Double-click to run. Press Ctrl+C to stop.

cd /d "%~dp0"

echo ============================================
echo   Glitch Controller Server
echo ============================================
echo.

REM Check Python
where python >nul 2>nul
if errorlevel 1 (
    echo [ERR] Python not found in PATH.
    echo       Install Python 3.9+ from https://python.org
    echo       Then:  pip install flask flask-socketio
    pause
    exit /b 1
)

REM Install deps if missing
python -c "import flask, flask_socketio" 2>nul
if errorlevel 1 (
    echo [..] Installing Python dependencies...
    python -m pip install --quiet flask flask-socketio
    if errorlevel 1 (
        echo [ERR] pip install failed.
        pause
        exit /b 1
    )
)

echo [..] Starting Flask server on http://192.168.5.1:5000
echo      Phone: open  http://192.168.5.1:5000
echo      Ctrl+C to stop.
echo.
python server.py
pause
