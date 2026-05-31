@echo off
setlocal

set "BASE=%~dp0blynk-server"
set "JAVA=%BASE%\jre11\jdk-11.0.31+11-jre\bin\java.exe"
set "JAR=%BASE%\server.jar"

if not exist "%JAVA%" (
  echo ERROR: Java runtime not found at "%JAVA%".
  echo Make sure the portable JRE folder exists.
  pause
  exit /b 1
)

if not exist "%JAR%" (
  echo ERROR: Server jar not found at "%JAR%".
  pause
  exit /b 1
)

pushd "%BASE%"
start "Blynk Server" "%JAVA%" -jar "%JAR%"
popd

echo Blynk server started.
pause
