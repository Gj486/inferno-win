@echo off
REM Register the Dante Bridge ASIO driver in the Windows registry.
REM Must be run as Administrator.

setlocal
cd /d "%~dp0"

set "DLL=%~dp0dante_asio.dll"
if not exist "%DLL%" (
    echo ERROR: dante_asio.dll not found next to this script.
    echo Expected: %DLL%
    pause
    exit /b 1
)

echo Registering %DLL% ...
regsvr32 /s "%DLL%"
if errorlevel 1 (
    echo.
    echo Failed to register the driver.
    echo Make sure you are running this file as Administrator.
    pause
    exit /b 1
)

echo.
echo Dante Bridge ASIO registered successfully.
echo Open your ASIO host and select "Dante Bridge ASIO".
echo.
pause
