@echo off
REM Unregister the Dante Bridge ASIO driver from the Windows registry.
REM Must be run as Administrator.

setlocal
cd /d "%~dp0"

set "DLL=%~dp0dante_asio.dll"
if not exist "%DLL%" (
    echo WARNING: dante_asio.dll not found next to this script.
    echo Unregistering by CLSID instead.
    reg delete "HKLM\SOFTWARE\Classes\CLSID\{7B1D3E52-9C4A-4F0E-8A2B-5D6E7F809A1B}" /f >nul 2>&1
    reg delete "HKLM\SOFTWARE\ASIO\Dante Bridge ASIO" /f >nul 2>&1
    echo Done.
    pause
    exit /b 0
)

echo Unregistering %DLL% ...
regsvr32 /s /u "%DLL%"

REM Clean up in case regsvr32 left anything behind.
reg delete "HKLM\SOFTWARE\Classes\CLSID\{7B1D3E52-9C4A-4F0E-8A2B-5D6E7F809A1B}" /f >nul 2>&1
reg delete "HKLM\SOFTWARE\ASIO\Dante Bridge ASIO" /f >nul 2>&1

echo.
echo Dante Bridge ASIO unregistered.
echo.
pause
