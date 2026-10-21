@echo off
REM Allows DanteBridge through Windows Firewall (any UDP port: Dante audio flows use
REM ports chosen by the operating system). Run as Administrator, from the folder
REM that contains dante_bridge.exe.
cd /d "%~dp0"
net session >nul 2>&1
if errorlevel 1 (
  echo Please right-click firewall.bat and choose "Run as administrator".
  pause
  exit /b 1
)
set "EXE=%~dp0dante_bridge.exe"
if not exist "%EXE%" (
  echo dante_bridge.exe was not found next to this file.
  pause
  exit /b 1
)
netsh advfirewall firewall delete rule name="DanteBridge" >nul 2>&1
netsh advfirewall firewall add rule name="DanteBridge" dir=in action=allow program="%EXE%" protocol=UDP profile=any enable=yes
if errorlevel 1 (
  echo FAILED to add the firewall rule.
) else (
  echo Firewall rule "DanteBridge" added for %EXE%
  echo If you move dante_bridge.exe, run this file again.
)
pause
