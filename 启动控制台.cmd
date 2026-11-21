@echo off
setlocal
"%USERPROFILE%\AppData\Local\Microsoft\WindowsApps\pwsh.exe" -NoLogo -NoProfile -File "%~dp0Start-Console.ps1" %*
if errorlevel 1 pause
