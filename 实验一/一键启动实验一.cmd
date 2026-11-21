@echo off
cd /d "%~dp0"
"C:\Users\asus\AppData\Local\Microsoft\WindowsApps\pwsh.exe" -NoLogo -NoProfile -File "%~dp0Start-Lab1.ps1" %*
set "labExit=%errorlevel%"
echo.
pause
exit /b %labExit%
