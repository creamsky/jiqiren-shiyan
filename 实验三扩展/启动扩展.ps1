#requires -Version 7.0
param([switch]$RealRobot)
$ErrorActionPreference='Stop'
if($PSVersionTable.PSVersion.Major -lt 7){throw 'PowerShell 7 required'}
& 'C:\Users\asus\AppData\Local\Microsoft\WindowsApps\pwsh.exe' -NoLogo -NoProfile -File "$PSScriptRoot/控制台/Start-Console.ps1" -RealRobot:$RealRobot
exit $LASTEXITCODE
