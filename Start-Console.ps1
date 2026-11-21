#requires -Version 7.0
param([switch]$Check, [switch]$RealRobot)
$ErrorActionPreference='Stop'
if ($PSVersionTable.PSVersion.Major -lt 7) { throw 'PowerShell 7 required' }
$arguments=@()
if ($Check) { $arguments+='-Check' }
if ($RealRobot) { $arguments+='-RealRobot' }
& 'C:\Users\asus\AppData\Local\Microsoft\WindowsApps\pwsh.exe' -NoLogo -NoProfile -File (Join-Path $PSScriptRoot '控制台/Start-Console.ps1') @arguments
exit $LASTEXITCODE
