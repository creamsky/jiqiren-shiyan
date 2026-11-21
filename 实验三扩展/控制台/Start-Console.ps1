#requires -Version 7.0
param([switch]$Check, [switch]$RealRobot)
$ErrorActionPreference = 'Stop'
if ($PSVersionTable.PSVersion.Major -lt 7) { throw 'PowerShell 7 required' }
$nodePath = (Get-Command node.exe -ErrorAction Stop).Source
$entry = Join-Path $PSScriptRoot 'launch.cjs'
if ($Check) { & $nodePath --check $entry; exit $LASTEXITCODE }
if ($RealRobot) { & $nodePath $entry --real } else { & $nodePath $entry }
exit $LASTEXITCODE
