#requires -Version 7.0
param([switch]$Legacy)
$ErrorActionPreference='Stop'
if($PSVersionTable.PSVersion.Major -lt 7){throw 'PowerShell 7 required'}
$project=Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$out=Join-Path $PSScriptRoot 'results'
New-Item -ItemType Directory -Force $out | Out-Null
& 'C:/Program Files/Microsoft Visual Studio/2022/Community/Common7/Tools/Launch-VsDevShell.ps1' -Arch amd64 -HostArch amd64 -SkipAutomaticLocation | Out-Null
$shared=Join-Path $project '实验二/lab26/catkin_ws/src/basic_control'
$arguments=@('/nologo','/std:c++14','/EHsc','/O2','/utf-8',('/I'+$shared+'/include'),('/I'+$project+'/实验三/include'),('/I'+$project+'/实验二扩展/include'),(Join-Path $PSScriptRoot 'navigation_scenarios.cpp'),($project+'/实验三/src/control_core.cpp'))
$name='baseline'
if($Legacy){$arguments=@(('/I'+$PSScriptRoot+'/legacy'))+$arguments}
if(!$Legacy){$arguments+='/DNAVIGATION_V2';$arguments+=($project+'/实验三/src/navigation.cpp');$name='improved'}
Push-Location $out
try {
  & cl.exe @arguments ('/Fe'+$name+'.exe')
  if($LASTEXITCODE -ne 0){throw 'Scenario compilation failed'}
  & (Join-Path $out ($name+'.exe')) (Join-Path $out ($name+'.csv')) | Tee-Object -FilePath (Join-Path $out ($name+'.txt'))
  $result=$LASTEXITCODE
} finally { Pop-Location }
exit $result
