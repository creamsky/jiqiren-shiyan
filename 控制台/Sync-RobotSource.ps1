#requires -Version 7.0
$ErrorActionPreference='Stop'
if($PSVersionTable.PSVersion.Major -lt 7){throw 'PowerShell 7 required'}
$projectRoot=Split-Path -Parent $PSScriptRoot
$logDir=Join-Path $PSScriptRoot 'logs'
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
$archivePath=Join-Path $logDir 'robot_source.tar.gz'
$sourcePaths=@('项目导航.md','机器人运动原理入门.md','机器人连接笔记.md','Start-Console.ps1','Stop-Console.ps1','启动控制台.cmd','关闭控制台.cmd','实验一/说明.md','实验一/运行脚本','实验一/源码','实验二/说明.md','实验二/lab26/catkin_ws/src/basic_control','实验二/lab26/launch','实验二扩展')
$sourcePaths+=Get-ChildItem -LiteralPath $PSScriptRoot -File | Where-Object {$_.Extension -in @('.cjs','.js','.json','.html','.css','.md','.ps1','.sh') -or $_.Name -eq 'robot-source.gitignore'} | ForEach-Object {'控制台/'+$_.Name}
$sourcePaths+=Get-ChildItem -LiteralPath (Join-Path $projectRoot '实验二/lab26') -File | ForEach-Object {'实验二/lab26/'+$_.Name}
$sourcePaths+=@('实验二/lab26/verification/local_update/manual_live_ack.json','实验二/lab26/verification/local_update/manual_ros_results.json','实验二/lab26/verification/local_update/eight_ros_results.json','实验二/lab26/verification/local_update/deploy_manual.sh')
$tarPath=Join-Path $env:WINDIR 'System32/tar.exe'
$sshPath=Join-Path $env:WINDIR 'System32/OpenSSH/ssh.exe'
$scpPath=Join-Path $env:WINDIR 'System32/OpenSSH/scp.exe'
$keyPath=Join-Path $env:USERPROFILE '.ssh/robot_lab_ed25519'
$sshOptions=@('-i',$keyPath,'-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','HostKeyAlias=10.226.235.72')
& $tarPath -czf $archivePath --exclude=__pycache__ --exclude=.git -C $projectRoot @sourcePaths
if($LASTEXITCODE -ne 0){throw 'Source archive failed'}
& $scpPath @sshOptions $archivePath 'nvidia@10.107.60.72:/home/nvidia/lab26-console-stage/robot_source.tar.gz'
if($LASTEXITCODE -ne 0){throw 'Source upload failed'}
& $scpPath @sshOptions (Join-Path $PSScriptRoot 'sync_robot_source.sh') 'nvidia@10.107.60.72:/home/nvidia/lab26-console-stage/sync_robot_source.sh'
if($LASTEXITCODE -ne 0){throw 'Sync script upload failed'}
& $sshPath @sshOptions 'nvidia@10.107.60.72' 'bash /home/nvidia/lab26-console-stage/sync_robot_source.sh'
if($LASTEXITCODE -ne 0){throw 'Robot source sync failed; inspect uncommitted robot changes'}
Write-Host 'Robot Git source repository updated. Running services were not changed.'
