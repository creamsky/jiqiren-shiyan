param([ValidateRange(5,600)][int]$Seconds=30)
#requires -Version 7.0
$ErrorActionPreference='Stop'
$keyPath=Join-Path $env:USERPROFILE '.ssh\robot_lab_ed25519'
$authArgs = @('-o', 'HostKeyAlias=10.226.235.72')
if (Test-Path -LiteralPath $keyPath) { $authArgs += @('-i', $keyPath, '-o', 'IdentitiesOnly=yes') }
else { Write-Host 'Dedicated SSH key missing. Use your robot account password when prompted.' -ForegroundColor Yellow }
$sshPath=Join-Path $env:WINDIR 'System32\OpenSSH\ssh.exe'
$scpPath=Join-Path $env:WINDIR 'System32\OpenSSH\scp.exe'
Write-Host "Experiment 1: map for $Seconds seconds, save PCD/bag, load map and verify localization."
Write-Host 'This program does not drive the robot. Keep it stationary for this automatic verification.'
& $sshPath -tt @authArgs -o StrictHostKeyChecking=ask -o ConnectTimeout=8 nvidia@10.107.60.72 "bash /home/nvidia/lab1/start.sh $Seconds"
if($LASTEXITCODE -ne 0) { Write-Host 'Experiment 1 did not complete. See the remote report and logs.' -ForegroundColor Yellow; exit 2 }
$stampOutput = & $sshPath @authArgs -o ConnectTimeout=8 nvidia@10.107.60.72 'cat /home/nvidia/lab1/latest_map.txt'
if ($LASTEXITCODE -ne 0) { throw 'Could not read remote map identifier.' }
$stamp = ($stampOutput -join '').Trim()
if($stamp -notmatch '^\d{8}_\d{6}$') { throw 'Invalid map identifier' }
$mapDir=Join-Path $PSScriptRoot 'maps'
New-Item -ItemType Directory -Path $mapDir -Force | Out-Null
& $scpPath @authArgs -o ConnectTimeout=8 "nvidia@10.107.60.72:/home/nvidia/lab1/maps/$stamp.bag" "nvidia@10.107.60.72:/home/nvidia/lab1/maps/$stamp.pcd" "nvidia@10.107.60.72:/home/nvidia/lab1/logs/${stamp}_report.json" $mapDir
if($LASTEXITCODE -ne 0) { Write-Host 'Robot experiment passed, but map download failed.' -ForegroundColor Yellow; exit 3 }
Write-Host "EXPERIMENT 1 COMPLETED. Map files: $mapDir. Live localization remains on the robot display." -ForegroundColor Green
