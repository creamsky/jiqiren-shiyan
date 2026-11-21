#requires -Version 7.0
param([ValidateSet('Prepare','Start','Status','Stop','Provision','Launch','StopLab','Forward','Viewer')][string]$Action='Status')
$ErrorActionPreference='Stop'
if($PSVersionTable.PSVersion.Major -lt 7){throw 'PowerShell 7 required'}
$runtimeDir=$PSScriptRoot
$privateDir=Join-Path $runtimeDir '.private'
$cacheDir=Join-Path $runtimeDir '.cache'
$vmDir=Join-Path $privateDir 'vm'
$vmx=Join-Path $vmDir 'scout-lab.vmx'
$vmrun='C:/Program Files (x86)/VMware/VMware Workstation/vmrun.exe'
$diskmanager='C:/Program Files (x86)/VMware/VMware Workstation/vmware-vdiskmanager.exe'
$python=Join-Path $env:USERPROFILE 'anaconda3/envs/embodied_sim/python.exe'
$ssh=Join-Path $env:WINDIR 'System32/OpenSSH/ssh.exe'
$scp=Join-Path $env:WINDIR 'System32/OpenSSH/scp.exe'
$key=Join-Path $privateDir 'ssh_key'
$known=Join-Path $privateDir 'known_hosts'
if(!(Test-Path -LiteralPath $vmrun)){throw 'VMware Workstation is required'}
if($Action -ne 'Status'){
    New-Item -ItemType Directory -Force $privateDir|Out-Null
    $lockName=if($Action -in @('Forward','Viewer')){'forward.lock'}else{'runtime.lock'}
    try{$runtimeLock=[IO.File]::Open((Join-Path $privateDir $lockName),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)}
    catch{throw 'Another simulation runtime operation is in progress'}
}
try {
function GuestIP {
    if(!(Test-Path -LiteralPath $vmx)){return $null}
    $out=& $vmrun getGuestIPAddress $vmx 2>$null
    if($LASTEXITCODE -eq 0 -and $out -match '^192\.168\.100\.\d+$'){return $out.Trim()}
    $leases='C:/ProgramData/VMware/vmnetdhcp.leases'
    if(Test-Path -LiteralPath $leases){
        $text=Get-Content -LiteralPath $leases -Raw
        $hits=[regex]::Matches($text,'lease\s+(192\.168\.100\.\d+)\s*\{([^}]+)\}')
        $match=$hits|Where-Object {$_.Groups[2].Value -match '00:50:56:2a:33:11'}|Select-Object -Last 1
        if($match){return $match.Groups[1].Value}
    }
    return $null
}
function SSHArgs {return @('-i',$key,'-o','BatchMode=yes','-o','StrictHostKeyChecking=accept-new','-o',"UserKnownHostsFile=$known",'-o','ConnectTimeout=8')}
function EnsureForward([int]$LocalPort,[int]$GuestPort,[string]$GuestIP,[string]$PidName,[string]$LogName){
    $forward="127.0.0.1:${LocalPort}:127.0.0.1:${GuestPort}"
    $pidPath=Join-Path $privateDir $PidName
    $owned=@(Get-CimInstance Win32_Process -Filter "Name='ssh.exe'" | Where-Object {
        $_.CommandLine -and $_.CommandLine.Contains($key) -and $_.CommandLine.Contains($forward) -and $_.CommandLine.Contains("sim@$GuestIP")
    })
    $listeners=@(Get-NetTCPConnection -State Listen -LocalPort $LocalPort -ErrorAction SilentlyContinue)
    foreach($listener in $listeners){
        if($listener.OwningProcess -notin $owned.ProcessId){throw "Port $LocalPort is occupied by an unrelated process"}
    }
    if($owned.Count -gt 1){throw "More than one project forwarding process exists for port $LocalPort"}
    if($owned.Count -eq 1){
        [IO.File]::WriteAllText($pidPath,[string]$owned[0].ProcessId)
        return
    }
    $tunnelArguments=@(SSHArgs)+@('-N','-o','ExitOnForwardFailure=yes','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=3','-L',$forward,"sim@$GuestIP")
    $quoted=$tunnelArguments|ForEach-Object {'"'+$_.Replace('"','\"')+'"'}
    $child=Start-Process -FilePath $ssh -ArgumentList $quoted -WindowStyle Hidden -PassThru -RedirectStandardError (Join-Path $privateDir $LogName)
    [IO.File]::WriteAllText($pidPath,[string]$child.Id)
    Start-Sleep -Milliseconds 500
    if($child.HasExited){throw "SSH forwarding for port $LocalPort failed"}
}
if($Action -eq 'Status'){
    $running=@(& $vmrun list 2>$null) -contains $vmx
    [pscustomobject]@{Installed=(Test-Path -LiteralPath $vmx);Running=$running;IP=(GuestIP);Directory=$vmDir}|ConvertTo-Json -Compress
    exit
}
if($Action -eq 'Prepare'){
    # The user lifted the original midnight-only download restriction.
    New-Item -ItemType Directory -Force $privateDir,$cacheDir,$vmDir|Out-Null
    if(Test-Path -LiteralPath $vmx){throw 'Private VM already exists; use Start or Provision'}
    $image='focal-server-cloudimg-amd64.vmdk'
    $manifest=Join-Path $cacheDir 'SHA256SUMS'
    $disk=Join-Path $cacheDir $image
    $mirror='https://mirrors.tuna.tsinghua.edu.cn/ubuntu-cloud-images/focal/current'
    Invoke-WebRequest "$mirror/SHA256SUMS" -OutFile $manifest -TimeoutSec 30
    $entry=Get-Content -LiteralPath $manifest|Where-Object {$_ -match "[ *]$([regex]::Escape($image))$"}|Select-Object -First 1
    if(!$entry){throw 'No checksum for requested cloud disk'}
    $expected=$entry.Split(' ',[StringSplitOptions]::RemoveEmptyEntries)[0]
    if(!(Test-Path -LiteralPath $disk) -or (Get-FileHash -LiteralPath $disk -Algorithm SHA256).Hash -ne $expected){
        Invoke-WebRequest "$mirror/$image" -OutFile "$disk.partial" -TimeoutSec 1200
        if((Get-FileHash -LiteralPath "$disk.partial" -Algorithm SHA256).Hash -ne $expected){throw 'Ubuntu disk checksum mismatch'}
        Move-Item -LiteralPath "$disk.partial" -Destination $disk
    }
    if(!(Test-Path -LiteralPath $key)){
        & (Join-Path $env:WINDIR 'System32/OpenSSH/ssh-keygen.exe') -q -t ed25519 -f $key -N '' -C 'scout-lab-local'
        if($LASTEXITCODE){throw 'SSH key creation failed'}
    }
    $pub=(Get-Content -LiteralPath "$key.pub" -Raw).Trim()
    $seed=Join-Path $privateDir 'seed'
    New-Item -ItemType Directory -Force $seed|Out-Null
    $userdata=@"
#cloud-config
hostname: scout-lab
users:
  - name: sim
    groups: [sudo]
    shell: /bin/bash
    sudo: ALL=(ALL) NOPASSWD:ALL
    lock_passwd: true
    ssh_authorized_keys:
      - $pub
ssh_pwauth: false
disable_root: true
apt:
  preserve_sources_list: false
  primary:
    - arches: [default]
      uri: https://mirrors.tuna.tsinghua.edu.cn/ubuntu/
  security:
    - arches: [default]
      uri: https://mirrors.tuna.tsinghua.edu.cn/ubuntu/
package_update: true
packages: [openssh-server, open-vm-tools, curl, ca-certificates, gnupg]
"@
    [IO.File]::WriteAllText((Join-Path $seed 'user-data'),$userdata.Replace("`r`n","`n"))
    [IO.File]::WriteAllText((Join-Path $seed 'meta-data'),"instance-id: scout-lab-20261119`nlocal-hostname: scout-lab`n")
    & $python (Join-Path $runtimeDir 'make_seed.py') $seed (Join-Path $vmDir 'seed.iso')
    if($LASTEXITCODE){throw 'NoCloud ISO generation failed'}
    & $diskmanager -r $disk -t 0 (Join-Path $vmDir 'ubuntu.vmdk')
    if($LASTEXITCODE){throw 'Cloud disk conversion failed'}
    & $diskmanager -x 30GB (Join-Path $vmDir 'ubuntu.vmdk')
    if($LASTEXITCODE){throw 'Disk expansion failed'}
    $config=@'
.encoding = "UTF-8"
config.version = "8"
virtualHW.version = "19"
displayName = "SCOUT Lab isolated Gazebo"
guestOS = "ubuntu-64"
memsize = "8192"
numvcpus = "4"
cpuid.coresPerSocket = "4"
firmware = "bios"
scsi0.present = "TRUE"
scsi0.virtualDev = "lsilogic"
scsi0:0.present = "TRUE"
scsi0:0.fileName = "ubuntu.vmdk"
ide1:0.present = "TRUE"
ide1:0.deviceType = "cdrom-image"
ide1:0.fileName = "seed.iso"
ethernet0.present = "TRUE"
ethernet0.connectionType = "nat"
ethernet0.virtualDev = "e1000"
ethernet0.addressType = "static"
ethernet0.address = "00:50:56:2a:33:11"
usb.present = "FALSE"
sound.present = "FALSE"
isolation.tools.hgfs.disable = "TRUE"
sharedFolder.maxNum = "0"
tools.syncTime = "TRUE"
svga.present = "TRUE"
mks.enable3d = "FALSE"
uuid.action = "create"
'@
    [IO.File]::WriteAllText($vmx,$config.Replace("`r`n","`n"))
    Write-Output "PRIVATE_VM_PREPARED=$vmx"
    exit
}
if(!(Test-Path -LiteralPath $vmx)){throw 'Private VM not prepared'}
if($Action -eq 'Forward'){
    $ip=GuestIP
    if(!$ip){throw 'Simulation VM has no IP address'}
    EnsureForward 9091 9091 $ip 'tunnel.pid' 'tunnel.log'
    EnsureForward 9092 6080 $ip 'viewer-tunnel.pid' 'viewer-tunnel.log'
    [pscustomobject]@{Forwarded=$true;IP=$ip}|ConvertTo-Json -Compress
    exit
}
if($Action -eq 'Viewer'){
    $ip=GuestIP
    if(!$ip){throw 'Simulation VM has no IP address'}
    $auth=SSHArgs
    & $scp @auth (Join-Path $runtimeDir 'native_viewer.py') "sim@${ip}:/home/sim/native-scout-viewer.py"
    if($LASTEXITCODE){throw 'Native viewer source deployment failed'}
    & $ssh @auth "sim@$ip" '/usr/bin/python3 /home/sim/native-scout-viewer.py'
    if($LASTEXITCODE){throw 'Native viewer could not be recovered'}
    EnsureForward 9092 6080 $ip 'viewer-tunnel.pid' 'viewer-tunnel.log'
    [pscustomobject]@{Viewer=$true;IP=$ip;NativeViewer='http://127.0.0.1:9092'}|ConvertTo-Json -Compress
    exit
}
if($Action -eq 'Start'){
    & $vmrun start $vmx nogui
    if($LASTEXITCODE){throw 'Private VM did not start'}
    exit
}
if($Action -eq 'Stop'){
    & $vmrun stop $vmx soft
    if($LASTEXITCODE){throw 'VM soft shutdown failed'}
    exit
}
if($Action -eq 'Provision'){
    $ip=GuestIP
    if(!$ip){throw 'Guest has not acquired an address yet'}
    $auth=SSHArgs
    & $ssh @auth "sim@$ip" 'cloud-init status --wait'
    if($LASTEXITCODE){throw 'Guest cloud-init is not ready'}
    & $scp @auth (Join-Path $runtimeDir 'guest-provision.sh') "sim@${ip}:/home/sim/guest-provision.sh"
    if($LASTEXITCODE){throw 'Guest provisioning upload failed'}
    & $ssh @auth "sim@$ip" 'bash /home/sim/guest-provision.sh'
    if($LASTEXITCODE){throw 'Guest package installation failed'}
    exit
}
if($Action -in @('Launch','StopLab')){
    if($Action -eq 'Launch'){
        $running=@(& $vmrun list 2>$null) -contains $vmx
        if(!$running){& $vmrun start $vmx nogui;if($LASTEXITCODE){throw 'Private VM failed to start'}}
    }
    $ip=GuestIP
    if(!$ip -and $Action -eq 'Launch'){
        $deadline=[DateTime]::UtcNow.AddSeconds(90)
        do{Start-Sleep -Seconds 2;$ip=GuestIP}while(!$ip -and [DateTime]::UtcNow -lt $deadline)
    }
    if(!$ip){throw 'Guest is starting; retry when it has acquired an IP'}
    $auth=SSHArgs
    if($Action -eq 'Launch'){
        $sshDeadline=[DateTime]::UtcNow.AddSeconds(90)
        do{
            & $ssh @auth "sim@$ip" true 2>$null
            $sshReady=$LASTEXITCODE -eq 0
            if(!$sshReady){Start-Sleep -Seconds 2;$newIP=GuestIP;if($newIP){$ip=$newIP}}
        }while(!$sshReady -and [DateTime]::UtcNow -lt $sshDeadline)
        if(!$sshReady){throw 'Private VM did not become available over SSH'}
    }
    & $scp @auth (Join-Path $runtimeDir 'stop_lab.py') "sim@${ip}:/home/sim/stop-scout-lab.py"
    if($LASTEXITCODE){throw 'Simulation process lifecycle helper upload failed'}
    if($Action -eq 'StopLab'){
        & $ssh @auth "sim@$ip" '/usr/bin/python3 /home/sim/stop-scout-lab.py'
        if($LASTEXITCODE){throw 'Isolated Gazebo processes did not finish shutting down'}
        foreach($file in 'tunnel.pid','viewer-tunnel.pid'){
            $tunnelPidFile=Join-Path $privateDir $file
            if(Test-Path -LiteralPath $tunnelPidFile){
                $tunnelId=[int](Get-Content -LiteralPath $tunnelPidFile)
                $tunnelProcess=Get-CimInstance Win32_Process -Filter "ProcessId=$tunnelId" -ErrorAction SilentlyContinue
                $forward=if($file -eq 'tunnel.pid'){'127.0.0.1:9091:127.0.0.1:9091'}else{'127.0.0.1:9092:127.0.0.1:6080'}
                if($tunnelProcess -and $tunnelProcess.Name -eq 'ssh.exe' -and $tunnelProcess.CommandLine.Contains($key) -and $tunnelProcess.CommandLine.Contains($forward)){
                    Stop-Process -Id $tunnelId -ErrorAction SilentlyContinue
                    Wait-Process -Id $tunnelId -Timeout 5 -ErrorAction SilentlyContinue
                }
                Remove-Item -LiteralPath $tunnelPidFile
            }
        }
        exit
    }
    $existing=& $ssh @auth "sim@$ip" 'if test -f /home/sim/scout-lab/gazebo.pid; then p=$(cat /home/sim/scout-lab/gazebo.pid); if ps -p "$p" -o args= | grep -q "scout_lab field.launch"; then echo RUNNING; fi; fi'
    if($LASTEXITCODE){throw 'SSH connection to simulation VM failed'}
    if($existing -ne 'RUNNING'){
        # A crashed roslaunch may leave its gzserver alive. Clear only processes
        # with this project's exact isolated master addresses and package path.
        & $ssh @auth "sim@$ip" '/usr/bin/python3 /home/sim/stop-scout-lab.py'
        if($LASTEXITCODE){throw 'Previous isolated simulation has not exited'}
        & $ssh @auth "sim@$ip" 'test -f /opt/ros/noetic/setup.bash'
        if($LASTEXITCODE){throw 'ROS/Gazebo dependencies are not installed; run Provision'}
        $sourceDir=Join-Path $privateDir 'source'
        $stageSim=Join-Path $sourceDir 'sim'
        $stageDwa=Join-Path $sourceDir 'dwa'
        New-Item -ItemType Directory -Force $stageSim,$stageDwa|Out-Null
        $simDir=Split-Path $runtimeDir
        Copy-Item -LiteralPath (Join-Path $simDir 'ros') -Destination $stageSim -Recurse -Force
        Copy-Item -LiteralPath (Join-Path $simDir 'start-gazebo.sh') -Destination $stageSim -Force
        $dwaDir=Join-Path (Split-Path $simDir) 'dwa'
        foreach($file in 'package.xml','navigation.yaml'){Copy-Item -LiteralPath (Join-Path $dwaDir $file) -Destination $stageDwa -Force}
        $limitsScript=Join-Path $simDir 'ros/scout_lab/scripts/source_limits.py'
        $limitsJson=& $python $limitsScript --root (Split-Path $simDir)
        if($LASTEXITCODE){throw 'Real control limits could not be read from the source'}
        $limitsConfig=Join-Path $stageSim 'ros/scout_lab/config'
        New-Item -ItemType Directory -Force $limitsConfig|Out-Null
        [IO.File]::WriteAllText((Join-Path $limitsConfig 'motion_limits.json'),($limitsJson -join "`n")+"`n")
        $archive=Join-Path $privateDir 'source.tar.gz'
        & tar -czf $archive -C $sourceDir sim dwa
        if($LASTEXITCODE){throw 'Simulation source archive creation failed'}
        & $scp @auth $archive "sim@${ip}:/home/sim/scout-source.tar.gz"
        if($LASTEXITCODE){throw 'Simulation source transfer failed'}
        & $ssh @auth "sim@$ip" 'mkdir -p /home/sim/scout-lab; tar -xzf /home/sim/scout-source.tar.gz -C /home/sim/scout-lab; find /home/sim/scout-lab/sim/ros/scout_lab/scripts -name "*.py" -exec chmod u+x {} +'
        if($LASTEXITCODE){throw 'Simulation source deployment failed'}
    }
    & $ssh @auth "sim@$ip" 'cd /home/sim/scout-lab; if test -f gazebo.pid; then p=$(cat gazebo.pid); if ps -p "$p" -o args= | grep -q "scout_lab field.launch"; then echo GAZEBO_ALREADY_RUNNING; exit 0; fi; fi; setsid bash sim/start-gazebo.sh gui:=false >gazebo.log 2>&1 </dev/null & echo $! >gazebo.pid; echo GAZEBO_STARTED'
    if($LASTEXITCODE){throw 'Gazebo process launch failed'}
    EnsureForward 9091 9091 $ip 'tunnel.pid' 'tunnel.log'
    [pscustomobject]@{Started=$true;IP=$ip;Log='/home/sim/scout-lab/gazebo.log';Bridge='ws://127.0.0.1:9091'}|ConvertTo-Json -Compress
}
} finally {
    if($runtimeLock){$runtimeLock.Dispose()}
}
