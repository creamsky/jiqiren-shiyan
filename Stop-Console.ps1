#requires -Version 7.0
param([switch]$Check, [switch]$RealRobot)
$ErrorActionPreference='Stop'
if ($PSVersionTable.PSVersion.Major -lt 7) { throw 'PowerShell 7 required' }
if ($Check) { exit 0 }
$url='http://127.0.0.1:8765'
$entry=Join-Path $PSScriptRoot '控制台/server.cjs'
$copy=Join-Path $PSScriptRoot '实验三扩展/控制台/server.cjs'
$nodes=@(Get-CimInstance Win32_Process -Filter "Name='node.exe'" | Where-Object {
    $line=([string]$_.CommandLine).Replace('/','\')
    $line.Contains($entry.Replace('/','\')) -or $line.Contains($copy.Replace('/','\'))
})
if ($nodes.Count) {
    $state=Invoke-RestMethod "$url/api/status" -TimeoutSec 3
    if ($state.service -ne 'robot-lab-console') { throw '8765 上的服务不是本项目控制台' }
    if ($state.connected) {
        $session=Invoke-RestMethod "$url/api/session" -TimeoutSec 3
        $headers=@{Origin=$url;'X-Control-Token'=$session.token}
        $null=Invoke-RestMethod "$url/api/command" -Method Post -Headers $headers -ContentType 'application/json' -Body '{"seq":1,"command":"stop"}' -TimeoutSec 3
        $confirmed=$false
        for($i=0;$i -lt 35;$i++){
            Start-Sleep -Milliseconds 200
            $state=Invoke-RestMethod "$url/api/status" -TimeoutSec 2
            $s=$state.status
            if ($state.connected -and $state.age -lt 500 -and $s.feedback_fresh -and $null -ne $s.feedback_v -and $null -ne $s.feedback_w -and
                [math]::Abs($s.feedback_v) -lt .025 -and [math]::Abs($s.feedback_w) -lt .04 -and !$state.room.stopping -and
                $state.task.phase -notin @('preparing','running','stopping')) { $confirmed=$true; break }
        }
        if (!$confirmed) { throw '尚未收到真机停稳反馈，控制台保持打开' }
    }
    if ($state.sharedVersion -eq 2) {
        $null=Invoke-RestMethod "$url/api/shutdown" -Method Post -Headers @{Origin=$url} -TimeoutSec 3
    } else {
        foreach($process in $nodes) { Stop-Process -Id $process.ProcessId }
    }
}
$runtime=Join-Path $PSScriptRoot '实验三扩展/仿真/runtime/Runtime.ps1'
if (Test-Path -LiteralPath ($runtime.Replace('Runtime.ps1','.private/vm/scout-lab.vmx'))) {
    $pwsh7='C:\Users\asus\AppData\Local\Microsoft\WindowsApps\pwsh.exe'
    $vm=(& $pwsh7 -NoLogo -NoProfile -File $runtime -Action Status) | ConvertFrom-Json
    if ($vm.Running) {
        & $pwsh7 -NoLogo -NoProfile -File $runtime -Action StopLab
        if ($LASTEXITCODE) { exit $LASTEXITCODE }
        & $pwsh7 -NoLogo -NoProfile -File $runtime -Action Stop
        if ($LASTEXITCODE) { exit $LASTEXITCODE }
    }
}
Write-Output '控制台已关闭。'
