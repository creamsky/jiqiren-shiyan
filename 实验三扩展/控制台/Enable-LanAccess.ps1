#requires -Version 7.0
$ErrorActionPreference='Stop'
$resultPath=Join-Path $PSScriptRoot 'logs\lan-access-result.json'
try {
    $ruleName='RobotLab-Console-LAN-8765'
    $rule=Get-NetFirewallRule -Name $ruleName -ErrorAction SilentlyContinue
    if (-not $rule) {
        $rule=New-NetFirewallRule -Name $ruleName -DisplayName '机器人实验共享控制台 · 本地子网 TCP 8765' -Direction Inbound -Action Allow -Protocol TCP -LocalPort 8765 -RemoteAddress LocalSubnet -InterfaceAlias WLAN -Profile Public,Private
    }
    @{ok=$true;name=$rule.Name;enabled=[string]$rule.Enabled;profile=[string]$rule.Profile} | ConvertTo-Json | Set-Content -LiteralPath $resultPath -Encoding utf8
} catch {
    @{ok=$false;error=$_.Exception.Message} | ConvertTo-Json | Set-Content -LiteralPath $resultPath -Encoding utf8
    exit 1
}
