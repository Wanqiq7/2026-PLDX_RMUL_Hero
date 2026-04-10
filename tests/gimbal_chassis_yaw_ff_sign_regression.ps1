$ErrorActionPreference = 'Stop'

function Assert-Pattern {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if (-not (Select-String -Path $Path -Pattern $Pattern -Quiet)) {
        throw $Message
    }
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$gimbalTask = Join-Path $repoRoot 'Gimbal\application\gimbal\gimbal.c'

Assert-Pattern `
    -Path $gimbalTask `
    -Pattern 'yaw_chassis_rate_feedforward_rad_s = -gimbal_cmd_recv\.chassis_rotate_wz;' `
    -Message 'Yaw chassis rate feedforward sign is not the expected reverse compensation.'

Write-Output 'PASS: gimbal chassis yaw ff sign regression checks'
