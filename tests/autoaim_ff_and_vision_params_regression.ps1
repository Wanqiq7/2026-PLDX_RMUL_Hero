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

function Assert-NoPattern {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if (Select-String -Path $Path -Pattern $Pattern -Quiet) {
        throw $Message
    }
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$gimbalTask = Join-Path $repoRoot 'Gimbal\application\gimbal\gimbal.c'
$visionCtrlHeader = Join-Path $repoRoot 'Gimbal\modules\algorithm\vision_control.h'
$visionApp = Join-Path $repoRoot 'Gimbal\application\vision\vision.c'
$gimbalDoc = Join-Path $repoRoot 'Gimbal\application\gimbal\gimbal.md'

$gimbalText = Get-Content $gimbalTask -Raw
$yawConfigPattern = 'Motor_Init_Config_s yaw_config = \{[\s\S]*?feedforward_flag = SPEED_FEEDFORWARD'
if (-not [regex]::IsMatch($gimbalText, $yawConfigPattern)) {
    throw 'Yaw vision speed feedforward is still not enabled.'
}

Assert-NoPattern $visionCtrlHeader 'yaw_pos_ki' 'vision_control.h still exposes dead yaw_pos_ki parameter.'
Assert-NoPattern $visionCtrlHeader 'yaw_rate_kp' 'vision_control.h still exposes dead yaw_rate_kp parameter.'
Assert-NoPattern $visionCtrlHeader 'yaw_rate_ki' 'vision_control.h still exposes dead yaw_rate_ki parameter.'
Assert-NoPattern $visionCtrlHeader 'yaw_current_max' 'vision_control.h still exposes dead yaw_current_max parameter.'
Assert-NoPattern $visionCtrlHeader 'pos_i' 'vision_control.h still exposes dead yaw integrator state.'
Assert-NoPattern $visionCtrlHeader 'rate_i' 'vision_control.h still exposes dead yaw integrator state.'

Assert-NoPattern $visionApp 'yaw_pos_ki' 'vision.c still initializes dead yaw_pos_ki parameter.'
Assert-NoPattern $visionApp 'yaw_rate_kp' 'vision.c still initializes dead yaw_rate_kp parameter.'
Assert-NoPattern $visionApp 'yaw_rate_ki' 'vision.c still initializes dead yaw_rate_ki parameter.'
Assert-NoPattern $visionApp 'yaw_current_max' 'vision.c still initializes dead yaw_current_max parameter.'

Assert-NoPattern $gimbalDoc 'OPEN_LOOP' 'gimbal.md still describes the Yaw mainline as OPEN_LOOP.'
Assert-NoPattern $gimbalDoc 'SetRawRef' 'gimbal.md still describes raw-current bypass as the current mainline.'

Write-Output 'PASS: autoaim ff and vision params regression checks'
