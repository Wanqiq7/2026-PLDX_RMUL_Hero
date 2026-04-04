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
$robotDef = Join-Path $repoRoot 'Gimbal\application\robot_def.h'
$visionCtrlHeader = Join-Path $repoRoot 'Gimbal\modules\algorithm\vision_control.h'
$visionApp = Join-Path $repoRoot 'Gimbal\application\vision\vision.c'
$gimbalTask = Join-Path $repoRoot 'Gimbal\application\gimbal\gimbal.c'

Assert-NoPattern $robotDef 'yaw_current_cmd' 'Vision upload data still exposes yaw_current_cmd.'
Assert-Pattern $robotDef 'yaw_ref_rad' 'Vision upload data is missing yaw_ref_rad.'
Assert-Pattern $robotDef 'pitch_ref_rad' 'Vision upload data is missing pitch_ref_rad.'
Assert-Pattern $robotDef 'yaw_rate_ff_rad_s' 'Vision upload data is missing yaw_rate_ff_rad_s.'
Assert-Pattern $robotDef 'pitch_rate_ff_rad_s' 'Vision upload data is missing pitch_rate_ff_rad_s.'

Assert-NoPattern $visionCtrlHeader 'yaw_current_cmd' 'Vision control output still exposes yaw_current_cmd.'
Assert-Pattern $visionCtrlHeader 'yaw_ref_rad' 'Vision control output is missing yaw_ref_rad.'
Assert-Pattern $visionCtrlHeader 'pitch_ref_rad' 'Vision control output is missing pitch_ref_rad.'

Assert-NoPattern $visionApp 'vision_upload_data\.yaw_current_cmd' 'Vision app still publishes yaw current command.'
Assert-Pattern $visionApp 'vision_upload_data\.yaw_ref_rad' 'Vision app is not publishing yaw reference.'

Assert-NoPattern $gimbalTask 'DJIMotorSetRawRef\(yaw_motor, vision_data_recv\.yaw_current_cmd\)' 'AutoAim Yaw still bypasses the closed-loop mainline.'
Assert-Pattern $gimbalTask 'yaw_ref_rad = vision_data_recv\.yaw_ref_rad' 'AutoAim Yaw is not consuming vision yaw reference.'
Assert-Pattern $gimbalTask 'DJIMotorSetEffort\(yaw_motor, &yaw_effort\)' 'AutoAim Yaw is not routed through SetEffort.'

Write-Output 'PASS: autoaim reference mainline regression checks'
