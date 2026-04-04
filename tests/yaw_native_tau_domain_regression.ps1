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
$djiMotor = Join-Path $repoRoot 'Gimbal\modules\motor\DJImotor\dji_motor.c'
$visionCtrl = Join-Path $repoRoot 'Gimbal\modules\algorithm\vision_control.c'

Assert-NoPattern $gimbalTask 'MaxOut = 16384\.0f' 'Yaw speed PID MaxOut still uses raw current domain.'
Assert-NoPattern $gimbalTask 'GM6020 电流指令（CAN 原始量）限幅' 'Yaw speed PID comment still describes raw current domain.'

Assert-NoPattern $djiMotor 'DJIMotorCurrentToTauRef\(&motor->physical_param, lqr_current_output\)' 'Yaw LQR still converts current-domain output at branch end.'
Assert-NoPattern $djiMotor 'DJIMotorRawCurrentToTauRef\(&motor->physical_param, smc_output\)' 'Yaw SMC still converts raw-domain output at branch end.'
Assert-NoPattern $djiMotor 'DJIMotorRawCurrentToTauRef\(&motor->physical_param, pid_ref\)' 'Yaw PID still converts raw-domain output at branch end.'

Assert-Pattern $visionCtrl 'if \(dt <= 0\.0f \|\| dt > 0\.02f\)' 'VisionCtrlStep is missing dt validity clamp.'
Assert-Pattern $visionCtrl 'dt = ROBOT_CTRL_PERIOD_S;' 'VisionCtrlStep is missing fallback control period.'

Write-Output 'PASS: yaw native tau domain regression checks'
