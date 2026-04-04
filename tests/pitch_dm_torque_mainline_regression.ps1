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
$dmMotor = Join-Path $repoRoot 'Gimbal\modules\motor\DMmotor\dmmotor.c'

Assert-Pattern $gimbalTask 'DMMotorSetRef\(pitch_motor, pitch_ref_rad, 0\.0f\)' 'Pitch mainline is not using the torque-only DM path.'
Assert-NoPattern $gimbalTask 'DMMotorSetMITTargetByProfile\(pitch_motor, pitch_ref_rad\)' 'Pitch mainline still uses MIT full-command profile path.'

Assert-Pattern $dmMotor 'PIDCalculate\(&motor->speed_PID,' 'DM torque mainline is missing the speed-to-torque inner loop.'
Assert-Pattern $dmMotor 'target_angle = 0\.0f;' 'DM torque mainline should zero MIT angle for torque-only path.'
Assert-Pattern $dmMotor 'target_velocity = 0\.0f;' 'DM torque mainline should zero MIT velocity for torque-only path.'
Assert-Pattern $dmMotor 'target_kp = 0\.0f;' 'DM torque mainline should zero MIT stiffness for torque-only path.'
Assert-Pattern $dmMotor 'target_kd = 0\.0f;' 'DM torque mainline should zero MIT damping for torque-only path.'

Write-Output 'PASS: pitch DM torque mainline regression checks'
