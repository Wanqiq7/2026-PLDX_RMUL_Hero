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
$sysidTask = Join-Path $repoRoot 'Gimbal\application\sysid\sysid_task.c'

Assert-NoPattern $sysidTask 'motor_controller\.pid_ref =' 'SYSID path should not write DJI internal pid_ref directly.'
Assert-Pattern $sysidTask 'DJIMotorSetRawRef\(sysid_yaw_motor, sysid_data\.step_input\)' 'SYSID Yaw step injection is not using explicit raw-current bypass.'
Assert-Pattern $sysidTask 'DJIMotorSetRef\(sysid_yaw_motor, 0\.0f\)' 'SYSID restore path is not using compatibility SetRef reset.'
Assert-Pattern $sysidTask 'DMMotorSetMITTargets\(sysid_pitch_motor, angle, 0\.0f, torque, 0\.0f, 0\.0f\)' 'SYSID Pitch hold path is not using explicit MIT full-command injection.'

Write-Output 'PASS: gimbal sysid interface boundary regression checks'
