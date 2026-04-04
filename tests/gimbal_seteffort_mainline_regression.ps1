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
$djiHeader = Join-Path $repoRoot 'Gimbal\modules\motor\DJImotor\dji_motor.h'
$dmHeader = Join-Path $repoRoot 'Gimbal\modules\motor\DMmotor\dmmotor.h'

Assert-Pattern $djiHeader 'DJIMotorCalculateEffort' 'Missing DJI effort calculation API.'
Assert-Pattern $dmHeader 'DMMotorCalculateTorqueEffort' 'Missing DM effort calculation API.'
Assert-Pattern $dmHeader 'DMMotorSetEffort' 'Missing DM effort setter API.'

Assert-Pattern $gimbalTask 'DJIMotorCalculateEffort' 'Yaw mainline is not calculating effort in module layer.'
Assert-Pattern $gimbalTask 'DJIMotorSetEffort\(yaw_motor,' 'Yaw mainline is not handing effort to SetEffort.'
Assert-Pattern $gimbalTask 'DMMotorCalculateTorqueEffort' 'Pitch mainline is not calculating torque effort in module layer.'
Assert-Pattern $gimbalTask 'DMMotorSetEffort\(pitch_motor,' 'Pitch mainline is not handing effort to SetEffort.'

Assert-NoPattern $gimbalTask 'DJIMotorSetRef\(yaw_motor,' 'Yaw mainline still calls SetRef directly.'
Assert-NoPattern $gimbalTask 'DMMotorSetRef\(pitch_motor,' 'Pitch mainline still calls SetRef directly.'

Write-Output 'PASS: gimbal SetEffort mainline regression checks'
