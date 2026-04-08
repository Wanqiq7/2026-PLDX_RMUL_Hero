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
$gimbalShoot = Join-Path $repoRoot 'Gimbal\application\shoot\shoot.c'
$chassisShoot = Join-Path $repoRoot 'Chassis\application\shoot\shoot.c'
$gimbalCtrl = Join-Path $repoRoot 'Gimbal\modules\algorithm\shoot_effort_controller.h'
$chassisCtrl = Join-Path $repoRoot 'Chassis\modules\algorithm\shoot_effort_controller.h'

Assert-Pattern $gimbalCtrl 'ShootFrictionCalculateEffort' 'Gimbal shoot effort controller is missing friction helper.'
Assert-Pattern $gimbalCtrl 'ShootLoaderCalculateAngleEffort' 'Gimbal shoot effort controller is missing loader angle helper.'
Assert-Pattern $gimbalCtrl 'ShootLoaderCalculateSpeedEffort' 'Gimbal shoot effort controller is missing loader speed helper.'
Assert-Pattern $chassisCtrl 'ShootFrictionCalculateEffort' 'Chassis shoot effort controller is missing friction helper.'

Assert-NoPattern $gimbalShoot 'DJIMotorSetRef\(' 'Gimbal shoot still uses SetRef.'
Assert-NoPattern $chassisShoot 'DJIMotorSetRef\(' 'Chassis shoot still uses SetRef.'
Assert-Pattern $gimbalShoot 'DJIMotorSetEffort\(loader, &effort\)' 'Gimbal shoot loader is not routed through SetEffort.'
Assert-Pattern $chassisShoot 'DJIMotorSetEffort\(loader, &effort\)' 'Chassis shoot loader is not routed through SetEffort.'

Write-Output 'PASS: shoot SetEffort mainline regression checks'
