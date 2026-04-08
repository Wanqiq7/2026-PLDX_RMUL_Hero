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
$gimbalShoot = Join-Path $repoRoot 'Gimbal\application\shoot\shoot.c'
$chassisShoot = Join-Path $repoRoot 'Chassis\application\shoot\shoot.c'

Assert-Pattern $gimbalShoot 'ApplyLoaderAngleTarget\(loader_discrete_shot_target_angle\)' 'Gimbal shoot discrete loader path is not using angle effort helper.'
Assert-Pattern $gimbalShoot 'ApplyLoaderSpeedTarget\(loader_target_speed\)' 'Gimbal shoot burstfire path is not using speed effort helper.'
Assert-Pattern $gimbalShoot 'ApplyLoaderSpeedTarget\(-LOADER_REVERSE_SPEED_DPS\)' 'Gimbal shoot reverse path is not using speed effort helper.'

Assert-Pattern $chassisShoot 'ApplyLoaderAngleTarget\(loader->measure.total_angle \+ ONE_BULLET_DELTA_ANGLE\)' 'Chassis shoot single-shot path is not using angle effort helper.'
Assert-Pattern $chassisShoot 'ApplyLoaderSpeedTarget\(shoot_cmd_recv.shoot_rate \* 360' 'Chassis shoot burstfire path is not using speed effort helper.'

Write-Output 'PASS: shoot loader state boundary regression checks'
