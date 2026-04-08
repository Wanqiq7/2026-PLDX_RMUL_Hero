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
$chassisShoot = Join-Path $repoRoot 'Chassis\application\shoot\shoot.c'
$shootText = Get-Content $chassisShoot -Raw

Assert-NoPattern $chassisShoot '\.kp = 0\.0f,' 'Chassis shoot still has zeroed KP placeholder values.'
Assert-Pattern $chassisShoot 'FRICTION_SPEED_TARGET_15MPS 2500\.0f' 'Chassis shoot is missing 15m/s friction target.'
Assert-Pattern $chassisShoot 'FRICTION_SPEED_TARGET_18MPS 3000\.0f' 'Chassis shoot is missing 18m/s friction target.'
Assert-Pattern $chassisShoot 'FRICTION_SPEED_TARGET_30MPS 5000\.0f' 'Chassis shoot is missing 30m/s friction target.'
if (-not [regex]::IsMatch($shootText, 'case SMALL_AMU_15:[\s\S]*?ApplyShootFrictionTarget\(friction_lf, FRICTION_SPEED_TARGET_15MPS\);')) {
    throw 'Chassis shoot is missing non-zero SMALL_AMU_15 friction target application.'
}
if (-not [regex]::IsMatch($shootText, 'case SMALL_AMU_18:[\s\S]*?ApplyShootFrictionTarget\(friction_lf, FRICTION_SPEED_TARGET_18MPS\);')) {
    throw 'Chassis shoot is missing non-zero SMALL_AMU_18 friction target application.'
}
if (-not [regex]::IsMatch($shootText, 'case SMALL_AMU_30:[\s\S]*?ApplyShootFrictionTarget\(friction_lf, FRICTION_SPEED_TARGET_30MPS\);')) {
    throw 'Chassis shoot is missing non-zero SMALL_AMU_30 friction target application.'
}

Write-Output 'PASS: chassis shoot runtime baseline regression checks'
