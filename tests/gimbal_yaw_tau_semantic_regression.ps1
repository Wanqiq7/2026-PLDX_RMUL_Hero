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
$gimbalDjiSource = Join-Path $repoRoot 'Gimbal\modules\motor\DJImotor\dji_motor.c'

Assert-Pattern $gimbalDjiSource 'CONTROLLER_OUTPUT_TAU_REF' 'Missing tau-ref semantic in Gimbal DJI motor control.'
Assert-NoPattern $gimbalDjiSource '\.semantic = CONTROLLER_OUTPUT_CURRENT_A' 'Gimbal yaw LQR path still emits current semantics.'
Assert-NoPattern $gimbalDjiSource '\.semantic = CONTROLLER_OUTPUT_RAW_CURRENT_CMD' 'Gimbal yaw PID/SMC path still emits raw current semantics.'

Write-Output 'PASS: gimbal yaw tau semantic regression checks'
