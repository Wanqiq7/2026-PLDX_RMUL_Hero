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
$djiSource = Join-Path $repoRoot 'Chassis\modules\motor\DJImotor\dji_motor.c'

Assert-Pattern $djiSource 'uint8_t DJIMotorCalculateEffort' 'Chassis DJI motor is missing CalculateEffort.'
Assert-Pattern $djiSource 'effort->semantic = CONTROLLER_OUTPUT_TAU_REF' 'Chassis DJI CalculateEffort is not emitting TAU_REF.'
Assert-Pattern $djiSource 'effort->tau_ref_nm = pid_ref;' 'Chassis DJI PID path is not native tau semantic.'
Assert-NoPattern $djiSource 'effort->tau_ref_nm = DJIMotorRawCurrentToTauRef' 'Chassis DJI CalculateEffort still converts raw/current to tau at branch end.'
Assert-NoPattern $djiSource 'CONTROLLER_OUTPUT_RAW_CURRENT_CMD' 'Chassis DJI control loop still emits raw-current semantics.'

Write-Output 'PASS: chassis DJI native tau semantic regression checks'
