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

$forceHeader = Join-Path $repoRoot 'Chassis\modules\algorithm\controllers\domain\chassis_force_control.h'
$forceSource = Join-Path $repoRoot 'Chassis\modules\algorithm\controllers\domain\chassis_force_control.c'
$djiHeader = Join-Path $repoRoot 'Chassis\modules\motor\DJImotor\dji_motor.h'
$chassisSource = Join-Path $repoRoot 'Chassis\application\chassis\chassis.c'

Assert-Pattern $forceHeader 'ChassisForceControlGetWheelTauRef' 'Missing torque-domain chassis force control getter.'
Assert-Pattern $forceSource 'wheel_tau_ref' 'Missing wheel_tau_ref storage in chassis force control.'
Assert-Pattern $djiHeader 'DJIMotorSetEffort' 'Missing explicit DJI effort-set API.'
Assert-Pattern $chassisSource 'PowerGetLimitedWheelTauRef' 'Chassis tau mainline is not using native tau-domain power limiting.'
Assert-Pattern $chassisSource 'DJIMotorSetEffort' 'Chassis mainline does not hand tau effort to DJI motors.'
Assert-NoPattern $chassisSource 'LegacyPowerBridge' 'Chassis tau mainline still leaks LegacyPowerBridge.'

Write-Output 'PASS: chassis tau mainline regression checks'
