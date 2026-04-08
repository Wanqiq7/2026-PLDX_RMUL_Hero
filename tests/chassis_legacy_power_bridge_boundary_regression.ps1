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
$chassisApp = Join-Path $repoRoot 'Chassis\application\chassis\chassis.c'
$chassisDoc = Join-Path $repoRoot 'Chassis\application\chassis\chassis.md'
$adapterDoc = Join-Path $repoRoot 'Chassis\modules\motor\tau_ref_adapter.md'
$adapterDocText = Get-Content $adapterDoc -Raw

Assert-NoPattern $chassisApp 'LegacyPowerBridge' 'Chassis mainline must not retain LegacyPowerBridge after Phase 4 migration.'
Assert-Pattern $chassisApp 'static void ApplyWheelTauRef' 'Chassis app is missing the direct wheel_tau_ref application helper.'
Assert-Pattern $chassisApp 'BuildPowerWheelObjs' 'Chassis app is missing the native tau-domain power-controller input builder.'
Assert-Pattern $chassisApp 'PowerWheelObj_t wheel_objs\[4\]' 'ChassisTask must prepare native tau-domain wheel objects.'
Assert-Pattern $chassisApp 'PowerGetLimitedWheelTauRef' 'ChassisTask must request native tau-domain limited wheel_tau_ref.'
Assert-NoPattern $chassisApp 'DJIMotorSetRef\(' 'Chassis mainline should not leak SetRef into the tau main path.'
Assert-NoPattern $chassisApp 'PowerMotorObj_t motor_objs\[4\]' 'ChassisTask should not keep legacy raw/current power objects in Phase 4.'

Assert-NoPattern $chassisDoc 'LegacyPowerBridge' 'Chassis doc still describes LegacyPowerBridge after Phase 4 migration.'
Assert-Pattern $chassisDoc 'power_controller\(native tau domain\)' 'Chassis doc is missing native tau-domain power-controller wording.'
Assert-Pattern $chassisDoc 'limited wheel_tau_ref' 'Chassis doc is missing limited wheel_tau_ref wording.'
Assert-Pattern $chassisDoc 'DJIMotorSetEffort\(\)' 'Chassis doc is missing direct SetEffort wording.'
if ([regex]::IsMatch($adapterDocText, 'LegacyPowerBridge.*current/raw')) {
    throw 'Chassis tau_ref adapter doc still describes LegacyPowerBridge as a compatibility island in code path.'
}
if (-not [regex]::IsMatch($adapterDocText, 'wheel_tau_ref[\s\S]*power_controller \(native tau domain\)[\s\S]*DJIMotorSetEffort')) {
    throw 'Chassis tau_ref adapter doc is missing the native tau-domain chassis flow.'
}

Write-Output 'PASS: chassis legacy power bridge boundary regression checks'
