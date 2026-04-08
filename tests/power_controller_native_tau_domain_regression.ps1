$ErrorActionPreference = 'Stop'

function Assert-PathPattern {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if (-not (Select-String -Path $Path -Pattern $Pattern -Quiet)) {
        throw $Message
    }
}

function Assert-NoPathPattern {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if (Select-String -Path $Path -Pattern $Pattern -Quiet) {
        throw $Message
    }
}

function Assert-TextRegex {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if (-not [regex]::IsMatch($Text, $Pattern, [System.Text.RegularExpressions.RegexOptions]::Multiline)) {
        throw $Message
    }
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$powerHeader = Join-Path $repoRoot 'Chassis\modules\power_controller\power_controller.h'
$powerSource = Join-Path $repoRoot 'Chassis\modules\power_controller\power_controller.c'
$chassisApp = Join-Path $repoRoot 'Chassis\application\chassis\chassis.c'
$adapterDoc = Join-Path $repoRoot 'Chassis\modules\motor\tau_ref_adapter.md'
$harnessHeader = Join-Path $repoRoot 'tests\harnesses\power_controller_sample_harness.h'
$harnessSource = Join-Path $repoRoot 'tests\harnesses\power_controller_sample_harness.c'
$powerHeaderText = Get-Content $powerHeader -Raw
$adapterDocText = Get-Content $adapterDoc -Raw

Assert-TextRegex $powerHeaderText 'typedef\s+struct\s*\{\s*[\s\S]*requested_tau_nm[\s\S]*feedback_speed_rad_s[\s\S]*feedback_tau_nm[\s\S]*max_tau_nm[\s\S]*\}\s*PowerWheelObj_t;' 'FAIL: power_controller public API is still legacy raw/current oriented.'
Assert-PathPattern $powerHeader 'PowerGetLimitedWheelTauRef' 'FAIL: power_controller public API is missing native tau-domain limit output API.'
Assert-PathPattern $powerSource 'PowerGetLimitedWheelTauRef' 'FAIL: power_controller implementation is missing native tau-domain limit entry.'
Assert-PathPattern $harnessHeader 'PowerWheelObj_t wheel_objs\[4\]' 'FAIL: sample harness input is still legacy raw/current oriented.'
Assert-PathPattern $harnessHeader 'limited_wheel_tau_ref\[4\]' 'FAIL: sample harness output is missing native tau-domain limited wheel_tau_ref snapshot.'
Assert-PathPattern $harnessSource 'PowerGetLimitedWheelTauRef' 'FAIL: sample harness is not exercising the native tau-domain limit API.'

Assert-NoPathPattern $chassisApp 'LegacyPowerBridgeTauToRawCurrentCmd' 'FAIL: chassis mainline still contains tau->raw bridge helper.'
Assert-NoPathPattern $chassisApp 'LegacyPowerBridgeRawCurrentCmdToTau' 'FAIL: chassis mainline still contains raw->tau bridge helper.'
Assert-NoPathPattern $chassisApp 'LegacyPowerBridgeApplyLimitedOutput' 'FAIL: chassis mainline still routes limited output through LegacyPowerBridge.'

if ([regex]::IsMatch($adapterDocText, 'wheel_tau_ref[\s\S]*LegacyPowerBridge[\s\S]*DJIMotorSetEffort')) {
    throw 'FAIL: tau_ref adapter doc still describes LegacyPowerBridge as the chassis main flow.'
}

Write-Output 'PASS: power controller native tau domain regression checks'
