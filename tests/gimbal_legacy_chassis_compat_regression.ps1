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
$legacyChassis = Join-Path $repoRoot 'Gimbal\application\chassis\chassis.c'
$legacyDoc = Join-Path $repoRoot 'Gimbal\application\chassis\chassis.md'
$legacyDocText = Get-Content $legacyDoc -Raw

Assert-Pattern $legacyChassis 'DJIMotorSetRef\(motor_lf, vt_lf\)' 'Legacy Gimbal chassis path should stay on compatibility SetRef for LF.'
Assert-Pattern $legacyChassis 'DJIMotorSetRef\(motor_rf, vt_rf\)' 'Legacy Gimbal chassis path should stay on compatibility SetRef for RF.'
Assert-NoPattern $legacyChassis 'DJIMotorCalculateEffort\(motor_lf, vt_lf, &effort\)' 'Legacy Gimbal chassis path should not half-migrate to SetEffort.'
if (-not [regex]::IsMatch($legacyDocText, 'DJIMotorSetRef\(\)')) {
    throw 'Legacy Gimbal chassis doc is missing legacy compatibility statement.'
}

Write-Output 'PASS: gimbal legacy chassis compatibility regression checks'
