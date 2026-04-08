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
$powerSource = Join-Path $repoRoot 'Chassis\modules\power_controller\power_controller.c'

Assert-Pattern $powerSource 'static float PredictPower' 'PredictPower helper is missing.'
Assert-Pattern $powerSource 'return torque \* speed \+ inst->k1 \* fabsf\(speed\) \+ inst->k2 \* torque \* torque \+' 'PredictPower formula is no longer matching the expected power model.'
Assert-Pattern $powerSource 'inst->k3 / 4\.0f' 'PredictPower is no longer distributing k3 per motor branch.'

Write-Output 'PASS: power predict equivalence regression checks'
