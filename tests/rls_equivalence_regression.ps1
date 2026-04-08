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

Assert-Pattern $powerSource 'static void PowerRLSUpdate' 'PowerRLSUpdate helper is missing.'
Assert-Pattern $powerSource 'sample_vector\[0\] \+= fabsf\(inst->motor.speeds\[i\]\)' 'RLS sample vector is missing Σ|ω| accumulation.'
Assert-Pattern $powerSource 'sample_vector\[1\] \+= inst->motor.torques\[i\] \* inst->motor.torques\[i\]' 'RLS sample vector is missing Στ² accumulation.'
Assert-Pattern $powerSource 'power_loss = feedback_power_w - mech_power_w - inst->k3' 'RLS power-loss baseline changed unexpectedly.'
Assert-Pattern $powerSource 'RLSUpdate\(&inst->rls, sample_vector, power_loss\)' 'RLS update call is missing.'
Assert-Pattern $powerSource 'inst->k1 = fmaxf\(inst->k1, 1e-5f\)' 'RLS k1 clamp is missing.'
Assert-Pattern $powerSource 'inst->k2 = fmaxf\(inst->k2, 1e-5f\)' 'RLS k2 clamp is missing.'

Write-Output 'PASS: RLS equivalence regression checks'
