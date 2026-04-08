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

Assert-Pattern $powerSource 'void PowerGetLimitedOutput' 'PowerGetLimitedOutput is missing.'
Assert-Pattern $powerSource 'cmd_power\[i\] = PredictPower\(instance, torque, speed\)' 'Limited output path is no longer using PredictPower.'
Assert-Pattern $powerSource 'if \(sum_positive_power <= max_power\)' 'Limited output fast path changed unexpectedly.'
Assert-Pattern $powerSource 'if \(sum_error > ERROR_POWER_DISTRIBUTION_THRESHOLD\)' 'Limited output is missing the error-distribution high threshold branch.'
Assert-Pattern $powerSource 'else if \(sum_error > PROP_POWER_DISTRIBUTION_THRESHOLD\)' 'Limited output is missing the proportional-distribution interpolation branch.'
Assert-Pattern $powerSource 'float max_torque = SolveMaxTorque' 'Limited output no longer solves max torque from allocated power.'
Assert-Pattern $powerSource 'torque_scale = float_constrain\(torque_scale, 0\.0f, 1\.0f\)' 'Limited output torque scaling clamp is missing.'

Write-Output 'PASS: limited output equivalence regression checks'
