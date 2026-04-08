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

Assert-Pattern $powerSource 'static void EnergyLoopControl' 'EnergyLoopControl helper is missing.'
Assert-Pattern $powerSource 'if \(inst->error_flags & POWER_ERROR_REFEREE_DISCONNECT\)' 'EnergyLoopControl no longer falls back on referee disconnect.'
Assert-Pattern $powerSource 'if \(inst->cap.online\) \{' 'EnergyLoopControl is missing capacitor-online hard-limit widening.'
Assert-Pattern $powerSource 'sqrtf\(full_buffer_target\) - sqrtf\(buffer_feedback\)' 'EnergyLoopControl is missing the full-buffer sqrt error path.'
Assert-Pattern $powerSource 'sqrtf\(base_buffer_target\) - sqrtf\(buffer_feedback\)' 'EnergyLoopControl is missing the base-buffer sqrt error path.'
Assert-Pattern $powerSource 'inst->limit.power_upper = float_constrain\(ref_limit_w - pd_output_full' 'EnergyLoopControl upper-limit clamp changed unexpectedly.'
Assert-Pattern $powerSource 'inst->limit.power_lower = float_constrain\(ref_limit_w - pd_output_base' 'EnergyLoopControl lower-limit clamp changed unexpectedly.'
Assert-Pattern $powerSource 'if \(cap_gg && ref_gg\)' 'EnergyLoopControl is missing the dual-disconnect conservative branch.'

Write-Output 'PASS: energy loop equivalence regression checks'
