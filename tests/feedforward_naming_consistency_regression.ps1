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
$gimbalTask = Join-Path $repoRoot 'Gimbal\application\gimbal\gimbal.c'
$robotDef = Join-Path $repoRoot 'Gimbal\application\robot_def.h'
$visionCtrl = Join-Path $repoRoot 'Gimbal\modules\algorithm\vision_control.h'
$refMgrHeader = Join-Path $repoRoot 'Gimbal\modules\algorithm\gimbal_ref_manager.h'
$gimbalDoc = Join-Path $repoRoot 'Gimbal\application\gimbal\gimbal.md'

Assert-Pattern $gimbalTask 'yaw_vision_rate_feedforward_rad_s' 'Gimbal app is missing the full feedforward naming for yaw.'
Assert-Pattern $gimbalTask 'pitch_vision_rate_feedforward_rad_s' 'Gimbal app is missing the full feedforward naming for pitch.'
Assert-NoPattern $gimbalTask 'yaw_vision_rate_ff_rad_s' 'Gimbal app still uses abbreviated ff naming for local feedforward state.'
Assert-NoPattern $gimbalTask 'pitch_vision_rate_ff_rad_s' 'Gimbal app still uses abbreviated ff naming for local feedforward state.'

Assert-Pattern $robotDef 'ff = feedforward' 'Vision upload struct is not documenting that ff means feedforward.'
Assert-Pattern $visionCtrl 'ff = feedforward' 'Vision control output is not documenting that ff means feedforward.'
Assert-Pattern $refMgrHeader 'ff = feedforward' 'Ref manager IO is not documenting that ff means feedforward.'
Assert-Pattern $gimbalDoc 'rate feedforward' 'Gimbal doc is not spelling out the feedforward semantic in the mainline description.'

Write-Output 'PASS: feedforward naming consistency regression checks'
