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
$visionCtrl = Join-Path $repoRoot 'Gimbal\modules\algorithm\controllers\domain\vision_control.h'
$refMgrHeader = Join-Path $repoRoot 'Gimbal\modules\algorithm\controllers\reference\gimbal_ref_manager.h'
$gimbalDoc = Join-Path $repoRoot 'Gimbal\application\gimbal\gimbal.md'

Assert-Pattern $gimbalTask 'yaw_ref_rate_feedforward_rad_s' 'Gimbal app is missing the reference-layer feedforward naming for yaw.'
Assert-Pattern $gimbalTask 'pitch_ref_rate_feedforward_rad_s' 'Gimbal app is missing the reference-layer feedforward naming for pitch.'
Assert-NoPattern $gimbalTask 'yaw_vision_rate_feedforward_rad_s' 'Gimbal app still uses vision-specific naming for the yaw reference-layer feedforward.'
Assert-NoPattern $gimbalTask 'pitch_vision_rate_feedforward_rad_s' 'Gimbal app still uses vision-specific naming for the pitch reference-layer feedforward.'
Assert-NoPattern $gimbalTask 'yaw_vision_rate_ff_rad_s' 'Gimbal app still uses abbreviated ff naming for local feedforward state.'
Assert-NoPattern $gimbalTask 'pitch_vision_rate_ff_rad_s' 'Gimbal app still uses abbreviated ff naming for local feedforward state.'

Assert-Pattern $robotDef 'ff = feedforward' 'Vision upload struct is not documenting that ff means feedforward.'
Assert-Pattern $visionCtrl 'ff = feedforward' 'Vision control output is not documenting that ff means feedforward.'
Assert-Pattern $refMgrHeader 'ff = feedforward' 'Ref manager IO is not documenting that ff means feedforward.'
Assert-Pattern $gimbalDoc 'rate feedforward' 'Gimbal doc is not spelling out the feedforward semantic in the mainline description.'

Write-Output 'PASS: feedforward naming consistency regression checks'
