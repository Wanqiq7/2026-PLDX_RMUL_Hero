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
$gimbalTask = Join-Path $repoRoot 'Gimbal\application\gimbal\gimbal.c'
$gimbalDoc = Join-Path $repoRoot 'Gimbal\application\gimbal\gimbal.md'
$algorithmDoc = Join-Path $repoRoot 'Gimbal\modules\algorithm\algorithm.md'
$shootCtrlDoc = Join-Path $repoRoot 'Gimbal\modules\algorithm\controllers\domain\shoot_effort_controller.md'
$adapterDoc = Join-Path $repoRoot 'Gimbal\modules\motor\tau_ref_adapter.md'

Assert-Pattern $gimbalTask 'static void BuildGimbalRefInput' 'Gimbal app is missing ref-input assembly helper.'
Assert-Pattern $gimbalTask 'static void RunYawMainline' 'Gimbal app is missing yaw mainline helper.'
Assert-Pattern $gimbalTask 'static void RunPitchMainline' 'Gimbal app is missing pitch mainline helper.'
Assert-Pattern $gimbalTask 'static void RunGimbalMainline' 'Gimbal app is missing shared mainline helper.'
Assert-Pattern $gimbalTask 'BuildGimbalRefInput\(&ref_input\);' 'GimbalTask is still assembling ref input inline.'
Assert-Pattern $gimbalTask 'RunGimbalMainline\(CONTROLLER_PID, &ref_output, &yaw_effort, &pitch_effort\);' 'Gimbal PID mainline is not routed through the shared helper.'
Assert-Pattern $gimbalTask 'RunGimbalMainline\(CONTROLLER_LQR, &ref_output, &yaw_effort, &pitch_effort\);' 'Gimbal LQR mainline is not routed through the shared helper.'

Assert-Pattern $gimbalDoc 'gimbal_ref_manager' 'Gimbal application doc is missing Ref Manager wording.'
Assert-Pattern $gimbalDoc 'Controller / CalculateEffort' 'Gimbal application doc is missing Effort Controller wording.'
Assert-Pattern $gimbalDoc 'adapter / protocol' 'Gimbal application doc is missing Adapter wording.'
Assert-Pattern $algorithmDoc 'Ref Manager / Arbiter' 'Algorithm overview is missing the Ref Manager layer wording.'
Assert-Pattern $shootCtrlDoc 'Effort Controller' 'shoot_effort_controller doc is not explicitly scoped as the effort-controller layer.'
Assert-Pattern $adapterDoc 'Ref Manager / Arbiter' 'tau_ref_adapter doc is missing the Ref Manager layer wording.'
Assert-Pattern $adapterDoc 'Effort Controller / CalculateEffort' 'tau_ref_adapter doc is missing the Effort Controller layer wording.'
Assert-Pattern $adapterDoc 'Adapter / Protocol' 'tau_ref_adapter doc is missing the Adapter layer wording.'

Write-Output 'PASS: gimbal application boundary regression checks'
