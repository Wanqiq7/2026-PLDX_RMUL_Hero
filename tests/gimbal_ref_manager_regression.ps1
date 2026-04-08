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
$refMgrHeader = Join-Path $repoRoot 'Gimbal\modules\algorithm\gimbal_ref_manager.h'
$refMgrSource = Join-Path $repoRoot 'Gimbal\modules\algorithm\gimbal_ref_manager.c'
$refMgrDoc = Join-Path $repoRoot 'Gimbal\modules\algorithm\gimbal_ref_manager.md'

Assert-Pattern $refMgrHeader 'Gimbal_Ref_Output_s' 'Missing Gimbal ref output definition.'
Assert-Pattern $refMgrHeader 'GimbalRefManagerStep' 'Missing GimbalRefManagerStep declaration.'
Assert-Pattern $refMgrHeader 'GIMBAL_REF_TRANSITION_VISION_ENTER' 'Missing vision-enter transition enum.'
Assert-Pattern $refMgrHeader 'GIMBAL_REF_TRANSITION_VISION_EXIT' 'Missing vision-exit transition enum.'
Assert-Pattern $refMgrHeader 'pitch_min_limit_rad' 'Missing pitch min limit input.'
Assert-Pattern $refMgrHeader 'pitch_max_limit_rad' 'Missing pitch max limit input.'
Assert-Pattern $refMgrHeader '\u53ea\u63d0\u4f9b enter/exit' 'Header is not documenting the minimal enter/exit scope.'
Assert-Pattern $refMgrSource 'output->yaw_ref_rad = input->manual_yaw_ref_rad' 'Manual yaw fallback is missing.'
Assert-Pattern $refMgrSource 'float_constrain\(input->manual_pitch_ref_rad' 'Manual pitch clamping is missing.'
Assert-Pattern $refMgrSource 'float_constrain\(input->vision_pitch_ref_rad' 'Vision pitch clamping is missing.'
Assert-Pattern $refMgrSource '!manager->vision_takeover_latched && requested_takeover' 'Enter-transition logic is missing.'
Assert-Pattern $refMgrSource 'manager->vision_takeover_latched && !requested_takeover' 'Exit-transition logic is missing.'
Assert-Pattern $refMgrSource 'manager->last_output_valid' 'Last-output hold state is missing.'
Assert-Pattern $refMgrDoc '\u6700\u5c0f\u4ef2\u88c1\u5668' 'Doc is not explicitly scoped as a minimal arbiter.'
Assert-Pattern $refMgrDoc '\u53ea\u8d1f\u8d23 enter/exit \u8fb9\u6cbf\u7684\u4e00\u62cd\u8fc7\u6e21' 'Doc is overstating continuous takeover behavior.'
Assert-Pattern $gimbalTask 'GimbalRefManagerStep\(&gimbal_ref_manager, &ref_input, &ref_output\)' 'GimbalTask is not using the ref manager.'
Assert-Pattern $gimbalTask 'BuildGimbalRefInput\(&ref_input\);' 'GimbalTask is not routing ref-input assembly through the helper.'
Assert-Pattern $gimbalTask 'ref_input->pitch_min_limit_rad = PITCH_MIN_ANGLE \* DEG_TO_RAD' 'Pitch min limit is not assembled into the ref manager input helper.'
Assert-Pattern $gimbalTask 'ref_input->pitch_max_limit_rad = PITCH_MAX_ANGLE \* DEG_TO_RAD' 'Pitch max limit is not assembled into the ref manager input helper.'

Write-Output 'PASS: gimbal ref manager regression checks'
