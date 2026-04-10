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
$robotDef = Join-Path $repoRoot 'Gimbal\application\robot_def.h'
$robotCmd = Join-Path $repoRoot 'Gimbal\application\cmd\robot_cmd.c'
$gimbalTask = Join-Path $repoRoot 'Gimbal\application\gimbal\gimbal.c'
$refMgrHeader = Join-Path $repoRoot 'Gimbal\modules\algorithm\controllers\reference\gimbal_ref_manager.h'
$refMgrSource = Join-Path $repoRoot 'Gimbal\modules\algorithm\controllers\reference\gimbal_ref_manager.c'

Assert-Pattern $robotDef 'manual_yaw_rate_ff_rad_s' 'Gimbal_Ctrl_Cmd_s is missing manual yaw rate feedforward.'
Assert-Pattern $robotDef 'manual_pitch_rate_ff_rad_s' 'Gimbal_Ctrl_Cmd_s is missing manual pitch rate feedforward.'

Assert-Pattern $robotCmd 'typedef struct \{' 'robot_cmd.c is missing the manual aim input state definition.'
Assert-Pattern $robotCmd 'Manual_Aim_Input_State_s' 'robot_cmd.c is missing a dedicated manual aim input state type.'
Assert-Pattern $robotCmd 'manual_aim_input_state' 'robot_cmd.c is missing a dedicated manual aim input state instance.'
Assert-Pattern $robotCmd 'ResetManualAimInputState\(uint8_t preserve_right_button_state\)' 'robot_cmd.c is missing a reset helper for manual aim input state.'
Assert-Pattern $robotCmd 'manual_aim_input_state\.yaw_rate_cmd_rad_s' 'Manual aim yaw rate filter state is not stored in a resettable state object.'
Assert-Pattern $robotCmd 'manual_aim_input_state\.pitch_rate_cmd_rad_s' 'Manual aim pitch rate filter state is not stored in a resettable state object.'
Assert-Pattern $robotCmd 'manual_aim_input_state\.dwt_cnt' 'Manual aim dt state is not stored in a resettable state object.'

Assert-Pattern $refMgrHeader 'manual_yaw_rate_ff_rad_s' 'Gimbal_Ref_Input_s is missing manual yaw rate feedforward.'
Assert-Pattern $refMgrHeader 'manual_pitch_rate_ff_rad_s' 'Gimbal_Ref_Input_s is missing manual pitch rate feedforward.'

Assert-Pattern $gimbalTask 'ref_input->manual_yaw_rate_ff_rad_s = gimbal_cmd_recv\.manual_yaw_rate_ff_rad_s' 'BuildGimbalRefInput is not routing manual yaw feedforward.'
Assert-Pattern $gimbalTask 'ref_input->manual_pitch_rate_ff_rad_s = gimbal_cmd_recv\.manual_pitch_rate_ff_rad_s' 'BuildGimbalRefInput is not routing manual pitch feedforward.'

Assert-Pattern $refMgrSource 'output->yaw_rate_ff_rad_s = input->manual_yaw_rate_ff_rad_s' 'Ref manager is not outputting manual yaw feedforward by default.'
Assert-Pattern $refMgrSource 'output->pitch_rate_ff_rad_s = input->manual_pitch_rate_ff_rad_s' 'Ref manager is not outputting manual pitch feedforward by default.'
Assert-Pattern $refMgrSource 'output->yaw_rate_ff_rad_s = 0\.0f;' 'Ref manager enter/exit transition is not clearing yaw feedforward.'
Assert-Pattern $refMgrSource 'output->pitch_rate_ff_rad_s = 0\.0f;' 'Ref manager enter/exit transition is not clearing pitch feedforward.'

Assert-Pattern $robotCmd 'gimbal_cmd_send\.yaw \+= yaw_rate_cmd_rad_s \* dt;' 'MouseKeySet is not integrating yaw reference from a rad/s command.'
Assert-Pattern $robotCmd 'gimbal_cmd_send\.pitch \+= pitch_rate_cmd_rad_s \* dt;' 'MouseKeySet is not integrating pitch reference from a rad/s command.'
Assert-Pattern $robotCmd 'gimbal_cmd_send\.manual_yaw_rate_ff_rad_s =' 'MouseKeySet is not assigning manual yaw feedforward.'
Assert-Pattern $robotCmd 'yaw_rate_cmd_rad_s \* MANUAL_YAW_RATE_FF_GAIN;' 'MouseKeySet is not scaling manual yaw feedforward with the configured gain.'
Assert-Pattern $robotCmd 'gimbal_cmd_send\.manual_pitch_rate_ff_rad_s =' 'MouseKeySet is not assigning manual pitch feedforward.'
Assert-Pattern $robotCmd 'pitch_rate_cmd_rad_s \* MANUAL_PITCH_RATE_FF_GAIN;' 'MouseKeySet is not scaling manual pitch feedforward with the configured gain.'
Assert-Pattern $robotCmd 'gimbal_cmd_send\.manual_yaw_rate_ff_rad_s = 0\.0f;' 'Manual yaw feedforward is not cleared on mode transitions.'
Assert-Pattern $robotCmd 'gimbal_cmd_send\.manual_pitch_rate_ff_rad_s = 0\.0f;' 'Manual pitch feedforward is not cleared on mode transitions.'
Assert-Pattern $robotCmd 'ResetManualAimInputState\(0U\);' 'Manual aim input state is not reset when leaving keyboard manual aim.'
Assert-Pattern $robotCmd 'ResetManualAimInputState\(1U\);' 'Manual aim input state is not reset on autoaim enter/exit transitions.'

Assert-Pattern $gimbalTask 'yaw_ref_rate_feedforward_rad_s' 'Gimbal app still uses vision-specific naming for the Yaw reference-layer feedforward.'
Assert-Pattern $gimbalTask 'pitch_ref_rate_feedforward_rad_s' 'Gimbal app still uses vision-specific naming for the Pitch reference-layer feedforward.'

Write-Output 'PASS: manual input feedforward regression checks'
