/**
 * @file gimbal_ref_manager.c
 * @brief 云台最小参考仲裁模块实现
 * @note Algorithm层：统一手操/自瞄参考与前馈选择，不直接输出执行量；
 *       当前只处理接管 enter/exit 边沿的一拍冻结，不提供持续平滑
 */

#include "controllers/reference/gimbal_ref_manager.h"

#include "utils/math/user_lib.h"
#include <string.h>

void GimbalRefManagerInit(Gimbal_Ref_Manager_s *manager) {
  if (manager == NULL) {
    return;
  }

  memset(manager, 0, sizeof(*manager));
}

void GimbalRefManagerReset(Gimbal_Ref_Manager_s *manager) {
  GimbalRefManagerInit(manager);
}

void GimbalRefManagerStep(Gimbal_Ref_Manager_s *manager,
                          const Gimbal_Ref_Input_s *input,
                          Gimbal_Ref_Output_s *output) {
  const uint8_t requested_takeover =
      (input != NULL && input->autoaim_mode && input->vision_takeover) ? 1U : 0U;
  float manual_pitch_ref_rad = 0.0f;
  float vision_pitch_ref_rad = 0.0f;

  if (manager == NULL || input == NULL || output == NULL) {
    return;
  }

  manual_pitch_ref_rad =
      float_constrain(input->manual_pitch_ref_rad, input->pitch_min_limit_rad,
                      input->pitch_max_limit_rad);
  vision_pitch_ref_rad =
      float_constrain(input->vision_pitch_ref_rad, input->pitch_min_limit_rad,
                      input->pitch_max_limit_rad);

  memset(output, 0, sizeof(*output));
  output->yaw_ref_rad = input->manual_yaw_ref_rad;
  output->pitch_ref_rad = manual_pitch_ref_rad;
  output->yaw_rate_ff_rad_s = input->manual_yaw_rate_ff_rad_s;
  output->pitch_rate_ff_rad_s = input->manual_pitch_rate_ff_rad_s;

  if (!manager->vision_takeover_latched && requested_takeover) {
    /* 进入视觉接管的当前拍，先冻结上一拍参考，避免边沿跳变。 */
    if (manager->last_output_valid) {
      output->yaw_ref_rad = manager->last_yaw_ref_rad;
      output->pitch_ref_rad = manager->last_pitch_ref_rad;
    }
    output->yaw_rate_ff_rad_s = 0.0f;
    output->pitch_rate_ff_rad_s = 0.0f;
    output->vision_takeover = 1U;
    output->transition = GIMBAL_REF_TRANSITION_VISION_ENTER;
  } else if (manager->vision_takeover_latched && !requested_takeover) {
    /* 退出视觉接管的当前拍，同样只做单拍冻结，不做持续缓释。 */
    if (manager->last_output_valid) {
      output->yaw_ref_rad = manager->last_yaw_ref_rad;
      output->pitch_ref_rad = manager->last_pitch_ref_rad;
    }
    output->yaw_rate_ff_rad_s = 0.0f;
    output->pitch_rate_ff_rad_s = 0.0f;
    output->vision_takeover = 0U;
    output->transition = GIMBAL_REF_TRANSITION_VISION_EXIT;
  } else if (requested_takeover) {
    output->yaw_ref_rad = input->vision_yaw_ref_rad;
    output->pitch_ref_rad = vision_pitch_ref_rad;
    output->yaw_rate_ff_rad_s = input->vision_yaw_rate_ff_rad_s;
    output->pitch_rate_ff_rad_s = input->vision_pitch_rate_ff_rad_s;
    output->vision_takeover = 1U;
  }

  manager->vision_takeover_latched = requested_takeover;
  manager->last_output_valid = 1U;
  manager->last_yaw_ref_rad = output->yaw_ref_rad;
  manager->last_pitch_ref_rad = output->pitch_ref_rad;
}
