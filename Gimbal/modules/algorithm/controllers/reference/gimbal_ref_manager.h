/**
 * @file gimbal_ref_manager.h
 * @brief 云台最小参考仲裁模块接口
 * @note Algorithm层：统一手操/自瞄参考与前馈选择，只提供 enter/exit
 *       过渡帧与单拍冻结，不承担持续平滑或完整接管状态机职责
 */

#ifndef GIMBAL_REF_MANAGER_H
#define GIMBAL_REF_MANAGER_H

#include <stdint.h>

typedef struct {
  float manual_yaw_ref_rad;
  float manual_pitch_ref_rad;
  float pitch_min_limit_rad;
  float pitch_max_limit_rad;
  uint8_t autoaim_mode;
  uint8_t vision_takeover;
  float vision_yaw_ref_rad;
  float vision_pitch_ref_rad;
  float vision_yaw_rate_ff_rad_s;   // ff = feedforward，视觉 Yaw 速度前馈
  float vision_pitch_rate_ff_rad_s; // ff = feedforward，视觉 Pitch 速度前馈
} Gimbal_Ref_Input_s;

typedef enum {
  GIMBAL_REF_TRANSITION_NONE = 0,
  GIMBAL_REF_TRANSITION_VISION_ENTER,
  GIMBAL_REF_TRANSITION_VISION_EXIT,
} Gimbal_Ref_Transition_e;

typedef struct {
  uint8_t vision_takeover_latched; // 上一拍接管状态，仅用于边沿检测
  uint8_t last_output_valid;       // 上一拍输出是否有效，仅用于单拍冻结
  float last_yaw_ref_rad;          // 上一拍 yaw 参考
  float last_pitch_ref_rad;        // 上一拍 pitch 参考
} Gimbal_Ref_Manager_s;

typedef struct {
  float yaw_ref_rad;
  float pitch_ref_rad;
  float yaw_rate_ff_rad_s;   // ff = feedforward，输出 Yaw 速度前馈
  float pitch_rate_ff_rad_s; // ff = feedforward，输出 Pitch 速度前馈
  uint8_t vision_takeover;
  Gimbal_Ref_Transition_e transition;
} Gimbal_Ref_Output_s;

void GimbalRefManagerInit(Gimbal_Ref_Manager_s *manager);
void GimbalRefManagerReset(Gimbal_Ref_Manager_s *manager);
void GimbalRefManagerStep(Gimbal_Ref_Manager_s *manager,
                          const Gimbal_Ref_Input_s *input,
                          Gimbal_Ref_Output_s *output);

#endif // GIMBAL_REF_MANAGER_H
