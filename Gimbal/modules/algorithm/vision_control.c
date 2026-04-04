/**
 * @file vision_control.c
 * @brief 视觉参考整形算法模块实现
 * @note Algorithm层：提供Yaw/Pitch参考与前馈，不直接输出执行量
 */

#include "vision_control.h"
#include "robot_def.h"
#include "user_lib.h"
#include <math.h>

#define TWO_PI_F (2.0f * (float)M_PI)
#define DEG_TO_RAD ((float)M_PI / 180.0f)

/* -------------------------参数约束边界------------------------- */
#define YAW_POS_KP_MAX         1000.0f  // 位置环Kp上限
#define YAW_RATE_REF_MIN       0.1f     // 角速度参考下限 [rad/s]
#define YAW_RATE_REF_MAX       60.0f    // 角速度参考上限 [rad/s]
#define PITCH_POS_KP_MAX       200.0f   // Pitch位置比例上限 [rad/s per rad]
#define PITCH_RATE_MIN         0.1f     // Pitch变化率下限 [rad/s]
#define PITCH_RATE_MAX         20.0f    // Pitch变化率上限 [rad/s]
#define PITCH_ERR_DEADBAND_MAX 5.0f     // Pitch误差死区上限 [deg]
#define PITCH_VEL_FF_MAX       20.0f    // Pitch前馈项上限 [rad/s]

static float WrapToPiRad(float rad) {
  while (rad > (float)M_PI) {
    rad -= TWO_PI_F;
  }
  while (rad <= -(float)M_PI) {
    rad += TWO_PI_F;
  }
  return rad;
}

static float ConstrainPitchRad(float pitch_rad, float pitch_max_deg,
                               float pitch_min_deg) {
  const float pitch_max_rad = pitch_max_deg * DEG_TO_RAD;
  const float pitch_min_rad = pitch_min_deg * DEG_TO_RAD;
  return float_constrain(pitch_rad, pitch_min_rad, pitch_max_rad);
}

void VisionCtrlInit(VisionCtrlState_s *state) {
  if (state == NULL) {
    return;
  }
  state->yaw.inited = 0;
  state->yaw.pos_i = 0.0f;
  state->yaw.rate_i = 0.0f;
  state->pitch.inited = 0;
  state->pitch.ref_rad = 0.0f;
  state->pitch.target_rad_filtered = 0.0f;
  state->dt = 0.0f;
  DWT_GetDeltaT(&state->DWT_CNT);  // 初始化DWT计数器
}

void VisionCtrlReset(VisionCtrlState_s *state, const VisionCtrlParams_s *params,
                     float pitch_feedback_rad) {
  if (state == NULL || params == NULL) {
    return;
  }
  state->yaw.inited = 0;
  state->yaw.pos_i = 0.0f;
  state->yaw.rate_i = 0.0f;

  state->pitch.inited = 0;
  state->pitch.ref_rad = ConstrainPitchRad(pitch_feedback_rad,
                                           params->pitch_max_angle,
                                           params->pitch_min_angle);
  state->pitch.target_rad_filtered = state->pitch.ref_rad;

  DWT_GetDeltaT(&state->DWT_CNT);  // 重置DWT计数器
}

void VisionCtrlStep(VisionCtrlState_s *state, const VisionCtrlParams_s *params,
                    const VisionCtrlInput_s *input, VisionCtrlOutput_s *output) {
  if (state == NULL || params == NULL || input == NULL || output == NULL) {
    return;
  }

  // 获取时间间隔
  state->dt = DWT_GetDeltaT(&state->DWT_CNT);
  float dt = state->dt;
  if (dt <= 0.0f || dt > 0.02f) {
    dt = ROBOT_CTRL_PERIOD_S;
    state->dt = dt;
  }

  // ---------- Yaw双环 ----------
  if (!state->yaw.inited) {
    state->yaw.inited = 1;
    output->yaw_ref_rad = input->yaw_target_rad;
    output->yaw_rate_ff_rad_s = 0.0f;
  } else {
    const float pos_err =
        WrapToPiRad(input->yaw_target_rad - input->yaw_angle_rad);
    const float pos_kp =
        float_constrain(params->yaw_pos_kp, 0.0f, YAW_POS_KP_MAX);
    const float rate_ref_max =
        float_constrain(params->yaw_rate_max, YAW_RATE_REF_MIN, YAW_RATE_REF_MAX);
    float yaw_rate_ff = pos_kp * pos_err;
    yaw_rate_ff += params->yaw_vel_ff * input->target_yaw_vel;
    yaw_rate_ff = float_constrain(yaw_rate_ff, -rate_ref_max, rate_ref_max);

    output->yaw_ref_rad = input->yaw_target_rad;
    output->yaw_rate_ff_rad_s = yaw_rate_ff;
  }

  // ---------- Pitch限速 + 速度前馈 ----------
  if (!state->pitch.inited) {
    state->pitch.inited = 1;
    state->pitch.target_rad_filtered = state->pitch.ref_rad;
    output->pitch_ref_rad = state->pitch.ref_rad;
    output->pitch_rate_ff_rad_s = 0.0f;
    return;
  }

  const float target_limited = ConstrainPitchRad(input->pitch_target_rad,
                                                 params->pitch_max_angle,
                                                 params->pitch_min_angle);
  const float target_lpf_k =
      float_constrain(params->pitch_target_lpf_k, 0.0f, 1.0f);
  const float pitch_pos_kp =
      float_constrain(params->pitch_pos_kp, 0.0f, PITCH_POS_KP_MAX);
  const float pitch_deadband_rad =
      float_constrain(params->pitch_err_deadband_deg, 0.0f,
                      PITCH_ERR_DEADBAND_MAX) *
      DEG_TO_RAD;
  const float rate_max =
      float_constrain(params->pitch_rate_max, PITCH_RATE_MIN, PITCH_RATE_MAX);
  const float pitch_vel_ff_max =
      float_constrain(params->pitch_vel_ff_max, 0.0f, PITCH_VEL_FF_MAX);
  const float step_max = rate_max * dt;

  if (target_lpf_k <= 0.0f) {
    // k=0 视为关闭低通，避免目标被上一帧值“冻结”
    state->pitch.target_rad_filtered = target_limited;
  } else {
    state->pitch.target_rad_filtered =
        target_lpf_k * target_limited +
        (1.0f - target_lpf_k) * state->pitch.target_rad_filtered;
  }

  float err = state->pitch.target_rad_filtered - state->pitch.ref_rad;
  if (fabsf(err) <= pitch_deadband_rad) {
    err = 0.0f;
  }

  float pitch_rate_ref = pitch_pos_kp * err;
  pitch_rate_ref +=
      float_constrain(params->pitch_vel_ff * input->target_pitch_vel,
                      -pitch_vel_ff_max, pitch_vel_ff_max);
  pitch_rate_ref = float_constrain(pitch_rate_ref, -rate_max, rate_max);

  state->pitch.ref_rad += float_constrain(pitch_rate_ref * dt, -step_max, step_max);
  state->pitch.ref_rad = ConstrainPitchRad(state->pitch.ref_rad,
                                           params->pitch_max_angle,
                                           params->pitch_min_angle);

  output->pitch_ref_rad = state->pitch.ref_rad;
  output->pitch_rate_ff_rad_s = pitch_rate_ref;
}
