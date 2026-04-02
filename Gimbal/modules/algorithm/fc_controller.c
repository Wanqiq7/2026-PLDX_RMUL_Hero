#include "fc_controller.h"
#include "bsp_dwt.h"
#include "robot_def.h"
#include "user_lib.h"
#include <string.h>

void FCInit(FC_State_s *state) {
  memset(state, 0, sizeof(*state));
  DWT_GetDeltaT(&state->dwt_cnt);
}

void FCReset(FC_State_s *state, float target_angle_rad) {
  state->angle_err_integral = 0.0f;
  state->rate_err_integral = 0.0f;
  state->current_cmd = 0.0f;
  state->target_rate_est = 0.0f;
  state->current_feedforward = 0.0f;
  state->last_target_angle_rad = target_angle_rad;
  state->inited = 1U;
  DWT_GetDeltaT(&state->dwt_cnt);
}

float FCCalculate(const FC_Init_Config_s *param, FC_State_s *state,
                  float target_angle_rad, float angle_feedback_rad,
                  float rate_feedback_rad_s, float dt) {
  float actual_dt = dt;

  if (actual_dt <= 0.0f) {
    actual_dt = DWT_GetDeltaT(&state->dwt_cnt);
  }
  if (actual_dt <= 0.0f || actual_dt > 0.02f) {
    actual_dt = ROBOT_CTRL_PERIOD_S;
  }

  if (!state->inited) {
    FCReset(state, target_angle_rad);
  }

  const float target_rate_lpf =
      float_constrain(param->target_rate_lpf_alpha, 0.0f, 1.0f);
  float target_rate_raw =
      (target_angle_rad - state->last_target_angle_rad) / actual_dt;
  state->last_target_angle_rad = target_angle_rad;

  if (target_rate_lpf <= 0.0f) {
    state->target_rate_est = target_rate_raw;
  } else {
    state->target_rate_est =
        target_rate_lpf * target_rate_raw +
        (1.0f - target_rate_lpf) * state->target_rate_est;
  }

  const float rate_ref_limit =
      float_constrain(param->rate_ref_max, 0.5f, 20.0f);
  state->target_rate_est =
      float_constrain(state->target_rate_est, -rate_ref_limit, rate_ref_limit);

  const float angle_error = target_angle_rad - angle_feedback_rad;
  state->angle_err_integral += angle_error * actual_dt;
  state->angle_err_integral =
      float_constrain(state->angle_err_integral,
                      -param->angle_err_integral_limit,
                      param->angle_err_integral_limit);

  state->current_feedforward =
      param->target_rate_ff_gain * state->target_rate_est;

  float rate_ref = param->angle_loop_kp * angle_error +
                   param->angle_loop_ki * state->angle_err_integral +
                   state->current_feedforward;
  rate_ref = float_constrain(rate_ref, -rate_ref_limit, rate_ref_limit);

  const float rate_error = rate_ref - rate_feedback_rad_s;
  state->rate_err_integral += rate_error * actual_dt;
  state->rate_err_integral =
      float_constrain(state->rate_err_integral,
                      -param->rate_err_integral_limit,
                      param->rate_err_integral_limit);

  state->current_cmd = param->rate_loop_kp * rate_error +
                       param->rate_loop_ki * state->rate_err_integral;
  state->current_cmd =
      float_constrain(state->current_cmd, -param->current_cmd_max,
                      param->current_cmd_max);

  return state->current_cmd;
}
