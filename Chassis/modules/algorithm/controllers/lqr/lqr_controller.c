/**
 ******************************************************************************
 * @file lqr_controller.c
 * @brief 底盘速度LQR控制器实现
 ******************************************************************************
 */
#include "controllers/lqr/lqr_controller.h"
#include "memory.h"

void LQR_Velocity_Init(LQR_Velocity_Instance *lqr,
                       LQR_Velocity_Init_Config_s *config) {
  memset(lqr, 0, sizeof(LQR_Velocity_Instance));

  lqr->K_velocity = config->K_velocity;
  lqr->K_integral = config->K_integral;
  lqr->max_out = config->max_out;
  lqr->enable_integral = config->enable_integral;
  lqr->integral_limit = config->integral_limit;
  lqr->integral_deadband = config->integral_deadband;
  lqr->integral_decay_coef = config->integral_decay_coef;

  DWT_GetDeltaT(&lqr->DWT_CNT);
}

float LQR_Velocity_Calculate(LQR_Velocity_Instance *lqr, float measure_vel,
                             float ref) {
  lqr->dt = DWT_GetDeltaT(&lqr->DWT_CNT);
  lqr->ref = ref;
  lqr->measure_vel = measure_vel;
  lqr->velocity_error = lqr->ref - lqr->measure_vel;

  const float output_feedback = lqr->K_velocity * lqr->velocity_error;
  float output_integral = 0.0f;

  if (lqr->enable_integral && lqr->dt > 0.0f) {
    if (fabsf(lqr->velocity_error) > lqr->integral_deadband) {
      float integral_term = lqr->K_integral * lqr->velocity_error * lqr->dt;

      if (lqr->integral_decay_coef > 0.0f &&
          lqr->integral_decay_coef < 1.0f &&
          lqr->velocity_error * lqr->integral > 0.0f) {
        integral_term *= (1.0f - lqr->integral_decay_coef);
      }

      lqr->integral += integral_term;

      if (lqr->integral > lqr->integral_limit) {
        lqr->integral = lqr->integral_limit;
      } else if (lqr->integral < -lqr->integral_limit) {
        lqr->integral = -lqr->integral_limit;
      }
    }

    output_integral = lqr->integral;
  }

  lqr->output = output_feedback + output_integral;

  if (lqr->output > lqr->max_out) {
    lqr->output = lqr->max_out;
  } else if (lqr->output < -lqr->max_out) {
    lqr->output = -lqr->max_out;
  }

  return lqr->output;
}

void LQR_Velocity_Reset(LQR_Velocity_Instance *lqr) {
  lqr->integral = 0.0f;
  lqr->velocity_error = 0.0f;
  lqr->output = 0.0f;
  DWT_GetDeltaT(&lqr->DWT_CNT);
}
