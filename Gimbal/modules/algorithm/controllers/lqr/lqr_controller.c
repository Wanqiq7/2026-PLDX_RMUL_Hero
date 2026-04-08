/**
 ******************************************************************************
 * @file lqr_controller.c
 * @brief LQR控制器实现
 ******************************************************************************
 */
#include "controllers/lqr/lqr_controller.h"
#include "memory.h"

void LQRInit(LQRInstance *lqr, LQR_Init_Config_s *config) {
  memset(lqr, 0, sizeof(LQRInstance));

  lqr->K_angle = config->K_angle;
  lqr->K_velocity = config->K_velocity;
  lqr->K_integral = config->K_integral;
  lqr->max_out = config->max_out;
  lqr->enable_integral = config->enable_integral;
  lqr->enable_angle_wrap = 1U;
  lqr->integral_limit = config->integral_limit;
  lqr->integral_deadband = config->integral_deadband;
  lqr->integral_decay_coef = config->integral_decay_coef;
  lqr->integral_decay_threshold =
      (config->integral_decay_threshold > 0.0f)
          ? config->integral_decay_threshold
          : 0.3f;

  DWT_GetDeltaT(&lqr->DWT_CNT);
}

float LQRCalculate(LQRInstance *lqr, float measure_angle, float measure_vel,
                   float ref) {
  lqr->dt = DWT_GetDeltaT(&lqr->DWT_CNT);
  lqr->ref = ref;
  lqr->measure_angle = measure_angle;
  lqr->measure_vel = measure_vel;
  lqr->angle_error = lqr->ref - lqr->measure_angle;

  if (lqr->enable_angle_wrap) {
    while (lqr->angle_error > PI) {
      lqr->angle_error -= 2.0f * PI;
    }
    while (lqr->angle_error < -PI) {
      lqr->angle_error += 2.0f * PI;
    }
  }

  const float output_feedback =
      lqr->K_angle * lqr->angle_error - lqr->K_velocity * lqr->measure_vel;
  float output_integral = 0.0f;

  if (lqr->enable_integral && lqr->dt > 0.0f) {
    if (fabsf(lqr->angle_error) > lqr->integral_deadband) {
      float integral_term = lqr->K_integral * lqr->angle_error * lqr->dt;

      if (lqr->integral_decay_coef > 0.0f &&
          lqr->integral_decay_coef < 1.0f &&
          lqr->angle_error * lqr->integral > 0.0f &&
          fabsf(lqr->angle_error) > lqr->integral_decay_threshold) {
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

void LQRReset(LQRInstance *lqr) {
  lqr->integral = 0.0f;
  lqr->angle_error = 0.0f;
  lqr->output = 0.0f;
  DWT_GetDeltaT(&lqr->DWT_CNT);
}
