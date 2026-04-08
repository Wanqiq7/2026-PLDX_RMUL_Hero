/**
 ******************************************************************************
 * @file lqr_controller.h
 * @brief LQR控制器接口定义
 ******************************************************************************
 */
#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include "main.h"
#include "utils/math/arm_math_compat.h"
#include "bsp_dwt.h"
#include "memory.h"
#include "stdint.h"
#include "stdlib.h"
#include <math.h>

typedef struct {
  float K_angle;
  float K_velocity;
  float K_integral;
  float max_out;

  uint8_t enable_integral;
  uint8_t enable_angle_wrap;
  float integral_limit;
  float integral_deadband;
  float integral_decay_coef;
  float integral_decay_threshold;

  float ref;
  float measure_angle;
  float measure_vel;

  float angle_error;
  float integral;

  float output;

  uint32_t DWT_CNT;
  float dt;
} LQRInstance;

typedef struct {
  float K_angle;
  float K_velocity;
  float K_integral;
  float max_out;

  uint8_t enable_integral;
  float integral_limit;
  float integral_deadband;
  float integral_decay_coef;
  float integral_decay_threshold;
} LQR_Init_Config_s;

void LQRInit(LQRInstance *lqr, LQR_Init_Config_s *config);
float LQRCalculate(LQRInstance *lqr, float measure_angle, float measure_vel,
                   float ref);
void LQRReset(LQRInstance *lqr);

#endif // LQR_CONTROLLER_H
