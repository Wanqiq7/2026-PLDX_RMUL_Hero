/**
 ******************************************************************************
 * @file lqr_controller.h
 * @brief 底盘速度LQR控制器接口定义
 ******************************************************************************
 */
#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include "bsp_dwt.h"
#include "stdint.h"
#include <math.h>

typedef struct {
  float K_velocity;
  float K_integral;
  float max_out;

  uint8_t enable_integral;
  float integral_limit;
  float integral_deadband;
  float integral_decay_coef;

  float ref;
  float measure_vel;
  float velocity_error;
  float integral;
  float output;

  uint32_t DWT_CNT;
  float dt;
} LQR_Velocity_Instance;

typedef struct {
  float K_velocity;
  float K_integral;
  float max_out;

  uint8_t enable_integral;
  float integral_limit;
  float integral_deadband;
  float integral_decay_coef;
} LQR_Velocity_Init_Config_s;

void LQR_Velocity_Init(LQR_Velocity_Instance *lqr,
                       LQR_Velocity_Init_Config_s *config);
float LQR_Velocity_Calculate(LQR_Velocity_Instance *lqr, float measure_vel,
                             float ref);
void LQR_Velocity_Reset(LQR_Velocity_Instance *lqr);

#endif // LQR_CONTROLLER_H
