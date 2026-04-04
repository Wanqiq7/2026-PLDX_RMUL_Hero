#ifndef CHASSIS_FORCE_CONTROL_H
#define CHASSIS_FORCE_CONTROL_H

#include "controller.h"
#include "dji_motor.h"
#include <stdint.h>

typedef struct {
  float wheel_radius;
  float reduction_ratio_wheel;
  float half_wheel_base;
  float half_track_width;
  float center_gimbal_offset_x;
  float center_gimbal_offset_y;
  float torque_feedforward_coeff;
  float velocity_lpf_alpha;
  float max_control_force;
  float max_control_torque;
  float max_wheel_tau_ref;
  float wheel_dynamic_tau_default;
  float wheel_resistance_omega_threshold_default;
  float wheel_speed_feedback_tau_gain_default;
  float torque_constant;
  PID_Init_Config_s force_x_pid_config;
  PID_Init_Config_s force_y_pid_config;
  PID_Init_Config_s torque_pid_config;
} ChassisForceControlConfig_s;

void ChassisForceControlInit(const ChassisForceControlConfig_s *config,
                             DJIMotorInstance *motor_rf,
                             DJIMotorInstance *motor_lf,
                             DJIMotorInstance *motor_lb,
                             DJIMotorInstance *motor_rb);
void ChassisForceControlReset(void);
void ChassisForceControlSetCommand(float projected_vx, float projected_vy,
                                   float target_wz);
void ChassisForceControlStep(void);
const float *ChassisForceControlGetWheelTauRef(void);
const float *ChassisForceControlGetTargetWheelOmega(void);

#endif
