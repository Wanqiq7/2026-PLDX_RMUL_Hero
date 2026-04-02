#include "chassis_force_control.h"
#include "general_def.h"
#include "user_lib.h"

#include <math.h>
#include <string.h>

typedef struct {
  ChassisForceControlConfig_s config;
  DJIMotorInstance *motors[4]; // {RF, LF, LB, RB}
  PIDInstance force_x_pid;
  PIDInstance force_y_pid;
  PIDInstance torque_pid;
  float wheel_center[4];
  float filtered_vx;
  float filtered_vy;
  float filtered_wz;
  float estimated_vx;
  float estimated_vy;
  float estimated_wz;
  float projected_vx;
  float projected_vy;
  float target_wz;
  float force_x;
  float force_y;
  float torque_z;
  float wheel_force[4];
  float wheel_current[4];
  float target_wheel_omega[4];
  uint8_t initialized;
} ChassisForceControlContext_s;

static ChassisForceControlContext_s chassis_force_ctx;

volatile float wheel_dynamic_resistance_current[4] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float wheel_resistance_omega_threshold = 0.0f;
volatile float wheel_speed_feedback_gain = 0.0f;

static void ResetPIDRuntimeState(PIDInstance *pid) {
  if (pid == NULL) {
    return;
  }

  pid->Measure = 0.0f;
  pid->Last_Measure = 0.0f;
  pid->Err = 0.0f;
  pid->Last_Err = 0.0f;
  pid->Last_ITerm = 0.0f;
  pid->Pout = 0.0f;
  pid->Iout = 0.0f;
  pid->Dout = 0.0f;
  pid->ITerm = 0.0f;
  pid->Output = 0.0f;
  pid->Last_Output = 0.0f;
  pid->Last_Dout = 0.0f;
  pid->Ref = 0.0f;
  pid->dt = 0.0f;
  pid->ERRORHandler.ERRORCount = 0U;
  pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
  DWT_GetDeltaT(&pid->DWT_CNT);
}

static float ApplyMotorDirectionToFeedback(const DJIMotorInstance *motor,
                                           float value) {
  if (motor != NULL &&
      motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
    return -value;
  }
  return value;
}

static void EstimateChassisVelocity(void) {
  const float speed_to_linear =
      DEGREE_2_RAD * chassis_force_ctx.config.wheel_radius /
      chassis_force_ctx.config.reduction_ratio_wheel;
  float rf_linear_vel = ApplyMotorDirectionToFeedback(
      chassis_force_ctx.motors[0],
      chassis_force_ctx.motors[0]->measure.speed_aps * speed_to_linear);
  float lf_linear_vel = ApplyMotorDirectionToFeedback(
      chassis_force_ctx.motors[1],
      chassis_force_ctx.motors[1]->measure.speed_aps * speed_to_linear);
  float lb_linear_vel = ApplyMotorDirectionToFeedback(
      chassis_force_ctx.motors[2],
      chassis_force_ctx.motors[2]->measure.speed_aps * speed_to_linear);
  float rb_linear_vel = ApplyMotorDirectionToFeedback(
      chassis_force_ctx.motors[3],
      chassis_force_ctx.motors[3]->measure.speed_aps * speed_to_linear);

  const float rotation_radius =
      chassis_force_ctx.config.half_wheel_base +
      chassis_force_ctx.config.half_track_width;
  const float inv_4 = 0.25f;
  float instant_vx =
      (-lf_linear_vel - rf_linear_vel + lb_linear_vel + rb_linear_vel) * inv_4;
  float instant_vy =
      (-lf_linear_vel + rf_linear_vel - lb_linear_vel + rb_linear_vel) * inv_4;
  float instant_wz =
      (lf_linear_vel + rf_linear_vel + lb_linear_vel + rb_linear_vel) /
      (4.0f * rotation_radius);

  chassis_force_ctx.estimated_vx = LowPassFilter_Float(
      instant_vx, chassis_force_ctx.config.velocity_lpf_alpha,
      &chassis_force_ctx.filtered_vx);
  chassis_force_ctx.estimated_vy = LowPassFilter_Float(
      instant_vy, chassis_force_ctx.config.velocity_lpf_alpha,
      &chassis_force_ctx.filtered_vy);
  chassis_force_ctx.estimated_wz = LowPassFilter_Float(
      instant_wz, chassis_force_ctx.config.velocity_lpf_alpha,
      &chassis_force_ctx.filtered_wz);
}

static float CalculateTorqueFeedforward(float target_wz) {
  return chassis_force_ctx.config.torque_feedforward_coeff * target_wz;
}

static void MecanumKinematicsCalculate(void) {
  float wheel_linear_vel[4] = {0.0f};
  float vx = chassis_force_ctx.projected_vx;
  float vy = chassis_force_ctx.projected_vy;
  float wz = chassis_force_ctx.target_wz;

  wheel_linear_vel[0] = (-vx + vy) + wz * chassis_force_ctx.wheel_center[0];
  wheel_linear_vel[1] = (-vx - vy) + wz * chassis_force_ctx.wheel_center[1];
  wheel_linear_vel[2] = (vx - vy) + wz * chassis_force_ctx.wheel_center[2];
  wheel_linear_vel[3] = (vx + vy) + wz * chassis_force_ctx.wheel_center[3];

  for (int i = 0; i < 4; i++) {
    float wheel_omega =
        wheel_linear_vel[i] / chassis_force_ctx.config.wheel_radius;
    chassis_force_ctx.target_wheel_omega[i] =
        wheel_omega * chassis_force_ctx.config.reduction_ratio_wheel;
  }
}

static void VelocityToForceControl(void) {
  float target_vx = chassis_force_ctx.projected_vx;
  float target_vy = chassis_force_ctx.projected_vy;
  float target_wz = chassis_force_ctx.target_wz;
  float torque_feedback = 0.0f;

  EstimateChassisVelocity();

  chassis_force_ctx.force_x = PIDCalculate(&chassis_force_ctx.force_x_pid,
                                           chassis_force_ctx.estimated_vx,
                                           target_vx);
  chassis_force_ctx.force_y = PIDCalculate(&chassis_force_ctx.force_y_pid,
                                           chassis_force_ctx.estimated_vy,
                                           target_vy);
  torque_feedback = PIDCalculate(&chassis_force_ctx.torque_pid,
                                 chassis_force_ctx.estimated_wz, target_wz);
  chassis_force_ctx.torque_z =
      torque_feedback + CalculateTorqueFeedforward(target_wz);

  chassis_force_ctx.force_x =
      float_constrain(chassis_force_ctx.force_x,
                      -chassis_force_ctx.config.max_control_force,
                      chassis_force_ctx.config.max_control_force);
  chassis_force_ctx.force_y =
      float_constrain(chassis_force_ctx.force_y,
                      -chassis_force_ctx.config.max_control_force,
                      chassis_force_ctx.config.max_control_force);
  chassis_force_ctx.torque_z =
      float_constrain(chassis_force_ctx.torque_z,
                      -chassis_force_ctx.config.max_control_torque,
                      chassis_force_ctx.config.max_control_torque);
}

static void ForceDynamicsInverseResolution(void) {
  chassis_force_ctx.wheel_force[0] =
      (-chassis_force_ctx.force_x + chassis_force_ctx.force_y) / 4.0f +
      chassis_force_ctx.torque_z / (4.0f * chassis_force_ctx.wheel_center[0]);
  chassis_force_ctx.wheel_force[1] =
      (-chassis_force_ctx.force_x - chassis_force_ctx.force_y) / 4.0f +
      chassis_force_ctx.torque_z / (4.0f * chassis_force_ctx.wheel_center[1]);
  chassis_force_ctx.wheel_force[2] =
      (chassis_force_ctx.force_x - chassis_force_ctx.force_y) / 4.0f +
      chassis_force_ctx.torque_z / (4.0f * chassis_force_ctx.wheel_center[2]);
  chassis_force_ctx.wheel_force[3] =
      (chassis_force_ctx.force_x + chassis_force_ctx.force_y) / 4.0f +
      chassis_force_ctx.torque_z / (4.0f * chassis_force_ctx.wheel_center[3]);
}

static float CalculateSpeedFeedback(float target_omega, float actual_omega) {
  return wheel_speed_feedback_gain * (target_omega - actual_omega);
}

static float CalculateFrictionCompensation(float target_omega, float actual_omega,
                                           uint8_t motor_index) {
  float dynamic_resistance_current =
      wheel_dynamic_resistance_current[motor_index];

  if (target_omega > wheel_resistance_omega_threshold) {
    return dynamic_resistance_current;
  }
  if (target_omega < -wheel_resistance_omega_threshold) {
    return -dynamic_resistance_current;
  }
  return actual_omega / wheel_resistance_omega_threshold *
         dynamic_resistance_current;
}

static void ForceToCurrentConversion(void) {
  static const uint8_t RF = 0U;
  const float force_to_current =
      chassis_force_ctx.config.wheel_radius / chassis_force_ctx.config.torque_constant;

  for (int i = 0; i < 4; i++) {
    float base_current = chassis_force_ctx.wheel_force[i] * force_to_current;
    float actual_omega_raw =
        chassis_force_ctx.motors[i]->measure.speed_aps * DEGREE_2_RAD;
    float actual_omega =
        ApplyMotorDirectionToFeedback(chassis_force_ctx.motors[i], actual_omega_raw);
    float speed_feedback = CalculateSpeedFeedback(
        chassis_force_ctx.target_wheel_omega[i], actual_omega);
    float friction_comp = CalculateFrictionCompensation(
        chassis_force_ctx.target_wheel_omega[i], actual_omega, (uint8_t)i);

    chassis_force_ctx.wheel_current[i] =
        base_current + speed_feedback + friction_comp;
    chassis_force_ctx.wheel_current[i] =
        float_constrain(chassis_force_ctx.wheel_current[i],
                        -chassis_force_ctx.config.max_wheel_current,
                        chassis_force_ctx.config.max_wheel_current);

    (void)RF;
  }
}

void ChassisForceControlInit(const ChassisForceControlConfig_s *config,
                             DJIMotorInstance *motor_rf,
                             DJIMotorInstance *motor_lf,
                             DJIMotorInstance *motor_lb,
                             DJIMotorInstance *motor_rb) {
  memset(&chassis_force_ctx, 0, sizeof(chassis_force_ctx));
  chassis_force_ctx.config = *config;
  chassis_force_ctx.motors[0] = motor_rf;
  chassis_force_ctx.motors[1] = motor_lf;
  chassis_force_ctx.motors[2] = motor_lb;
  chassis_force_ctx.motors[3] = motor_rb;
  chassis_force_ctx.wheel_center[0] =
      fabsf(config->half_track_width - config->center_gimbal_offset_x) +
      fabsf(config->half_wheel_base - config->center_gimbal_offset_y);
  chassis_force_ctx.wheel_center[1] =
      fabsf(config->half_track_width + config->center_gimbal_offset_x) +
      fabsf(config->half_wheel_base - config->center_gimbal_offset_y);
  chassis_force_ctx.wheel_center[2] =
      fabsf(config->half_track_width + config->center_gimbal_offset_x) +
      fabsf(config->half_wheel_base + config->center_gimbal_offset_y);
  chassis_force_ctx.wheel_center[3] =
      fabsf(config->half_track_width - config->center_gimbal_offset_x) +
      fabsf(config->half_wheel_base + config->center_gimbal_offset_y);
  PID_Init_Config_s force_x_pid_config = config->force_x_pid_config;
  PID_Init_Config_s force_y_pid_config = config->force_y_pid_config;
  PID_Init_Config_s torque_pid_config = config->torque_pid_config;

  PIDInit(&chassis_force_ctx.force_x_pid, &force_x_pid_config);
  PIDInit(&chassis_force_ctx.force_y_pid, &force_y_pid_config);
  PIDInit(&chassis_force_ctx.torque_pid, &torque_pid_config);

  for (int i = 0; i < 4; i++) {
    wheel_dynamic_resistance_current[i] =
        config->wheel_dynamic_current_default;
  }
  wheel_resistance_omega_threshold =
      config->wheel_resistance_omega_threshold_default;
  wheel_speed_feedback_gain = config->wheel_speed_feedback_gain_default;

  chassis_force_ctx.initialized = 1U;
}

void ChassisForceControlReset(void) {
  ResetPIDRuntimeState(&chassis_force_ctx.force_x_pid);
  ResetPIDRuntimeState(&chassis_force_ctx.force_y_pid);
  ResetPIDRuntimeState(&chassis_force_ctx.torque_pid);
  chassis_force_ctx.filtered_vx = 0.0f;
  chassis_force_ctx.filtered_vy = 0.0f;
  chassis_force_ctx.filtered_wz = 0.0f;
  chassis_force_ctx.estimated_vx = 0.0f;
  chassis_force_ctx.estimated_vy = 0.0f;
  chassis_force_ctx.estimated_wz = 0.0f;
  chassis_force_ctx.projected_vx = 0.0f;
  chassis_force_ctx.projected_vy = 0.0f;
  chassis_force_ctx.target_wz = 0.0f;
  chassis_force_ctx.force_x = 0.0f;
  chassis_force_ctx.force_y = 0.0f;
  chassis_force_ctx.torque_z = 0.0f;
  memset(chassis_force_ctx.wheel_force, 0, sizeof(chassis_force_ctx.wheel_force));
  memset(chassis_force_ctx.wheel_current, 0,
         sizeof(chassis_force_ctx.wheel_current));
  memset(chassis_force_ctx.target_wheel_omega, 0,
         sizeof(chassis_force_ctx.target_wheel_omega));
}

void ChassisForceControlSetCommand(float projected_vx, float projected_vy,
                                   float target_wz) {
  chassis_force_ctx.projected_vx = projected_vx;
  chassis_force_ctx.projected_vy = projected_vy;
  chassis_force_ctx.target_wz = target_wz;
}

void ChassisForceControlStep(void) {
  if (!chassis_force_ctx.initialized) {
    return;
  }

  MecanumKinematicsCalculate();
  VelocityToForceControl();
  ForceDynamicsInverseResolution();
  ForceToCurrentConversion();
}

const float *ChassisForceControlGetWheelCurrent(void) {
  return chassis_force_ctx.wheel_current;
}

const float *ChassisForceControlGetTargetWheelOmega(void) {
  return chassis_force_ctx.target_wheel_omega;
}
