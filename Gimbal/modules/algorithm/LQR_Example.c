/**
 * @file LQR_Example.c
 * @brief LQR控制器使用示例代码
 * @note  本文件展示如何在云台应用中使用LQR控制器
 *        参考Mas2025云台Yaw轴实现
 * @date 2025-11-03
 */

#include "controller.h"
#include "dji_motor.h"
#include "ins_task.h"

/* ==================== 示例1：云台Yaw轴LQR控制（纯LQR） ==================== */

/**
 * @brief Yaw轴LQR控制示例（不带积分）
 * @note  参考 Mas2025 Gimbal.c line 166-214
 *        增益来自MATLAB设计: Q=diag([1000,1]), R=1
 */
void Example_GimbalYawLQR(void) {
  // 定义LQR实例（静态变量，保持状态）
  static LQRInstance yaw_lqr;
  static uint8_t lqr_initialized = 0;

  // 首次调用时初始化
  if (!lqr_initialized) {
    LQR_Init_Config_s config = {
        // LQR增益（从MATLAB离线计算）
        .K_angle = 31.6228f,   // [A/rad]
        .K_velocity = 3.3113f, // [A·s/rad]
        .K_integral = 0.0f,    // 不使用积分
        .max_out = 15.0f,      // 最大电流限制 [A]

        // 积分参数（此例不启用）
        .enable_integral = 0,
        .integral_limit = 0.0f,
        .integral_deadband = 0.0f,
        .integral_decay_coef = 0.0f,
    };

    LQRInit(&yaw_lqr, &config);
    lqr_initialized = 1;
  }

  // ===== 获取状态反馈 =====
  // 假设已有IMU数据指针: attitude_t *imu_data
  extern attitude_t *gimbal_imu_data; // IMU数据

  float current_angle = gimbal_imu_data->YawTotalAngle; // 当前角度 [rad]

  // 注意：陀螺仪方向可能与电机相反，需要根据实际情况决定是否取反
  float current_velocity = -gimbal_imu_data->Gyro[2]; // 角速度 [rad/s]

  // ===== 设置目标角度 =====
  static float target_angle = 0.0f; // [rad]

  // 从遥控器或其他源更新目标角度
  // target_angle += remote_input * dt;

  // ===== LQR计算 =====
  float current_cmd =
      LQRCalculate(&yaw_lqr, current_angle, current_velocity, target_angle);

  // current_cmd 单位: [A] (安培)

  // ===== 转换为CAN指令 =====
  // GM6020: CAN值 = 电流(A) × (16384 / 20)
  int16_t can_value = (int16_t)(current_cmd * 819.2f);

  // 确保在范围内
  if (can_value > 16384)
    can_value = 16384;
  if (can_value < -16384)
    can_value = -16384;

  // ===== 发送CAN指令 =====
  // 根据电机安装方向，可能需要取反
  // GM6020_CAN2SetLIDCurrent(-can_value, 0, 0, 0);
}

/* ==================== 示例2：云台Pitch轴LQR+I控制 ==================== */

/**
 * @brief Pitch轴LQR+积分控制示例
 * @note  参考 Mas2025 Gimbal.c line 72-157
 *        带变增益积分，用于消除稳态误差（如重力干扰）
 */
void Example_GimbalPitchLQR(void) {
  static LQRInstance pitch_lqr;
  static uint8_t lqr_initialized = 0;

  if (!lqr_initialized) {
    LQR_Init_Config_s config = {
        // LQR增益
        .K_angle = 40.0f,      // [A/rad]
        .K_velocity = 2.3802f, // [A·s/rad]
        .K_integral = 0.15f,   // 积分增益 [A/(rad·s)]
        .max_out = 18.0f,      // 最大电流 [A]

        // 积分优化（参考Mas2025实现）
        .enable_integral = 1,        // 启用积分消除稳态误差
        .integral_limit = 2.0f,      // 积分限幅
        .integral_deadband = 0.01f,  // 死区 0.57度
        .integral_decay_coef = 0.5f, // 大误差时衰减
    };

    LQRInit(&pitch_lqr, &config);
    lqr_initialized = 1;
  }

  extern attitude_t *gimbal_imu_data;

  // 获取反馈（注意：可能用电机编码器或IMU，根据实际选择）
  float current_angle = gimbal_imu_data->Pitch;       // [rad]
  float current_velocity = -gimbal_imu_data->Gyro[0]; // [rad/s]

  static float target_angle = 0.0f; // [rad]

  // LQR+积分计算
  float current_cmd =
      LQRCalculate(&pitch_lqr, current_angle, current_velocity, target_angle);

  // 转换并发送...
}

/* ==================== 示例3：在电机控制中集成LQR ==================== */

/**
 * @brief 在DJI电机控制器中使用LQR
 * @note  展示如何替代PID使用LQR
 */
void Example_MotorWithLQR(void) {
  // 假设有一个电机实例
  extern DJIMotorInstance *yaw_motor;
  extern LQRInstance yaw_motor_lqr;

  // 获取当前状态
  float angle = yaw_motor->measure.total_angle * DEG_TO_RAD;  // 转为rad
  float velocity = yaw_motor->measure.speed_aps * DEG_TO_RAD; // 转为rad/s

  // 获取目标值（从上层应用）
  float target = yaw_motor->motor_controller.pid_ref * DEG_TO_RAD;

  // LQR计算
  float current_A = LQRCalculate(&yaw_motor_lqr, angle, velocity, target);

  // 转换为CAN指令
  int16_t can_cmd = (int16_t)(current_A * 819.2f);

  // 这里通常会由电机模块统一发送
  // 或者直接设置到电机的输出缓冲区
}

/* ==================== 示例4：模式切换 ==================== */

/**
 * @brief 在PID和LQR之间切换
 */
void Example_SwitchControl(void) {
  static enum {
    MODE_PID,
    MODE_LQR,
  } control_mode = MODE_PID;

  static PIDInstance yaw_pid;
  static LQRInstance yaw_lqr;

  extern attitude_t *imu_data;
  float angle = imu_data->YawTotalAngle;
  float velocity = -imu_data->Gyro[2];
  float target = 0.0f;

  float output_current = 0.0f;

  switch (control_mode) {
  case MODE_PID:
    // 使用传统PID（串级或单环）
    output_current = PIDCalculate(&yaw_pid, angle, target);
    break;

  case MODE_LQR:
    // 使用LQR控制
    output_current = LQRCalculate(&yaw_lqr, angle, velocity, target);
    break;
  }

  // 模式切换时重置
  if (/* 检测到模式切换 */) {
    if (control_mode == MODE_LQR) {
      LQRReset(&yaw_lqr);
    }
  }
}

/* ==================== 调试辅助函数 ==================== */

/**
 * @brief 获取LQR内部状态（用于调试和监控）
 * @note  在Ozone中监控这些变量以分析控制性能
 */
typedef struct {
  float angle_error;    // 角度误差 [rad]
  float velocity;       // 角速度 [rad/s]
  float integral;       // 积分项
  float output_current; // 输出电流 [A]
  float dt;             // 计算周期 [s]
} LQR_Debug_Info_t;

void GetLQRDebugInfo(LQRInstance *lqr, LQR_Debug_Info_t *debug_info) {
  debug_info->angle_error = lqr->angle_error;
  debug_info->velocity = lqr->measure_vel;
  debug_info->integral = lqr->integral;
  debug_info->output_current = lqr->output;
  debug_info->dt = lqr->dt;
}
