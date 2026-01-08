/**
 * @file power_controller.c
 * @author Refactored from chassis.c
 * @brief 功率控制器核心实现
 */

#include "power_controller.h"
#include "arm_math.h"
#include "user_lib.h"
#include <math.h>
#include <stdlib.h>

/* ======================== 内部变量 ======================== */

// RLS参数辨识实例
static RLSInstance power_rls;

// 功率模型参数（动态更新）
static float power_k1 = 0.22f;
static float power_k2 = 1.2f;
static float power_k3 = 5.2f;

// 控制器配置
static PowerControllerConfig_t controller_config;

// 裁判系统数据
static struct {
  float power_limit;
  float power_buffer;
  float chassis_power;
} referee_data = {80.0f, 60.0f, 0.0f};

// 超级电容数据
static struct {
  uint8_t voltage;
  uint8_t online;
  float energy; // 估算能量 (J)
} cap_data = {0, 0, 0.0f};

// 电机反馈数据
static struct {
  float speeds[4];  // rad/s
  float torques[4]; // Nm
} motor_feedback;

// PD控制器状态（base和full各自独立）
static float power_pd_last_error_full = 0.0f;
static float power_pd_last_error_base = 0.0f;

// 功率限制状态
static struct {
  float max_power;
  float power_upper;
  float power_lower;
} power_limit_state = {100.0f, 80.0f, 15.0f};

// 控制器状态（供外部查询）
static PowerControllerStatus_t controller_status = {
    .k1 = 0.22f,
    .k2 = 1.2f,
    .max_power_limit = 100.0f,
    .rls_enabled = 1,
};

/* ======================== 内部函数 ======================== */

/**
 * @brief 能量环控制：计算功率上下限
 */
static void EnergyLoopControl(void) {
  float max_power = referee_data.power_limit;
  float energy_feedback = referee_data.power_buffer;
  float full_buff_set = REFEREE_FULL_BUFF;
  float base_buff_set = REFEREE_BASE_BUFF;

  // 优先使用超级电容数据
  if (cap_data.online) {
    // 电容能量百分比：直接使用0-255范围值，与PD控制器阈值单位一致
    energy_feedback = (float)cap_data.voltage;
    full_buff_set = CAP_FULL_BUFF;
    base_buff_set = CAP_BASE_BUFF;
    // 有电容时可以增加功率上限
    max_power = referee_data.power_limit + MAX_CAP_POWER_OUT;
  }

  // PD控制器调节功率限制
  // 使用sqrt变换使能量较低时更敏感
  float error_full = sqrtf(full_buff_set) - sqrtf(energy_feedback);
  float error_base = sqrtf(base_buff_set) - sqrtf(energy_feedback);

  // base和full各自独立的PD控制器
  float pd_output_full = POWER_PD_KP * error_full +
                         POWER_PD_KD * (error_full - power_pd_last_error_full);
  float pd_output_base = POWER_PD_KP * error_base +
                         POWER_PD_KD * (error_base - power_pd_last_error_base);

  power_pd_last_error_full = error_full;
  power_pd_last_error_base = error_base;

  // 计算功率上下限
  power_limit_state.power_upper =
      fmaxf(referee_data.power_limit - pd_output_full, 15.0f);
  power_limit_state.power_lower =
      fmaxf(referee_data.power_limit - pd_output_base, 15.0f);

  // 限制在上下限之间
  if (max_power > power_limit_state.power_upper)
    max_power = power_limit_state.power_upper;
  if (max_power < power_limit_state.power_lower)
    max_power = power_limit_state.power_lower;

  power_limit_state.max_power = max_power;

  // 更新状态
  controller_status.max_power_limit = max_power;
  controller_status.power_upper = power_limit_state.power_upper;
  controller_status.power_lower = power_limit_state.power_lower;
  controller_status.energy_feedback = energy_feedback;
  controller_status.cap_online = cap_data.online;
}

/**
 * @brief RLS参数更新（内部函数，避免与controller.h中的RLSUpdate冲突）
 */
static void PowerRLSUpdate(void) {
  if (!controller_status.rls_enabled) {
    return;
  }

  float measured_power = referee_data.chassis_power;

  // 只在功率大于阈值时更新，避免噪声
  if (fabsf(measured_power) < 5.0f) {
    return;
  }

  float sample_vector[2] = {0.0f, 0.0f};
  float effective_power = 0.0f;

  // 计算采样向量 [Σ|ω|, Στ²]
  for (int i = 0; i < 4; i++) {
    sample_vector[0] += fabsf(motor_feedback.speeds[i]);
    sample_vector[1] += motor_feedback.torques[i] * motor_feedback.torques[i];
    effective_power += motor_feedback.torques[i] * motor_feedback.speeds[i];
  }

  // 功率损耗 = 实测功率 - 有效功率 - 静态损耗
  float power_loss = measured_power - effective_power - power_k3;

  // RLS更新
  RLSUpdate(&power_rls, sample_vector, power_loss);

  // 获取更新后的参数并限幅，防止发散
  RLSGetParams(&power_rls, &power_k1, &power_k2);

  // 参数限幅（只做下限保护，防止发散到负数）
  power_k1 = fmaxf(power_k1, 1e-5f);
  power_k2 = fmaxf(power_k2, 1e-5f);

  // 更新状态
  controller_status.k1 = power_k1;
  controller_status.k2 = power_k2;
}

/**
 * @brief 预测功率消耗
 */
static float PredictPower(float torque, float speed) {
  return torque * speed + power_k1 * fabsf(speed) + power_k2 * torque * torque +
         power_k3 / 4.0f;
}

/**
 * @brief 计算二次方程最大转矩
 */
static float SolveMaxTorque(float speed, float power_allocated,
                            float current_torque) {
  // 求解二次方程: k2*τ² + ω*τ + (k1|ω| + k3/4 - P) = 0
  float A = power_k2;
  float B = speed;
  float C = power_k1 * fabsf(speed) + power_k3 / 4.0f - power_allocated;

  float delta = B * B - 4.0f * A * C;
  float max_torque = 0.0f;

  if (delta <= 0.0f) {
    // 无实数解或重根，取顶点
    max_torque = -B / (2.0f * A);
  } else {
    // 两个实数解，选择与当前转矩符号一致的解
    float sqrt_delta = sqrtf(delta);
    float torque_pos = (-B + sqrt_delta) / (2.0f * A);
    float torque_neg = (-B - sqrt_delta) / (2.0f * A);

    if (current_torque >= 0.0f) {
      max_torque = torque_pos;
    } else {
      max_torque = torque_neg;
    }
  }

  return max_torque;
}

/* ======================== 接口函数实现 ======================== */

void PowerControllerInit(const PowerControllerConfig_t *config) {
  // 保存配置
  controller_config = *config;

  // 初始化功率模型参数
  power_k1 = config->k1_init;
  power_k2 = config->k2_init;
  power_k3 = config->k3;

  // RLS初始化
  RLS_Init_Config_s rls_config = {
      .lambda = config->rls_lambda,
      .delta =
          1e-5f, // 协方差矩阵初始值，推荐1e-5~1e-3，过大会导致首次更新参数剧变
      .init_k1 = config->k1_init,
      .init_k2 = config->k2_init,
  };
  RLSInit(&power_rls, &rls_config);

  // 初始化状态
  controller_status.k1 = power_k1;
  controller_status.k2 = power_k2;
  controller_status.rls_enabled = RLS_ENABLE;
}

void PowerControllerTask(void) {
  // 1. 能量环控制
  EnergyLoopControl();

  // 2. RLS参数更新
  PowerRLSUpdate();
}

void PowerGetLimitedOutput(PowerMotorObj_t motor_objs[4], float output[4]) {
  float max_power = power_limit_state.max_power;

  // 1. 预测功率消耗
  float cmd_power[4];
  float sum_cmd_power = 0.0f;
  float sum_positive_power = 0.0f;

  for (int i = 0; i < 4; i++) {
    // 转换PID输出为转矩
    float torque = motor_objs[i].pid_output * controller_config.current_scale *
                   controller_config.torque_constant;
    float speed = motor_objs[i].current_av;

    cmd_power[i] = PredictPower(torque, speed);
    sum_cmd_power += cmd_power[i];

    // 统计正功率
    if (cmd_power[i] > 0.0f) {
      sum_positive_power += cmd_power[i];
    } else {
      // 负功率回收
      max_power += (-cmd_power[i]);
    }
  }

  controller_status.sum_cmd_power = sum_cmd_power;
  controller_status.estimated_power = sum_cmd_power;

  // 2. 如果功率不超限，直接输出
  if (sum_positive_power <= max_power) {
    for (int i = 0; i < 4; i++) {
      output[i] = motor_objs[i].pid_output;
    }
    return;
  }

  // 3. 功率超限，进行智能分配
  float speed_error[4];
  float sum_error = 0.0f;

  for (int i = 0; i < 4; i++) {
    speed_error[i] = fabsf(motor_objs[i].target_av - motor_objs[i].current_av);
    sum_error += speed_error[i];
  }

  // 计算error分配的置信度
  float error_confidence = 0.0f;
  if (sum_error > ERROR_POWER_DISTRIBUTION_THRESHOLD) {
    error_confidence = 1.0f;
  } else if (sum_error > PROP_POWER_DISTRIBUTION_THRESHOLD) {
    error_confidence = (sum_error - PROP_POWER_DISTRIBUTION_THRESHOLD) /
                       (ERROR_POWER_DISTRIBUTION_THRESHOLD -
                        PROP_POWER_DISTRIBUTION_THRESHOLD);
  }

  // 为每个电机分配功率并计算限制
  for (int i = 0; i < 4; i++) {
    if (cmd_power[i] <= 0.0f) {
      output[i] = motor_objs[i].pid_output;
      continue;
    }

    // 混合分配策略
    float weight_error =
        (sum_error > 1e-6f) ? (speed_error[i] / sum_error) : 0.25f;
    float weight_prop = (sum_positive_power > 1e-6f)
                            ? (cmd_power[i] / sum_positive_power)
                            : 0.25f;
    float weight = error_confidence * weight_error +
                   (1.0f - error_confidence) * weight_prop;

    float power_allocated = weight * max_power;

    // 转换当前PID输出为转矩
    float current_torque = motor_objs[i].pid_output *
                           controller_config.current_scale *
                           controller_config.torque_constant;

    // 计算允许的最大转矩
    float max_torque = SolveMaxTorque(motor_objs[i].current_av, power_allocated,
                                      current_torque);

    // 转换回PID输出
    if (fabsf(current_torque) > 1e-6f) {
      float torque_scale = max_torque / current_torque;
      torque_scale = float_constrain(torque_scale, 0.0f, 1.0f);
      output[i] = motor_objs[i].pid_output * torque_scale;
    } else {
      output[i] = motor_objs[i].pid_output;
    }
  }
}

void PowerUpdateRefereeData(float chassis_power_limit,
                            float chassis_power_buffer, float chassis_power) {
  referee_data.power_limit = chassis_power_limit;
  referee_data.power_buffer = chassis_power_buffer;
  referee_data.chassis_power = chassis_power;
}

void PowerUpdateCapData(uint8_t cap_voltage, uint8_t cap_online) {
  cap_data.voltage = cap_voltage;
  cap_data.online = cap_online;
}

void PowerUpdateMotorFeedback(float motor_speeds[4], float motor_torques[4]) {
  for (int i = 0; i < 4; i++) {
    motor_feedback.speeds[i] = motor_speeds[i];
    motor_feedback.torques[i] = motor_torques[i];
  }
}

const PowerControllerStatus_t *PowerGetStatus(void) {
  return &controller_status;
}

void PowerSetRLSEnable(uint8_t enable) {
  controller_status.rls_enabled = enable;
}

void PowerSetUserLimit(float power_limit) {
  referee_data.power_limit = power_limit;
}
