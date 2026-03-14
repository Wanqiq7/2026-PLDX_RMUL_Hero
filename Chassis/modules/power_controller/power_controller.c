/**
 * @file power_controller.c
 * @author Refactored from chassis.c
 * @brief 功率控制器核心实现
 */

#include "power_controller.h"
#include "arm_math_compat.h"
#include "user_lib.h"
#include <math.h>
#include <stdlib.h>

/* ======================== 内部变量 ======================== */

// 功率限制查表（参考港科大实现，RM2024规则）
static const uint8_t HeroChassisPowerLimit[MAX_ROBOT_LEVEL] = {
    95U, 60U, 65U, 70U, 75U, 80U, 85U, 90U, 100U, 120U};
static const uint8_t InfantryChassisPowerLimit[MAX_ROBOT_LEVEL] = {
    45U, 50U, 55U, 60U, 65U, 70U, 75U, 80U, 90U, 100U};
static const uint8_t SentryChassisPowerLimit = 100U;

/**
 * @brief 功率控制器实例结构体（内部实现）
 */
struct PowerControllerInstance {
  // 配置参数
  PowerControllerConfig_t config;

  // RLS参数辨识
  RLSInstance rls;
  float k1, k2, k3;

  // 裁判系统数据
  struct {
    float limit_w;
    float buffer_energy;
    float power_w;
    uint8_t online;
    uint8_t robot_level;
  } referee;

  // 超级电容数据
  struct {
    uint8_t energy_percent;
    uint8_t online;
  } cap;

  // 电机反馈
  struct {
    float speeds[4];
    float torques[4];
    uint8_t online[4];
    uint16_t disconnect_cnt[4];
  } motor;

  // PD控制器状态
  float pd_last_error_full;
  float pd_last_error_base;
  float pd_d_error_full;
  float pd_d_error_base;

  // 功率限制状态
  struct {
    float max_power;
    float power_upper;
    float power_lower;
  } limit;

  // 错误标志
  uint8_t error_flags;
  uint8_t last_robot_level;

  // 状态（供外部查询）
  PowerControllerStatus_t status;
};

// 实例存储（单实例模式）
static PowerControllerInstance power_ctrl_instance;
static uint8_t instance_initialized = 0;

/* ======================== 内部函数 ======================== */

/**
 * @brief 更新功率统计量（基于电机反馈）
 * @param inst 功率控制器实例
 */
static void UpdatePowerStatistics(PowerControllerInstance *inst) {
  float sum_abs_speed = 0.0f;
  float sum_torque_sq = 0.0f;
  float mech_power_w = 0.0f;

  for (int i = 0; i < 4; i++) {
    if (inst->motor.disconnect_cnt[i] >= MOTOR_DISCONNECT_TIMEOUT) {
      continue;
    }
    sum_abs_speed += fabsf(inst->motor.speeds[i]);
    sum_torque_sq += inst->motor.torques[i] * inst->motor.torques[i];
    mech_power_w += inst->motor.torques[i] * inst->motor.speeds[i];
  }

  inst->status.mech_power_w = mech_power_w;
  inst->status.est_power_w =
      mech_power_w + inst->k1 * sum_abs_speed + inst->k2 * sum_torque_sq +
      inst->k3;
  inst->status.measured_power_w = inst->referee.power_w;
  inst->status.loss_power_w = inst->status.measured_power_w - mech_power_w;

  if (inst->cap.online) {
    inst->status.cap_energy_est = (float)inst->cap.energy_percent;
  } else {
    // 无电容反馈时，用裁判缓冲做等效估计（0~255）
    float buffer_ratio = inst->referee.buffer_energy / REFEREE_FULL_BUFF;
    inst->status.cap_energy_est =
        float_constrain(buffer_ratio * 255.0f, 0.0f, 255.0f);
  }
}

/**
 * @brief 更新错误标志
 * @param inst 功率控制器实例
 */
static void UpdateErrorFlags(PowerControllerInstance *inst) {
  inst->error_flags = POWER_ERROR_NONE;

  // 检查电容状态
  if (!inst->cap.online) {
    inst->error_flags |= POWER_ERROR_CAP_DISCONNECT;
  }

  // 检查裁判系统状态
  if (!inst->referee.online) {
    inst->error_flags |= POWER_ERROR_REFEREE_DISCONNECT;
  }

  // 检查电机状态
  for (int i = 0; i < 4; i++) {
    if (!inst->motor.online[i]) {
      inst->error_flags |= POWER_ERROR_MOTOR_DISCONNECT;
      break;
    }
  }

  inst->status.error_flags = inst->error_flags;
}

/**
 * @brief 根据机器人类型和等级获取功率限制
 */
static float GetPowerLimitByLevel(RobotDivision_e division, uint8_t level) {
  if (level < 1)
    level = 1;
  if (level > MAX_ROBOT_LEVEL)
    level = MAX_ROBOT_LEVEL;

  switch (division) {
  case ROBOT_HERO:
    return (float)HeroChassisPowerLimit[level - 1];
  case ROBOT_INFANTRY:
    return (float)InfantryChassisPowerLimit[level - 1];
  case ROBOT_SENTRY:
    return (float)SentryChassisPowerLimit;
  default:
    return (float)InfantryChassisPowerLimit[level - 1];
  }
}

/**
 * @brief 能量环控制：计算功率上下限
 * @param inst 功率控制器实例
 */
static void EnergyLoopControl(PowerControllerInstance *inst) {
  // 更新错误标志
  UpdateErrorFlags(inst);

  float ref_limit_w = inst->referee.limit_w;
  float hard_limit_w = ref_limit_w;
  float buffer_feedback = inst->referee.buffer_energy;
  float full_buffer_target = REFEREE_FULL_BUFF;
  float base_buffer_target = REFEREE_BASE_BUFF;

  // 裁判系统断连时，按等级查表获取功率限制
  if (inst->error_flags & POWER_ERROR_REFEREE_DISCONNECT) {
    ref_limit_w = GetPowerLimitByLevel(inst->config.robot_division,
                                       inst->last_robot_level);
  } else {
    // 更新最后记录的等级
    if (inst->referee.robot_level >= 1 &&
        inst->referee.robot_level <= MAX_ROBOT_LEVEL) {
      inst->last_robot_level = inst->referee.robot_level;
    }
  }

  // 保留电容在线时的额外功率上限能力，但缓冲反馈优先使用裁判系统
  if (inst->cap.online) {
    hard_limit_w = ref_limit_w + MAX_CAP_POWER_OUT;
  } else {
    hard_limit_w = ref_limit_w;
  }

  // 仅在裁判系统断连时，才回退使用超级电容能量作为缓冲反馈
  if ((inst->error_flags & POWER_ERROR_REFEREE_DISCONNECT) && inst->cap.online) {
    buffer_feedback = (float)inst->cap.energy_percent;
    full_buffer_target = CAP_FULL_BUFF;
    base_buffer_target = CAP_BASE_BUFF;
  }

  // 避免 sqrt 输入为负数导致 NaN
  buffer_feedback = fmaxf(buffer_feedback, 0.0f);

  // PD 控制器调节功率限制
  float error_full = sqrtf(full_buffer_target) - sqrtf(buffer_feedback);
  float error_base = sqrtf(base_buffer_target) - sqrtf(buffer_feedback);

  // base 和 full 各自独立的 PD 控制器（微分项做低通抑制噪声）
  float d_error_full = error_full - inst->pd_last_error_full;
  float d_error_base = error_base - inst->pd_last_error_base;
  inst->pd_d_error_full = LowPassFilter_Float(
      d_error_full, POWER_PD_D_FILTER_ALPHA, &inst->pd_d_error_full);
  inst->pd_d_error_base = LowPassFilter_Float(
      d_error_base, POWER_PD_D_FILTER_ALPHA, &inst->pd_d_error_base);
  float pd_output_full =
      POWER_PD_KP * error_full + POWER_PD_KD * inst->pd_d_error_full;
  float pd_output_base =
      POWER_PD_KP * error_base + POWER_PD_KD * inst->pd_d_error_base;

  inst->pd_last_error_full = error_full;
  inst->pd_last_error_base = error_base;

  // 计算功率上下限
  float allowed_power_w = hard_limit_w;
  inst->limit.power_upper = float_constrain(ref_limit_w - pd_output_full,
                                            MIN_POWER_CONFIGURED,
                                            hard_limit_w);
  inst->limit.power_lower = float_constrain(ref_limit_w - pd_output_base,
                                            MIN_POWER_CONFIGURED,
                                            hard_limit_w);

  // 防止上下限交叉导致异常功率跳变
  if (inst->limit.power_lower > inst->limit.power_upper) {
    inst->limit.power_lower = inst->limit.power_upper;
  }

  // 双断连保守策略
  uint8_t cap_gg = (inst->error_flags & POWER_ERROR_CAP_DISCONNECT);
  uint8_t ref_gg = (inst->error_flags & POWER_ERROR_REFEREE_DISCONNECT);

  if (cap_gg && ref_gg) {
    allowed_power_w = ref_limit_w * CAP_REFEREE_BOTH_GG_COE;
    allowed_power_w =
        float_constrain(allowed_power_w, MIN_POWER_CONFIGURED, hard_limit_w);
    inst->limit.power_upper = allowed_power_w;
    inst->limit.power_lower = allowed_power_w;
    inst->pd_last_error_full = 0.0f;
    inst->pd_last_error_base = 0.0f;
    inst->pd_d_error_full = 0.0f;
    inst->pd_d_error_base = 0.0f;
  } else {
    allowed_power_w = float_constrain(allowed_power_w, inst->limit.power_lower,
                                      inst->limit.power_upper);
  }

  // 最终安全钳位：任何路径都不允许超过绝对上限
  allowed_power_w = float_constrain(allowed_power_w, MIN_POWER_CONFIGURED,
                                    hard_limit_w);

  inst->limit.max_power = allowed_power_w;

  // 更新状态
  inst->status.allowed_power_w = allowed_power_w;
  inst->status.upper_limit_w = inst->limit.power_upper;
  inst->status.lower_limit_w = inst->limit.power_lower;
  inst->status.ref_limit_w = ref_limit_w;
  inst->status.hard_limit_w = hard_limit_w;
  inst->status.buffer_feedback = buffer_feedback;
  inst->status.cap_online = inst->cap.online;
  inst->status.referee_online = inst->referee.online;
  inst->status.robot_level = inst->last_robot_level;
}

/**
 * @brief RLS参数更新
 * @param inst 功率控制器实例
 */
static void PowerRLSUpdate(PowerControllerInstance *inst) {
  if (!inst->status.rls_enabled) {
    return;
  }

  float feedback_power_w = inst->referee.power_w;

  // 只在功率大于阈值时更新，避免噪声
  if (fabsf(feedback_power_w) < 5.0f) {
    return;
  }

  float sample_vector[2] = {0.0f, 0.0f};
  float mech_power_w = 0.0f;

  // 计算采样向量 [Σ|ω|, Στ²]
  for (int i = 0; i < 4; i++) {
    sample_vector[0] += fabsf(inst->motor.speeds[i]);
    sample_vector[1] += inst->motor.torques[i] * inst->motor.torques[i];
    mech_power_w += inst->motor.torques[i] * inst->motor.speeds[i];
  }

  // 功率损耗 = 实测功率 - 有效功率 - 静态损耗
  float power_loss = feedback_power_w - mech_power_w - inst->k3;

  // RLS 更新
  RLSUpdate(&inst->rls, sample_vector, power_loss);

  // 获取更新后的参数并限幅
  RLSGetParams(&inst->rls, &inst->k1, &inst->k2);

  // 参数限幅（防止发散到负数）
  inst->k1 = fmaxf(inst->k1, 1e-5f);
  inst->k2 = fmaxf(inst->k2, 1e-5f);

  // 更新状态
  inst->status.k1 = inst->k1;
  inst->status.k2 = inst->k2;
}

/**
 * @brief 预测功率消耗
 * @param inst 功率控制器实例
 */
static float PredictPower(PowerControllerInstance *inst, float torque,
                          float speed) {
  return torque * speed + inst->k1 * fabsf(speed) + inst->k2 * torque * torque +
         inst->k3 / 4.0f;
}

/**
 * @brief 计算二次方程最大转矩
 * @param inst 功率控制器实例
 */
static float SolveMaxTorque(PowerControllerInstance *inst, float speed,
                            float power_allocated, float current_torque) {
  // 求解二次方程: k2*τ² + ω*τ + (k1|ω| + k3/4 - P) = 0
  float A = inst->k2;
  float B = speed;
  float C = inst->k1 * fabsf(speed) + inst->k3 / 4.0f - power_allocated;

  float delta = B * B - 4.0f * A * C;
  float max_torque = 0.0f;

  if (delta <= 0.0f) {
    max_torque = -B / (2.0f * A);
  } else {
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

PowerControllerInstance *
PowerControllerRegister(const PowerControllerConfig_t *config) {
  // 检查是否已初始化（单实例模式）
  if (instance_initialized) {
    return &power_ctrl_instance;
  }

  PowerControllerInstance *inst = &power_ctrl_instance;

  // 保存配置
  inst->config = *config;

  // 初始化功率模型参数
  inst->k1 = config->k1_init;
  inst->k2 = config->k2_init;
  inst->k3 = config->k3;

  // RLS初始化
  RLS_Init_Config_s rls_config = {
      .lambda = config->rls_lambda,
      .delta = 1e-5f,
      .init_k1 = config->k1_init,
      .init_k2 = config->k2_init,
  };
  RLSInit(&inst->rls, &rls_config);

  // 初始化默认值
  inst->referee.limit_w = 80.0f;
  inst->referee.buffer_energy = 60.0f;
  inst->referee.robot_level = 1;
  inst->last_robot_level = 1;
  inst->limit.max_power = 100.0f;
  inst->limit.power_upper = 80.0f;
  inst->limit.power_lower = 15.0f;
  inst->pd_last_error_full = 0.0f;
  inst->pd_last_error_base = 0.0f;
  inst->pd_d_error_full = 0.0f;
  inst->pd_d_error_base = 0.0f;

  for (int i = 0; i < 4; i++) {
    inst->motor.online[i] = 1;
  }

  // 初始化状态
  inst->status.k1 = inst->k1;
  inst->status.k2 = inst->k2;
  inst->status.allowed_power_w = 100.0f;
  inst->status.rls_enabled = RLS_ENABLE;

  instance_initialized = 1;
  return inst;
}

void PowerControllerTask(PowerControllerInstance *instance) {
  if (instance == NULL)
    return;

  // 1. 能量环控制
  EnergyLoopControl(instance);

  // 2. RLS参数更新
  PowerRLSUpdate(instance);

  // 3. 更新对外状态量（用于Ozone/上位机观测）
  UpdatePowerStatistics(instance);
}

void PowerGetLimitedOutput(PowerControllerInstance *instance,
                           PowerMotorObj_t motor_objs[4], float output[4]) {
  if (instance == NULL)
    return;

  float max_power = instance->limit.max_power;

  // 1. 预测功率消耗
  float cmd_power[4];
  float sum_cmd_power = 0.0f;
  float sum_positive_power = 0.0f;

  for (int i = 0; i < 4; i++) {
    if (instance->motor.disconnect_cnt[i] >= MOTOR_DISCONNECT_TIMEOUT) {
      cmd_power[i] = 0.0f;
      output[i] = 0.0f;
      continue;
    }

    float torque = motor_objs[i].pid_output * instance->config.current_scale *
                   instance->config.torque_constant;
    float speed = motor_objs[i].current_av;

    cmd_power[i] = PredictPower(instance, torque, speed);
    sum_cmd_power += cmd_power[i];

    if (cmd_power[i] > 0.0f) {
      sum_positive_power += cmd_power[i];
    } else {
      max_power += (-cmd_power[i]);
    }
  }

  instance->status.cmd_power_sum_w = sum_cmd_power;

  // 2. 功率不超限，直接输出
  if (sum_positive_power <= max_power) {
    for (int i = 0; i < 4; i++) {
      output[i] = motor_objs[i].pid_output;
    }
    return;
  }

  // 3. 功率超限，智能分配
  float speed_error[4];
  float sum_error = 0.0f;

  for (int i = 0; i < 4; i++) {
    speed_error[i] = fabsf(motor_objs[i].target_av - motor_objs[i].current_av);
    sum_error += speed_error[i];
  }

  float error_confidence = 0.0f;
  if (sum_error > ERROR_POWER_DISTRIBUTION_THRESHOLD) {
    error_confidence = 1.0f;
  } else if (sum_error > PROP_POWER_DISTRIBUTION_THRESHOLD) {
    error_confidence = (sum_error - PROP_POWER_DISTRIBUTION_THRESHOLD) /
                       (ERROR_POWER_DISTRIBUTION_THRESHOLD -
                        PROP_POWER_DISTRIBUTION_THRESHOLD);
  }

  for (int i = 0; i < 4; i++) {
    if (cmd_power[i] <= 0.0f) {
      output[i] = motor_objs[i].pid_output;
      continue;
    }

    float weight_error =
        (sum_error > 1e-6f) ? (speed_error[i] / sum_error) : 0.25f;
    float weight_prop = (sum_positive_power > 1e-6f)
                            ? (cmd_power[i] / sum_positive_power)
                            : 0.25f;
    float weight = error_confidence * weight_error +
                   (1.0f - error_confidence) * weight_prop;

    float power_allocated = weight * max_power;

    float current_torque = motor_objs[i].pid_output *
                           instance->config.current_scale *
                           instance->config.torque_constant;

    float max_torque = SolveMaxTorque(instance, motor_objs[i].current_av,
                                      power_allocated, current_torque);

    if (fabsf(current_torque) > 1e-6f) {
      float torque_scale = max_torque / current_torque;
      torque_scale = float_constrain(torque_scale, 0.0f, 1.0f);
      output[i] = motor_objs[i].pid_output * torque_scale;
    } else {
      output[i] = motor_objs[i].pid_output;
    }
  }
}

void PowerUpdateRefereeData(PowerControllerInstance *instance, float limit_w,
                            float buffer_energy, float power_w) {
  if (instance == NULL)
    return;
  instance->referee.limit_w = limit_w;
  instance->referee.buffer_energy = buffer_energy;
  instance->referee.power_w = power_w;
}

void PowerUpdateCapData(PowerControllerInstance *instance, uint8_t cap_energy,
                        uint8_t cap_online) {
  if (instance == NULL)
    return;
  instance->cap.energy_percent = cap_energy;
  instance->cap.online = cap_online;
}

void PowerUpdateMotorFeedback(PowerControllerInstance *instance,
                              float motor_speeds[4], float motor_torques[4]) {
  if (instance == NULL)
    return;
  for (int i = 0; i < 4; i++) {
    instance->motor.speeds[i] = motor_speeds[i];
    instance->motor.torques[i] = motor_torques[i];
  }
}

const PowerControllerStatus_t *
PowerGetStatus(PowerControllerInstance *instance) {
  if (instance == NULL)
    return NULL;
  return &instance->status;
}

void PowerSetRLSEnable(PowerControllerInstance *instance, uint8_t enable) {
  if (instance == NULL)
    return;
  instance->status.rls_enabled = enable;
}

void PowerSetUserLimit(PowerControllerInstance *instance, float limit_w) {
  if (instance == NULL)
    return;
  instance->referee.limit_w = limit_w;
}

void PowerUpdateRefereeOnline(PowerControllerInstance *instance, uint8_t online,
                              uint8_t robot_level) {
  if (instance == NULL)
    return;
  instance->referee.online = online;
  if (robot_level >= 1 && robot_level <= MAX_ROBOT_LEVEL) {
    instance->referee.robot_level = robot_level;
  }
}

void PowerUpdateMotorOnline(PowerControllerInstance *instance,
                            uint8_t motor_index, uint8_t online) {
  if (instance == NULL || motor_index >= 4)
    return;
  instance->motor.online[motor_index] = online;
  if (online) {
    instance->motor.disconnect_cnt[motor_index] = 0;
  } else {
    if (instance->motor.disconnect_cnt[motor_index] <
        MOTOR_DISCONNECT_TIMEOUT) {
      instance->motor.disconnect_cnt[motor_index]++;
    }
  }
}

uint8_t PowerGetErrorFlags(PowerControllerInstance *instance) {
  if (instance == NULL)
    return 0xFF;
  return instance->error_flags;
}
