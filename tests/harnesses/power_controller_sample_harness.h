#ifndef POWER_CONTROLLER_SAMPLE_HARNESS_H
#define POWER_CONTROLLER_SAMPLE_HARNESS_H

#include "power_controller.h"
#include <stdint.h>

/**
 * @brief 单步样本输入。
 * @note 该结构体只描述样本契约，优先对齐 Phase 4 原生 tau 域 public API。
 */
typedef struct {
  PowerControllerConfig_t config;
  float referee_limit_w;
  float referee_buffer_energy;
  float referee_power_w;
  uint8_t referee_online;
  uint8_t robot_level;
  uint8_t cap_energy_percent;
  uint8_t cap_online;
  float motor_speeds[4];
  float motor_torques[4];
  uint8_t motor_online[4];
  uint16_t motor_disconnect_cycles[4];
  uint8_t rls_enabled;
  uint8_t run_task_iterations;
  PowerWheelObj_t wheel_objs[4];
} PowerControllerSampleInput;

/**
 * @brief 多步样本步骤。
 * @note 用于 RLS/状态演化类黄金回归。
 */
typedef struct {
  float referee_limit_w;
  float referee_buffer_energy;
  float referee_power_w;
  uint8_t referee_online;
  uint8_t robot_level;
  uint8_t cap_energy_percent;
  uint8_t cap_online;
  float motor_speeds[4];
  float motor_torques[4];
  uint8_t motor_online[4];
  uint16_t motor_disconnect_cycles[4];
  uint8_t rls_enabled;
  uint8_t run_task_iterations;
} PowerControllerSampleStep;

/**
 * @brief 样本输出快照。
 * @note 除对外状态外，额外补充黄金回归需要冻结的中间统计量。
 */
typedef struct {
  float predicted_power_w;
  float allowed_power_w;
  float upper_limit_w;
  float lower_limit_w;
  float ref_limit_w;
  float hard_limit_w;
  float buffer_feedback;
  float cmd_power_sum_w;
  float effective_max_power_w;
  float k1_after;
  float k2_after;
  uint8_t error_flags;
  uint8_t rls_updated;
  float limited_wheel_tau_ref[4];
  float limited_output[4];
} PowerControllerSampleOutput;

/**
 * @brief 执行单个样本。
 * @return 1 表示执行完成，0 表示参数非法。
 * @note 该接口按“单进程单样本”使用，确保底层静态实例在进程启动时处于干净状态。
 */
uint8_t PowerControllerRunSample(const PowerControllerSampleInput *input,
                                 PowerControllerSampleOutput *output);

/**
 * @brief 执行多步状态演化样本。
 * @return 1 表示执行完成，0 表示参数非法。
 */
uint8_t PowerControllerRunSampleSequence(
    const PowerControllerConfig_t *config,
    const PowerControllerSampleStep *steps,
    uint32_t step_count,
    PowerControllerSampleOutput *output);

#endif // POWER_CONTROLLER_SAMPLE_HARNESS_H
