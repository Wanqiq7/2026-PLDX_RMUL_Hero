#include "power_controller_sample_harness.h"

#include <string.h>

/**
 * @brief 说明：
 * 1. 该 harness 只复用 power_controller public API。
 * 2. 由于模块当前是静态单实例实现，推荐按“单进程单样本”方式运行。
 * 3. 本文件提供稳定的样本输入/输出契约，便于后续接入原生 host runner。
 */

static void PowerControllerApplyMotorOnline(
    PowerControllerInstance *instance, const uint8_t motor_online[4],
    const uint16_t motor_disconnect_cycles[4]) {
  for (uint8_t i = 0; i < 4; i++) {
    if (motor_online != NULL) {
      PowerUpdateMotorOnline(instance, i, motor_online[i]);
    }

    if (motor_disconnect_cycles != NULL) {
      for (uint16_t cycle = 0; cycle < motor_disconnect_cycles[i]; cycle++) {
        PowerUpdateMotorOnline(instance, i, 0U);
      }
    }
  }
}

static void PowerControllerSnapshot(const PowerControllerStatus_t *status,
                                    PowerControllerSampleOutput *output) {
  if (status == NULL || output == NULL) {
    return;
  }

  output->allowed_power_w = status->allowed_power_w;
  output->upper_limit_w = status->upper_limit_w;
  output->lower_limit_w = status->lower_limit_w;
  output->ref_limit_w = status->ref_limit_w;
  output->hard_limit_w = status->hard_limit_w;
  output->buffer_feedback = status->buffer_feedback;
  output->cmd_power_sum_w = status->cmd_power_sum_w;
  output->k1_after = status->k1;
  output->k2_after = status->k2;
  output->error_flags = status->error_flags;
}

static void PowerControllerMirrorLimitedWheelTauRef(
    PowerControllerSampleOutput *output) {
  if (output == NULL) {
    return;
  }

  memcpy(output->limited_output, output->limited_wheel_tau_ref,
         sizeof(output->limited_output));
}

uint8_t PowerControllerRunSample(const PowerControllerSampleInput *input,
                                 PowerControllerSampleOutput *output) {
  if (input == NULL || output == NULL) {
    return 0U;
  }

  memset(output, 0, sizeof(*output));

  PowerControllerInstance *instance = PowerControllerRegister(&input->config);
  if (instance == NULL) {
    return 0U;
  }

  PowerSetRLSEnable(instance, input->rls_enabled);
  PowerUpdateRefereeOnline(instance, input->referee_online, input->robot_level);
  PowerUpdateRefereeData(instance, input->referee_limit_w,
                         input->referee_buffer_energy, input->referee_power_w);
  PowerUpdateCapData(instance, input->cap_energy_percent, input->cap_online);
  PowerUpdateMotorFeedback(instance, (float *)input->motor_speeds,
                           (float *)input->motor_torques);
  PowerControllerApplyMotorOnline(instance, input->motor_online,
                                  input->motor_disconnect_cycles);

  for (uint8_t iteration = 0; iteration < input->run_task_iterations;
       iteration++) {
    PowerControllerTask(instance);
  }

  PowerGetLimitedWheelTauRef(instance, input->wheel_objs,
                             output->limited_wheel_tau_ref);
  PowerControllerMirrorLimitedWheelTauRef(output);
  PowerControllerSnapshot(PowerGetStatus(instance), output);
  return 1U;
}

uint8_t PowerControllerRunSampleSequence(
    const PowerControllerConfig_t *config,
    const PowerControllerSampleStep *steps,
    uint32_t step_count,
    PowerControllerSampleOutput *output) {
  if (config == NULL || steps == NULL || step_count == 0U || output == NULL) {
    return 0U;
  }

  memset(output, 0, sizeof(*output));

  PowerControllerInstance *instance = PowerControllerRegister(config);
  if (instance == NULL) {
    return 0U;
  }

  for (uint32_t step_index = 0; step_index < step_count; step_index++) {
    const PowerControllerSampleStep *step = &steps[step_index];
    PowerSetRLSEnable(instance, step->rls_enabled);
    PowerUpdateRefereeOnline(instance, step->referee_online, step->robot_level);
    PowerUpdateRefereeData(instance, step->referee_limit_w,
                           step->referee_buffer_energy, step->referee_power_w);
    PowerUpdateCapData(instance, step->cap_energy_percent, step->cap_online);
    PowerUpdateMotorFeedback(instance, (float *)step->motor_speeds,
                             (float *)step->motor_torques);
    PowerControllerApplyMotorOnline(instance, step->motor_online,
                                    step->motor_disconnect_cycles);

    for (uint8_t iteration = 0; iteration < step->run_task_iterations;
         iteration++) {
      PowerControllerTask(instance);
      output->rls_updated = 1U;
    }
  }

  PowerControllerSnapshot(PowerGetStatus(instance), output);
  return 1U;
}
