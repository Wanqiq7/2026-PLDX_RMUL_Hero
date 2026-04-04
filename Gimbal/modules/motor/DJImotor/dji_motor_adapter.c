#include "dji_motor_adapter.h"

#include <math.h>
#include <string.h>

static uint8_t DJIMotorNormalizeToTauRef(
    const Motor_Physical_Param_s *param,
    const Controller_Effort_Output_s *input,
    Controller_Effort_Output_s *output) {
  // output_shaft_torque: 统一将 legacy 输出归一为输出轴扭矩参考
  if (param == NULL || input == NULL || output == NULL) {
    return 0U;
  }

  memset(output, 0, sizeof(*output));
  output->semantic = CONTROLLER_OUTPUT_TAU_REF;

  switch (input->semantic) {
  case CONTROLLER_OUTPUT_TAU_REF:
    output->tau_ref_nm = input->tau_ref_nm;
    return 1U;

  case CONTROLLER_OUTPUT_CURRENT_A:
    if (param->torque_constant_nm_per_a <= 0.0f) {
      return 0U;
    }
    output->current_ref_a = input->current_ref_a;
    output->tau_ref_nm =
        input->current_ref_a * param->torque_constant_nm_per_a;
    return 1U;

  case CONTROLLER_OUTPUT_RAW_CURRENT_CMD:
    if (param->raw_to_current_coeff <= 0.0f ||
        param->torque_constant_nm_per_a <= 0.0f) {
      return 0U;
    }
    output->raw_current_cmd = input->raw_current_cmd;
    output->current_ref_a = input->raw_current_cmd * param->raw_to_current_coeff;
    output->tau_ref_nm =
        output->current_ref_a * param->torque_constant_nm_per_a;
    return 1U;

  case CONTROLLER_OUTPUT_INVALID:
  default:
    return 0U;
  }
}

static void DJIMotorApplyDefaultPhysicalParam(Motor_Type_e motor_type,
                                              Motor_Physical_Param_s *param) {
  if (param == NULL) {
    return;
  }

  switch (motor_type) {
  case M3508:
    if (param->torque_constant_nm_per_a <= 0.0f) {
      param->torque_constant_nm_per_a = 0.3f;
    }
    if (param->current_limit_a <= 0.0f) {
      param->current_limit_a = 20.0f;
    }
    if (param->current_to_raw_coeff <= 0.0f) {
      param->current_to_raw_coeff = 16384.0f / 20.0f;
    }
    if (param->raw_to_current_coeff <= 0.0f) {
      param->raw_to_current_coeff = 20.0f / 16384.0f;
    }
    break;

  case M2006:
    if (param->torque_constant_nm_per_a <= 0.0f) {
      param->torque_constant_nm_per_a = 0.18f;
    }
    if (param->current_limit_a <= 0.0f) {
      param->current_limit_a = 10.0f;
    }
    if (param->current_to_raw_coeff <= 0.0f) {
      param->current_to_raw_coeff = 10000.0f / 10.0f;
    }
    if (param->raw_to_current_coeff <= 0.0f) {
      param->raw_to_current_coeff = 10.0f / 10000.0f;
    }
    break;

  case GM6020:
    if (param->torque_constant_nm_per_a <= 0.0f) {
      param->torque_constant_nm_per_a = 0.741f;
    }
    if (param->current_limit_a <= 0.0f) {
      param->current_limit_a = 20.0f;
    }
    if (param->current_to_raw_coeff <= 0.0f) {
      param->current_to_raw_coeff = 16384.0f / 20.0f;
    }
    if (param->raw_to_current_coeff <= 0.0f) {
      param->raw_to_current_coeff = 20.0f / 16384.0f;
    }
    break;

  default:
    break;
  }
  if (param->torque_limit_nm <= 0.0f && param->current_limit_a > 0.0f &&
      param->torque_constant_nm_per_a > 0.0f) {
    param->torque_limit_nm =
        param->current_limit_a * param->torque_constant_nm_per_a;
  }
}

static uint8_t DJIMotorPhysicalParamIsValid(
    const Motor_Physical_Param_s *param) {
  return (param != NULL && param->torque_constant_nm_per_a > 0.0f &&
          param->current_to_raw_coeff > 0.0f &&
          param->raw_to_current_coeff > 0.0f && param->current_limit_a > 0.0f)
             ? 1U
             : 0U;
}

uint8_t DJIMotorResolvePhysicalParam(Motor_Type_e motor_type,
                                     const Motor_Physical_Param_s *configured,
                                     Motor_Physical_Param_s *resolved) {
  if (resolved == NULL) {
    return 0U;
  }

  memset(resolved, 0, sizeof(*resolved));
  if (configured != NULL) {
    *resolved = *configured;
  }

  DJIMotorApplyDefaultPhysicalParam(motor_type, resolved);
  return DJIMotorPhysicalParamIsValid(resolved);
}

uint8_t DJIMotorBuildRawCommandFromEffort(
    const Motor_Physical_Param_s *param,
    const Controller_Effort_Output_s *input, int16_t *raw_cmd,
    Controller_Effort_Output_s *normalized_output) {
  // output_shaft_torque -> phase current -> DJI raw current command
  Controller_Effort_Output_s normalized = {0};
  float current_ref_a = 0.0f;
  float raw_value = 0.0f;

  if (param == NULL || input == NULL || raw_cmd == NULL) {
    return 0U;
  }

  if (!DJIMotorNormalizeToTauRef(param, input, &normalized)) {
    return 0U;
  }

  if (param->torque_constant_nm_per_a <= 0.0f ||
      param->current_to_raw_coeff <= 0.0f) {
    return 0U;
  }

  current_ref_a = normalized.tau_ref_nm / param->torque_constant_nm_per_a;

  if (param->current_limit_a > 0.0f) {
    if (current_ref_a > param->current_limit_a) {
      current_ref_a = param->current_limit_a;
    } else if (current_ref_a < -param->current_limit_a) {
      current_ref_a = -param->current_limit_a;
    }
  }

  raw_value = current_ref_a * param->current_to_raw_coeff;
  *raw_cmd = (int16_t)lroundf(raw_value);

  if (normalized_output != NULL) {
    *normalized_output = normalized;
    normalized_output->current_ref_a = current_ref_a;
    normalized_output->raw_current_cmd = (float)(*raw_cmd);
  }

  return 1U;
}
