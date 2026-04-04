#include "dmmotor_adapter.h"

#include <string.h>

uint8_t DMMotorBuildMitTorqueCommand(
    const Controller_Effort_Output_s *input, float torque_limit_nm,
    Actuator_Command_s *command,
    Controller_Effort_Output_s *normalized_output) {
  float tau_ref_nm = 0.0f;

  if (input == NULL || command == NULL) {
    return 0U;
  }

  if (input->semantic != CONTROLLER_OUTPUT_TAU_REF) {
    return 0U;
  }

  tau_ref_nm = input->tau_ref_nm;
  if (torque_limit_nm > 0.0f) {
    if (tau_ref_nm > torque_limit_nm) {
      tau_ref_nm = torque_limit_nm;
    } else if (tau_ref_nm < -torque_limit_nm) {
      tau_ref_nm = -torque_limit_nm;
    }
  }

  memset(command, 0, sizeof(*command));
  command->type = ACTUATOR_COMMAND_DM_MIT_TORQUE;
  command->mit_torque_nm = tau_ref_nm;

  if (normalized_output != NULL) {
    memset(normalized_output, 0, sizeof(*normalized_output));
    normalized_output->semantic = CONTROLLER_OUTPUT_TAU_REF;
    normalized_output->tau_ref_nm = tau_ref_nm;
  }

  return 1U;
}
