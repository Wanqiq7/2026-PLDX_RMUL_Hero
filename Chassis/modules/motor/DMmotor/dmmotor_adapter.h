#ifndef DMMOTOR_ADAPTER_H
#define DMMOTOR_ADAPTER_H

#include "bsp_can.h"
#include "motor_def.h"

#include <stdint.h>

/**
 * @brief 将 tau_ref 归一输出转换为达妙 MIT 扭矩命令
 *
 * @param input 已归一为 tau_ref 的控制器输出
 * @param torque_limit_nm 扭矩限幅
 * @param command 输出命令
 * @param normalized_output 限幅后的 tau_ref 输出，可为 NULL
 * @return uint8_t 1-成功，0-失败
 */
uint8_t DMMotorBuildMitTorqueCommand(
    const Controller_Effort_Output_s *input, float torque_limit_nm,
    Actuator_Command_s *command,
    Controller_Effort_Output_s *normalized_output);

#endif
