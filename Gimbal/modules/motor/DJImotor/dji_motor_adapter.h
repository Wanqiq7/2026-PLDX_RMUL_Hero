#ifndef DJI_MOTOR_ADAPTER_H
#define DJI_MOTOR_ADAPTER_H

#include "bsp_can.h"
#include "motor_def.h"

#include <stdint.h>

/**
 * @brief 解析并补全 DJI 动力电机的物理参数
 *
 * @param motor_type 电机类型
 * @param configured 用户显式配置的参数，可为 NULL
 * @param resolved 解析后的参数输出
 */
uint8_t DJIMotorResolvePhysicalParam(Motor_Type_e motor_type,
                                     const Motor_Physical_Param_s *configured,
                                     Motor_Physical_Param_s *resolved);

/**
 * @brief 将控制器输出转换为 DJI 原始电流指令
 *
 * @param param 电机物理参数
 * @param input 控制器输出
 * @param raw_cmd 原始指令输出
 * @param normalized_output 归一化后的 tau_ref 输出，可为 NULL
 * @return uint8_t 1-成功，0-失败
 */
uint8_t DJIMotorBuildRawCommandFromEffort(
    const Motor_Physical_Param_s *param,
    const Controller_Effort_Output_s *input, int16_t *raw_cmd,
    Controller_Effort_Output_s *normalized_output);

#endif
