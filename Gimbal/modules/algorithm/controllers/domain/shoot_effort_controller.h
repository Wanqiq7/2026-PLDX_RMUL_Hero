/**
 * @file shoot_effort_controller.h
 * @brief 发射执行器统一 effort 计算接口
 * @note Algorithm层：将 shoot 的速度/角度参考收敛为 TAU_REF
 */

#ifndef SHOOT_EFFORT_CONTROLLER_H
#define SHOOT_EFFORT_CONTROLLER_H

#include "dji_motor.h"

uint8_t ShootFrictionCalculateEffort(DJIMotorInstance *motor,
                                     float target_speed_aps,
                                     Controller_Effort_Output_s *effort);

uint8_t ShootLoaderCalculateAngleEffort(DJIMotorInstance *motor,
                                        float target_angle_deg,
                                        Controller_Effort_Output_s *effort);

uint8_t ShootLoaderCalculateSpeedEffort(DJIMotorInstance *motor,
                                        float target_speed_aps,
                                        Controller_Effort_Output_s *effort);

#endif // SHOOT_EFFORT_CONTROLLER_H
