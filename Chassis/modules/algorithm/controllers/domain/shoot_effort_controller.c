/**
 * @file shoot_effort_controller.c
 * @brief 发射执行器统一 effort 计算实现
 * @note Algorithm层：将 shoot 的速度/角度参考收敛为 TAU_REF
 */

#include "controllers/domain/shoot_effort_controller.h"

#include <string.h>

static uint8_t ShootCalculateDJIEffort(DJIMotorInstance *motor,
                                       Closeloop_Type_e outer_loop,
                                       float ref,
                                       Controller_Effort_Output_s *effort) {
  if (motor == NULL || effort == NULL) {
    return 0U;
  }

  DJIMotorOuterLoop(motor, outer_loop);
  memset(effort, 0, sizeof(*effort));
  return DJIMotorCalculateEffort(motor, ref, effort);
}

uint8_t ShootFrictionCalculateEffort(DJIMotorInstance *motor,
                                     float target_speed_aps,
                                     Controller_Effort_Output_s *effort) {
  return ShootCalculateDJIEffort(motor, SPEED_LOOP, target_speed_aps, effort);
}

uint8_t ShootLoaderCalculateAngleEffort(DJIMotorInstance *motor,
                                        float target_angle_deg,
                                        Controller_Effort_Output_s *effort) {
  return ShootCalculateDJIEffort(motor, ANGLE_LOOP, target_angle_deg, effort);
}

uint8_t ShootLoaderCalculateSpeedEffort(DJIMotorInstance *motor,
                                        float target_speed_aps,
                                        Controller_Effort_Output_s *effort) {
  return ShootCalculateDJIEffort(motor, SPEED_LOOP, target_speed_aps, effort);
}
