#ifndef FC_CONTROLLER_H
#define FC_CONTROLLER_H

#include "bsp_can.h"
#include "motor_def.h"

/**
 * @brief 初始化力控状态
 *
 * @param state 力控状态指针
 */
void FCInit(FC_State_s *state);

/**
 * @brief 重置力控状态
 *
 * @param state 力控状态指针
 * @param target_angle_rad 当前目标角度，单位 rad
 */
void FCReset(FC_State_s *state, float target_angle_rad);

/**
 * @brief 执行一次力控计算
 *
 * @param param 力控参数，单位使用 rad/rad/s/raw
 * @param state 力控状态
 * @param target_angle_rad 目标角度，单位 rad
 * @param angle_feedback_rad 角度反馈，单位 rad
 * @param rate_feedback_rad_s 角速度反馈，单位 rad/s
 * @param dt 控制周期，单位 s
 * @return float 输出电流原始量，单位 raw
 */
float FCCalculate(const FC_Init_Config_s *param, FC_State_s *state,
                  float target_angle_rad, float angle_feedback_rad,
                  float rate_feedback_rad_s, float dt);

#endif
