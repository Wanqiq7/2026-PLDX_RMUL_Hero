/**
 * @file sysid_task.h
 * @brief 底盘轮速系统辨识独立任务头文件
 * @author RoboMaster EC Team
 * @date 2025-01-04
 */

#ifndef SYSID_TASK_H
#define SYSID_TASK_H

#include "dji_motor.h"
#include "stdint.h"

/**
 * @brief 底盘系统辨识任务初始化
 * @param motor_lf 左前轮电机实例指针
 * @param motor_rf 右前轮电机实例指针
 * @param motor_lb 左后轮电机实例指针
 * @param motor_rb 右后轮电机实例指针
 */
void Chassis_SysIDTaskInit(DJIMotorInstance *motor_lf,
                           DJIMotorInstance *motor_rf,
                           DJIMotorInstance *motor_lb,
                           DJIMotorInstance *motor_rb);

/**
 * @brief 底盘系统辨识任务主循环
 * @note 在1kHz FreeRTOS任务中调用
 */
void Chassis_SysIDTask(void);

/**
 * @brief 检查底盘系统辨识任务是否激活
 * @return 1-激活，0-未激活
 */
uint8_t Chassis_SysIDIsActive(void);

#endif // SYSID_TASK_H
