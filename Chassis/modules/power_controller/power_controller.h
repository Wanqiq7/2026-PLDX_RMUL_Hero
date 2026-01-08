/**
 * @file power_controller.h
 * @author Wanqiq
 * @brief 独立的底盘功率控制模块，包含RLS参数辨识和双环功率控制
 * @reference 参考港科大24年开源
 * @version 2.0
 * @date 2024-10-30
 * @note 功率模型：P = τω + k1|ω| + k2τ² + k3
 * @note 双环控制：能量环(外环) + 功率环(内环)
 */

#ifndef POWER_CONTROLLER_H
#define POWER_CONTROLLER_H

#include "controller.h"
#include "stdint.h"

/* ======================== 配置宏定义 ======================== */
// 使能开关
#define POWER_CONTROLLER_ENABLE 0 // 功率控制总开关
#define RLS_ENABLE 0              // RLS参数辨识使能

// RLS保护参数（防止协方差爆炸）
#define RLS_MIN_TORQUE_SQ 0.08f            // 最小力矩平方和阈值 (Nm²)
#define RLS_MIN_POWER_LOSS -10.0f          // 最小功率损耗 (W)
#define RLS_MAX_POWER_LOSS_RATIO 1.5f      // 最大功率损耗比例（相对于功率限制）
#define RLS_COVARIANCE_TRACE_LIMIT 1000.0f // 协方差矩阵迹限制

// 能量环PD控制器参数
#define POWER_PD_KP 25.0f // 降低比例增益，减少超调
#define POWER_PD_KD 0.0f  // 增加微分增益，增加阻尼

// 功率分配参数
#define ERROR_POWER_DISTRIBUTION_THRESHOLD 30.0f // error分配阈值上限
#define PROP_POWER_DISTRIBUTION_THRESHOLD 20.0f  // error分配阈值下限

// 电容和裁判系统状态
#define REFEREE_FULL_BUFF 60.0f
#define REFEREE_BASE_BUFF 55.0f
#define CAP_FULL_BUFF 235.0f
#define CAP_BASE_BUFF 50.0f
#define MAX_CAP_POWER_OUT 300.0f
#define CAP_OFFLINE_THRESHOLD 43.0f

/* ======================== 数据结构定义 ======================== */

/**
 * @brief 单个电机的功率对象
 */
typedef struct {
  float pid_output;     // PID输出（CAN指令值）
  float current_av;     // 当前角速度 (rad/s)
  float target_av;      // 目标角速度 (rad/s)
  float pid_max_output; // PID最大输出限制
} PowerMotorObj_t;

/**
 * @brief 功率控制器配置
 */
typedef struct {
  // RLS初始参数
  float k1_init;    // 转速损耗系数初始值
  float k2_init;    // 力矩损耗系数初始值
  float k3;         // 静态功率损耗（固定）
  float rls_lambda; // RLS遗忘因子

  // 电机参数
  float torque_constant; // 电机转矩常数 (Nm/A)
  float current_scale;   // CAN指令到电流的转换系数
} PowerControllerConfig_t;

/**
 * @brief 功率控制状态（可供外部查询）
 */
typedef struct {
  // 实时参数
  float k1; // 当前k1参数
  float k2; // 当前k2参数

  // 功率限制
  float max_power_limit; // 当前功率上限
  float power_upper;     // 功率上限
  float power_lower;     // 功率下限

  // 功率统计
  float estimated_power; // 估算功率
  float sum_cmd_power;   // 指令功率总和

  // 能量状态
  float energy_feedback; // 能量反馈
  uint8_t cap_online;    // 电容在线标志

  // 错误标志
  uint8_t rls_enabled; // RLS使能状态
} PowerControllerStatus_t;

/* ======================== 接口函数声明 ======================== */

/**
 * @brief 功率控制器初始化
 * @param config 配置参数
 * @note 必须在调度器启动前调用
 */
void PowerControllerInit(const PowerControllerConfig_t *config);

/**
 * @brief 功率控制器任务（独立任务）
 * @note 建议1-5ms周期调用，处理RLS更新和能量环控制
 */
void PowerControllerTask(void);

/**
 * @brief 获取功率限制后的电机输出
 * @param motor_objs 四个电机的功率对象数组
 * @param output 输出数组（由调用者提供空间）
 * @note 在底盘控制任务中调用，实时性要求高
 */
void PowerGetLimitedOutput(PowerMotorObj_t motor_objs[4], float output[4]);

/**
 * @brief 更新裁判系统数据
 * @param chassis_power_limit 底盘功率上限
 * @param chassis_power_buffer 功率缓冲
 * @param chassis_power 当前功率
 */
void PowerUpdateRefereeData(float chassis_power_limit,
                            float chassis_power_buffer, float chassis_power);

/**
 * @brief 更新超级电容数据
 * @param cap_voltage 电容电压 (0-255)
 * @param cap_online 电容在线标志
 */
void PowerUpdateCapData(uint8_t cap_voltage, uint8_t cap_online);

/**
 * @brief 更新电机反馈数据
 * @param motor_speeds 电机转速数组 (rad/s)
 * @param motor_torques 电机转矩数组 (Nm)
 */
void PowerUpdateMotorFeedback(float motor_speeds[4], float motor_torques[4]);

/**
 * @brief 获取功率控制器状态
 * @return 功率控制器状态结构体指针
 */
const PowerControllerStatus_t *PowerGetStatus(void);

/**
 * @brief 设置RLS使能状态
 * @param enable 1:使能, 0:禁用
 */
void PowerSetRLSEnable(uint8_t enable);

/**
 * @brief 设置用户自定义功率限制
 * @param power_limit 功率限制值 (W)
 */
void PowerSetUserLimit(float power_limit);

#endif // POWER_CONTROLLER_H
