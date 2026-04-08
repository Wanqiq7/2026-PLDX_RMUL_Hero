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

#include "stdint.h"

/* ======================== 配置宏定义 ======================== */
// 使能开关
#define POWER_CONTROLLER_ENABLE 1 // 功率控制总开关（正常比赛模式开启）
#define RLS_ENABLE 1              // RLS参数辨识使能（调试力控PID时关闭）

// RLS保护参数（防止协方差爆炸）
#define RLS_MIN_TORQUE_SQ 0.08f            // 最小力矩平方和阈值 (Nm²)
#define RLS_MIN_POWER_LOSS -10.0f          // 最小功率损耗 (W)
#define RLS_MAX_POWER_LOSS_RATIO 1.5f      // 最大功率损耗比例（相对于功率限制）
#define RLS_COVARIANCE_TRACE_LIMIT 1000.0f // 协方差矩阵迹限制

// 能量环PD控制器参数（参考港科大实现）
#define POWER_PD_KP 50.0f          // 比例增益
#define POWER_PD_KD 0.20f          // 微分增益（恢复阻尼）
#define POWER_PD_D_FILTER_ALPHA 0.3f // 微分项低通滤波系数(0~1)

// 功率分配参数（参考港科大实现）
#define ERROR_POWER_DISTRIBUTION_THRESHOLD 20.0f // error分配阈值上限
#define PROP_POWER_DISTRIBUTION_THRESHOLD 15.0f  // error分配阈值下限

// 电容和裁判系统状态（参考港科大实现）
#define REFEREE_FULL_BUFF 60.0f
#define REFEREE_BASE_BUFF 50.0f // 降低以增加安全裕度
#define CAP_FULL_BUFF 230.0f
#define CAP_BASE_BUFF 30.0f // 降低以增加安全裕度
#define MAX_CAP_POWER_OUT 300.0f
#define CAP_OFFLINE_THRESHOLD 43.0f

// 错误处理参数
#define CAP_REFEREE_BOTH_GG_COE 0.85f // 双断连保守系数
#define REFEREE_GG_COE 0.95f          // 裁判系统断连保守系数
#define MOTOR_DISCONNECT_TIMEOUT 1000 // 电机断连超时计数 (ms)
#define MIN_POWER_CONFIGURED 30.0f    // 最小功率配置值

// 机器人等级最大值
#define MAX_ROBOT_LEVEL 10

/* ======================== 数据结构定义 ======================== */

/**
 * @brief 功率控制器实例（前向声明）
 * @note 内部实现细节对App隐藏，App层仅持有指针
 */
typedef struct PowerControllerInstance PowerControllerInstance;

/**
 * @brief 机器人类型枚举
 */
typedef enum {
  ROBOT_INFANTRY = 0, // 步兵
  ROBOT_HERO,         // 英雄
  ROBOT_SENTRY        // 哨兵
} RobotDivision_e;

/**
 * @brief 错误标志枚举
 */
typedef enum {
  POWER_ERROR_NONE = 0x00,
  POWER_ERROR_MOTOR_DISCONNECT = 0x01,   // 电机断连
  POWER_ERROR_REFEREE_DISCONNECT = 0x02, // 裁判系统断连
  POWER_ERROR_CAP_DISCONNECT = 0x04      // 电容断连
} PowerErrorFlags_e;

/**
 * @brief 原生轮端功率对象
 * @note Phase 4 起推荐外部主线使用该结构，语义保持在轮端 tau 域
 */
typedef struct {
  float requested_tau_nm;      // 请求的轮端扭矩参考 (Nm)
  float feedback_speed_rad_s;  // 当前轮端角速度 (rad/s)
  float target_speed_rad_s;    // 目标轮端角速度 (rad/s)
  float feedback_tau_nm;       // 当前轮端反馈扭矩 (Nm)
  float max_tau_nm;            // 当前轮端最大允许扭矩 (Nm)
  uint8_t online;              // 当前轮是否在线
} PowerWheelObj_t;

/**
 * @brief 单个电机的旧域功率对象
 * @note 仅用于 Phase 4 迁移过渡，避免一次性打断现有样本与兼容入口
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

  // 机器人类型（用于断连时查表）
  RobotDivision_e robot_division;
} PowerControllerConfig_t;

/**
 * @brief 功率控制状态（可供外部查询）
 */
typedef struct {
  // 实时参数
  float k1; // 当前k1参数
  float k2; // 当前k2参数

  // 功率限制
  float allowed_power_w; // 当前实际生效的功率上限
  float upper_limit_w;   // 上限缓冲线（full）
  float lower_limit_w;   // 下限缓冲线（base）
  float ref_limit_w;     // 当前参考功率上限（优先来自裁判）
  float hard_limit_w;    // 当前绝对硬上限（含超电放宽）

  // 功率统计
  float est_power_w;      // 估算功率
  float measured_power_w; // 实测功率
  float cmd_power_sum_w;  // 指令功率总和
  float mech_power_w;     // 有效机械功率（τω）
  float loss_power_w;     // 功率损耗

  // 能量状态
  float buffer_feedback; // 当前用于能量环的缓冲反馈量
  float cap_energy_est;  // 统一量纲后的估算储能状态
  uint8_t cap_online;         // 超级电容在线标志
  uint8_t referee_online;     // 裁判系统在线标志

  // 错误标志
  uint8_t error_flags; // 错误标志位
  uint8_t rls_enabled; // RLS使能状态
  uint8_t robot_level; // 当前机器人等级
} PowerControllerStatus_t;

/* ======================== 接口函数声明 ======================== */

/**
 * @brief 注册功率控制器实例
 * @param config 配置参数
 * @return 功率控制器实例指针，失败返回 NULL
 * @note 符合框架规范：App层调用注册函数创建实例
 */
PowerControllerInstance *
PowerControllerRegister(const PowerControllerConfig_t *config);

/**
 * @brief 功率控制器任务（独立任务）
 * @param instance 功率控制器实例
 * @note 建议1-5ms周期调用，处理RLS更新和能量环控制
 */
void PowerControllerTask(PowerControllerInstance *instance);

/**
 * @brief 获取功率限制后的轮端扭矩参考
 * @param instance 功率控制器实例
 * @param wheel_objs 四个轮端对象数组
 * @param limited_wheel_tau_ref 输出数组（轮端 tau_ref）
 */
void PowerGetLimitedWheelTauRef(PowerControllerInstance *instance,
                                const PowerWheelObj_t wheel_objs[4],
                                float limited_wheel_tau_ref[4]);

/**
 * @brief 获取功率限制后的电机输出
 * @param instance 功率控制器实例
 * @param motor_objs 四个电机的功率对象数组
 * @param output 输出数组（由调用者提供空间）
 * @note 旧域兼容接口，内部会桥接到原生轮端 tau 域
 */
void PowerGetLimitedOutput(PowerControllerInstance *instance,
                           PowerMotorObj_t motor_objs[4], float output[4]);

/**
 * @brief 更新裁判系统功率反馈
 * @param instance 功率控制器实例
 * @param limit_w 当前生效的功率上限 (W)
 * @param buffer_energy 裁判系统缓冲能量
 * @param power_w 裁判系统反馈功率 (W)
 */
void PowerUpdateRefereeData(PowerControllerInstance *instance, float limit_w,
                            float buffer_energy, float power_w);

/**
 * @brief 更新超级电容数据
 * @param instance 功率控制器实例
 * @param cap_energy 电容能量百分比 (0-255, 255表示100%)
 * @param cap_online 电容在线标志
 */
void PowerUpdateCapData(PowerControllerInstance *instance, uint8_t cap_energy,
                        uint8_t cap_online);

/**
 * @brief 更新电机反馈数据
 * @param instance 功率控制器实例
 * @param motor_speeds 电机转速数组 (rad/s)
 * @param motor_torques 电机转矩数组 (Nm)
 */
void PowerUpdateMotorFeedback(PowerControllerInstance *instance,
                              float motor_speeds[4], float motor_torques[4]);

/**
 * @brief 获取功率控制器状态
 * @param instance 功率控制器实例
 * @return 功率控制器状态结构体指针
 */
const PowerControllerStatus_t *
PowerGetStatus(PowerControllerInstance *instance);

/**
 * @brief 设置RLS使能状态
 * @param instance 功率控制器实例
 * @param enable 1:使能, 0:禁用
 */
void PowerSetRLSEnable(PowerControllerInstance *instance, uint8_t enable);

/**
 * @brief 设置当前生效的目标功率上限
 * @param instance 功率控制器实例
 * @param limit_w 目标功率上限 (W)
 */
void PowerSetUserLimit(PowerControllerInstance *instance, float limit_w);

/**
 * @brief 更新裁判系统在线状态
 * @param instance 功率控制器实例
 * @param online 1:在线, 0:离线
 * @param robot_level 机器人等级 (1-10)
 */
void PowerUpdateRefereeOnline(PowerControllerInstance *instance, uint8_t online,
                              uint8_t robot_level);

/**
 * @brief 更新电机在线状态
 * @param instance 功率控制器实例
 * @param motor_index 电机索引 (0-3)
 * @param online 1:在线, 0:离线
 */
void PowerUpdateMotorOnline(PowerControllerInstance *instance,
                            uint8_t motor_index, uint8_t online);

/**
 * @brief 获取当前错误标志
 * @param instance 功率控制器实例
 * @return 错误标志位
 */
uint8_t PowerGetErrorFlags(PowerControllerInstance *instance);

#endif // POWER_CONTROLLER_H
