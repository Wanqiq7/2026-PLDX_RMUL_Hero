/**
 ******************************************************************************
 * @file	 controller.h
 * @author  Wang Hongxi
 * @version V1.1.3
 * @date    2021/7/3
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "arm_math.h"
#include "bsp_dwt.h"
#include "main.h"
#include "memory.h"
#include "stdint.h"
#include "stdlib.h"
#include <math.h>

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

// PID 优化环节使能标志位,通过位与可以判断启用的优化环节;也可以改成位域的形式
typedef enum {
  PID_IMPROVE_NONE = 0b00000000,                // 0000 0000
  PID_Integral_Limit = 0b00000001,              // 0000 0001
  PID_Derivative_On_Measurement = 0b00000010,   // 0000 0010
  PID_Trapezoid_Intergral = 0b00000100,         // 0000 0100
  PID_Proportional_On_Measurement = 0b00001000, // 0000 1000
  PID_OutputFilter = 0b00010000,                // 0001 0000
  PID_ChangingIntegrationRate = 0b00100000,     // 0010 0000
  PID_DerivativeFilter = 0b01000000,            // 0100 0000
  PID_ErrorHandle = 0b10000000,                 // 1000 0000
} PID_Improvement_e;

/* PID 报错类型枚举*/
typedef enum errorType_e {
  PID_ERROR_NONE = 0x00U,
  PID_MOTOR_BLOCKED_ERROR = 0x01U
} ErrorType_e;

typedef struct {
  uint64_t ERRORCount;
  ErrorType_e ERRORType;
} PID_ErrorHandler_t;

/* PID结构体 */
typedef struct {
  //---------------------------------- init config block
  // config parameter
  float Kp;
  float Ki;
  float Kd;
  float MaxOut;
  float DeadBand;

  // improve parameter
  PID_Improvement_e Improve;
  float IntegralLimit; // 积分限幅
  float CoefA;         // 变速积分 For Changing Integral
  float CoefB; // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
  float Output_LPF_RC;     // 输出滤波器 RC = 1/omegac
  float Derivative_LPF_RC; // 微分滤波器系数

  //-----------------------------------
  // for calculating
  float Measure;
  float Last_Measure;
  float Err;
  float Last_Err;
  float Last_ITerm;

  float Pout;
  float Iout;
  float Dout;
  float ITerm;

  float Output;
  float Last_Output;
  float Last_Dout;

  float Ref;

  uint32_t DWT_CNT;
  float dt;

  PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

/* 用于PID初始化的结构体*/
typedef struct // config parameter
{
  // basic parameter
  float Kp;
  float Ki;
  float Kd;
  float MaxOut;   // 输出限幅
  float DeadBand; // 死区

  // improve parameter
  PID_Improvement_e Improve;
  float IntegralLimit; // 积分限幅
  float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
  float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
  float Output_LPF_RC; // RC = 1/omegac
  float Derivative_LPF_RC;
} PID_Init_Config_s;

/**
 * @brief 初始化PID实例
 * @todo 待修改为统一的PIDRegister风格
 * @param pid    PID实例指针
 * @param config PID初始化配置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);

/**
 * @brief 计算PID输出
 *
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param ref     设定值
 * @return float  PID计算输出
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref);

/* SMC滑模控制器结构体 */
typedef struct {
  //---------------------------------- init config block
  // config parameter
  float k1;             // 滑模面系数1
  float k2;             // 滑模面系数2
  float alpha;          // 趋近律系数alpha
  float beta;           // 趋近律系数beta
  float max_out;        // 最大输出限幅
  float feedforward_k1; // 前馈系数1
  float feedforward_k2; // 前馈系数2

  //-----------------------------------
  // for calculating
  float ref;           // 目标值(角度,单位:度)
  float last_ref;      // 上次目标值
  float last_last_ref; // 上上次目标值
  float measure;       // 当前反馈值(角度,单位:度)
  float measure_vel;   // 当前角速度反馈(单位:度/s)

  float error;           // 误差
  float error_dot;       // 误差微分
  float sliding_surface; // 滑模面

  float output; // 控制器输出

  uint32_t DWT_CNT; // 用于计算时间间隔
  float dt;         // 计算周期
} SMCInstance;

/* 用于SMC初始化的结构体*/
typedef struct {
  float k1;             // 滑模面系数1
  float k2;             // 滑模面系数2
  float alpha;          // 趋近律系数alpha
  float beta;           // 趋近律系数beta
  float max_out;        // 最大输出限幅
  float feedforward_k1; // 前馈系数1
  float feedforward_k2; // 前馈系数2
} SMC_Init_Config_s;

/**
 * @brief 初始化SMC滑模控制器实例
 *
 * @param smc    SMC实例指针
 * @param config SMC初始化配置
 */
void SMCInit(SMCInstance *smc, SMC_Init_Config_s *config);

/**
 * @brief 计算SMC滑模控制器输出
 *
 * @param smc         SMC实例指针
 * @param measure     当前角度反馈值(单位:度)
 * @param measure_vel 当前角速度反馈值(单位:度/s)
 * @param ref         目标角度值(单位:度)
 * @return float      SMC控制器输出
 */
float SMCCalculate(SMCInstance *smc, float measure, float measure_vel,
                   float ref);

/* LQR线性二次调节器结构体 */
typedef struct {
  //---------------------------------- init config block
  // config parameter
  float K_angle;    // 角度反馈增益 [A/rad]
  float K_velocity; // 角速度反馈增益 [A·s/rad]
  float K_integral; // 积分增益 (可选) [A/(rad·s)]
  float max_out;    // 最大输出限幅 [A]

  // improve parameter
  uint8_t enable_integral;   // 是否启用积分项
  float integral_limit;      // 积分限幅
  float integral_deadband;   // 积分死区 [rad]
  float integral_decay_coef; // 积分衰减系数 (变增益积分)

  //-----------------------------------
  // for calculating
  float ref;           // 目标角度 [rad]
  float measure_angle; // 当前角度反馈 [rad]
  float measure_vel;   // 当前角速度反馈 [rad/s]

  float angle_error; // 角度误差 [rad]
  float integral;    // 积分项累积值

  float output; // 控制器输出 [A]

  uint32_t DWT_CNT; // 用于计算时间间隔
  float dt;         // 计算周期 [s]
} LQRInstance;

/* 用于LQR初始化的结构体 */
typedef struct {
  // basic parameter
  float K_angle;    // 角度反馈增益 [A/rad]
  float K_velocity; // 角速度反馈增益 [A·s/rad]
  float K_integral; // 积分增益 [A/(rad·s)]
  float max_out;    // 最大输出限幅 [A]

  // improve parameter
  uint8_t enable_integral;   // 是否启用积分项 (0:禁用, 1:启用)
  float integral_limit;      // 积分限幅
  float integral_deadband;   // 积分死区 [rad]
  float integral_decay_coef; // 积分衰减系数 (0-1, 用于变增益积分)
} LQR_Init_Config_s;

/**
 * @brief 初始化LQR控制器实例
 *
 * @param lqr    LQR实例指针
 * @param config LQR初始化配置
 * @note  LQR适用于已建立系统模型的线性系统
 *        增益K通过MATLAB离线计算: [K, S, e] = lqr(A, B, Q, R)
 */
void LQRInit(LQRInstance *lqr, LQR_Init_Config_s *config);

/**
 * @brief 计算LQR控制器输出
 *
 * @param lqr         LQR实例指针
 * @param measure_angle 当前角度反馈值 [rad]
 * @param measure_vel   当前角速度反馈值 [rad/s]
 * @param ref          目标角度值 [rad]
 * @return float       LQR控制器输出 (电流 [A])
 *
 * @note  控制律: u = -K1*(θ_ref - θ) - K2*ω + Ki*∫e·dt
 *        所有物理量使用标准单位制 (弧度、弧度/秒、安培)
 */
float LQRCalculate(LQRInstance *lqr, float measure_angle, float measure_vel,
                   float ref);

/**
 * @brief 重置LQR控制器状态 (清除积分项)
 *
 * @param lqr LQR实例指针
 */
void LQRReset(LQRInstance *lqr);

#endif