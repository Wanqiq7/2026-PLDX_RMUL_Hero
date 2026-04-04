/**
 * @file motor_def.h
 * @author neozng
 * @brief  电机通用的数据结构定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

#include "bsp_can.h"
#include "SMC_Controller.h"
#include "controller.h"
#include "stdint.h"

#define LIMIT_MIN_MAX(x, min, max)                                             \
  (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

typedef enum {
  CONTROLLER_OUTPUT_INVALID = 0,
  CONTROLLER_OUTPUT_TAU_REF,
  CONTROLLER_OUTPUT_CURRENT_A,
  CONTROLLER_OUTPUT_RAW_CURRENT_CMD,
} Controller_Output_Semantic_e;

typedef struct {
  Controller_Output_Semantic_e semantic;
  float tau_ref_nm;      // output_shaft_torque [N·m]
  float current_ref_a;
  float raw_current_cmd;
} Controller_Effort_Output_s;

typedef struct {
  float torque_constant_nm_per_a; // output_shaft_torque constant [N·m/A]
  float current_to_raw_coeff;
  float raw_to_current_coeff;
  float torque_limit_nm;
  float current_limit_a;
} Motor_Physical_Param_s;

typedef enum {
  ACTUATOR_COMMAND_NONE = 0,
  ACTUATOR_COMMAND_DJI_RAW_CURRENT,
  ACTUATOR_COMMAND_DM_MIT_TORQUE,
  ACTUATOR_COMMAND_DM_MIT_FULL,
  ACTUATOR_COMMAND_DM_PVT,
} Actuator_Command_Type_e;

typedef struct {
  Actuator_Command_Type_e type;
  float raw_current_cmd;
  float mit_angle_rad;
  float mit_velocity_rad_s;
  float mit_torque_nm;
  float mit_kp;
  float mit_kd;
  float pvt_angle_rad;
  float pvt_velocity_limit_rad_s;
  float pvt_current_ratio;
} Actuator_Command_s;

/**
 * @brief 闭环类型,如果需要多个闭环,则使用或运算
 *        例如需要速度环和电流环: CURRENT_LOOP|SPEED_LOOP
 */
typedef enum {
  OPEN_LOOP = 0b0000,
  CURRENT_LOOP = 0b0001,
  SPEED_LOOP = 0b0010,
  ANGLE_LOOP = 0b0100,

  // only for checking
  SPEED_AND_CURRENT_LOOP = 0b0011,
  ANGLE_AND_SPEED_LOOP = 0b0110,
  ALL_THREE_LOOP = 0b0111,
} Closeloop_Type_e;

typedef enum {
  FEEDFORWARD_NONE = 0b00,
  CURRENT_FEEDFORWARD = 0b01,
  SPEED_FEEDFORWARD = 0b10,
  CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
} Feedfoward_Type_e;

/* 反馈来源设定,若设为OTHER_FEED则需要指定数据来源指针,详见Motor_Controller_s*/
typedef enum {
  MOTOR_FEED = 0,
  OTHER_FEED,
} Feedback_Source_e;

/* 电机正反转标志 */
typedef enum {
  MOTOR_DIRECTION_NORMAL = 0,
  MOTOR_DIRECTION_REVERSE = 1
} Motor_Reverse_Flag_e;

/* 反馈量正反标志 */
typedef enum {
  FEEDBACK_DIRECTION_NORMAL = 0,
  FEEDBACK_DIRECTION_REVERSE = 1
} Feedback_Reverse_Flag_e;

/* 控制器类型 */
typedef enum {
  CONTROLLER_PID = 0, // 使用PID控制器（默认）
  CONTROLLER_LQR = 1, // 使用LQR控制器
  CONTROLLER_SMC = 2, // 使用滑模控制器（Sliding Mode Control）
} Controller_Type_e;

typedef enum {
  MOTOR_STOP = 0,
  MOTOR_ENALBED = 1,
} Motor_Working_Type_e;

/* 达妙 MIT 配置结构体（用于驱动层 MIT 模式参数） */
typedef struct {
  float angle_min;
  float angle_max;
  float omega_min;
  float omega_max;
  float torque_min;
  float torque_max;
  float kp_min;
  float kp_max;
  float kd_min;
  float kd_max;
  float current_max;
} DM_MIT_Limit_s;

typedef struct {
  float control_angle;
  float control_omega;
  float control_torque;
  float kp;
  float kd;
} DM_MIT_State_s;

typedef struct {
  float kp;
  float kd;
  float v_des;
  float torque_des;
} DM_MIT_Profile_s;

typedef struct {
  DM_MIT_Limit_s limit;
  float default_kp;
  float default_kd;
  DM_MIT_Profile_s manual_profile;
  DM_MIT_Profile_s vision_profile;
  uint8_t auto_clear_error;
  uint8_t auto_enter_mode;
} DM_MIT_Config_s;

/* 达妙力位混控（PVT）配置 */
typedef struct {
  float v_limit_max; // 速度上限(rad/s)，协议要求不超过100
  float i_limit_max; // 电流标幺上限(0~1.0)
  float kt_out;      // 扭矩到相电流的换算系数，缺省可为0留作外部使用
} DM_PVT_Config_s;

/* 电机控制设置,包括闭环类型,反转标志和反馈来源 */
typedef struct {
  Closeloop_Type_e outer_loop_type; // 最外层的闭环,未设置时默认为最高级的闭环
  Closeloop_Type_e close_loop_type; // 使用几个闭环(串级)
  Motor_Reverse_Flag_e motor_reverse_flag;       // 是否反转
  Feedback_Reverse_Flag_e feedback_reverse_flag; // 反馈是否反向
  Feedback_Source_e angle_feedback_source;       // 角度反馈类型
  Feedback_Source_e speed_feedback_source;       // 速度反馈类型
  Feedfoward_Type_e feedforward_flag;            // 前馈标志
  Controller_Type_e controller_type;             // 控制器类型 (PID/LQR/SMC)

} Motor_Control_Setting_s;

/* 电机控制器,包括其他来源的反馈数据指针,3环控制器和电机的参考输入*/
// 后续增加前馈数据指针
typedef struct {
  float *other_angle_feedback_ptr; // 其他反馈来源的反馈数据指针
  float *other_speed_feedback_ptr;
  float *speed_feedforward_ptr;
  float *current_feedforward_ptr;

  PIDInstance current_PID;
  PIDInstance speed_PID;
  PIDInstance angle_PID;

  LQRInstance LQR; // LQR控制器实例
  SMC_ControllerInstance smc;
  uint32_t smc_dwt_cnt;

  float pid_ref; // 将会作为每个环的输入和输出顺次通过串级闭环
  float output;  // 控制器最终输出（供LQR等直接控制使用）
  Controller_Effort_Output_s controller_output;
} Motor_Controller_s;

/* 电机类型枚举 */
typedef enum {
  MOTOR_TYPE_NONE = 0,
  GM6020,
  M3508,
  M2006,
} Motor_Type_e;

/**
 * @brief 电机控制器初始化结构体,包括三环PID的配置以及两个反馈数据来源指针
 *        如果不需要某个控制环,可以不设置对应的pid config
 *        需要其他数据来源进行反馈闭环,不仅要设置这里的指针还需要在Motor_Control_Setting_s启用其他数据来源标志
 */
typedef struct {
  float *other_angle_feedback_ptr; // 角度反馈数据指针,注意电机使用total_angle
  float *other_speed_feedback_ptr; // 速度反馈数据指针,单位为angle per sec

  float *speed_feedforward_ptr;   // 速度前馈数据指针
  float *current_feedforward_ptr; // 电流前馈数据指针

  PID_Init_Config_s current_PID;
  PID_Init_Config_s speed_PID;
  PID_Init_Config_s angle_PID;
  LQR_Init_Config_s LQR; // LQR控制器初始化配置
  SMC_ControllerInitConfig_s smc; // 滑模控制器初始化配置
} Motor_Controller_Init_s;

/* 用于初始化CAN电机的结构体,各类电机通用 */
typedef struct {
  Motor_Controller_Init_s controller_param_init_config;
  Motor_Control_Setting_s controller_setting_init_config;
  Motor_Physical_Param_s physical_param;
  DM_MIT_Config_s mit_config; // 达妙 MIT 配置, 其他电机可忽略
  DM_PVT_Config_s pvt_config; // 达妙 PVT 配置，可选
  Motor_Type_e motor_type;
  CAN_Init_Config_s can_init_config;
} Motor_Init_Config_s;

#endif // !MOTOR_DEF_H
