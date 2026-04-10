/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "math.h"
#include "stdint.h"

// 开发板角色固定为底盘板，当前工作树不再保留单板/云台板兼容入口
#define CHASSIS_BOARD // 底盘板

// 视觉通信默认走虚拟串口
#define VISION_USE_VCP
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD                                                  \
  5646
#define YAW_ECD_GREATER_THAN_4096                                              \
  1
#define PITCH_HORIZON_ECD                                                      \
  0
#define PITCH_MAX_ANGLE                                                        \
  53.5f
#define PITCH_MIN_ANGLE                                                        \
  -15.5f

#define ONE_BULLET_DELTA_ANGLE 60
#define REDUCTION_RATIO_LOADER 51.0f
#define NUM_PER_CIRCLE 6

#define HEAT_PER_SHOT_D 100.0f
#define FEEDFORWARD_T_TARGET_S 1.0f
#define SHOOT_RATE_MIN 0.5f
#define SHOOT_RATE_MAX 3.0f
#define SHOOT_RATE_SAFE 1.0f

#define WHEEL_BASE 0.412f
#define TRACK_WIDTH 0.412f
#define CENTER_GIMBAL_OFFSET_X                                                 \
  0.0f
#define CENTER_GIMBAL_OFFSET_Y                                                 \
  0.0f
#define RADIUS_WHEEL 0.077f // 轮子半径(单位m,注意不是直径)
#define REDUCTION_RATIO_WHEEL                                                  \
  19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
#define CHASSIS_MASS 25.0f
#define GRAVITY_ACCEL 9.81f      // 重力加速度,单位:m/s^2
#define DIST_CG_FRONT_AXLE 0.28f
#define DIST_CG_REAR_AXLE 0.28f
#define CG_HEIGHT 0.132f
// 底盘跟随就近回中参数
#define CHASSIS_FOLLOW_ALLOW_FLIP 0
#define CHASSIS_FOLLOW_MAX_ERR 135.0f
// 键盘控制相关参数


#define KEYBOARD_CMD_MAX_SPEED_X 1.0f
#define KEYBOARD_CMD_MAX_SPEED_Y 1.0f


#define KEYBOARD_RAMP_ACCEL 3.0f
// 兼容旧宏名：键盘速度最大值（如有旧调用）
#define CHASSIS_KB_MAX_SPEED_X KEYBOARD_CMD_MAX_SPEED_X
#define CHASSIS_KB_MAX_SPEED_Y KEYBOARD_CMD_MAX_SPEED_Y


#define KEYBOARD_RAMP_DECEL 4.0f

// 可以设置得非常大, 实现凌厉的转向和制动
#define KEYBOARD_RAMP_BRAKE_DECEL 6.0f
/**
 * @brief M3508 电机扭矩到 CAN 指令值的换算系数
 * @note C620 电调 -20A~20A 对应 -16384~16384
 */
#define M3508_TORQUE_TO_CURRENT_CMD_COEFF 2730.67f

/* ----------------力控策略相关物理参数---------------- */
/**
 * @brief M3508电机转矩常数 (N·m/A)
 * @note 根据官方参数表：




 */
#define M3508_TORQUE_CONSTANT 0.3f

/**


 */
#define M3508_CMD_TO_CURRENT_COEFF (20.0f / 16384.0f)

/**
 * @brief 力控策略摩擦补偿参数
 */

#define FRICTION_STATIC_CURRENT 0.75f

#define FRICTION_DYNAMIC_CURRENT 0.15f

// 最大控制力 (N)
#define MAX_CONTROL_FORCE 300.0f

#define MAX_CONTROL_TORQUE 100.0f

#define MAX_WHEEL_CURRENT 20.0f

/* ----------------底盘运行时配置结构体---------------- */
/**
 * @brief 底盘遥控器控制配置
 */
typedef struct {
  float max_linear_speed;  // m/s - 最大线速度
  float max_angular_speed; // rad/s - 最大角速度
} Chassis_RC_Config_t;

/**
 * @brief 底盘力控策略配置
 */
typedef struct {
  float torque_feedforward_coeff;   // N·m/(rad/s) - 扭矩前馈系数
  float friction_threshold_omega;
  float wheel_speed_feedback_coeff;
  float omega_error_lpf_alpha;      // 角速度误差滤波系数
  float omega_threshold;
} Chassis_Force_Control_Config_t;

/**
 * @brief 底盘运动学配置
 */
typedef struct {
  float velocity_lpf_alpha; // 速度估算滤波系数
  float speed_deadband;
  float rotate_speed;       // 小陀螺模式旋转速度 (rad/s)
} Chassis_Kinematics_Config_t;

/**
 * @brief 底盘完整配置集合
 */
typedef struct {
  Chassis_RC_Config_t rc;
  Chassis_Force_Control_Config_t force;   // 力控配置
  Chassis_Kinematics_Config_t kinematics;
} Chassis_Runtime_Config_t;

#define GYRO2GIMBAL_DIR_YAW                                                    \
  1
#define GYRO2GIMBAL_DIR_PITCH                                                  \
  1
#define GYRO2GIMBAL_DIR_ROLL                                                   \
  1


#if defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD)
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1)

/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */

typedef enum {
  ROBOT_STOP = 0,
  ROBOT_READY,
} Robot_Status_e;


typedef enum {
  APP_OFFLINE = 0,
  APP_ONLINE,
  APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**

 *
 */
typedef enum {
  CHASSIS_ZERO_FORCE = 0,
  CHASSIS_ROTATE,
  CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
  CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

typedef enum {
  CHASSIS_SAFETY_STATUS_NONE = 0x00U,
  CHASSIS_SAFETY_STATUS_DEAD = 0x01U,
  CHASSIS_SAFETY_STATUS_READY = 0x02U,
} chassis_safety_status_e;

// 云台模式设置
typedef enum {
  GIMBAL_ZERO_FORCE = 0,
  GIMBAL_FREE_MODE, // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
  GIMBAL_GYRO_MODE, // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
  GIMBAL_SYS_ID_CHIRP, // 云台正弦扫频辨识模式,用于系统辨识和PID整定
} gimbal_mode_e;

// 发射模式设置
typedef enum {
  SHOOT_OFF = 0,
  SHOOT_ON,
} shoot_mode_e;
typedef enum {
  FRICTION_OFF = 0,
  FRICTION_ON,
} friction_mode_e;

typedef enum {
  LOAD_STOP = 0,  // 停止发射
  LOAD_REVERSE,   // 反转
  LOAD_1_BULLET,  // 单发
  LOAD_3_BULLET,  // 三发
  LOAD_BURSTFIRE, // 连发
} loader_mode_e;

typedef enum {
  BULLET_SPEED_NONE = 0,
  BIG_AMU_10 = 10,
  SMALL_AMU_15 = 15,
  BIG_AMU_16 = 16,
  SMALL_AMU_18 = 18,
  SMALL_AMU_30 = 30,
} Bullet_Speed_e;


typedef struct { // 功率控制
  float chassis_power_mx;
} Chassis_Power_Data_s;

/**
 * @brief CMD 控制数据与应用反馈结构体
 */

typedef struct {
  // 控制部分
  float vx;                // 前进方向速度
  float vy;                // 横移方向速度
  float wz;                // 旋转速度
  float offset_angle;      // 底盘和归中位置的夹角
  float near_center_error;
  chassis_mode_e chassis_mode;
  int chassis_speed_buff;
  uint8_t ui_friction_on;
  uint8_t ui_autoaim_enabled;
  uint8_t ui_fire_allow;
  uint8_t ui_stuck_active;
  uint8_t ui_loader_mode;
  uint16_t ui_refresh_request_seq;
  // UI部分
  //  ...

} Chassis_Ctrl_Cmd_s;


typedef struct { // 云台角度控制
  float yaw;
  float pitch;
  float chassis_rotate_wz;

  gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;


typedef struct {
  shoot_mode_e shoot_mode;
  loader_mode_e load_mode;
  friction_mode_e friction_mode;
  Bullet_Speed_e bullet_speed;
  uint8_t rest_heat;
  float shoot_rate;
} Shoot_Ctrl_Cmd_s;


/**

 *
 */


typedef struct {
#if defined(CHASSIS_BOARD) ||                                                  \
    defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
                          // attitude_t chassis_imu_data;
#endif
  // 后续增加底盘的真实速度
  // float real_vx;
  // float real_vy;
  float real_wz; // 底盘实际角速度 [rad/s]
  uint8_t chassis_safety_status; // bit0-阵亡 bit1-底盘ready

  uint8_t referee_online;      // 裁判系统在线标志(1:在线,0:离线)
  uint8_t rest_heat;           // 剩余枪口热量
  Bullet_Speed_e bullet_speed;
  uint16_t chassis_power_limit;  // 裁判系统底盘功率上限
  uint16_t barrel_heat;          // 当前枪口热量
  uint16_t barrel_heat_limit;    // 枪口热量上限
  uint16_t barrel_cooling_value;
  float bullet_speed_limit;      // 弹速上限（当前使用实测弹速近似）

} Chassis_Upload_Data_s;

#pragma pack(1)
typedef struct {
  float vx;
  float vy;
  float wz;
  float offset_angle;
  float near_center_error;
  chassis_mode_e chassis_mode;
} Chassis_Ctrl_Fast_Pkt_s;

typedef struct {
  int32_t chassis_speed_buff;
} Chassis_Ctrl_State_Pkt_s;

typedef struct {
  uint8_t ui_friction_on;
  uint8_t ui_autoaim_enabled;
  uint8_t ui_fire_allow;
  uint8_t ui_stuck_active;
  uint8_t ui_loader_mode;
} Chassis_Ctrl_UI_Pkt_s;

typedef struct {
  uint16_t ui_refresh_request_seq;
} Chassis_Ctrl_Event_Pkt_s;

typedef struct {
  uint8_t referee_online;
  uint8_t rest_heat;
  uint16_t barrel_heat;
  uint16_t barrel_heat_limit;
  uint16_t barrel_cooling_value;
  float bullet_speed_limit;
  float real_wz; // 底盘实际角速度 [rad/s]
  uint8_t chassis_safety_status;
} Chassis_Feed_Fast_Pkt_s;

typedef struct {
  Bullet_Speed_e bullet_speed;
  uint16_t chassis_power_limit;
} Chassis_Feed_State_Pkt_s;
#pragma pack()

// 双板CAN通信单包最大有效负载为60字节，防止结构体膨胀导致越界
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(Chassis_Ctrl_Cmd_s) <= 60,
               "Chassis_Ctrl_Cmd_s too large for CAN comm");
_Static_assert(sizeof(Chassis_Upload_Data_s) <= 60,
               "Chassis_Upload_Data_s too large for CAN comm");
_Static_assert(sizeof(Chassis_Ctrl_Fast_Pkt_s) <= 60,
               "Chassis_Ctrl_Fast_Pkt_s too large for CAN comm");
_Static_assert(sizeof(Chassis_Ctrl_State_Pkt_s) <= 60,
               "Chassis_Ctrl_State_Pkt_s too large for CAN comm");
_Static_assert(sizeof(Chassis_Ctrl_UI_Pkt_s) <= 60,
               "Chassis_Ctrl_UI_Pkt_s too large for CAN comm");
_Static_assert(sizeof(Chassis_Ctrl_Event_Pkt_s) <= 60,
               "Chassis_Ctrl_Event_Pkt_s too large for CAN comm");
_Static_assert(sizeof(Chassis_Feed_Fast_Pkt_s) <= 60,
               "Chassis_Feed_Fast_Pkt_s too large for CAN comm");
_Static_assert(sizeof(Chassis_Feed_State_Pkt_s) <= 60,
               "Chassis_Feed_State_Pkt_s too large for CAN comm");
#else
typedef char
    chassis_ctrl_cmd_size_check[(sizeof(Chassis_Ctrl_Cmd_s) <= 60) ? 1 : -1];
typedef char
    chassis_upload_data_size_check[(sizeof(Chassis_Upload_Data_s) <= 60) ? 1
                                                                         : -1];
typedef char
    chassis_ctrl_fast_pkt_size_check[(sizeof(Chassis_Ctrl_Fast_Pkt_s) <= 60)
                                         ? 1
                                         : -1];
typedef char
    chassis_ctrl_state_pkt_size_check[(sizeof(Chassis_Ctrl_State_Pkt_s) <= 60)
                                          ? 1
                                          : -1];
typedef char
    chassis_ctrl_ui_pkt_size_check[(sizeof(Chassis_Ctrl_UI_Pkt_s) <= 60) ? 1
                                                                          : -1];
typedef char
    chassis_ctrl_event_pkt_size_check[(sizeof(Chassis_Ctrl_Event_Pkt_s) <= 60)
                                          ? 1
                                          : -1];
typedef char
    chassis_feed_fast_pkt_size_check[(sizeof(Chassis_Feed_Fast_Pkt_s) <= 60)
                                         ? 1
                                         : -1];
typedef char
    chassis_feed_state_pkt_size_check[(sizeof(Chassis_Feed_State_Pkt_s) <= 60)
                                          ? 1
                                          : -1];
#endif

typedef struct {
  attitude_t gimbal_imu_data;
  uint16_t yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

typedef struct {
  // code to go here
  // ...
} Shoot_Upload_Data_s;

/* ----------------系统辨识任务相关定义----------------*/
// 底盘系统辨识控制指令（cmd任务发布，系统辨识任务订阅）
typedef struct {
  uint8_t enable;
  uint8_t target_motor;
} Chassis_SysID_Ctrl_Cmd_s;


typedef struct {
  float step_input;      // 方波输入信号（电流CAN指令值）
  float motor_output;
  float time_elapsed;
  uint8_t is_finished;   // 辨识完成标志
  uint8_t step_state;
  uint32_t call_counter; // 任务调用次数
  float actual_dt;       // 实际测量的dt [s]
  float task_freq;       // 实际任务频率 [Hz]
} Chassis_SysID_Feedback_s;

#pragma pack()

#endif // !ROBOT_DEF_H



