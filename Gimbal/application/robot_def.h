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
#include "vision_comm.h"

// 开发板类型定义，编译时只保留一个
// #define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD // 底盘板
#define GIMBAL_BOARD   // 云台板

/* -------------------------视觉通信链路选择-------------------------
 */
/**
 * @brief 视觉链路选择开关（编译期）
 * @note 默认使用 CAN，如需改回 USB
 *       虚拟串口模式请将 VISION_LINK_TYPE 设为 VISION_LINK_VCP
 */
#define VISION_LINK_VCP 0
#define VISION_LINK_CAN 1

#ifndef VISION_LINK_TYPE
#define VISION_LINK_TYPE VISION_LINK_CAN
#endif

/* 视觉链路参数仅在 CAN 模式下生效 */
// 控制周期参数
#define ROBOT_CTRL_PERIOD_S 0.001f
#define VISION_CTRL_PERIOD_S                                                   \
  0.001f

// 云台参数
#define YAW_CHASSIS_ALIGN_ECD                                                  \
  5695
#define PITCH_MAX_ANGLE                                                        \
  39.5f
#define PITCH_MIN_ANGLE                                                        \
  -10.5f


#define PITCH_GRAVITY_K 0.0f
#define PITCH_GRAVITY_GAMMA_DEG 0.0f
// 发射参数
#define REDUCTION_RATIO_LOADER                                                 \
  51.0f
#define LOAD_ANGLE_PER_BULLET                                                  \
  60 // 拨盘输出轴每发弹丸转动角度（机械设计值）
#define ONE_BULLET_DELTA_ANGLE                                                 \
  (LOAD_ANGLE_PER_BULLET *                                                     \
   REDUCTION_RATIO_LOADER)
                           // 减速比 = 60×51 = 3060°
#define NUM_PER_CIRCLE 6

#define HEAT_PER_SHOT_D 1.0f
#define FEEDFORWARD_T_TARGET_S 1.0f
#define SHOOT_RATE_MIN 0.5f
#define SHOOT_RATE_MAX 3.0f
#define SHOOT_RATE_SAFE 1.0f
#define SHOOT_FIXED_BULLET_SPEED 15.0f
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 560  // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 330 // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X                                                 \
  0 // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y                                                 \
  0 // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL                                                           \
  0.077f // 轮子半径(单位m,注意不是直径)
#define REDUCTION_RATIO_WHEEL                                                  \
  19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
#define CHASSIS_MASS 17.0f
#define GRAVITY_ACCEL 9.81f // 重力加速度,单位m/s^2,用于功率计算
#define DIST_CG_FRONT_AXLE 280
#define DIST_CG_REAR_AXLE 280
#define CG_HEIGHT 132
// 底盘跟随就近回中参数
#define CHASSIS_FOLLOW_ALLOW_FLIP                                              \
  1
#define CHASSIS_FOLLOW_FLIP_THRESHOLD 90.0f
#define CHASSIS_FOLLOW_MAX_ERR                                                 \
  135.0f
// 键盘控制相关参数
//  当前键盘路径直接使用归一化目标值[-1,




#define KEYBOARD_RAMP_ACCEL 2.0f

// dt=5ms 时，每周期约减小 0.015，松手回零约需 0.33s
#define KEYBOARD_RAMP_DECEL 3.0f

// dt=5ms 时，每周期约变化 0.02，换向过零约需 0.25s
#define KEYBOARD_RAMP_BRAKE_DECEL 4.0f
/**
 * @brief M3508 电机扭矩到 CAN 指令值的换算系数
 * @note C620 电调 -20A~20A 对应 -16384~16384
 */
#define M3508_TORQUE_TO_CURRENT_CMD_COEFF 2730.67f

#define GYRO2GIMBAL_DIR_YAW                                                    \
  1
#define GYRO2GIMBAL_DIR_PITCH                                                  \
  1
#define GYRO2GIMBAL_DIR_ROLL                                                   \
  1


#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) ||                          \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||                           \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(                                                                  \
    1)

/**
 * @brief
 * 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
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
 * @brief

 *
 */
typedef enum {
  CHASSIS_ZERO_FORCE = 0,
  CHASSIS_ROTATE,
  CHASSIS_NO_FOLLOW,      // 不跟随，允许全向平移
  CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

// 云台模式设置
typedef enum {
  GIMBAL_ZERO_FORCE = 0,
  GIMBAL_GYRO_MODE = 2,
  GIMBAL_AUTOAIM_MODE =
      3,
  GIMBAL_LQR_MODE = 4,
  GIMBAL_SYS_ID_CHIRP = 5,
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

// 视觉控制模式设置
typedef enum {
  VISION_MODE_OFF = 0,        // 视觉关闭
  VISION_MODE_AUTO_AIM = 1,   // 自动瞄准
  VISION_MODE_SMALL_BUFF = 2, // 小符
  VISION_MODE_BIG_BUFF = 3,   // 大符
  VISION_MODE_ENERGY_HIT =
      VISION_MODE_SMALL_BUFF,
  VISION_MODE_MANUAL_AIM = 4, // 手动辅助瞄准
} vision_mode_e;


typedef struct { // 功率控制
  float chassis_power_mx;
} Chassis_Power_Data_s;

/**
 * @brief CMD 控制数据与应用反馈结构体
 */

typedef struct {
  // 控制部分
  float vx;           // 前进方向速度
  float vy;           // 横移方向速度
  float wz;           // 旋转速度
  float offset_angle; // 底盘和归中位置的夹角
  float
      near_center_error;
  chassis_mode_e chassis_mode;
  int chassis_speed_buff;
  uint8_t
      vision_is_tracking;
  uint8_t image_online;
  uint8_t image_target_locked;
  uint8_t image_auto_fire_request;
  uint8_t image_should_fire;
  uint16_t image_cmd_seq;          // 图传控制序号
  uint32_t image_ts_ms;
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


  uint8_t
      vision_yaw_direct; // 是否使用视觉Yaw电流直接控制
  float vision_yaw_current;    // 视觉Yaw电流指令 [raw]
  uint8_t vision_pitch_direct; // 是否使用视觉Pitch目标
  float vision_pitch_ref;      // 视觉Pitch目标角度 [rad]
} Gimbal_Ctrl_Cmd_s;


typedef struct {
  shoot_mode_e shoot_mode;
  loader_mode_e load_mode;
  friction_mode_e friction_mode;
  Bullet_Speed_e bullet_speed;
  uint8_t rest_heat;
  float shoot_rate;
} Shoot_Ctrl_Cmd_s;


typedef struct {
  vision_mode_e vision_mode; // 视觉控制模式
  uint8_t allow_auto_fire;   // 允许自动射击
  float bullet_speed_limit;  // 实时上传给视觉端的当前弹速
  float manual_yaw_offset;
  float manual_pitch_offset;
} Vision_Ctrl_Cmd_s;


/**
 * @brief

 *
 */

// 常规扩展命令桥接契约版本与能力位（Chassis

#define REGULAR_BRIDGE_VERSION 1u
#define REGULAR_BRIDGE_CAP_0303 (1u << 0)
#define REGULAR_BRIDGE_CAP_0305 (1u << 1)
#define REGULAR_BRIDGE_CAP_0307 (1u << 2)
#define REGULAR_BRIDGE_CAP_0308 (1u << 3)
#define REGULAR_BRIDGE_CAP_MASK                                                \
  (REGULAR_BRIDGE_CAP_0303 | REGULAR_BRIDGE_CAP_0305 |                         \
   REGULAR_BRIDGE_CAP_0307 | REGULAR_BRIDGE_CAP_0308)
#define REGULAR_BRIDGE_VALID_0303 REGULAR_BRIDGE_CAP_0303
#define REGULAR_BRIDGE_VALID_0305 REGULAR_BRIDGE_CAP_0305
#define REGULAR_BRIDGE_VALID_0307 REGULAR_BRIDGE_CAP_0307
#define REGULAR_BRIDGE_VALID_0308 REGULAR_BRIDGE_CAP_0308

typedef struct {
#if defined(CHASSIS_BOARD) ||                                                  \
    defined(                                                                   \
        GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
                      // attitude_t chassis_imu_data;
#endif
  // 后续增加底盘的真实速度
  // float real_vx;
  // float real_vy;
  // float real_wz;

  uint8_t referee_online;      // 裁判系统在线标志(1:在线,0:离线)
  uint16_t current_hp;
  uint16_t buffer_energy;      // 裁判系统功率缓冲能量
  uint8_t rest_heat;           // 剩余枪口热量
  Bullet_Speed_e bullet_speed;
  Enemy_Color_e enemy_color;   // 0 for blue, 1 for red

  uint8_t
      regular_online;
  uint8_t robot_id;              // 裁判系统机器人ID
  uint16_t chassis_power_limit;  // 裁判系统底盘功率上限
  uint16_t barrel_heat;          // 当前枪口热量
  uint16_t barrel_heat_limit;    // 枪口热量上限
  uint16_t barrel_cooling_value;
  float bullet_speed_limit; // 弹速上限（当前使用实测弹速近似）
  uint32_t referee_ts_ms;

  uint8_t regular_bridge_version;    // 桥接协议版本
  uint8_t regular_bridge_capability;
  uint8_t regular_cmd_valid_mask;
  uint8_t
      regular_cmd_seq;

} Chassis_Upload_Data_s;

// 双板CAN通信单包最大有效负载为60字节，防止结构体膨胀导致越界
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(Chassis_Ctrl_Cmd_s) <= 60,
               "Chassis_Ctrl_Cmd_s too large for CAN comm");
_Static_assert(sizeof(Chassis_Upload_Data_s) <= 60,
               "Chassis_Upload_Data_s too large for CAN comm");
#else
typedef char
    chassis_ctrl_cmd_size_check[(sizeof(Chassis_Ctrl_Cmd_s) <= 60) ? 1 : -1];
typedef char
    chassis_upload_data_size_check[(sizeof(Chassis_Upload_Data_s) <= 60) ? 1
                                                                         : -1];
#endif

typedef struct {
  attitude_t gimbal_imu_data;
  uint16_t yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

typedef struct {
  uint8_t loader_jam_state; // 拨盘卡弹状态机
  uint8_t
      loader_jam_active; // 拨盘是否处于卡弹恢复流程
  uint16_t loader_jam_recovery_count; // 累计卡弹恢复次数
  int16_t
      loader_real_current; // 拨盘当前反馈电流（电调原始量纲）
  float loader_speed_aps;   // 拨盘当前角速度 [deg/s]
  float loader_total_angle;
} Shoot_Upload_Data_s;


typedef struct {
  uint8_t vision_valid;  // 视觉数据有效标志
  uint8_t target_locked; // 目标锁定标志
  uint8_t should_fire;   // 建议射击标志


  uint8_t vision_takeover; // 视觉接管标志
  float yaw_current_cmd;   // Yaw电流指令
                           // [raw]（视觉双环输出）
  float
      pitch_ref_limited;


  float yaw;   // 原始目标yaw角度 [rad]
  float pitch; // 原始目标pitch角度 [rad]
} Vision_Upload_Data_s;

/* ----------------系统辨识任务相关定义----------------*/
// 云台系统辨识轴选择枚举
typedef enum {
  SYSID_AXIS_YAW = 0,
  SYSID_AXIS_PITCH = 1,
  SYS_ID_DISABLED_AXIS = 2
} SysID_TargetAxis_e;

// 云台系统辨识控制指令（gimbal任务发布，系统辨识任务订阅）
typedef struct {
  uint8_t
      enable;
  uint8_t axis; // 目标轴：0-Yaw 1-Pitch
  float
      yaw_ref;
  float
      pitch_ref;
} SysID_Ctrl_Cmd_s;


typedef struct {
  float step_input; // 方波输入信号（电流指令）
  float
      motor_output;
  float time_elapsed;
  uint8_t is_finished;   // 辨识完成标志
  uint8_t step_state;
  uint32_t call_counter; // 任务调用次数
  float actual_dt;       // 实际测量的dt [s]
  float task_freq;       // 实际任务频率 [Hz]
} SysID_Feedback_s;

#pragma pack()
               // pack(1)

#endif // !ROBOT_DEF_H
