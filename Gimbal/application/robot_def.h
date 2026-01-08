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
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "math.h"
#include "stdint.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义!
 */
// #define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD // 底盘板
#define GIMBAL_BOARD // 云台板
/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾
 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD                                                  \
  5642 // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE                                                        \
  38.0f // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE                                                        \
  -18.5f // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// Pitch 重力补偿参数（参考 derivation_process.md 2.2）
// k = m * g * r，单位 N·m，gamma 为重心相对枪线的偏置角（度）
#define PITCH_GRAVITY_K 0.0f
#define PITCH_GRAVITY_GAMMA_DEG 0.0f
// 发射参数
#define REDUCTION_RATIO_LOADER 19.0f // M3508拨盘电机的减速比
#define LOAD_ANGLE_PER_BULLET 60     // 拨盘输出轴每发弹丸转动角度（机械设计值）
#define ONE_BULLET_DELTA_ANGLE                                                 \
  (LOAD_ANGLE_PER_BULLET *                                                     \
   REDUCTION_RATIO_LOADER) // 电机轴角度 = 输出轴角度 × 减速比 = 60×51 = 3060°
#define NUM_PER_CIRCLE 6   // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 560  // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 330 // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X                                                 \
  0 // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y                                                 \
  0 // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 0.077f // 轮子半径(单位m,注意不是直径)
#define REDUCTION_RATIO_WHEEL                                                  \
  19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
#define CHASSIS_MASS 17.0f     // 机器人整备质量,单位kg,用于功率计算
#define GRAVITY_ACCEL 9.81f    // 重力加速度,单位m/s^2,用于功率计算
#define DIST_CG_FRONT_AXLE 280 // 重心距前轴距离,单位mm
#define DIST_CG_REAR_AXLE 280  // 重心距后轴距离,单位mm
#define CG_HEIGHT 132          // 重心距底盘中心高度,单位mm
// 底盘跟随就近回中参数
#define CHASSIS_FOLLOW_ALLOW_FLIP 1 // 是否允许车头翻转(0:不允许, 1:允许)
#define CHASSIS_FOLLOW_FLIP_THRESHOLD 90.0f // 车头翻转触发阈值(度)
#define CHASSIS_FOLLOW_MAX_ERR 135.0f       // 最大允许误差(度),避免控制量过大
// 键盘控制相关参数
//  键盘按下时的最大目标指令值 (遥控器摇杆最大值为660,
//  这里可以参考设置或设定的更大)
#define KEYBOARD_CMD_MAX_SPEED_X 16000.0f
#define KEYBOARD_CMD_MAX_SPEED_Y 16000.0f
// 加速度 (单位: 指令值/秒)
// 值越大, 响应越快。可以从 20000 开始尝试
#define KEYBOARD_RAMP_ACCEL 12000.0f
// 兼容旧宏名：键盘速度最大值（如有旧调用）
#define CHASSIS_KB_MAX_SPEED_X KEYBOARD_CMD_MAX_SPEED_X
#define CHASSIS_KB_MAX_SPEED_Y KEYBOARD_CMD_MAX_SPEED_Y
// 减速度 (单位: 指令值/秒)
// 通常可以设置得比ACCEL大, 实现更快的刹车
#define KEYBOARD_RAMP_DECEL 15000.0f
// 反向制动减速度 (单位: 指令值/秒)
// 可以设置得非常大, 实现凌厉的转向和制动
#define KEYBOARD_RAMP_BRAKE_DECEL 20000.0f
/**
 * @brief M3508电机扭矩到CAN指令值的转换系数,1N·m对应2730.67的CAN指令值
 * @note 根据官方数据：额定扭矩3N·m @ 10A电流, C620电调 -20A~20A 对应
 * -16384~16384                                计算公式: (10A / 20A * 16384) /
 * 3N·m = 8192 / 3 ≈ 2730.67
 */
#define M3508_TORQUE_TO_CURRENT_CMD_COEFF 2730.67f

#define GYRO2GIMBAL_DIR_YAW                                                    \
  1 // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH                                                  \
  1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL                                                   \
  1 // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) ||                          \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||                           \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum {
  ROBOT_STOP = 0,
  ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum {
  APP_OFFLINE = 0,
  APP_ONLINE,
  APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum {
  CHASSIS_ZERO_FORCE = 0,    // 电流零输入
  CHASSIS_ROTATE,            // 小陀螺模式
  CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
  CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

// 云台模式设置
typedef enum {
  GIMBAL_ZERO_FORCE = 0, // 电流零输入
  GIMBAL_FREE_MODE, // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
  GIMBAL_GYRO_MODE, // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
  GIMBAL_LQR_MODE, // 云台LQR控制模式,Yaw轴使用LQR最优控制,Pitch轴使用PID(或LQR)
  GIMBAL_SYS_ID_CHIRP, // 云台正弦扫频辨识模式,用于系统辨识和PID整定
} gimbal_mode_e;

// 发射模式设置
typedef enum {
  SHOOT_OFF = 0,
  SHOOT_ON,
} shoot_mode_e;
typedef enum {
  FRICTION_OFF = 0, // 摩擦轮关闭
  FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum {
  LID_OPEN = 0, // 弹舱盖打开
  LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

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
      VISION_MODE_SMALL_BUFF, // 兼容旧能量机关枚举，映射为小符
  VISION_MODE_MANUAL_AIM = 4, // 手动辅助瞄准
} vision_mode_e;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct { // 功率控制
  float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅----------------
 */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct {
  // 控制部分
  float vx;                // 前进方向速度
  float vy;                // 横移方向速度
  float wz;                // 旋转速度
  float offset_angle;      // 底盘和归中位置的夹角
  float near_center_error; // 就近回中误差(考虑翻转优化后,供PID使用)
  chassis_mode_e chassis_mode;
  int chassis_speed_buff;
  // UI部分
  //  ...

} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct { // 云台角度控制
  float yaw;
  float pitch;
  float chassis_rotate_wz;

  gimbal_mode_e gimbal_mode;
  float yaw_vel_ff;   // 未启用的yaw速度前馈(弧度/秒)
  float yaw_acc_ff;   // 未启用的yaw加速度前馈(弧度/秒^2)
  float pitch_vel_ff; // 未启用的pitch速度前馈(弧度/秒)
  float pitch_acc_ff; // 未启用的pitch加速度前馈(弧度/秒^2)
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct {
  shoot_mode_e shoot_mode;
  loader_mode_e load_mode;
  lid_mode_e lid_mode;
  friction_mode_e friction_mode;
  Bullet_Speed_e bullet_speed; // 弹速枚举
  uint8_t rest_heat;
  float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

// cmd发布的视觉控制数据,由vision订阅
typedef struct {
  vision_mode_e vision_mode; // 视觉控制模式
  uint8_t allow_auto_fire;   // 允许自动射击
  float manual_yaw_offset;   // 手动微调yaw偏移量
  float manual_pitch_offset; // 手动微调pitch偏移量
} Vision_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
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
  // float real_wz;

  uint8_t rest_heat;           // 剩余枪口热量
  Bullet_Speed_e bullet_speed; // 弹速限制
  Enemy_Color_e enemy_color;   // 0 for blue, 1 for red

} Chassis_Upload_Data_s;

typedef struct {
  attitude_t gimbal_imu_data;
  uint16_t yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

typedef struct {
  // code to go here
  // ...
} Shoot_Upload_Data_s;

// vision发布的视觉处理数据,由cmd订阅用于融合控制
typedef struct {
  uint8_t vision_valid;  // 视觉数据有效标志
  uint8_t target_locked; // 目标锁定标志
  float yaw;             // 目标yaw角度(弧度)
  float pitch;           // 目标pitch角度(弧度)
  uint8_t should_fire;   // 建议射击标志
  float yaw_vel_ff;      // 目标yaw角速度前馈(弧度/秒)
  float yaw_acc_ff;      // 目标yaw角加速度前馈(弧度/秒^2)
  float pitch_vel_ff;    // 目标pitch角速度前馈(弧度/秒)
  float pitch_acc_ff;    // 目标pitch角加速度前馈(弧度/秒^2)
} Vision_Upload_Data_s;

/* ----------------系统辨识任务相关定义----------------*/
// 云台系统辨识轴选择枚举
typedef enum {
  SYSID_AXIS_YAW = 0,      // 辨识Yaw轴
  SYSID_AXIS_PITCH = 1,    // 辨识Pitch轴
  SYS_ID_DISABLED_AXIS = 2 // 未选择任何轴
} SysID_TargetAxis_e;

// 云台系统辨识控制指令（gimbal任务发布，系统辨识任务订阅）
typedef struct {
  uint8_t enable;  // 使能标志：1-启动辨识，0-停止辨识
  uint8_t axis;    // 目标轴：0-Yaw 1-Pitch
  float yaw_ref;   // Yaw轴位置参考值（用于保持非辨识轴位置）
  float pitch_ref; // Pitch轴位置参考值（用于保持非辨识轴位置）
} SysID_Ctrl_Cmd_s;

// 系统辨识反馈数据（系统辨识任务发布，云台任务订阅）
typedef struct {
  float step_input;      // 方波输入信号（电流指令）
  float motor_output;    // 电机输出反馈（陀螺仪角速度）
  float time_elapsed;    // 已运行时间 [s]
  uint8_t is_finished;   // 辨识完成标志
  uint8_t step_state;    // 当前阶跃状态
  uint32_t call_counter; // 任务调用次数
  float actual_dt;       // 实际测量的dt [s]
  float task_freq;       // 实际任务频率 [Hz]
} SysID_Feedback_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H
