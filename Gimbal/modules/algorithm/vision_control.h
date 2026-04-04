/**
 * @file vision_control.h
 * @brief 视觉控制算法模块接口
 * @note Algorithm层：提供Yaw双环控制与Pitch限速参考生成
 */

#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

#include <stdint.h>
#include "bsp_dwt.h"

/* ==================== 参数结构体 ==================== */
typedef struct {
  // Yaw位置环参数
  float yaw_pos_kp; // [rad/s per rad] 位置环比例增益
  float yaw_pos_ki; // [rad/s per rad*s] 位置环积分增益
  // Yaw速度环参数
  float yaw_rate_kp;     // [raw per rad/s] 速度环比例增益
  float yaw_rate_ki;     // [raw per rad/s*s] 速度环积分增益
  float yaw_rate_max;    // [rad/s] 角速度参考限幅
  float yaw_current_max; // [raw] 电流指令限幅
  // Pitch参考生成参数（视觉接管专用）
  float pitch_pos_kp;          // [rad/s per rad] Pitch位置误差比例系数
  float pitch_rate_max;        // [rad/s] Pitch参考角速度限幅
  float pitch_err_deadband_deg; // [deg] Pitch误差死区
  float pitch_target_lpf_k;    // [1] 0表示关闭低通，(0,1] 越大越跟手
  float pitch_max_angle;       // [deg] Pitch轴最大角度限位
  float pitch_min_angle;       // [deg] Pitch轴最小角度限位
  // 前馈参数
  float yaw_vel_ff;       // [1] Yaw速度前馈系数，作用于位置环输出
  float pitch_vel_ff;     // [1] Pitch速度前馈系数，作用于角速度参考
  float pitch_vel_ff_max; // [rad/s] Pitch速度前馈项限幅
} VisionCtrlParams_s;

/* ==================== 控制器状态 ==================== */
typedef struct {
  uint8_t inited;
  float pos_i;  // 位置环积分项
  float rate_i; // 速度环积分项
} VisionYawDualLoop_s;

typedef struct {
  uint8_t inited;
  float ref_rad;             // 当前参考值
  float target_rad_filtered; // 低通后的目标值
} VisionPitchRefGen_s;

typedef struct {
  VisionYawDualLoop_s yaw;
  VisionPitchRefGen_s pitch;
  uint32_t DWT_CNT;  // DWT计数器，用于计算时间间隔
  float dt;          // 计算周期 [s]
} VisionCtrlState_s;

/* ==================== 输入/输出结构体 ==================== */
typedef struct {
  float yaw_target_rad;
  float pitch_target_rad;
  float yaw_angle_rad;
  float yaw_gyro_rad_s;
  float pitch_feedback_rad;
  // 视觉提供的目标速度（用于前馈）
  float target_yaw_vel;   // [rad/s] 目标Yaw角速度
  float target_pitch_vel; // [rad/s] 目标Pitch角速度
} VisionCtrlInput_s;

typedef struct {
  float yaw_current_cmd;
  float pitch_ref_limited;
} VisionCtrlOutput_s;

/* ==================== 接口函数 ==================== */
void VisionCtrlInit(VisionCtrlState_s *state);
void VisionCtrlReset(VisionCtrlState_s *state, const VisionCtrlParams_s *params,
                     float pitch_feedback_rad);
void VisionCtrlStep(VisionCtrlState_s *state, const VisionCtrlParams_s *params,
                    const VisionCtrlInput_s *input, VisionCtrlOutput_s *output);

#endif // VISION_CONTROL_H
