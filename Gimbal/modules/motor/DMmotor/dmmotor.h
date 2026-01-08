#ifndef DMMOTOR_H
#define DMMOTOR_H
#include "bsp_can.h"
#include "controller.h"
#include "daemon.h"
#include "motor_def.h"
#include <stdint.h>

#define DM_MOTOR_CNT 4

#define DM_P_MIN (-12.5f)
#define DM_P_MAX 12.5f
#define DM_V_MIN (-45.0f)
#define DM_V_MAX 45.0f
#define DM_T_MIN (-18.0f)
#define DM_T_MAX 18.0f
#define DM_KP_MAX 500.0f
#define DM_KD_MAX 5.0f

typedef enum {
  DM_MODE_MIT = 1,
  DM_MODE_POSVEL = 2,
  DM_MODE_VEL = 3,
  DM_MODE_PVT = 4,
} DM_Mode_e;

typedef struct {
  uint8_t id;
  uint8_t state;
  float velocity;
  float last_position;
  float position;
  float torque;
  float T_Mos;
  float T_Rotor;
  int32_t total_round;
} DM_Motor_Measure_s;

typedef struct {
  uint16_t position_des;
  uint16_t velocity_des;
  uint16_t torque_des;
  uint16_t Kp;
  uint16_t Kd;
} DMMotor_Send_s;

typedef struct {
  DM_Motor_Measure_s measure;
  Motor_Control_Setting_s motor_settings;
  PIDInstance current_PID;
  PIDInstance speed_PID;
  PIDInstance angle_PID;
  DM_MIT_Limit_s limit; // 当前电机的 MIT 量程配置（未配置时退回默认）
  DM_Mode_e mode;       // 当前电机的控制模式
  DM_PVT_Config_s pvt_cfg;
  float *other_angle_feedback_ptr;
  float *other_speed_feedback_ptr;
  float *speed_feedforward_ptr;
  float *current_feedforward_ptr;
  float pid_ref; // 保留兼容：raw MIT 模式使用
  float angle_ref_deg;
  float torque_ff_cmd;
  float kd_cmd;
  uint8_t use_pid_path; // 1: module 内部 PID 角度->速度->力矩
  uint8_t use_raw_mit;  // 1: 使用 DMMotorSetMITTargets 下发的原始 MIT 目标
  float mit_default_kp;
  float mit_default_kd;

  /* MIT目标（位置/速度/力矩/KP/KD） */
  float mit_target_angle;
  float mit_target_velocity;
  float mit_target_torque;
  float mit_target_kp;
  float mit_target_kd;

  /* 直接MIT速度/阻尼/力矩指令（位置/刚度锁零） */
  float mit_direct_velocity;
  float mit_direct_torque_ff;
  float mit_direct_kd;
  uint8_t mit_use_direct_velocity;
  float pvt_pos;
  float pvt_v_lim;
  float pvt_i_lim;
  uint8_t use_pvt;

  Motor_Working_Type_e stop_flag;
  CANInstance *motor_can_instace;
  DaemonInstance *motor_daemon;
  uint32_t tx_base_id;
  uint32_t lost_cnt;
} DMMotorInstance;

typedef enum {
  DM_CMD_MOTOR_MODE = 0xfc,    // 使能,会响应指令
  DM_CMD_RESET_MODE = 0xfd,    // 停止
  DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
  DM_CMD_CLEAR_ERROR = 0xfb    // 清除电机过热错误
} DMMotor_Mode_e;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

void DMMotorSetRef(DMMotorInstance *motor, float angle_deg, float torque_ff);
void DMMotorSetMITTargets(DMMotorInstance *motor, float angle, float omega,
                          float torque, float kp, float kd);
void DMMotorSetPVT(DMMotorInstance *motor, float pos_rad, float v_limit_rad_s,
                   float i_ratio);
void DMMotorSetMode(DMMotorInstance *motor, DM_Mode_e mode);

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);
void DMMotorControlInit();
void DMMotorSetMITVelocity(DMMotorInstance *motor, float omega, float torque_ff,
                           float kd);
#endif // !DMMOTOR
