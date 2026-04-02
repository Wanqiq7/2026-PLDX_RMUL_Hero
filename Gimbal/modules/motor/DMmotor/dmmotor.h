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
#define DM_FB_STATE_DISABLED 0x0u
#define DM_FB_STATE_ENABLED 0x1u
#define DM_FB_STATE_COMMUNICATION_LOST 0xDu
#define DM_ENABLE_RETRY_INTERVAL_MS 20U

typedef enum {
  DM_MODE_MIT = 1,
  DM_MODE_POSVEL = 2,
  DM_MODE_VEL = 3,
  DM_MODE_PVT = 4,
} DM_Mode_e;

typedef enum {
  DM_MIT_PROFILE_MANUAL = 0,
  DM_MIT_PROFILE_VISION = 1,
} DM_MIT_Profile_e;

typedef struct {
  uint8_t motor_id;
  uint8_t motor_state;
  float output_velocity_rad_s;
  float last_output_angle_rad;
  float output_angle_rad;
  float output_torque_nm;
  float mos_temp_c;
  float rotor_temp_c;
  int32_t output_total_round;
} DM_Motor_Measure_s;

typedef struct {
  uint16_t angle_cmd_raw;
  uint16_t velocity_cmd_raw;
  uint16_t torque_cmd_raw;
  uint16_t kp_cmd_raw;
  uint16_t kd_cmd_raw;
} DMMotor_Send_s;

typedef struct {
  DM_Motor_Measure_s measure;
  Motor_Control_Setting_s motor_settings;
  PIDInstance current_PID;
  PIDInstance speed_PID;
  PIDInstance angle_PID;

  /* MIT 协议量程与模式 */
  DM_MIT_Limit_s mit_limit;
  DM_Mode_e drive_mode;
  DM_PVT_Config_s pvt_config;

  /* 外部反馈与前馈接口 */
  float *external_angle_feedback_ptr;
  float *external_speed_feedback_ptr;
  float *external_speed_feedforward_ptr;
  float *external_torque_feedforward_ptr;

  /* 模块内串级控制缓存 */
  float mit_torque_fallback_nm;
  float cascade_angle_ref_deg;
  float cascade_torque_ff_nm;
  float cascade_damping_kd;
  uint8_t use_cascade_pid_path;
  uint8_t use_mit_full_command;
  float mit_default_stiffness_kp;
  float mit_default_damping_kd;
  DM_MIT_Profile_s mit_manual_profile;
  DM_MIT_Profile_s mit_vision_profile;
  DM_MIT_Profile_e active_mit_profile;

  /* MIT 五元组目标：角度/速度/力矩/刚度/阻尼 */
  float mit_target_angle_rad;
  float mit_target_velocity_rad_s;
  float mit_target_torque_nm;
  float mit_target_stiffness_kp;
  float mit_target_damping_kd;

  /* MIT 速度直控：位置锁零，仅保留速度/力矩前馈/阻尼 */
  float mit_velocity_only_rad_s;
  float mit_velocity_only_torque_ff_nm;
  float mit_velocity_only_damping_kd;
  uint8_t use_mit_velocity_only;

  /* PVT 目标：位置/速度上限/电流比例 */
  float pvt_target_angle_rad;
  float pvt_velocity_limit_rad_s;
  float pvt_current_limit_ratio;
  uint8_t use_pvt_command_frame;

  Motor_Working_Type_e stop_flag;
  CANInstance *motor_can_instance;
  DaemonInstance *motor_daemon;
  uint32_t command_tx_id;
  uint32_t lost_count;
  uint32_t last_enable_cmd_ms;
} DMMotorInstance;

typedef enum {
  DM_CMD_MOTOR_MODE = 0xfc,
  DM_CMD_RESET_MODE = 0xfd,
  DM_CMD_ZERO_POSITION = 0xfe,
  DM_CMD_CLEAR_ERROR = 0xfb
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
void DMMotorSelectMITProfile(DMMotorInstance *motor, DM_MIT_Profile_e profile);
void DMMotorSetMITTargetByProfile(DMMotorInstance *motor, float angle);

#endif // DMMOTOR_H
