#include "gimbal.h"
#include "arm_math.h"
#include "bmi088.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "controller.h" // 包含LQR控制器
#include "dji_motor.h"
#include "dmmotor.h"
#include "general_def.h"
#include "ins_task.h"
#include "message_center.h"
#include "robot_def.h"
#include "sysid_task.h"
#include "user_lib.h"
#include <math.h>

/* ============================================================
 * 系统辨识控制开关（测试时改为1，完成后改为0）
 * ============================================================ */
#define ENABLE_GIMBAL_SYSID 0            // 0-关闭，1-启动辨识
#define SYSID_TARGET_AXIS SYSID_AXIS_YAW // SYSID_AXIS_YAW 或 SYSID_AXIS_PITCH
#define DEG_TO_RAD ((float)M_PI / 180.0f)

#if ENABLE_GIMBAL_SYSID
static Publisher_t *sysid_macro_pub = NULL;
static uint8_t sysid_macro_done = 0;

static void GimbalSystemIDSwitch() {
  static uint8_t sysid_stop_sent = 0;

  if (!sysid_macro_done && DWT_GetTimeline_s() > 3.0f) {
    if (sysid_macro_pub == NULL) {
      sysid_macro_pub =
          PubRegister("gimbal_sysid_cmd", sizeof(SysID_Ctrl_Cmd_s));
    }

    SysID_Ctrl_Cmd_s sysid_cmd = {
        .enable = 1,
        .axis = SYSID_TARGET_AXIS,
        .yaw_ref = 0.0f,
        .pitch_ref = 0.0f,
    };
    PubPushMessage(sysid_macro_pub, &sysid_cmd);
    sysid_macro_done = 1;
  }

  if (sysid_macro_done && !sysid_stop_sent && DWT_GetTimeline_s() > 25.0f) {
    SysID_Ctrl_Cmd_s sysid_cmd = {
        .enable = 0,
        .axis = SYSID_TARGET_AXIS,
        .yaw_ref = 0.0f,
        .pitch_ref = 0.0f,
    };
    PubPushMessage(sysid_macro_pub, &sysid_cmd);
    sysid_stop_sent = 1;
  }
}
#endif
/* ============================================================ */

/* ==================== 系统辨识任务交互 ==================== */
static Publisher_t *sysid_pub;
static Subscriber_t *sysid_sub;
/* ================================================================== */

static attitude_t *gimba_IMU_data;
static DJIMotorInstance *yaw_motor;
static DMMotorInstance *pitch_motor;

static float gimbal_yaw_cur_ff = 0.0f;
static float damiao_pitch_gravity_ff = 0.0f;
static float pitch_gravity_k =
    PITCH_GRAVITY_K * GYRO2GIMBAL_DIR_PITCH; // m*g*r，符号随IMU方向
static float pitch_gamma_rad =
    PITCH_GRAVITY_GAMMA_DEG * (float)M_PI / 180.0f; // 重心偏置角(弧度)
static float pitch_torque_ff = 0.0f;
static const float pitch_velocity_limit_rad_s = 8.0f; // 默认位置阶跃速度限幅

static Publisher_t *gimbal_pub;
static Subscriber_t *gimbal_sub;
static Gimbal_Upload_Data_s gimbal_feedback_data;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;

static float GimbalPitchGravityFF(float pitch_rad) {
  /* IMU 已输出弧度，这里直接用弧度做重力补偿 */
  float theta = pitch_rad + pitch_gamma_rad;
  pitch_torque_ff = pitch_gravity_k * cosf(theta); // theta 为弧度
  damiao_pitch_gravity_ff = pitch_torque_ff;
  return pitch_torque_ff;
}

void GimbalInit() {
  gimba_IMU_data = INS_Init();

  Motor_Init_Config_s yaw_config = {
      .can_init_config =
          {
              .can_handle = &hcan2,
              .tx_id = 2,
          },
      .controller_param_init_config =
          {
              .LQR =
                  {
                      .K_angle = 345.0f,
                      .K_velocity = 24.0f,
                      .K_integral = 0.0f,
                      .max_out = 20.0f,
                      .enable_integral = 0,
                  },
              .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle_rad,
              .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
              .current_feedforward_ptr = &gimbal_yaw_cur_ff,
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = OTHER_FEED,
              .speed_feedback_source = OTHER_FEED,
              .outer_loop_type = ANGLE_LOOP,
              .close_loop_type = SPEED_LOOP | ANGLE_LOOP, // LQR设置中不启用
              .feedforward_flag = FEEDFORWARD_NONE,
              .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
              .controller_type = CONTROLLER_LQR,
          },
      .motor_type = GM6020,
  };

  Motor_Init_Config_s pitch_config = {
      .can_init_config =
          {
              .can_handle = &hcan2,
              .tx_id = 0x01,
              .rx_id = 0x11,
          },
      .controller_param_init_config =
          {
              .angle_PID =
                  {
                      .Kp = 1.0f,
                      .Ki = 0.0f,
                      .Kd = 0.1f,
                      .MaxOut = 8.0f,
                      .DeadBand = 0.01f,
                      .Improve = PID_DerivativeFilter | PID_Integral_Limit,
                      .IntegralLimit = 3.0f,
                      .Derivative_LPF_RC = 0.05f,
                  },
              .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
              .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[0],
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = OTHER_FEED,
              .speed_feedback_source = OTHER_FEED,
              .outer_loop_type = OPEN_LOOP,
              .close_loop_type = OPEN_LOOP,
              .feedforward_flag = FEEDFORWARD_NONE,
              .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
              .controller_type = CONTROLLER_PID,
          },
      .pvt_config =
          {
              .v_limit_max = 12.0f,
              .i_limit_max = 1.0f,
              .kt_out = 0.0f,
          },
      .motor_type = MOTOR_TYPE_NONE,
  };
  yaw_motor = DJIMotorInit(&yaw_config);
  pitch_motor = DMMotorInit(&pitch_config);
  DMMotorSetMode(pitch_motor, DM_MODE_PVT);
  DMMotorEnable(pitch_motor);
  DMMotorControlInit();

  gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
  gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
  sysid_pub = PubRegister("gimbal_sysid_cmd", sizeof(SysID_Ctrl_Cmd_s));
  sysid_sub = SubRegister("gimbal_sysid_feedback", sizeof(SysID_Feedback_s));

  Gimbal_SysIDTaskInit(yaw_motor, pitch_motor, gimba_IMU_data);
}

void GimbalTask() {
#if ENABLE_GIMBAL_SYSID
  GimbalSystemIDSwitch();
#endif

  SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
  // GimbalPitchGravityFF(gimba_IMU_data->Pitch_rad);

  float pitch_pos_rad = gimbal_cmd_recv.pitch;
  float pitch_v_lim = pitch_velocity_limit_rad_s;
  // 固定PVT电流标幺上限，用于上位机参数调试；避免重力补偿关闭时i_ratio长期卡在0.05
  float pitch_i_ratio = 0.5f;

  switch (gimbal_cmd_recv.gimbal_mode) {
  case GIMBAL_ZERO_FORCE: {
    DJIMotorStop(yaw_motor);
    DMMotorStop(pitch_motor);
    damiao_pitch_gravity_ff = 0.0fs;
    break;
  }
  case GIMBAL_GYRO_MODE: {
    DJIMotorEnable(yaw_motor);
    DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
    DMMotorEnable(pitch_motor);
    DMMotorSetPVT(pitch_motor, pitch_pos_rad, pitch_v_lim, pitch_i_ratio);
    break;
  }
  case GIMBAL_FREE_MODE: {
    DJIMotorEnable(yaw_motor);
    DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
    DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
    DMMotorEnable(pitch_motor);
    DMMotorSetPVT(pitch_motor, pitch_pos_rad, pitch_v_lim, pitch_i_ratio);
    break;
  }
  case GIMBAL_LQR_MODE: {
    DJIMotorEnable(yaw_motor);
    DJIMotorChangeController(yaw_motor, CONTROLLER_LQR);
    DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
    DMMotorEnable(pitch_motor);
    DMMotorSetPVT(pitch_motor, pitch_pos_rad, pitch_v_lim, pitch_i_ratio);
    break;
  }
  case GIMBAL_SYS_ID_CHIRP: {
    static SysID_Feedback_s sysid_feedback;
    SubGetMessage(sysid_sub, &sysid_feedback);

    SysID_Ctrl_Cmd_s sysid_cmd = {
        .enable = 1,
        .axis = SYSID_AXIS_YAW,
        .yaw_ref = gimbal_cmd_recv.yaw,
        .pitch_ref = gimbal_cmd_recv.pitch,
    };
    PubPushMessage(sysid_pub, &sysid_cmd);
    break;
  }
  default: {
    break;
  }
  }
  // 设置反馈数据,主要是imu和yaw的ecd
  gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
  gimbal_feedback_data.yaw_motor_single_round_angle =
      yaw_motor->measure.angle_single_round;
  // 推送消息
  PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
