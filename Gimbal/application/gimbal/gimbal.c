#include "gimbal.h"
#include "arm_math_compat.h"
#include "bmi088.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "controller.h" // 包含LQR控制器
#include "dji_motor.h"
#include "dmmotor.h"
#include "general_def.h"
#include "ins_task.h"
#include "main.h"
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

/* ============================================================
 * 视觉自瞄与 Yaw 控制链路
 * ============================================================
 * 思路：
 * - 云台应用层负责模式编排与视觉接管。
 * - Yaw：手动/回退链路使用 DJI 电机层 PID 控制器。
 * - Yaw：视觉直接给电流时，仍沿用 OPEN_LOOP 原始电流发送链路。
 * - Pitch：继续使用达妙 MIT 目标组装逻辑。
 * ============================================================ */

#if ENABLE_GIMBAL_SYSID
static Publisher_t *sysid_macro_pub = NULL;
static uint8_t sysid_macro_done = 0;

static void GimbalSystemIDSwitch() {
  static uint8_t sysid_stop_sent = 0;

  if (!sysid_macro_done && DWT_GetTimeline_s() > 3.0f) {
    if (sysid_macro_pub == NULL) {
      sysid_macro_pub =
          RegisterPublisher("gimbal_sysid_cmd", sizeof(SysID_Ctrl_Cmd_s));
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

static Publisher_t *gimbal_pub;
static Subscriber_t *gimbal_sub;
static Subscriber_t *vision_sub;
static Gimbal_Upload_Data_s gimbal_feedback_data;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;
static Vision_Upload_Data_s vision_data_recv;

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
              // 串级
              // PID：角度环输出角速度参考（rad/s），速度环输出电机电流指令（CAN
              // 原始量） 注意：Yaw 反馈来自 IMU（弧度/弧度每秒），这里按
              // rad/rad/s 进行调参。
              .angle_PID =
                  {
                      .kp = 16.8f,
                      .ki = 0.0f,
                      .kd = 1.58f,
                      .MaxOut = 100.0f,   // 角速度参考限幅 [rad/s]
                      .DeadBand = 0.001f, // 约 0.06°，抑制零点抖动
                      .Improve = PID_Derivative_On_Measurement |
                                 PID_DerivativeFilter, // 微分先行 + 微分滤波
                      .Derivative_LPF_RC =
                          0.008f, // 微分滤波时间常数，单位秒（根据经验和实物调试确定，过大响应慢，过小噪声大）
                  },
              .speed_PID =
                  {
                      .kp = 22500.0f,
                      .ki = 2000.0f,
                      .kd = 0.0f,
                      .MaxOut = 16384.0f, // GM6020 电流指令（CAN 原始量）限幅
                      .DeadBand = 0.0f,
                      .Improve = PID_Integral_Limit,
                      .IntegralLimit = 2000.0f,
                  },
              .smc =
                  {
                      .sample_period = ROBOT_CTRL_PERIOD_S,
                  },
              .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle_rad,
              .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = OTHER_FEED,
              .speed_feedback_source = OTHER_FEED,
              .outer_loop_type = ANGLE_LOOP,
              .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
              .feedforward_flag = FEEDFORWARD_NONE,
              .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
              .controller_type = CONTROLLER_PID,
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
      .controller_setting_init_config =
          {
              .angle_feedback_source = OTHER_FEED,
              .speed_feedback_source = OTHER_FEED,
              .outer_loop_type = ANGLE_LOOP,
              .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
              .feedforward_flag = FEEDFORWARD_NONE,
              .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
              .controller_type = CONTROLLER_PID,
          },
      .controller_param_init_config =
          {
              // Pitch 扭矩主线：角度外环输出角速度参考(rad/s)，
              // 速度内环输出输出轴扭矩参考(N·m)。
              .angle_PID =
                  {
                      .kp = 20.0f,
                      .ki = 0.0f,
                      .kd = 0.0f,
                      .MaxOut = 12.0f, // 约等于 720 deg/s
                      .DeadBand = 0.001f,
                      .Improve = PID_Derivative_On_Measurement |
                                 PID_DerivativeFilter,
                      .Derivative_LPF_RC = 0.01f,
                  },
              .speed_PID =
                  {
                      .kp = 1.15f,
                      .ki = 0.0f,
                      .kd = 0.0f,
                      .MaxOut = 3.0f, // 先按 rmpp 的 3N·m 保守迁移
                      .DeadBand = 0.0f,
                      .Improve = PID_Integral_Limit,
                      .IntegralLimit = 1.5f,
                  },
              .other_angle_feedback_ptr = &gimba_IMU_data->Pitch_rad,
              .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[0],
          },
      .mit_config =
          {
              .limit =
                  {
                      .angle_min = DM_P_MIN,
                      .angle_max = DM_P_MAX,
                      .omega_min = DM_V_MIN,
                      .omega_max = DM_V_MAX,
                      .torque_min = DM_T_MIN,
                      .torque_max = DM_T_MAX,
                      .kp_min = 0.0f,
                      .kp_max = DM_KP_MAX,
                      .kd_min = 0.0f,
                      .kd_max = DM_KD_MAX,
                      .current_max = 0.0f,
                  },
              .default_kp = 25.0f,
              .default_kd = 1.0f,
              .manual_profile =
                  {
                      .kp = 18.8f,
                      .kd = 1.48f,
                      .v_des = 0.0f,
                      .torque_des = 0.0f,
                  },
              .vision_profile =
                  {
                      .kp = 21.75f,
                      .kd = 2.15f,
                      .v_des = 0.0f,
                      .torque_des = 0.0f,
                  },
              .auto_clear_error = 1,
              .auto_enter_mode = 1,
          },
      .motor_type = MOTOR_TYPE_NONE,
  };
  yaw_motor = DJIMotorInit(&yaw_config);
  pitch_motor = DMMotorInit(&pitch_config);
  DMMotorSetMode(pitch_motor, DM_MODE_MIT);
  DMMotorEnable(pitch_motor);
  DMMotorControlInit();

  gimbal_pub = RegisterPublisher("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
  gimbal_sub = RegisterSubscriber("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
  vision_sub = RegisterSubscriber("vision_data", sizeof(Vision_Upload_Data_s));
  sysid_pub = RegisterPublisher("gimbal_sysid_cmd", sizeof(SysID_Ctrl_Cmd_s));
  sysid_sub =
      RegisterSubscriber("gimbal_sysid_feedback", sizeof(SysID_Feedback_s));

  Gimbal_SysIDTaskInit(yaw_motor, pitch_motor, gimba_IMU_data);
}

void GimbalTask() {
#if ENABLE_GIMBAL_SYSID
  GimbalSystemIDSwitch();
#endif

  SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
  SubGetMessage(vision_sub, &vision_data_recv);

  float yaw_ref_rad = gimbal_cmd_recv.yaw;
  float pitch_ref_rad = gimbal_cmd_recv.pitch;

  const uint8_t autoaim_mode =
      (gimbal_cmd_recv.gimbal_mode == GIMBAL_AUTOAIM_MODE);
  const uint8_t vision_takeover =
      (autoaim_mode && vision_data_recv.vision_takeover);

  switch (gimbal_cmd_recv.gimbal_mode) {
  case GIMBAL_ZERO_FORCE: {
    DJIMotorStop(yaw_motor);
    DMMotorStop(pitch_motor);
    break;
  }
  case GIMBAL_GYRO_MODE: {
    DJIMotorEnable(yaw_motor);
    yaw_motor->motor_settings.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
    yaw_motor->motor_settings.outer_loop_type = ANGLE_LOOP;
    DJIMotorSetRef(yaw_motor, yaw_ref_rad);
    DJIMotorChangeController(yaw_motor, CONTROLLER_PID);
    DMMotorEnable(pitch_motor);
    DMMotorSetRef(pitch_motor, pitch_ref_rad, 0.0f);
    break;
  }
  case GIMBAL_AUTOAIM_MODE: {
    // 自瞄模式：有目标则视觉接管，否则保持遥控器/鼠标控制
    DJIMotorEnable(yaw_motor);

    if (vision_takeover) {
      yaw_motor->motor_settings.close_loop_type = OPEN_LOOP;
      yaw_motor->motor_settings.outer_loop_type = OPEN_LOOP;
      DJIMotorChangeController(yaw_motor, CONTROLLER_PID);
      DJIMotorSetRawRef(yaw_motor, vision_data_recv.yaw_current_cmd);
    } else {
      yaw_motor->motor_settings.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
      yaw_motor->motor_settings.outer_loop_type = ANGLE_LOOP;
      DJIMotorSetRef(yaw_motor, yaw_ref_rad);
      DJIMotorChangeController(yaw_motor, CONTROLLER_PID);
    }

    if (vision_takeover) {
      pitch_ref_rad = vision_data_recv.pitch_ref_limited;
    }
    DMMotorEnable(pitch_motor);
    DMMotorSetRef(pitch_motor, pitch_ref_rad, 0.0f);
    break;
  }
  case GIMBAL_LQR_MODE: {
    DJIMotorEnable(yaw_motor);
    yaw_motor->motor_settings.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
    yaw_motor->motor_settings.outer_loop_type = ANGLE_LOOP;
    DJIMotorChangeController(yaw_motor, CONTROLLER_LQR);
    DJIMotorSetRef(yaw_motor, yaw_ref_rad);
    DMMotorEnable(pitch_motor);
    DMMotorSetRef(pitch_motor, pitch_ref_rad, 0.0f);
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
