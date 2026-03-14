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
 * 视觉自瞄（对齐哨兵V3.22：Yaw 独立视觉双环）
 * ============================================================
 * 思路：
 * - cmd 层不直接覆盖云台角度目标（避免丢目标时突跳），视觉接管在 gimbal
 * 层完成。
 * -
 * Yaw：视觉接管时启用“位置环(角度误差→角速度参考)+速度环(角速度误差→电流指令)”。
 *        并将 yaw_motor 置为 OPEN_LOOP，使 DJI
 * 电机库仅负责组帧发送，不再计算串级PID。
 * - Pitch：保持达妙 MIT 位置刚度控制，仅对视觉 pitch
 * 目标做限速/限幅参考生成，避免突跳。
 *
 * 注意：
 * - Yaw 位置误差使用 IMU 单圈角 YawAngle_rad，并做 wrap_to_pi，天然避免跨 ±pi
 * 产生 2pi 跳变。
 * - Yaw 双环参数为保守默认值，需结合实物调参（先内环后外环）。
 * ============================================================ */
#define TWO_PI_F (2.0f * (float)M_PI)
// 视觉 pitch 参考变化率限幅（可按实际云台能力调整）
#define VISION_PITCH_REF_RATE_MAX_RAD_S 4.0f

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

static float gimbal_yaw_cur_ff = 0.0f;

/* ==================== Ozone 联调变量（Pitch/MIT） ====================
 * 说明：MIT 模式下需要下发 kp/kd（刚度/阻尼）。为便于 Ozone
 * 在线试参，这里提供两个 全局可改的参数（单位/含义与达妙 MIT 协议一致）。
 *
 * 调参第一步：先将力矩前馈 t_ff 置 0，只调
 * kp/kd，确保系统稳定且无啸叫，再考虑加入前馈。
 * ================================================================ */
volatile float gimbal_pitch_mit_kp = 18.8f;
volatile float gimbal_pitch_mit_kd = 1.48f;
/* MIT 期望角速度 v_des（单位：rad/s），默认置 0；可在 Ozone 实时修改 */
volatile float gimbal_pitch_mit_v_des = 0.0f;
/* MIT 期望力矩 τ_des（单位与达妙协议一致），默认置 0；可在 Ozone 实时修改 */
volatile float gimbal_pitch_mit_torque_des = 0.0f;

static Publisher_t *gimbal_pub;
static Subscriber_t *gimbal_sub;
static Gimbal_Upload_Data_s gimbal_feedback_data;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;

// 视觉接管状态（用于模式切换时的平滑过渡）
static uint8_t vision_takeover_last = 0;

static float DJIMotorApplyInternalSign(const DJIMotorInstance *motor,
                                       float desired_set) {
  // dji_motor.c 的 PID 分支会根据 motor_reverse_flag 和 feedback_reverse_flag
  // 对 pid_ref 做取反。这里提前乘同样的符号，确保最终发到 CAN 的 set 为
  // desired_set。
  float sign = 1.0f;
  if (motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
    sign *= -1.0f;
  }
  if (motor->motor_settings.feedback_reverse_flag ==
      FEEDBACK_DIRECTION_REVERSE) {
    sign *= -1.0f;
  }
  return desired_set * sign;
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
              .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle_rad,
              .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
              .current_feedforward_ptr = &gimbal_yaw_cur_ff,
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
              .outer_loop_type = OPEN_LOOP,
              .close_loop_type = OPEN_LOOP,
              .feedforward_flag = FEEDFORWARD_NONE,
              .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
              .controller_type = CONTROLLER_PID,
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
  // 注意：视觉订阅已移除，视觉控制量通过gimbal_cmd传递
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

  float yaw_ref_rad = gimbal_cmd_recv.yaw;
  float pitch_ref_rad = gimbal_cmd_recv.pitch;

  // 视觉接管判断：通过gimbal_cmd中的标志位判断
  const uint8_t autoaim_mode =
      (gimbal_cmd_recv.gimbal_mode == GIMBAL_AUTOAIM_MODE);
  const uint8_t vision_yaw_takeover =
      (autoaim_mode && gimbal_cmd_recv.vision_yaw_direct);
  const uint8_t vision_pitch_takeover =
      (autoaim_mode && gimbal_cmd_recv.vision_pitch_direct);

  // 视觉接管沿处理：Yaw 切到 OPEN_LOOP 直出电流
  static Closeloop_Type_e yaw_close_loop_backup = OPEN_LOOP;
  static Closeloop_Type_e yaw_outer_loop_backup = OPEN_LOOP;
  static Controller_Type_e yaw_controller_backup = CONTROLLER_PID;
  static uint8_t yaw_backup_valid = 0;

  if (vision_yaw_takeover && !vision_takeover_last) {
    // 进入视觉接管：备份并切换 yaw 为 OPEN_LOOP（仅组帧发送）
    yaw_close_loop_backup = yaw_motor->motor_settings.close_loop_type;
    yaw_outer_loop_backup = yaw_motor->motor_settings.outer_loop_type;
    yaw_controller_backup = yaw_motor->motor_settings.controller_type;
    yaw_backup_valid = 1;

    yaw_motor->motor_settings.controller_type = CONTROLLER_PID;
    yaw_motor->motor_settings.close_loop_type = OPEN_LOOP;
    yaw_motor->motor_settings.outer_loop_type = OPEN_LOOP;
  } else if (!vision_yaw_takeover && vision_takeover_last) {
    // 退出视觉接管：清零输出并恢复闭环配置
    DJIMotorSetRef(yaw_motor, 0.0f);
    if (yaw_backup_valid) {
      yaw_motor->motor_settings.close_loop_type = yaw_close_loop_backup;
      yaw_motor->motor_settings.outer_loop_type = yaw_outer_loop_backup;
      yaw_motor->motor_settings.controller_type = yaw_controller_backup;
    }
    // 防止切回闭环时积分残留导致突跳
    PIDReset(&yaw_motor->motor_controller.angle_PID);
    PIDReset(&yaw_motor->motor_controller.speed_PID);
  }
  vision_takeover_last = vision_yaw_takeover;

  float mit_kp = float_constrain(gimbal_pitch_mit_kp, 0.0f, DM_KP_MAX);
  float mit_kd = float_constrain(gimbal_pitch_mit_kd, 0.0f, DM_KD_MAX);
  float mit_v_des = float_constrain(gimbal_pitch_mit_v_des, DM_V_MIN, DM_V_MAX);
  float mit_torque =
      float_constrain(gimbal_pitch_mit_torque_des, DM_T_MIN, DM_T_MAX);

  // 视觉Pitch接管：使用vision层计算好的限速目标
  if (vision_pitch_takeover) {
    pitch_ref_rad = gimbal_cmd_recv.vision_pitch_ref;
  }

  switch (gimbal_cmd_recv.gimbal_mode) {
  case GIMBAL_ZERO_FORCE: {
    DJIMotorStop(yaw_motor);
    DMMotorStop(pitch_motor);
    break;
  }
  case GIMBAL_GYRO_MODE: {
    DJIMotorEnable(yaw_motor);
    DJIMotorSetRef(yaw_motor, yaw_ref_rad);
    DMMotorEnable(pitch_motor);
    DMMotorSetMITTargets(pitch_motor, pitch_ref_rad, mit_v_des, mit_torque,
                         mit_kp, mit_kd);
    break;
  }
  case GIMBAL_AUTOAIM_MODE: {
    // 自瞄模式：有目标则视觉接管，否则保持遥控器/鼠标控制
    DJIMotorEnable(yaw_motor);
    DJIMotorChangeController(yaw_motor, CONTROLLER_PID);

    if (vision_yaw_takeover) {
      // 使用vision层计算好的电流指令
      const float set_ref = DJIMotorApplyInternalSign(
          yaw_motor, gimbal_cmd_recv.vision_yaw_current);
      DJIMotorSetRef(yaw_motor, set_ref);
    } else {
      DJIMotorSetRef(yaw_motor, yaw_ref_rad);
    }

    DMMotorEnable(pitch_motor);
    DMMotorSetMITTargets(pitch_motor, pitch_ref_rad, mit_v_des, mit_torque,
                         mit_kp, mit_kd);
    break;
  }
  case GIMBAL_LQR_MODE: {
    DJIMotorEnable(yaw_motor);
    DJIMotorChangeController(yaw_motor, CONTROLLER_LQR);
    DJIMotorSetRef(yaw_motor, yaw_ref_rad);
    DMMotorEnable(pitch_motor);
    DMMotorSetMITTargets(pitch_motor, pitch_ref_rad, mit_v_des, mit_torque,
                         mit_kp, mit_kd);
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
