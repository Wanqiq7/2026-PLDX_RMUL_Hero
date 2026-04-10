#include "chassis.h"
#include "chassis_can_link.h"
#include "VOFA.h"
#include "utils/math/arm_math_compat.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"
#include "controllers/domain/chassis_force_control.h"
#include "controllers/pid/pid_controller.h"
#include "dji_motor.h"
#include "general_def.h"
#include "main.h"
#include "math.h"
#include "message_center.h"
#include "power_controller.h"
#include "referee_UI.h"
#include "referee_task.h"
#include "robot_def.h"
#include "string.h"
#include "super_cap.h"
#include "sysid_task.h"
#include "utils/math/user_lib.h"

/* ============================================================
 * 系统辨识控制开关（测试时改为1，完成后改为0）
 * ============================================================ */
#define ENABLE_CHASSIS_SYSID 0 // 0-关闭，1-启动辨识
#define SYSID_TARGET_MOTOR 0   // 0-左前lf, 1-右前rf, 2-左后lb, 3-右后rb

#if ENABLE_CHASSIS_SYSID
static Publisher_t *sysid_macro_pub = NULL;
static uint8_t sysid_macro_done = 0;

/**
 * @brief 系统辨识宏开关触发函数
 * @note 当ENABLE_CHASSIS_SYSID=1时，上电3秒后自动启动辨识
 *       辨识运行20秒后自动停止，电机完全停止
 *       测试完成后，将宏改为0，重新编译即可恢复正常
 *
 * 使用说明：
 *   1. 架起机器人（轮子悬空）
 *   2. 修改上面的宏：ENABLE_CHASSIS_SYSID = 1
 *   3. 选择目标电机：SYSID_TARGET_MOTOR = 0~3
 *   4. 编译烧录
 *   5. 配置Ozone记录变量（见文档）
 *   6. 上电等待3秒 → 自动启动辨识（持续20秒）
 *   7. 20秒后电机自动停止
 *   8. Ozone导出CSV，运行MATLAB分析
 *   9. 测试完成后，改回：ENABLE_CHASSIS_SYSID = 0
 */
static void ChassisSystemIDSwitch() {
  static uint8_t sysid_stop_sent = 0;

  // 第一阶段：启动辨识（上电3秒后）
  if (!sysid_macro_done && DWT_GetTimeline_s() > 3.0f) {
    // 首次调用，注册发布者
    if (sysid_macro_pub == NULL) {
      sysid_macro_pub =
          PubRegister("chassis_sysid_cmd", sizeof(Chassis_SysID_Ctrl_Cmd_s));
    }

    // 发布启动指令
    Chassis_SysID_Ctrl_Cmd_s sysid_cmd = {
        .enable = 1,
        .target_motor = SYSID_TARGET_MOTOR,
    };
    PubPushMessage(sysid_macro_pub, &sysid_cmd);

    sysid_macro_done = 1; // 标记已触发
  }

  // 第二阶段：确保停止（上电25秒后，辨识应该已完成）
  if (sysid_macro_done && !sysid_stop_sent && DWT_GetTimeline_s() > 25.0f) {
    // 发送停止指令（确保辨识任务退出）
    Chassis_SysID_Ctrl_Cmd_s sysid_cmd = {
        .enable = 0,
        .target_motor = SYSID_TARGET_MOTOR,
    };
    PubPushMessage(sysid_macro_pub, &sysid_cmd);

    sysid_stop_sent = 1; // 标记已发送停止指令
    // 注意：电机停止由sysid_task.c中的逻辑处理
  }
}
#endif
/* ============================================================ */

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)   // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f) // 半轮距

/* 底盘运行时配置实例 - const 优化，编译时优化 */
static const Chassis_Runtime_Config_t chassis_config = {
    .rc =
        {
            .max_linear_speed = 4.0f, // m/s - 最大线速度
            .max_angular_speed = 6.0f // rad/s - 最大角速度
        },
    .force =
        {
            .torque_feedforward_coeff = 0.5f, // 扭矩前馈系数(比例系数)
            .friction_threshold_omega = 2.0f, // rad/s - 摩擦补偿速度阈值
            .wheel_speed_feedback_coeff =
                0.02f, // A·s/rad - 轮速反馈系数（降低增益减少振荡）
            .omega_error_lpf_alpha =
                0.85f,              // 角速度误差滤波系数（增强滤波抑制噪声）
            .omega_threshold = 4.0f // rad/s - 过零保护角速度阈值（扩大死区）
        },
    .kinematics = {
        .velocity_lpf_alpha = 0.85f, // 速度估算滤波系数
        .speed_deadband = 25.0f,     // 速度死区阈值 (deg/s)
        .rotate_speed = 4.5f         // 小陀螺模式旋转速度 (rad/s)
    }};

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体
 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "ins_task.h"
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;          // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data;  // 底盘回传的反馈数据

static referee_info_t *referee_data; // 用于获取裁判系统的数据
static Referee_UI_Generated_State_t
    ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap; // 超级电容

#if POWER_CONTROLLER_ENABLE
static PowerControllerInstance *power_ctrl = NULL; // 功率控制器实例
static USARTInstance *vofa_usart = NULL;
static uint8_t vofa_tx_buf[32];
static VofaJustFloatSender_s vofa_sender;
static uint32_t power_limit_hard_clamp_count = 0;

static uint8_t VofaUsartIsReady(void *ctx) {
  USARTInstance *ins = (USARTInstance *)ctx;
  if (ins == NULL || ins->usart_handle == NULL) {
    return 0;
  }

  return (ins->usart_handle->gState == HAL_UART_STATE_READY) ? 1 : 0;
}

static void VofaUsartSend(void *ctx, uint8_t *buf, uint16_t len,
                          VofaTransferMode_e mode) {
  USART_TRANSFER_MODE bsp_mode = USART_TRANSFER_DMA;
  switch (mode) {
  case VOFA_TRANSFER_BLOCKING:
    bsp_mode = USART_TRANSFER_BLOCKING;
    break;
  case VOFA_TRANSFER_IT:
    bsp_mode = USART_TRANSFER_IT;
    break;
  case VOFA_TRANSFER_DMA:
  default:
    bsp_mode = USART_TRANSFER_DMA;
    break;
  }

  USARTSend((USARTInstance *)ctx, buf, len, bsp_mode);
}
#endif
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb,
    *motor_rb; // left right forward back

static PIDInstance chassis_follow_pid; // 底盘跟随云台PID控制器
static uint32_t chassis_last_telemetry_ms = 0;
static const uint32_t CHASSIS_TELEMETRY_PERIOD_MS = 500U;
#define CHASSIS_ALL_WHEELS_STABLE_MS 150U

typedef enum {
  CHASSIS_RUNTIME_HOLD = 0,
  CHASSIS_RUNTIME_WAIT_STABLE = 1,
  CHASSIS_RUNTIME_READY = 2,
} chassis_runtime_state_e;

typedef enum {
  CHASSIS_LOCAL_INHIBIT_NONE = 0,
  CHASSIS_LOCAL_INHIBIT_LINK_LOST = 1,
  CHASSIS_LOCAL_INHIBIT_DEAD = 2,
  CHASSIS_LOCAL_INHIBIT_WHEEL_OFFLINE = 3,
  CHASSIS_LOCAL_INHIBIT_WAIT_STABLE = 4,
} chassis_local_inhibit_reason_e;

typedef struct {
  uint32_t now_ms;
  uint8_t comm_online;
  uint8_t referee_online;
  uint8_t wheel_online_mask;
  uint8_t robot_dead;
} ChassisSafetyInput_s;

typedef struct {
  uint8_t should_hold;
  uint8_t chassis_ready;
  uint8_t chassis_safety_status;
  uint8_t local_inhibit_reason;
} ChassisSafetyResult_s;

static chassis_runtime_state_e chassis_runtime_state = CHASSIS_RUNTIME_HOLD;
static uint32_t chassis_all_online_since_ms = 0U;
static uint8_t chassis_last_dead_state = 0xFFU;
static uint8_t chassis_last_ready_state = 0xFFU;
static uint8_t chassis_last_inhibit_reason = 0xFFU;
static uint8_t chassis_last_wheel_online_mask = 0xFFU;

static void ResetFollowPidRuntimeState(void);

static void ApplyMotorTauRef(DJIMotorInstance *motor, float tau_ref_nm) {
  Controller_Effort_Output_s effort = {
      .semantic = CONTROLLER_OUTPUT_TAU_REF,
      .tau_ref_nm = tau_ref_nm,
  };
  DJIMotorSetEffort(motor, &effort);
}

static void ApplyWheelTauRef(const float wheel_tau_ref[4]) {
  if (wheel_tau_ref == NULL) {
    return;
  }

  ApplyMotorTauRef(motor_rf, wheel_tau_ref[0]); // RF
  ApplyMotorTauRef(motor_lf, wheel_tau_ref[1]); // LF
  ApplyMotorTauRef(motor_lb, wheel_tau_ref[2]); // LB
  ApplyMotorTauRef(motor_rb, wheel_tau_ref[3]); // RB
}

static uint8_t BuildWheelOnlineMask(void) {
  uint8_t mask = 0U;

  if (motor_rf != NULL && motor_rf->daemon != NULL &&
      DaemonIsOnline(motor_rf->daemon)) {
    mask |= 0x01U; // bit0 = RF
  }
  if (motor_lf != NULL && motor_lf->daemon != NULL &&
      DaemonIsOnline(motor_lf->daemon)) {
    mask |= 0x02U; // bit1 = LF
  }
  if (motor_lb != NULL && motor_lb->daemon != NULL &&
      DaemonIsOnline(motor_lb->daemon)) {
    mask |= 0x04U; // bit2 = LB
  }
  if (motor_rb != NULL && motor_rb->daemon != NULL &&
      DaemonIsOnline(motor_rb->daemon)) {
    mask |= 0x08U; // bit3 = RB
  }

  return mask;
}

static uint8_t IsRobotDeadLocal(uint8_t referee_online) {
  if (!referee_online || referee_data == NULL) {
    return 0U;
  }

  return (referee_data->GameRobotState.current_HP == 0U ||
          referee_data->GameRobotState.power_management_chassis_output == 0U)
             ? 1U
             : 0U;
}

static void BuildChassisSafetyInput(ChassisSafetyInput_s *input, uint32_t now_ms,
                                    uint8_t comm_online,
                                    uint8_t referee_online) {
  if (input == NULL) {
    return;
  }

  memset(input, 0, sizeof(*input));
  input->now_ms = now_ms;
  input->comm_online = comm_online;
  input->referee_online = referee_online;
  input->wheel_online_mask = BuildWheelOnlineMask();
  input->robot_dead = IsRobotDeadLocal(referee_online);
}

static void StepChassisSafetyGate(const ChassisSafetyInput_s *input,
                                  ChassisSafetyResult_s *result) {
  uint8_t all_wheels_online = 0U;

  if (input == NULL || result == NULL) {
    return;
  }

  memset(result, 0, sizeof(*result));
  all_wheels_online = (input->wheel_online_mask == 0x0FU) ? 1U : 0U;

  if (!input->comm_online) {
    chassis_runtime_state = CHASSIS_RUNTIME_HOLD;
    chassis_all_online_since_ms = 0U;
    result->local_inhibit_reason = CHASSIS_LOCAL_INHIBIT_LINK_LOST;
  } else if (input->robot_dead) {
    chassis_runtime_state = CHASSIS_RUNTIME_HOLD;
    chassis_all_online_since_ms = 0U;
    result->local_inhibit_reason = CHASSIS_LOCAL_INHIBIT_DEAD;
  } else if (!all_wheels_online) {
    chassis_runtime_state = CHASSIS_RUNTIME_HOLD;
    chassis_all_online_since_ms = 0U;
    result->local_inhibit_reason = CHASSIS_LOCAL_INHIBIT_WHEEL_OFFLINE;
  } else if (chassis_runtime_state == CHASSIS_RUNTIME_READY) {
    result->local_inhibit_reason = CHASSIS_LOCAL_INHIBIT_NONE;
  } else {
    if (chassis_runtime_state != CHASSIS_RUNTIME_WAIT_STABLE) {
      chassis_runtime_state = CHASSIS_RUNTIME_WAIT_STABLE;
      chassis_all_online_since_ms = input->now_ms;
    }

    result->local_inhibit_reason = CHASSIS_LOCAL_INHIBIT_WAIT_STABLE;
    if ((input->now_ms - chassis_all_online_since_ms) >=
        CHASSIS_ALL_WHEELS_STABLE_MS) {
      chassis_runtime_state = CHASSIS_RUNTIME_READY;
      result->local_inhibit_reason = CHASSIS_LOCAL_INHIBIT_NONE;
    }
  }

  result->chassis_ready =
      (chassis_runtime_state == CHASSIS_RUNTIME_READY) ? 1U : 0U;
  result->should_hold = result->chassis_ready ? 0U : 1U;
  result->chassis_safety_status = CHASSIS_SAFETY_STATUS_NONE;
  if (input->robot_dead) {
    result->chassis_safety_status |= CHASSIS_SAFETY_STATUS_DEAD;
  }
  if (result->chassis_ready) {
    result->chassis_safety_status |= CHASSIS_SAFETY_STATUS_READY;
  }
}

static uint8_t ShouldHoldChassisControl(const Chassis_Ctrl_Cmd_s *cmd,
                                        const ChassisSafetyResult_s *result) {
  const uint8_t command_zero_force =
      (cmd != NULL && cmd->chassis_mode == CHASSIS_ZERO_FORCE) ? 1U : 0U;
  const uint8_t local_hold = (result != NULL) ? result->should_hold : 1U;

  return command_zero_force || local_hold;
}

static void HoldChassisOutputs(void) {
  DJIMotorStop(motor_lf);
  DJIMotorStop(motor_rf);
  DJIMotorStop(motor_lb);
  DJIMotorStop(motor_rb);
  ApplyMotorTauRef(motor_lf, 0.0f);
  ApplyMotorTauRef(motor_rf, 0.0f);
  ApplyMotorTauRef(motor_lb, 0.0f);
  ApplyMotorTauRef(motor_rb, 0.0f);
  ResetFollowPidRuntimeState();
  ChassisForceControlReset();
}

static void ApplyChassisSafetyFeedback(const ChassisSafetyResult_s *result) {
  chassis_feedback_data.chassis_safety_status =
      (result != NULL) ? result->chassis_safety_status
                       : CHASSIS_SAFETY_STATUS_NONE;
}

static void LogChassisSafetyTransition(const ChassisSafetyResult_s *result,
                                       const ChassisSafetyInput_s *input) {
  if (result == NULL || input == NULL) {
    return;
  }

  if (input->robot_dead != chassis_last_dead_state ||
      result->chassis_ready != chassis_last_ready_state ||
      result->local_inhibit_reason != chassis_last_inhibit_reason ||
      input->wheel_online_mask != chassis_last_wheel_online_mask) {
    LOGWARNING("[chassis-safe] dead=%u ready=%u wheel_mask=0x%02X reason=%u",
               input->robot_dead, result->chassis_ready,
               input->wheel_online_mask, result->local_inhibit_reason);
    chassis_last_dead_state = input->robot_dead;
    chassis_last_ready_state = result->chassis_ready;
    chassis_last_inhibit_reason = result->local_inhibit_reason;
    chassis_last_wheel_online_mask = input->wheel_online_mask;
  }
}

#if POWER_CONTROLLER_ENABLE
static void BuildPowerWheelObjs(const float wheel_tau_ref[4],
                                const float motor_speeds[4],
                                const float target_wheel_omega[4],
                                const float motor_torques[4],
                                PowerWheelObj_t wheel_objs[4]) {
  if (wheel_tau_ref == NULL || motor_speeds == NULL ||
      target_wheel_omega == NULL || motor_torques == NULL ||
      wheel_objs == NULL) {
    return;
  }

  for (size_t i = 0; i < 4; ++i) {
    wheel_objs[i].requested_tau_nm = wheel_tau_ref[i];
    wheel_objs[i].feedback_speed_rad_s = motor_speeds[i];
    wheel_objs[i].target_speed_rad_s =
        target_wheel_omega[i] / REDUCTION_RATIO_WHEEL;
    wheel_objs[i].feedback_tau_nm = motor_torques[i];
    wheel_objs[i].max_tau_nm = MAX_WHEEL_CURRENT * M3508_TORQUE_CONSTANT;
    wheel_objs[i].online = 1U;
  }
}
#endif

static void SyncGeneratedUIState(uint8_t comm_online, uint8_t cap_online,
                                 uint16_t cap_energy) {
  const int mild_threshold_percent = 90;
  const int wild_threshold_percent = 110;
  uint16_t heat_value = 0U;
  uint16_t fire_allowance_count = 0U;

  if (chassis_feedback_data.referee_online && referee_data != NULL) {
    const uint16_t heat_limit =
        referee_data->GameRobotState.shooter_barrel_heat_limit;
    const uint16_t barrel_heat =
        referee_data->PowerHeatData.shooter_42mm_barrel_heat;
    heat_value = (heat_limit > barrel_heat) ? (heat_limit - barrel_heat) : 0U;
    fire_allowance_count = (uint16_t)(heat_value / (uint16_t)HEAT_PER_SHOT_D);
  }

  ui_data.chassis_online = comm_online;
  ui_data.chassis_mode = chassis_cmd_recv.chassis_mode;
  ui_data.autoaim_on = chassis_cmd_recv.ui_autoaim_enabled ? 1U : 0U;
  ui_data.fri_on = chassis_cmd_recv.ui_friction_on ? 1U : 0U;
  ui_data.fire_allow = chassis_cmd_recv.ui_fire_allow ? 1U : 0U;
  ui_data.cap_online = cap_online;
  ui_data.stuck_active = chassis_cmd_recv.ui_stuck_active ? 1U : 0U;
  ui_data.shoot_mode_display = chassis_cmd_recv.ui_loader_mode;
  ui_data.refresh_request_seq = chassis_cmd_recv.ui_refresh_request_seq;
  ui_data.heat_value = heat_value;
  ui_data.fire_allowance_count = fire_allowance_count;
  ui_data.cap_energy = cap_energy;

  if (chassis_cmd_recv.chassis_speed_buff <= mild_threshold_percent) {
    ui_data.power_mode = REFEREE_UI_POWER_MILD;
  } else if (chassis_cmd_recv.chassis_speed_buff >= wild_threshold_percent) {
    ui_data.power_mode = REFEREE_UI_POWER_WILD;
  } else {
    ui_data.power_mode = REFEREE_UI_POWER_NOR;
  }
}

/* 功率控制相关变量已移至独立的power_controller模块 */

// 键鼠功率档位参数（对应 cmd 层 C 键循环：100% -> 80% -> 120%）
static const int POWER_MODE_ECO_PERCENT = 80;
static const int POWER_MODE_LIMIT_PERCENT = 100;
static const int POWER_MODE_OVERDRIVE_PERCENT = 120;
static const int POWER_MODE_ECO_THRESHOLD_PERCENT =
    (POWER_MODE_ECO_PERCENT + POWER_MODE_LIMIT_PERCENT) / 2;
static const int POWER_MODE_OVERDRIVE_THRESHOLD_PERCENT =
    (POWER_MODE_LIMIT_PERCENT + POWER_MODE_OVERDRIVE_PERCENT) / 2;
static const float POWER_MODE_ECO_SCALE = 0.80f;
static const float POWER_MODE_LIMIT_SCALE = 1.00f;
static const float POWER_MODE_OVERDRIVE_SCALE = 1.20f;
static const uint8_t POWER_MODE_OVERDRIVE_MIN_CAP_ENERGY =
    120; // 约 47% 电量阈值，低于阈值时超功率自动回退

#if POWER_CONTROLLER_ENABLE
/**
 * @brief 根据键鼠档位和电容状态选择目标功率上限
 * @param source_limit_w 基准功率上限（通常来自裁判/超电反馈）
 * @param power_mode_percent cmd 层透传档位值（80/100/120）
 * @param cap_online 超级电容在线标志
 * @param cap_energy 超级电容能量(0~255)
 * @param referee_limit_valid 裁判功率上限是否有效
 * @param referee_limit_w 裁判系统实时功率上限
 */
static float SelectPowerLimitByMode(float source_limit_w,
                                    int power_mode_percent, uint8_t cap_online,
                                    uint8_t cap_energy,
                                    uint8_t referee_limit_valid,
                                    float referee_limit_w) {
  float limit_scale = POWER_MODE_LIMIT_SCALE;

  if (power_mode_percent >= POWER_MODE_OVERDRIVE_THRESHOLD_PERCENT) {
    limit_scale = POWER_MODE_OVERDRIVE_SCALE;
  } else if (power_mode_percent <= POWER_MODE_ECO_THRESHOLD_PERCENT) {
    limit_scale = POWER_MODE_ECO_SCALE;
  }

  // 裁判离线时，禁止超功率档，避免越权输出
  if (!referee_limit_valid && limit_scale > POWER_MODE_LIMIT_SCALE) {
    limit_scale = POWER_MODE_LIMIT_SCALE;
  }

  // 超功率模式要求电容在线且能量充足，否则自动降级到额定档
  if (limit_scale > POWER_MODE_LIMIT_SCALE &&
      (!cap_online || cap_energy < POWER_MODE_OVERDRIVE_MIN_CAP_ENERGY)) {
    limit_scale = POWER_MODE_LIMIT_SCALE;
  }

  float target_limit_w = source_limit_w * limit_scale;
  float hard_limit_w = source_limit_w + MAX_CAP_POWER_OUT;
  if (referee_limit_valid) {
    hard_limit_w = referee_limit_w;
  }
  return float_constrain(target_limit_w, MIN_POWER_CONFIGURED, hard_limit_w);
}

static float ApplyPowerLimitHardClamp(float target_limit_w,
                                      uint8_t referee_limit_valid,
                                      float referee_limit_w) {
  float clamped_limit_w = target_limit_w;
  if (referee_limit_valid && referee_limit_w > MIN_POWER_CONFIGURED) {
    clamped_limit_w = fminf(clamped_limit_w, referee_limit_w);
  }
  if (clamped_limit_w + 1e-3f < target_limit_w) {
    power_limit_hard_clamp_count++;
  }
  if (clamped_limit_w < MIN_POWER_CONFIGURED) {
    clamped_limit_w = MIN_POWER_CONFIGURED;
  }
  return clamped_limit_w;
}
#endif // POWER_CONTROLLER_ENABLE

static Bullet_Speed_e SelectBulletSpeedByReferee(float bullet_speed_mps) {
  if (bullet_speed_mps >= 24.0f) {
    return SMALL_AMU_30;
  }
  if (bullet_speed_mps >= 16.5f) {
    return SMALL_AMU_18;
  }
  return SMALL_AMU_15;
}

void ChassisInit() {
  // 四个轮子的参数一样,改tx_id和反转标志位即可
  Motor_Init_Config_s chassis_motor_config = {
      .can_init_config = {.can_handle = &hcan1},
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,
              .outer_loop_type = OPEN_LOOP,
              .close_loop_type = OPEN_LOOP,
              .feedforward_flag = FEEDFORWARD_NONE,
          },
      .motor_type = M3508,
  };

  chassis_motor_config.controller_param_init_config.speed_feedforward_ptr =
      NULL;
  // 右前轮（RF）- 电机ID=1
  chassis_motor_config.can_init_config.tx_id = 1;
  chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE;
  motor_rf = DJIMotorInit(&chassis_motor_config);

  // 左前轮（LF）- 电机ID=2
  chassis_motor_config.can_init_config.tx_id = 2;
  chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE;
  motor_lf = DJIMotorInit(&chassis_motor_config);

  // 左后轮（LB）- 电机ID=3
  chassis_motor_config.can_init_config.tx_id = 3;
  chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE;
  motor_lb = DJIMotorInit(&chassis_motor_config);

  // 右后轮（RB）- 电机ID=4
  chassis_motor_config.can_init_config.tx_id = 4;
  chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE;
  motor_rb = DJIMotorInit(&chassis_motor_config);

  referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI

  SuperCap_Init_Config_s cap_conf = {
      .can_config = {
          .can_handle = &hcan1,
          .tx_id = 0x061, // 超级电容控制器实际接收的 CAN ID（修复）
          .rx_id = 0x051, // 超级电容控制器实际发送的 CAN ID（修复）
      }};
  cap = SuperCapInit(&cap_conf); // 超级电容初始化

#if POWER_CONTROLLER_ENABLE
  // VOFA+ JustFloat：用于将超级电容实时功率发送到上位机
  // 注意：USARTRegister() 会启动 DMA 接收，因此 recv_buff_size 不能为 0
  // 串口选择原则：避免与遥控器/裁判系统/视觉等已占用串口冲突
  USART_Init_Config_s vofa_usart_conf = {
      .recv_buff_size = 1,
      .usart_handle = &huart1,
      .module_callback = NULL,
  };
  vofa_usart = USARTRegister(&vofa_usart_conf);

  vofa_sender.ctx = vofa_usart;
  vofa_sender.is_ready = VofaUsartIsReady;
  vofa_sender.send = VofaUsartSend;
  vofa_sender.tx_buf = vofa_tx_buf;
  vofa_sender.tx_buf_len = (uint16_t)sizeof(vofa_tx_buf);
  vofa_sender.max_channels = 1;
  vofa_sender.transfer_mode = VOFA_TRANSFER_DMA;
#endif

  // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
  Chassis_IMU_data = INS_Init(); // 底盘IMU初始化
  ChassisCanLinkInit(&hcan2, (uint32_t)DWT_GetTimeline_ms());
#endif                                        // CHASSIS_BOARD

#if POWER_CONTROLLER_ENABLE
  // 功率控制器初始化（独立模块）
  // ⚠️ 所有量纲统一到输出轴侧（与功率模型k1、k2参数匹配）
  PowerControllerConfig_t power_config = {
      .k1_init = 0.22f,      // 转速损耗系数初始值（输出轴量纲）
      .k2_init = 1.2f,       // 力矩损耗系数初始值（输出轴量纲）
      .k3 = 5.10f,           // 静态功率损耗（参考港科大）
      .rls_lambda = 0.9999f, // RLS遗忘因子
      // 输出轴转矩常数 = 0.3 Nm/A
      .torque_constant = M3508_TORQUE_CONSTANT,
      .current_scale = 20.0f / 16384.0f, // CAN指令值转电流系数
      .robot_division = ROBOT_HERO,      // 机器人类型（用于断连时查表）
  };
  power_ctrl = PowerControllerRegister(&power_config);
#endif // POWER_CONTROLLER_ENABLE

  // 底盘跟随云台PID控制器初始化（统一力控：输出角速度）
  // 注意：输出单位为角速度（rad/s），后续通过力控PID转换为扭矩
  PID_Init_Config_s follow_pid_config = {
      .kp = 0.125f, // 单位：(rad/s)/度（角度误差→角速度）
      .ki = 0.0f,   // 积分增益（可选开启）
      .kd = 0.0f,   // 微分增益
      // 微分低通滤波时间常数RC（秒）
      // RobotTask周期约2ms(500Hz)，取微分截止频率fc≈20Hz：
      // RC = 1/(2πfc) ≈ 1/(2π·20) ≈ 0.00796s
      .IntegralLimit = 0.0f, // 积分限幅：1 rad/s
      .MaxOut = 12.0f,       // 最大输出：5 rad/s（底盘旋转角速度）
      .DeadBand = 0.3f,      // 死区：0.25度（静止时更稳定）
      .Improve = PID_Derivative_On_Measurement | PID_DerivativeFilter,
  };
  PIDInit(&chassis_follow_pid, &follow_pid_config);

  /* ----------------力控策略模块初始化---------------- */
  PID_Init_Config_s force_x_pid_config = {
      .kp = 525.0f,                // 比例增益 [N/(m/s)]
      .ki = 0.0f,                  // 积分增益 [N/(m·s)]
      .kd = 0.0f,                  // 微分增益 [N·s/m]（抑制超调）
      .Derivative_LPF_RC = 0.12f,  // 微分低通滤波
      .IntegralLimit = 100.0f,     // 积分限幅 [N]
      .MaxOut = MAX_CONTROL_FORCE, // 最大输出力 [N]
      .DeadBand = 0.03f,           // 死区 [m/s]
      .Improve = PID_Integral_Limit,
  };
  PID_Init_Config_s force_y_pid_config = {
      .kp = 625.0f,
      .ki = 0.0f,
      .kd = 0.0f,
      .Derivative_LPF_RC = 0.02f,
      .IntegralLimit = 100.0f,
      .MaxOut = MAX_CONTROL_FORCE,
      .DeadBand = 0.03f,
      .Improve = PID_Integral_Limit,
  };
  PID_Init_Config_s torque_pid_config = {
      .kp = 80.0f, // 比例增益 [N·m/(rad/s)] - 从2.5降至1.5减少过冲
      .ki = 0.0f,  // 积分增益 [N·m/rad]
      .kd = 0.0f,  // 微分增益 [N·m·s/rad] - 增加阻尼抑制振荡
      .MaxOut = 100.0f,
      .DeadBand = 0.05f, // 死区 [rad/s] - 修复：从0.15增至0.3，减少回位后震荡
  };
  ChassisForceControlConfig_s force_ctrl_config = {
      .wheel_radius = RADIUS_WHEEL,
      .reduction_ratio_wheel = REDUCTION_RATIO_WHEEL,
      .half_wheel_base = HALF_WHEEL_BASE,
      .half_track_width = HALF_TRACK_WIDTH,
      .center_gimbal_offset_x = CENTER_GIMBAL_OFFSET_X,
      .center_gimbal_offset_y = CENTER_GIMBAL_OFFSET_Y,
      .torque_feedforward_coeff = chassis_config.force.torque_feedforward_coeff,
      .velocity_lpf_alpha = chassis_config.kinematics.velocity_lpf_alpha,
      .max_control_force = MAX_CONTROL_FORCE,
      .max_control_torque = MAX_CONTROL_TORQUE,
      .max_wheel_tau_ref = MAX_WHEEL_CURRENT * M3508_TORQUE_CONSTANT,
      .wheel_dynamic_tau_default =
          FRICTION_DYNAMIC_CURRENT * M3508_TORQUE_CONSTANT,
      .wheel_resistance_omega_threshold_default = 20.0f,
      .wheel_speed_feedback_tau_gain_default =
          0.005f * M3508_TORQUE_CONSTANT,
      .torque_constant = M3508_TORQUE_CONSTANT,
      .force_x_pid_config = force_x_pid_config,
      .force_y_pid_config = force_y_pid_config,
      .torque_pid_config = torque_pid_config,
  };
  ChassisForceControlInit(&force_ctrl_config, motor_rf, motor_lf, motor_lb,
                          motor_rb);

  /* ----------------底盘系统辨识初始化---------------- */
  // 初始化底盘轮速系统辨识任务（用于LQR参数标定）
  Chassis_SysIDTaskInit(motor_lf, motor_rf, motor_lb, motor_rb);
}

static void ResetFollowPidRuntimeState(void) {
  chassis_follow_pid.Measure = 0.0f;
  chassis_follow_pid.Last_Measure = 0.0f;
  chassis_follow_pid.Err = 0.0f;
  chassis_follow_pid.Last_Err = 0.0f;
  chassis_follow_pid.Last_ITerm = 0.0f;
  chassis_follow_pid.Pout = 0.0f;
  chassis_follow_pid.Iout = 0.0f;
  chassis_follow_pid.Dout = 0.0f;
  chassis_follow_pid.ITerm = 0.0f;
  chassis_follow_pid.Output = 0.0f;
  chassis_follow_pid.Last_Output = 0.0f;
  chassis_follow_pid.Last_Dout = 0.0f;
  chassis_follow_pid.Ref = 0.0f;
  chassis_follow_pid.dt = 0.0f;
  chassis_follow_pid.ERRORHandler.ERRORCount = 0U;
  chassis_follow_pid.ERRORHandler.ERRORType = PID_ERROR_NONE;
  DWT_GetDeltaT(&chassis_follow_pid.DWT_CNT);
}

void ChassisTask() {
#if ENABLE_CHASSIS_SYSID
  // 系统辨识宏开关触发（ENABLE_CHASSIS_SYSID=1时有效）
  ChassisSystemIDSwitch();
#endif

  // 检查系统辨识是否激活
  if (Chassis_SysIDIsActive()) {
    // 系统辨识激活时，完全由辨识任务控制电机
    // 底盘控制任务不干预，直接返回
    return;
  }

  uint8_t comm_online = 1U;
  uint8_t referee_online = RefereeIsOnline();
  SuperCap_Rx_Data_s cap_rx_data = {0};
  uint8_t cap_online = 0U;
  uint16_t ui_cap_energy = 0U;
  uint32_t now_ms = (uint32_t)DWT_GetTimeline_ms();
  ChassisSafetyInput_s safety_input = {0};
  ChassisSafetyResult_s safety_result = {0};

  if (cap != NULL) {
    cap_rx_data = SuperCapGetData(cap);
    if (cap->can_ins != NULL && (cap->can_ins->rx_len > 0U) &&
        ((cap_rx_data.error_code & 0x80U) == 0U)) {
      float cap_energy_value = cap_rx_data.cap_energy;
      if (cap_energy_value < 0.0f) {
        cap_energy_value = 0.0f;
      } else if (cap_energy_value > 65535.0f) {
        cap_energy_value = 65535.0f;
      }
      cap_online = 1U;
      ui_cap_energy = (uint16_t)cap_energy_value;
    }
  }

  // 双板控制链路只接受来自云台板的 CAN 指令
  comm_online = ChassisCanLinkUpdateCommand(&chassis_cmd_recv);
  if (!comm_online) {
    // 双板链路失联：强制零力并清零速度，禁止沿用旧指令
    chassis_cmd_recv.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_recv.vx = 0.0f;
    chassis_cmd_recv.vy = 0.0f;
    chassis_cmd_recv.wz = 0.0f;
    chassis_cmd_recv.offset_angle = 0.0f;
    chassis_cmd_recv.near_center_error = 0.0f;
    chassis_cmd_recv.ui_friction_on = 0U;
    chassis_cmd_recv.ui_autoaim_enabled = 0U;
  }

  // === 应用遥控器速度增益 ===
  // 从Gimbal接收到的vx/vy是归一化值(-1.0~1.0)
  // 需要乘以增益转换为实际速度(m/s)
  chassis_cmd_recv.vx *= chassis_config.rc.max_linear_speed;
  chassis_cmd_recv.vy *= chassis_config.rc.max_linear_speed;
  // 注意: wz(角速度)由底盘根据模式自动设定，不需要在这里处理

  BuildChassisSafetyInput(&safety_input, now_ms, comm_online, referee_online);
  StepChassisSafetyGate(&safety_input, &safety_result);
  if (ShouldHoldChassisControl(&chassis_cmd_recv, &safety_result)) {
    HoldChassisOutputs();
    goto feedback_only;
  }

  DJIMotorEnable(motor_lf);
  DJIMotorEnable(motor_rf);
  DJIMotorEnable(motor_lb);
  DJIMotorEnable(motor_rb);

  // 根据控制模式设定旋转速度
  switch (chassis_cmd_recv.chassis_mode) {
  case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
    chassis_cmd_recv.wz = 0;
    break;

  case CHASSIS_FOLLOW_GIMBAL_YAW: { // 跟随云台,统一力控链路
    // PID输出目标角速度（rad/s）
    // 统一进入力控PID转换为扭矩
    // 输入：角度误差（度）
    // 输出：目标角速度（rad/s）
    // 修复：交换PID参数顺序，使云台右偏时底盘顺时针旋转
    float follow_angular_vel = -PIDCalculate(
        &chassis_follow_pid, chassis_cmd_recv.near_center_error, 0.0f);
    // 对外环输出做轻微低通，降低 offset_angle/误差台阶对 wz 参考的直接冲击
    static float follow_angular_vel_filtered = 0.0f;
    follow_angular_vel = LowPassFilter_Float(follow_angular_vel, 0.85f,
                                             &follow_angular_vel_filtered);
    // wz统一为角速度（rad/s），后续通过chassis_torque_pid转换为扭矩
    chassis_cmd_recv.wz = follow_angular_vel;
    break;
  }

  case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
    chassis_cmd_recv.wz = chassis_config.kinematics.rotate_speed; // rad/s
    break;

  default:
    break;
  }

  // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
  // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
  static float sin_theta, cos_theta;
  cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
  sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
  float chassis_vx =
      chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
  float chassis_vy =
      chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

  /* ================== 力控策略控制流程 ================== */
  ChassisForceControlSetCommand(chassis_vx, chassis_vy, chassis_cmd_recv.wz);
  ChassisForceControlStep();

  const float *wheel_tau_ref = ChassisForceControlGetWheelTauRef();
  const float *target_wheel_omega = ChassisForceControlGetTargetWheelOmega();

#if POWER_CONTROLLER_ENABLE
  // 4. 功率限制（必须在发送电机指令前执行）
  // 4.1 获取超级电容数据

  // 4.1.1 通过 VOFA 上位机实时发送超级电容反馈功率（W）
  // 说明：VOFA 走 DMA/IT
  // 时不建议在高频控制环里“每次都发”，否则会打断上一次发送或占满带宽
  static float last_vofa_send_s = 0.0f;
  float now_s = DWT_GetTimeline_s();
  if ((now_s - last_vofa_send_s) >= 0.05f) { // 20Hz
    float vofa_values[1] = {cap_rx_data.chassis_power};
    (void)VofaJustFloatSend(&vofa_sender, vofa_values, 1);
    last_vofa_send_s = now_s;
  }

  // 4.2 更新超级电容在线状态
  // 在线判断：CAN有数据 且 错误码bit7=0（输出未关闭）
  PowerUpdateCapData(power_ctrl, cap_rx_data.cap_energy, cap_online);

  // 4.3 更新功率控制器数据（C键三档：100% / 80% / 120%）
  // 基准功率优先使用裁判系统上限，异常时回退到超电反馈，再回退到80W
  uint8_t robot_level = 1; // 默认等级1
  float referee_limit_w = 0.0f;
  float referee_buffer_energy = 0.0f;
  if (referee_online && referee_data != NULL) {
    if (referee_data->GameRobotState.chassis_power_limit > 1U) {
      referee_limit_w = (float)referee_data->GameRobotState.chassis_power_limit;
    }
    referee_buffer_energy = (float)referee_data->PowerHeatData.buffer_energy;
    if (referee_data->GameRobotState.robot_level >= 1 &&
        referee_data->GameRobotState.robot_level <= MAX_ROBOT_LEVEL) {
      robot_level = referee_data->GameRobotState.robot_level;
    }
  }
  uint8_t referee_limit_valid =
      (referee_online && referee_limit_w > 1.0f) ? 1U : 0U;
  float cap_limit_fallback_w = (cap_rx_data.chassis_power_limit > 1)
                                   ? (float)cap_rx_data.chassis_power_limit
                                   : 0.0f;
  float source_limit_w =
      referee_limit_valid
          ? referee_limit_w
          : ((cap_limit_fallback_w > 1.0f) ? cap_limit_fallback_w : 80.0f);
  float target_limit_w = SelectPowerLimitByMode(
      source_limit_w, chassis_cmd_recv.chassis_speed_buff, cap_online,
      cap_rx_data.cap_energy, referee_limit_valid, referee_limit_w);
  target_limit_w = ApplyPowerLimitHardClamp(target_limit_w, referee_limit_valid,
                                            referee_limit_w);

  // 反馈优先使用裁判系统数据，裁判离线时仅功率测量回退到超级电容
  float feedback_power_w = (referee_online && referee_data != NULL)
                               ? referee_data->PowerHeatData.chassis_power
                               : cap_rx_data.chassis_power; // 底盘功率 (W)

  PowerSetUserLimit(power_ctrl, target_limit_w);
  PowerUpdateRefereeData(power_ctrl, target_limit_w, referee_buffer_energy,
                         feedback_power_w);

  // 4.3.1 更新裁判系统在线状态
  PowerUpdateRefereeOnline(power_ctrl, referee_online, robot_level);

  // 4.3.2 更新电机在线状态
  PowerUpdateMotorOnline(power_ctrl, 0, DaemonIsOnline(motor_rf->daemon)); // RF
  PowerUpdateMotorOnline(power_ctrl, 1, DaemonIsOnline(motor_lf->daemon)); // LF
  PowerUpdateMotorOnline(power_ctrl, 2, DaemonIsOnline(motor_lb->daemon)); // LB
  PowerUpdateMotorOnline(power_ctrl, 3, DaemonIsOnline(motor_rb->daemon)); // RB

  // 4.4 更新电机反馈数据（用于RLS参数辨识）
  // ⚠️ 量纲统一到输出轴侧（与功率模型k1、k2参数匹配）
  // ⚠️ 参考系统一到"轮子侧"（与wheel_tau_ref一致）
  // ⚠️ 数组顺序必须与 motors[] = {RF, LF, LB, RB} 一致（556行）
  // 电机方向配置：RF=REVERSE, LF=REVERSE, LB=REVERSE, RB=REVERSE
  //    所有电机都是REVERSE，需要统一取反
  //    确保pid_output和motor_speeds参考系一致，避免"符号毁灭"导致功率预测失准
  // 输出轴角速度 = 转子角速度 / 减速比（REVERSE电机需加负号）
  float motor_speeds[4] = {
      -motor_rf->measure.speed_aps * DEGREE_2_RAD /
          REDUCTION_RATIO_WHEEL, // RF: REVERSE
      -motor_lf->measure.speed_aps * DEGREE_2_RAD /
          REDUCTION_RATIO_WHEEL, // LF: REVERSE
      -motor_lb->measure.speed_aps * DEGREE_2_RAD /
          REDUCTION_RATIO_WHEEL, // LB: REVERSE
      -motor_rb->measure.speed_aps * DEGREE_2_RAD /
          REDUCTION_RATIO_WHEEL, // RB: REVERSE
  };

  // 使用电机实测电流反馈计算转矩（而非控制指令）
  // 这对RLS参数辨识至关重要：必须使用真实消耗的电流，而非期望指令
  // real_current范围：-16384~16384，需乘以M3508_CMD_TO_CURRENT_COEFF转换为安培
  // ⚠️ 使用输出轴转矩常数（与motor_speeds的输出轴角速度匹配）
  // ⚠️ real_current也需要根据电机方向配置取反，与motor_speeds保持一致
  const float TORQUE_CONSTANT = M3508_TORQUE_CONSTANT; // 0.3 Nm/A（输出轴）
  float motor_torques[4] = {
      -motor_rf->measure.real_current * M3508_CMD_TO_CURRENT_COEFF *
          TORQUE_CONSTANT, // RF: REVERSE
      -motor_lf->measure.real_current * M3508_CMD_TO_CURRENT_COEFF *
          TORQUE_CONSTANT, // LF: REVERSE
      -motor_lb->measure.real_current * M3508_CMD_TO_CURRENT_COEFF *
          TORQUE_CONSTANT, // LB: REVERSE
      -motor_rb->measure.real_current * M3508_CMD_TO_CURRENT_COEFF *
          TORQUE_CONSTANT, // RB: REVERSE
  };
  PowerUpdateMotorFeedback(power_ctrl, motor_speeds, motor_torques);

  // 4.5 执行能量环控制和RLS更新
  PowerControllerTask(power_ctrl);

  // 4.6 功率限制：对电流进行限幅
  // ⚠️ 量纲说明：
  //   - target_wheel_omega 是目标转子角速度(rad/s)，需转换为输出轴
  //   - motor_speeds 是当前输出轴角速度(rad/s)
  //   - 功率控制器需要统一量纲，都基于输出轴侧
  PowerWheelObj_t wheel_objs[4] = {0};
  float limited_wheel_tau_ref[4] = {0};
  BuildPowerWheelObjs(wheel_tau_ref, motor_speeds, target_wheel_omega,
                      motor_torques, wheel_objs);
  PowerGetLimitedWheelTauRef(power_ctrl, wheel_objs, limited_wheel_tau_ref);

  // 5. 原生 tau 域限幅后，直接交给 DJI 电机 effort 主线
  ApplyWheelTauRef(limited_wheel_tau_ref);
#else
  // 5. 无功率限制时直接保持 tau_ref 主线，将 effort 交给 DJI motor adapter
  ApplyWheelTauRef(wheel_tau_ref);
#endif

  // 5. 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
  // LimitChassisOutput();  // 力控策略中已在ForceToCurrentConversion中完成限幅

  // 6. 根据电机的反馈速度和IMU(如果有)计算真实速度
  // EstimateSpeed();

feedback_only:
  ApplyChassisSafetyFeedback(&safety_result);
  LogChassisSafetyTransition(&safety_result, &safety_input);

  // 双板摘要链路回传底盘实际角速度，统一使用国际标准量纲 rad/s。
  chassis_feedback_data.real_wz = 0.0f;
#ifdef CHASSIS_BOARD
  if (Chassis_IMU_data != NULL) {
    chassis_feedback_data.real_wz = Chassis_IMU_data->Gyro[2];
  }
#endif
  chassis_feedback_data.referee_online = referee_online;
  if (referee_online && referee_data != NULL) {
    chassis_feedback_data.chassis_power_limit =
        referee_data->GameRobotState.chassis_power_limit;
    chassis_feedback_data.barrel_heat =
        referee_data->PowerHeatData.shooter_42mm_barrel_heat;
    chassis_feedback_data.barrel_heat_limit =
        referee_data->GameRobotState.shooter_barrel_heat_limit;
    chassis_feedback_data.barrel_cooling_value =
        referee_data->GameRobotState.shooter_barrel_cooling_value;
    chassis_feedback_data.bullet_speed_limit =
        referee_data->ShootData.bullet_speed;

    uint16_t heat_limit =
        referee_data->GameRobotState.shooter_barrel_heat_limit;
    uint16_t barrel_heat = referee_data->PowerHeatData.shooter_42mm_barrel_heat;
    uint16_t rest_heat =
        (heat_limit > barrel_heat) ? (heat_limit - barrel_heat) : 0U;
    chassis_feedback_data.rest_heat =
        (rest_heat > 255U) ? 255U : (uint8_t)rest_heat;

    chassis_feedback_data.bullet_speed =
        SelectBulletSpeedByReferee(referee_data->ShootData.bullet_speed);
  } else {
    chassis_feedback_data.chassis_power_limit = 0U;
    chassis_feedback_data.barrel_heat = 0U;
    chassis_feedback_data.barrel_heat_limit = 0U;
    chassis_feedback_data.barrel_cooling_value = 0U;
    chassis_feedback_data.rest_heat = 0;
    chassis_feedback_data.bullet_speed_limit = 0.0f;
    chassis_feedback_data.bullet_speed = SMALL_AMU_15;
  }

  SyncGeneratedUIState(comm_online, cap_online, ui_cap_energy);

  if ((now_ms - chassis_last_telemetry_ms) >= CHASSIS_TELEMETRY_PERIOD_MS) {
    LOGINFO("[chassis-link] referee=%u comm=%u safety=0x%02X reason=%u wheel=0x%02X "
            "power_mode=%d heat=%u",
            referee_online, comm_online,
            chassis_feedback_data.chassis_safety_status,
            safety_result.local_inhibit_reason, safety_input.wheel_online_mask,
            chassis_cmd_recv.chassis_speed_buff, chassis_feedback_data.rest_heat);
    chassis_last_telemetry_ms = now_ms;
  }

  // 推送反馈消息
  ChassisCanLinkSendFeedbackIfDue(&chassis_feedback_data);
}
