#include "dmmotor.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "daemon.h"
#include "general_def.h"
#include "memory.h"
#include "stdlib.h"
#include "string.h"
#include <math.h>
#include "user_lib.h"

#define RAD_TO_DEG 57.29577951308232f
#define DEG_TO_RAD 0.01745329251994329576f

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static inline float DMMotorApplyMotorDirection(const Motor_Control_Setting_s *setting,
                                               float value) {
  if (setting && setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
    return -value;
  return value;
}

static inline float DMMotorApplyFeedbackDirection(
    const Motor_Control_Setting_s *setting, float value) {
  if (setting && setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
    return -value;
  return value;
}

/* 将可选的 MIT 量程配置下沉到实例；未提供时回退到默认常量 */
static DM_MIT_Limit_s DMMotorResolveLimit(const DM_MIT_Config_s *mit_cfg) {
  DM_MIT_Limit_s limit = {
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
  };

  if (mit_cfg == NULL)
    return limit;

  const DM_MIT_Limit_s *cfg_limit = &mit_cfg->limit;

  if (cfg_limit->angle_max > cfg_limit->angle_min) {
    limit.angle_min = cfg_limit->angle_min;
    limit.angle_max = cfg_limit->angle_max;
  }
  if (cfg_limit->omega_max > cfg_limit->omega_min) {
    limit.omega_min = cfg_limit->omega_min;
    limit.omega_max = cfg_limit->omega_max;
  }
  if (cfg_limit->torque_max > cfg_limit->torque_min) {
    limit.torque_min = cfg_limit->torque_min;
    limit.torque_max = cfg_limit->torque_max;
  }
  /* 仅当上限大于下限时才采纳 kp/kd 量程，避免 span<=0 造成映射异常 */
  if (cfg_limit->kp_max > cfg_limit->kp_min) {
    limit.kp_min = cfg_limit->kp_min;
    limit.kp_max = cfg_limit->kp_max;
  }
  if (cfg_limit->kd_max > cfg_limit->kd_min) {
    limit.kd_min = cfg_limit->kd_min;
    limit.kd_max = cfg_limit->kd_max;
  }
  if (cfg_limit->current_max > 0.0f) {
    limit.current_max = cfg_limit->current_max;
  }

  return limit;
}

static inline uint32_t DMMotorTargetStdId(const DMMotorInstance *motor) {
  return motor->tx_base_id + (motor->use_pvt ? 0x300u : 0u);
}

static void DMMotorSendCommand(DMMotor_Mode_e cmd, DMMotorInstance *motor) {
  if (!motor || !motor->motor_can_instace)
    return;
  uint32_t prev_id = motor->motor_can_instace->txconf.StdId;
  motor->motor_can_instace->txconf.StdId = DMMotorTargetStdId(motor);
  memset(motor->motor_can_instace->tx_buff, 0xff, 8);
  motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd;
  CANTransmit(motor->motor_can_instace, 1);
  motor->motor_can_instace->txconf.StdId = prev_id;
}

static void DMMotorDecode(CANInstance *motor_can) {
  uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
  uint8_t *rxbuff = motor_can->rx_buff;
  DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
  DM_Motor_Measure_s *measure =
      &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

  DaemonReload(motor->motor_daemon);

  measure->last_position = measure->position;
  tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
  measure->position =
      uint_to_float(tmp, motor->limit.angle_min, motor->limit.angle_max, 16);

  tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
  measure->velocity =
      uint_to_float(tmp, motor->limit.omega_min, motor->limit.omega_max, 12);

  tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
  measure->torque =
      uint_to_float(tmp, motor->limit.torque_min, motor->limit.torque_max, 12);

  measure->T_Mos = (float)rxbuff[6];
  measure->T_Rotor = (float)rxbuff[7];
}

static void DMMotorLostCallback(void *motor_ptr) {
  DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
  uint16_t can_bus = motor->motor_can_instace->can_handle == &hcan1 ? 1 : 2;
  LOGWARNING("[dm_motor] Motor lost, can bus [%d], tx_id [0x%X]", can_bus,
             motor->motor_can_instace->tx_id);
  motor->stop_flag = MOTOR_STOP;
}
void DMMotorCaliEncoder(DMMotorInstance *motor) {
  DMMotorSendCommand(DM_CMD_ZERO_POSITION, motor);
  DWT_Delay(0.1);
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config) {
  DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
  memset(motor, 0, sizeof(DMMotorInstance));

  motor->motor_settings = config->controller_setting_init_config;
  PIDInit(&motor->current_PID,
          &config->controller_param_init_config.current_PID);
  PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
  PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);

  motor->mit_default_kp = config->mit_config.default_kp;
  motor->mit_default_kd = config->mit_config.default_kd;

  motor->mit_target_angle = 0.0f;
  motor->mit_target_velocity = 0.0f;
  motor->mit_target_torque = 0.0f;
  motor->mit_target_kp = 0.0f;
  motor->mit_target_kd = 0.0f;
  motor->mit_direct_velocity = 0.0f;
  motor->mit_direct_torque_ff = 0.0f;
  motor->mit_direct_kd = 0.0f;
  motor->mit_use_direct_velocity = 0;
  motor->angle_ref_deg = 0.0f;
  motor->torque_ff_cmd = 0.0f;
  motor->kd_cmd = config->mit_config.default_kd;
  motor->use_pid_path = 0;
  motor->use_raw_mit = 0;
  motor->limit = DMMotorResolveLimit(&config->mit_config);
  motor->mode = DM_MODE_MIT;
  motor->pvt_cfg = config->pvt_config;
  if (motor->pvt_cfg.v_limit_max <= 0.0f ||
      motor->pvt_cfg.v_limit_max > 100.0f) {
    motor->pvt_cfg.v_limit_max =
        (motor->limit.omega_max > 0.0f) ? motor->limit.omega_max : 100.0f;
  }
  if (motor->pvt_cfg.i_limit_max <= 0.0f || motor->pvt_cfg.i_limit_max > 1.0f) {
    motor->pvt_cfg.i_limit_max = 1.0f;
  }
  motor->pvt_pos = 0.0f;
  motor->pvt_v_lim = 0.0f;
  motor->pvt_i_lim = 0.0f;
  motor->use_pvt = 0;
  motor->stop_flag = MOTOR_STOP;

  motor->other_angle_feedback_ptr =
      config->controller_param_init_config.other_angle_feedback_ptr;
  motor->other_speed_feedback_ptr =
      config->controller_param_init_config.other_speed_feedback_ptr;

  config->can_init_config.can_module_callback = DMMotorDecode;
  config->can_init_config.id = motor;
  motor->motor_can_instace = CANRegister(&config->can_init_config);
  motor->tx_base_id = motor->motor_can_instace->txconf.StdId;

  Daemon_Init_Config_s conf = {
      .callback = DMMotorLostCallback,
      .owner_id = motor,
      .reload_count = 10,
  };
  motor->motor_daemon = DaemonRegister(&conf);

  /* 固定上电流程：清错 -> 进入工作模式 */
  DMMotorEnable(motor);
  dm_motor_instance[idx++] = motor;
  return motor;
}

void DMMotorSetMode(DMMotorInstance *motor, DM_Mode_e mode) {
  if (!motor || !motor->motor_can_instace)
    return;
  motor->mode = mode;
  motor->use_pvt = (mode == DM_MODE_PVT);
  uint32_t prev_id = motor->motor_can_instace->txconf.StdId;
  motor->motor_can_instace->txconf.StdId = 0x7FF;
  uint32_t can_id = motor->tx_base_id;
  motor->motor_can_instace->tx_buff[0] = (uint8_t)(can_id & 0xFF);
  motor->motor_can_instace->tx_buff[1] = (uint8_t)((can_id >> 8) & 0xFF);
  motor->motor_can_instace->tx_buff[2] = 0x55;
  motor->motor_can_instace->tx_buff[3] = 0x0A;
  uint32_t mode_val = (uint32_t)mode;
  memcpy(&motor->motor_can_instace->tx_buff[4], &mode_val, sizeof(uint32_t));
  CANTransmit(motor->motor_can_instace, 1);
  motor->motor_can_instace->txconf.StdId = prev_id;
}

void DMMotorSetRef(DMMotorInstance *motor, float angle_deg, float torque_ff) {
  if (!motor)
    return;
  motor->use_pvt = 0;
  motor->angle_ref_deg = angle_deg;
  motor->torque_ff_cmd = torque_ff;
  motor->mit_use_direct_velocity = 0;
  motor->use_pid_path = 1;
  motor->use_raw_mit = 0;
}

void DMMotorSetMITTargets(DMMotorInstance *motor, float angle, float omega,
                          float torque, float kp, float kd) {
  if (!motor)
    return;
  motor->use_pvt = 0;
  motor->mit_use_direct_velocity = 0;
  motor->use_pid_path = 0;
  motor->use_raw_mit = 1;
  motor->mit_target_angle = angle;
  motor->mit_target_velocity = omega;
  motor->mit_target_torque = torque;
  motor->mit_target_kp = kp;
  motor->mit_target_kd = kd;
  motor->pid_ref = torque; // 兼容旧流程，力矩优先
}

void DMMotorSetMITVelocity(DMMotorInstance *motor, float omega, float torque_ff,
                           float kd) {
  if (!motor)
    return;
  motor->use_pvt = 0;
  motor->mit_direct_velocity = omega;
  motor->mit_direct_torque_ff = torque_ff;
  motor->mit_direct_kd = kd;
  motor->mit_use_direct_velocity = 1;
  motor->use_pid_path = 0;
  motor->use_raw_mit = 0;
}

void DMMotorSetPVT(DMMotorInstance *motor, float pos_rad,
                   float v_limit_rad_s, float i_ratio) {
  if (!motor)
    return;
  motor->use_pvt = 1;
  motor->use_pid_path = 0;
  motor->use_raw_mit = 0;
  motor->mit_use_direct_velocity = 0;
  motor->pvt_pos = pos_rad;
  motor->pvt_v_lim =
      float_constrain(fabsf(v_limit_rad_s), 0.0f, motor->pvt_cfg.v_limit_max);
  motor->pvt_i_lim =
      float_constrain(fabsf(i_ratio), 0.0f, motor->pvt_cfg.i_limit_max);
}

void DMMotorEnable(DMMotorInstance *motor) {
  if (!motor)
    return;
  if (motor->stop_flag == MOTOR_ENALBED)
    return;
  DMMotorSendCommand(DM_CMD_CLEAR_ERROR, motor);
  DWT_Delay(0.01);
  DMMotorSendCommand(DM_CMD_MOTOR_MODE, motor);
  motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor) {
  if (!motor)
    return;
  if (motor->stop_flag == MOTOR_STOP)
    return;
  DMMotorSendCommand(DM_CMD_RESET_MODE, motor);
  motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type) {
  motor->motor_settings.outer_loop_type = type;
}

//@Todo: 目前只实现了力控，更多位控PID等请自行添加
void DMMotorTask(void const *argument) {
  DMMotorInstance *motor = (DMMotorInstance *)argument;
  Motor_Control_Setting_s *setting = &motor->motor_settings;
  DMMotor_Send_s motor_send_mailbox;
  const float speed_limit_deg_max = motor->limit.omega_max * RAD_TO_DEG;
  const float speed_limit_deg_min = motor->limit.omega_min * RAD_TO_DEG;
  while (1) {
    if (motor->measure.T_Mos > 100.0f || motor->measure.T_Rotor > 100.0f) {
      motor->stop_flag = MOTOR_STOP;
    }

    /* PVT 通道：直接发送力位混控帧 */
    if (motor->use_pvt) {
      uint32_t prev_id = motor->motor_can_instace->txconf.StdId;
      motor->motor_can_instace->txconf.StdId = DMMotorTargetStdId(motor);
      if (motor->stop_flag == MOTOR_STOP) {
        memset(motor->motor_can_instace->tx_buff, 0xff, 8);
        motor->motor_can_instace->tx_buff[7] = (uint8_t)DM_CMD_RESET_MODE;
      } else {
        float pos = motor->pvt_pos;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
          pos = -pos;
        }
        uint16_t v_raw = (uint16_t)(motor->pvt_v_lim * 100.0f);
        if (v_raw > 10000)
          v_raw = 10000;
        uint16_t i_raw = (uint16_t)(motor->pvt_i_lim * 10000.0f);
        if (i_raw > 10000)
          i_raw = 10000;
        memcpy(motor->motor_can_instace->tx_buff, &pos, sizeof(float));
        motor->motor_can_instace->tx_buff[4] = (uint8_t)(v_raw & 0xFF);
        motor->motor_can_instace->tx_buff[5] = (uint8_t)(v_raw >> 8);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)(i_raw & 0xFF);
        motor->motor_can_instace->tx_buff[7] = (uint8_t)(i_raw >> 8);
      }
      CANTransmit(motor->motor_can_instace, 1);
      motor->motor_can_instace->txconf.StdId = prev_id;
      osDelay(1);
      continue;
    }

    float target_angle = 0.0f;
    float target_velocity = 0.0f;
    float target_torque = 0.0f;
    float target_kp = 0.0f;
    float target_kd = motor->mit_default_kd;

    if (motor->stop_flag == MOTOR_STOP) {
      goto pack_and_send;
    }

    if (motor->mit_use_direct_velocity) {
      /* 速度直控：位置/Kp 置零，仅速度+阻尼+力矩前馈 */
      target_angle = 0.0f;
      target_kp = 0.0f;
      target_velocity = motor->mit_direct_velocity;
      target_torque = motor->mit_direct_torque_ff;
      target_kd = (motor->mit_direct_kd > 0.0f) ? motor->mit_direct_kd
                                                : motor->mit_default_kd;
    } else if (motor->use_raw_mit) {
      /* 原生 MIT 目标：兼容系统辨识/其他直发场景 */
      target_angle = motor->mit_target_angle;
      target_velocity = motor->mit_target_velocity;
      target_torque = (motor->mit_target_torque != 0.0f)
                          ? motor->mit_target_torque
                          : motor->pid_ref;
      target_kp = (motor->mit_target_kp > 0.0f) ? motor->mit_target_kp
                                                : motor->mit_default_kp;
      target_kd = (motor->mit_target_kd > 0.0f) ? motor->mit_target_kd
                                                : motor->mit_default_kd;
    } else if (motor->use_pid_path) {
      /* module 内部：角度(°) -> 速度(°/s) -> 力矩(Nm) */
      float angle_ref_deg = motor->angle_ref_deg;
      float torque_ff = motor->torque_ff_cmd;

      float angle_feedback_deg = (motor->other_angle_feedback_ptr)
                                     ? *motor->other_angle_feedback_ptr
                                     : 0.0f;
      float speed_feedback_deg =
          (motor->other_speed_feedback_ptr)
              ? (*motor->other_speed_feedback_ptr) * RAD_TO_DEG
              : 0.0f;

      angle_feedback_deg =
          DMMotorApplyFeedbackDirection(setting, angle_feedback_deg);
      speed_feedback_deg =
          DMMotorApplyFeedbackDirection(setting, speed_feedback_deg);

      float speed_ref_deg =
          PIDCalculate(&motor->angle_PID, angle_feedback_deg, angle_ref_deg);
      speed_ref_deg = float_constrain(speed_ref_deg, speed_limit_deg_min,
                                      speed_limit_deg_max);

      /* 核心修改：速度环交给电机端 Kd，MCU 只提供重力等力矩前馈 */
      target_torque = torque_ff;
      target_angle = 0.0f; // MIT 位置锁定为 0
      target_kp = 0.0f;    // 位置刚度锁零，避免与 MCU 位置环冲突
      target_velocity = speed_ref_deg * DEG_TO_RAD; // 速度帧用 rad/s
      target_kd =
          (motor->kd_cmd > 0.0f) ? motor->kd_cmd : motor->mit_default_kd;
    } else {
      /* 默认兜底：保持零输出 */
      target_angle = 0.0f;
      target_velocity = 0.0f;
      target_torque = 0.0f;
      target_kp = 0.0f;
      target_kd = motor->mit_default_kd;
    }

    if (motor->current_feedforward_ptr)
      target_torque += *motor->current_feedforward_ptr;

    /* 电机方向映射：将“逻辑目标”转换到电机物理方向 */
    target_angle = DMMotorApplyMotorDirection(setting, target_angle);
    target_velocity = DMMotorApplyMotorDirection(setting, target_velocity);
    target_torque = DMMotorApplyMotorDirection(setting, target_torque);

    LIMIT_MIN_MAX(target_angle, motor->limit.angle_min, motor->limit.angle_max);
    LIMIT_MIN_MAX(target_velocity, motor->limit.omega_min, motor->limit.omega_max);
    LIMIT_MIN_MAX(target_torque, motor->limit.torque_min, motor->limit.torque_max);
    LIMIT_MIN_MAX(target_kp, motor->limit.kp_min, motor->limit.kp_max);
    LIMIT_MIN_MAX(target_kd, motor->limit.kd_min, motor->limit.kd_max);

  pack_and_send:
    motor_send_mailbox.position_des =
        float_to_uint(target_angle, motor->limit.angle_min, motor->limit.angle_max,
                      16);
    motor_send_mailbox.velocity_des =
        float_to_uint(target_velocity, motor->limit.omega_min, motor->limit.omega_max,
                      12);
    motor_send_mailbox.torque_des =
        float_to_uint(target_torque, motor->limit.torque_min, motor->limit.torque_max,
                      12);
    motor_send_mailbox.Kp =
        float_to_uint(target_kp, motor->limit.kp_min, motor->limit.kp_max, 12);
    motor_send_mailbox.Kd =
        float_to_uint(target_kd, motor->limit.kd_min, motor->limit.kd_max, 12);

    if (motor->stop_flag == MOTOR_STOP) {
      motor_send_mailbox.torque_des =
          float_to_uint(0, motor->limit.torque_min, motor->limit.torque_max, 12);
      motor_send_mailbox.velocity_des =
          float_to_uint(0, motor->limit.omega_min, motor->limit.omega_max, 12);
    }

    motor->motor_can_instace->tx_buff[0] =
        (uint8_t)(motor_send_mailbox.position_des >> 8);
    motor->motor_can_instace->tx_buff[1] =
        (uint8_t)(motor_send_mailbox.position_des);
    motor->motor_can_instace->tx_buff[2] =
        (uint8_t)(motor_send_mailbox.velocity_des >> 4);
    motor->motor_can_instace->tx_buff[3] =
        (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) |
                  (motor_send_mailbox.Kp >> 8));
    motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
    motor->motor_can_instace->tx_buff[5] =
        (uint8_t)(motor_send_mailbox.Kd >> 4);
    motor->motor_can_instace->tx_buff[6] =
        (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) |
                  (motor_send_mailbox.torque_des >> 8));
    motor->motor_can_instace->tx_buff[7] =
        (uint8_t)(motor_send_mailbox.torque_des);

    CANTransmit(motor->motor_can_instace, 1);

    osDelay(1);
  }
}
void DMMotorControlInit() {
  char dm_task_name[5] = "dm";
  // 遍历所有电机实例,创建任务
  if (!idx)
    return;
  for (size_t i = 0; i < idx; i++) {
    char dm_id_buff[2] = {0};
    __itoa(i, dm_id_buff, 10);
    strcat(dm_task_name, dm_id_buff);
    osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
    dm_task_handle[i] =
        osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
  }
}
