#include "dmmotor.h"
#include "dmmotor_adapter.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "daemon.h"
#include "general_def.h"
#include "memory.h"
#include "stdlib.h"
#include "string.h"
#include "user_lib.h"

#include <math.h>

#define RAD_TO_DEG 57.29577951308232f
#define DEG_TO_RAD 0.01745329251994329576f

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];

/* 将浮点物理量映射到达妙协议原始整数 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/* 将达妙协议原始整数还原成浮点物理量 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static inline float DMMotorApplyMotorDirection(
    const Motor_Control_Setting_s *setting, float value) {
  if (setting && setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
    return -value;
  }
  return value;
}

static inline float DMMotorApplyFeedbackDirection(
    const Motor_Control_Setting_s *setting, float value) {
  if (setting && setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) {
    return -value;
  }
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

  if (mit_cfg == NULL) {
    return limit;
  }

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

static uint8_t DMMotorProfileIsZero(const DM_MIT_Profile_s *profile) {
  if (profile == NULL) {
    return 1U;
  }

  return (profile->kp == 0.0f && profile->kd == 0.0f &&
          profile->v_des == 0.0f && profile->torque_des == 0.0f)
             ? 1U
             : 0U;
}

static DM_MIT_Profile_s
DMMotorResolveMITProfile(const DM_MIT_Config_s *mit_cfg, uint8_t use_vision) {
  DM_MIT_Profile_s manual_profile = {
      .kp = (mit_cfg != NULL) ? mit_cfg->default_kp : 0.0f,
      .kd = (mit_cfg != NULL) ? mit_cfg->default_kd : 0.0f,
      .v_des = 0.0f,
      .torque_des = 0.0f,
  };
  DM_MIT_Profile_s vision_profile = manual_profile;

  if (mit_cfg == NULL) {
    return use_vision ? vision_profile : manual_profile;
  }

  if (!DMMotorProfileIsZero(&mit_cfg->manual_profile)) {
    manual_profile = mit_cfg->manual_profile;
  }
  if (!DMMotorProfileIsZero(&mit_cfg->vision_profile)) {
    vision_profile = mit_cfg->vision_profile;
  }

  return use_vision ? vision_profile : manual_profile;
}

static inline uint32_t DMMotorTargetStdId(const DMMotorInstance *motor) {
  return motor->command_tx_id + (motor->use_pvt_command_frame ? 0x300u : 0u);
}

static uint8_t DMMotorNeedEnableRetry(const DMMotorInstance *motor) {
  if (!motor) {
    return 0U;
  }

  if (motor->motor_daemon == NULL || !DaemonIsOnline(motor->motor_daemon)) {
    return 1U;
  }

  return (motor->measure.motor_state == DM_FB_STATE_DISABLED) ? 1U : 0U;
}

static void DMMotorClearDirectEffort(DMMotorInstance *motor) {
  if (!motor) {
    return;
  }
  memset(&motor->ref_effort, 0, sizeof(motor->ref_effort));
}

static void DMMotorSendCommand(DMMotor_Mode_e cmd, DMMotorInstance *motor) {
  if (!motor || !motor->motor_can_instance) {
    return;
  }

  uint32_t prev_id = motor->motor_can_instance->txconf.StdId;
  motor->motor_can_instance->txconf.StdId = DMMotorTargetStdId(motor);
  memset(motor->motor_can_instance->tx_buff, 0xff, 8);
  motor->motor_can_instance->tx_buff[7] = (uint8_t)cmd;
  CANTransmit(motor->motor_can_instance, 1);
  motor->motor_can_instance->txconf.StdId = prev_id;
}

static void DMMotorDecode(CANInstance *motor_can) {
  uint16_t tmp;
  uint8_t *rxbuff = motor_can->rx_buff;
  DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
  DM_Motor_Measure_s *measure = &motor->measure;

  DaemonReload(motor->motor_daemon);

  measure->motor_id = rxbuff[0] & 0x0f;
  measure->motor_state = (rxbuff[0] >> 4) & 0x0f;
  if (measure->motor_state == DM_FB_STATE_ENABLED) {
    motor->stop_flag = MOTOR_ENALBED;
  } else if (measure->motor_state == DM_FB_STATE_DISABLED) {
    motor->stop_flag = MOTOR_STOP;
  }

  measure->last_output_angle_rad = measure->output_angle_rad;
  tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
  measure->output_angle_rad = uint_to_float(
      tmp, motor->mit_limit.angle_min, motor->mit_limit.angle_max, 16);

  tmp = (uint16_t)((rxbuff[3] << 4) | (rxbuff[4] >> 4));
  measure->output_velocity_rad_s =
      uint_to_float(tmp, motor->mit_limit.omega_min, motor->mit_limit.omega_max,
                    12);

  tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
  measure->output_torque_nm = uint_to_float(
      tmp, motor->mit_limit.torque_min, motor->mit_limit.torque_max, 12);

  measure->mos_temp_c = (float)rxbuff[6];
  measure->rotor_temp_c = (float)rxbuff[7];
}

static void DMMotorLostCallback(void *motor_ptr) {
  DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
  uint16_t can_bus = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
  LOGWARNING("[dm_motor] Motor lost, can bus [%d], tx_id [0x%X]", can_bus,
             motor->motor_can_instance->tx_id);
  motor->measure.motor_state = DM_FB_STATE_COMMUNICATION_LOST;
  motor->stop_flag = MOTOR_STOP;
  motor->lost_count++;
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

  motor->mit_default_stiffness_kp = config->mit_config.default_kp;
  motor->mit_default_damping_kd = config->mit_config.default_kd;
  motor->mit_manual_profile = DMMotorResolveMITProfile(&config->mit_config, 0U);
  motor->mit_vision_profile = DMMotorResolveMITProfile(&config->mit_config, 1U);
  motor->active_mit_profile = DM_MIT_PROFILE_MANUAL;

  motor->mit_limit = DMMotorResolveLimit(&config->mit_config);
  motor->drive_mode = DM_MODE_MIT;
  motor->measure.motor_state = DM_FB_STATE_DISABLED;
  motor->pvt_config = config->pvt_config;
  if (motor->pvt_config.v_limit_max <= 0.0f ||
      motor->pvt_config.v_limit_max > 100.0f) {
    motor->pvt_config.v_limit_max =
        (motor->mit_limit.omega_max > 0.0f) ? motor->mit_limit.omega_max
                                            : 100.0f;
  }
  if (motor->pvt_config.i_limit_max <= 0.0f ||
      motor->pvt_config.i_limit_max > 1.0f) {
    motor->pvt_config.i_limit_max = 1.0f;
  }

  motor->external_angle_feedback_ptr =
      config->controller_param_init_config.other_angle_feedback_ptr;
  motor->external_speed_feedback_ptr =
      config->controller_param_init_config.other_speed_feedback_ptr;
  motor->external_speed_feedforward_ptr = NULL;
  motor->external_torque_feedforward_ptr = NULL;

  config->can_init_config.can_module_callback = DMMotorDecode;
  config->can_init_config.id = motor;
  motor->motor_can_instance = CANRegister(&config->can_init_config);
  motor->command_tx_id = motor->motor_can_instance->txconf.StdId;

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
  if (!motor || !motor->motor_can_instance) {
    return;
  }

  motor->drive_mode = mode;
  motor->use_pvt_command_frame = (mode == DM_MODE_PVT);

  uint32_t prev_id = motor->motor_can_instance->txconf.StdId;
  motor->motor_can_instance->txconf.StdId = 0x7FF;
  uint32_t can_id = motor->command_tx_id;
  motor->motor_can_instance->tx_buff[0] = (uint8_t)(can_id & 0xFF);
  motor->motor_can_instance->tx_buff[1] = (uint8_t)((can_id >> 8) & 0xFF);
  motor->motor_can_instance->tx_buff[2] = 0x55;
  motor->motor_can_instance->tx_buff[3] = 0x0A;
  uint32_t mode_val = (uint32_t)mode;
  memcpy(&motor->motor_can_instance->tx_buff[4], &mode_val, sizeof(uint32_t));
  CANTransmit(motor->motor_can_instance, 1);
  motor->motor_can_instance->txconf.StdId = prev_id;
}

void DMMotorSetRef(DMMotorInstance *motor, float angle_rad, float torque_ff) {
  if (!motor) {
    return;
  }

  DMMotorClearDirectEffort(motor);
  motor->use_pvt_command_frame = 0;
  motor->use_mit_velocity_only = 0;
  motor->use_cascade_pid_path = 1;
  motor->use_mit_full_command = 0;
  motor->cascade_angle_ref_rad = angle_rad;
  motor->cascade_torque_ff_nm = torque_ff;
}

uint8_t DMMotorCalculateTorqueEffort(DMMotorInstance *motor, float angle_rad,
                                     float torque_ff,
                                     Controller_Effort_Output_s *effort) {
  Motor_Control_Setting_s *setting = NULL;
  float angle_feedback_rad = 0.0f;
  float speed_feedback_rad_s = 0.0f;
  float speed_ref_rad_s = 0.0f;
  float torque_ref_nm = 0.0f;

  if (!motor || !effort) {
    return 0U;
  }

  setting = &motor->motor_settings;
  angle_feedback_rad =
      (motor->external_angle_feedback_ptr) ? *motor->external_angle_feedback_ptr
                                           : 0.0f;
  speed_feedback_rad_s =
      (motor->external_speed_feedback_ptr) ? *motor->external_speed_feedback_ptr
                                           : 0.0f;

  angle_feedback_rad =
      DMMotorApplyFeedbackDirection(setting, angle_feedback_rad);
  speed_feedback_rad_s =
      DMMotorApplyFeedbackDirection(setting, speed_feedback_rad_s);

  speed_ref_rad_s =
      PIDCalculate(&motor->angle_PID, angle_feedback_rad, angle_rad);
  speed_ref_rad_s = float_constrain(speed_ref_rad_s, motor->mit_limit.omega_min,
                                    motor->mit_limit.omega_max);
  if (motor->external_speed_feedforward_ptr) {
    speed_ref_rad_s += *motor->external_speed_feedforward_ptr;
  }

  torque_ref_nm =
      PIDCalculate(&motor->speed_PID, speed_feedback_rad_s, speed_ref_rad_s);
  torque_ref_nm += torque_ff;
  if (motor->external_torque_feedforward_ptr) {
    torque_ref_nm += *motor->external_torque_feedforward_ptr;
  }
  torque_ref_nm =
      float_constrain(torque_ref_nm, motor->mit_limit.torque_min,
                      motor->mit_limit.torque_max);

  memset(effort, 0, sizeof(*effort));
  effort->semantic = CONTROLLER_OUTPUT_TAU_REF;
  effort->tau_ref_nm = torque_ref_nm;
  return 1U;
}

void DMMotorSetEffort(DMMotorInstance *motor,
                      const Controller_Effort_Output_s *effort) {
  if (!motor) {
    return;
  }

  DMMotorClearDirectEffort(motor);
  if (effort != NULL) {
    motor->ref_effort = *effort;
  }
  motor->use_pvt_command_frame = 0;
  motor->use_mit_velocity_only = 0;
  motor->use_cascade_pid_path = 0;
  motor->use_mit_full_command = 0;
}

void DMMotorSetMITTargets(DMMotorInstance *motor, float angle, float omega,
                          float torque, float kp, float kd) {
  if (!motor) {
    return;
  }

  DMMotorClearDirectEffort(motor);
  motor->use_pvt_command_frame = 0;
  motor->use_mit_velocity_only = 0;
  motor->use_cascade_pid_path = 0;
  motor->use_mit_full_command = 1;
  motor->mit_target_angle_rad = angle;
  motor->mit_target_velocity_rad_s = omega;
  motor->mit_target_torque_nm = torque;
  motor->mit_target_stiffness_kp = kp;
  motor->mit_target_damping_kd = kd;
  motor->mit_torque_fallback_nm = torque;
}

void DMMotorSelectMITProfile(DMMotorInstance *motor, DM_MIT_Profile_e profile) {
  if (!motor) {
    return;
  }

  motor->active_mit_profile = profile;
}

void DMMotorSetMITTargetByProfile(DMMotorInstance *motor, float angle) {
  DM_MIT_Profile_s *profile = NULL;

  if (!motor) {
    return;
  }

  profile = (motor->active_mit_profile == DM_MIT_PROFILE_VISION)
                ? &motor->mit_vision_profile
                : &motor->mit_manual_profile;

  DMMotorSetMITTargets(motor, angle, profile->v_des, profile->torque_des,
                       profile->kp, profile->kd);
}

void DMMotorSetMITVelocity(DMMotorInstance *motor, float omega, float torque_ff,
                           float kd) {
  if (!motor) {
    return;
  }

  DMMotorClearDirectEffort(motor);
  motor->use_pvt_command_frame = 0;
  motor->use_mit_velocity_only = 1;
  motor->use_cascade_pid_path = 0;
  motor->use_mit_full_command = 0;
  motor->mit_velocity_only_rad_s = omega;
  motor->mit_velocity_only_torque_ff_nm = torque_ff;
  motor->mit_velocity_only_damping_kd = kd;
}

void DMMotorSetPVT(DMMotorInstance *motor, float pos_rad,
                   float v_limit_rad_s, float i_ratio) {
  if (!motor) {
    return;
  }

  DMMotorClearDirectEffort(motor);
  motor->use_pvt_command_frame = 1;
  motor->use_cascade_pid_path = 0;
  motor->use_mit_full_command = 0;
  motor->use_mit_velocity_only = 0;
  motor->pvt_target_angle_rad = pos_rad;
  motor->pvt_velocity_limit_rad_s = float_constrain(
      fabsf(v_limit_rad_s), 0.0f, motor->pvt_config.v_limit_max);
  motor->pvt_current_limit_ratio =
      float_constrain(fabsf(i_ratio), 0.0f, motor->pvt_config.i_limit_max);
}

void DMMotorEnable(DMMotorInstance *motor) {
  if (!motor) {
    return;
  }
  if (motor->stop_flag == MOTOR_ENALBED &&
      motor->measure.motor_state == DM_FB_STATE_ENABLED) {
    return;
  }
  if (motor->stop_flag == MOTOR_ENALBED && !DMMotorNeedEnableRetry(motor)) {
    return;
  }

  uint32_t now_ms = (uint32_t)DWT_GetTimeline_ms();
  if (motor->last_enable_cmd_ms != 0U &&
      (now_ms - motor->last_enable_cmd_ms) < DM_ENABLE_RETRY_INTERVAL_MS) {
    return;
  }

  DMMotorSendCommand(DM_CMD_CLEAR_ERROR, motor);
  DWT_Delay(0.01);
  DMMotorSendCommand(DM_CMD_MOTOR_MODE, motor);
  motor->stop_flag = MOTOR_ENALBED;
  motor->last_enable_cmd_ms = now_ms;
}

void DMMotorStop(DMMotorInstance *motor) {
  if (!motor) {
    return;
  }
  if (motor->stop_flag == MOTOR_STOP) {
    return;
  }

  DMMotorSendCommand(DM_CMD_RESET_MODE, motor);
  motor->stop_flag = MOTOR_STOP;
  motor->last_enable_cmd_ms = 0U;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type) {
  motor->motor_settings.outer_loop_type = type;
}

//@Todo: 目前只实现了力控，更多位控PID等请自行添加
void DMMotorTask(void const *argument) {
  DMMotorInstance *motor = (DMMotorInstance *)argument;
  Motor_Control_Setting_s *setting = &motor->motor_settings;
  DMMotor_Send_s motor_send_mailbox;

  while (1) {
    Controller_Effort_Output_s effort_output = {
        .semantic = CONTROLLER_OUTPUT_TAU_REF,
    };
    Controller_Effort_Output_s normalized_output = {0};
    Actuator_Command_s actuator_command = {0};
    if (motor->measure.mos_temp_c > 100.0f ||
        motor->measure.rotor_temp_c > 100.0f) {
      motor->stop_flag = MOTOR_STOP;
    }

    /* PVT 帧：对应说明书里的位置/速度上限/电流比例三元组 */
    if (motor->use_pvt_command_frame) {
      uint32_t prev_id = motor->motor_can_instance->txconf.StdId;
      motor->motor_can_instance->txconf.StdId = DMMotorTargetStdId(motor);
      if (motor->stop_flag == MOTOR_STOP) {
        memset(motor->motor_can_instance->tx_buff, 0xff, 8);
        motor->motor_can_instance->tx_buff[7] = (uint8_t)DM_CMD_RESET_MODE;
      } else {
        float pos = motor->pvt_target_angle_rad;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
          pos = -pos;
        }

        uint16_t v_raw =
            (uint16_t)(motor->pvt_velocity_limit_rad_s * 100.0f);
        if (v_raw > 10000) {
          v_raw = 10000;
        }

        uint16_t i_raw =
            (uint16_t)(motor->pvt_current_limit_ratio * 10000.0f);
        if (i_raw > 10000) {
          i_raw = 10000;
        }

        memcpy(motor->motor_can_instance->tx_buff, &pos, sizeof(float));
        motor->motor_can_instance->tx_buff[4] = (uint8_t)(v_raw & 0xFF);
        motor->motor_can_instance->tx_buff[5] = (uint8_t)(v_raw >> 8);
        motor->motor_can_instance->tx_buff[6] = (uint8_t)(i_raw & 0xFF);
        motor->motor_can_instance->tx_buff[7] = (uint8_t)(i_raw >> 8);
      }
      CANTransmit(motor->motor_can_instance, 1);
      motor->motor_can_instance->txconf.StdId = prev_id;
      osDelay(1);
      continue;
    }

    float target_angle = 0.0f;
    float target_velocity = 0.0f;
    float target_torque = 0.0f;
    float target_kp = 0.0f;
    float target_kd = motor->mit_default_damping_kd;

    if (motor->stop_flag == MOTOR_STOP) {
      goto pack_and_send;
    }

    if (motor->ref_effort.semantic != CONTROLLER_OUTPUT_INVALID) {
      target_angle = 0.0f;
      target_velocity = 0.0f;
      target_torque = motor->ref_effort.tau_ref_nm;
      target_kp = 0.0f;
      target_kd = 0.0f;
    } else if (motor->use_mit_velocity_only) {
      target_angle = 0.0f;
      target_kp = 0.0f;
      target_velocity = motor->mit_velocity_only_rad_s;
      target_torque = motor->mit_velocity_only_torque_ff_nm;
      target_kd = (motor->mit_velocity_only_damping_kd > 0.0f)
                      ? motor->mit_velocity_only_damping_kd
                      : motor->mit_default_damping_kd;
    } else if (motor->use_mit_full_command) {
      target_angle = motor->mit_target_angle_rad;
      target_velocity = motor->mit_target_velocity_rad_s;
      target_torque = (motor->mit_target_torque_nm != 0.0f)
                          ? motor->mit_target_torque_nm
                          : motor->mit_torque_fallback_nm;
      target_kp = (motor->mit_target_stiffness_kp > 0.0f)
                      ? motor->mit_target_stiffness_kp
                      : motor->mit_default_stiffness_kp;
      target_kd = (motor->mit_target_damping_kd > 0.0f)
                      ? motor->mit_target_damping_kd
                      : motor->mit_default_damping_kd;
    } else if (motor->use_cascade_pid_path) {
      float angle_ref_rad = motor->cascade_angle_ref_rad;
      float torque_ff = motor->cascade_torque_ff_nm;

      float angle_feedback_rad = (motor->external_angle_feedback_ptr)
                                     ? *motor->external_angle_feedback_ptr
                                     : 0.0f;
      float speed_feedback_rad_s =
          (motor->external_speed_feedback_ptr)
              ? *motor->external_speed_feedback_ptr
              : 0.0f;

      angle_feedback_rad =
          DMMotorApplyFeedbackDirection(setting, angle_feedback_rad);
      speed_feedback_rad_s =
          DMMotorApplyFeedbackDirection(setting, speed_feedback_rad_s);

      float speed_ref_rad_s =
          PIDCalculate(&motor->angle_PID, angle_feedback_rad, angle_ref_rad);
      speed_ref_rad_s =
          float_constrain(speed_ref_rad_s, motor->mit_limit.omega_min,
                          motor->mit_limit.omega_max);

      if (motor->external_speed_feedforward_ptr) {
        speed_ref_rad_s += *motor->external_speed_feedforward_ptr;
      }

      float torque_ref_nm = PIDCalculate(&motor->speed_PID,
                                         speed_feedback_rad_s, speed_ref_rad_s);

      target_torque = torque_ref_nm + torque_ff;
      target_angle = 0.0f;
      target_kp = 0.0f;
      target_velocity = 0.0f;
      target_kd = 0.0f;
    } else {
      target_angle = 0.0f;
      target_velocity = 0.0f;
      target_torque = 0.0f;
      target_kp = 0.0f;
      target_kd = motor->mit_default_damping_kd;
    }

    if (motor->external_torque_feedforward_ptr) {
      target_torque += *motor->external_torque_feedforward_ptr;
    }

    target_angle = DMMotorApplyMotorDirection(setting, target_angle);
    target_velocity = DMMotorApplyMotorDirection(setting, target_velocity);
    target_torque = DMMotorApplyMotorDirection(setting, target_torque);

    LIMIT_MIN_MAX(target_angle, motor->mit_limit.angle_min,
                  motor->mit_limit.angle_max);
    LIMIT_MIN_MAX(target_velocity, motor->mit_limit.omega_min,
                  motor->mit_limit.omega_max);
    LIMIT_MIN_MAX(target_torque, motor->mit_limit.torque_min,
                  motor->mit_limit.torque_max);
    LIMIT_MIN_MAX(target_kp, motor->mit_limit.kp_min, motor->mit_limit.kp_max);
    LIMIT_MIN_MAX(target_kd, motor->mit_limit.kd_min, motor->mit_limit.kd_max);

    effort_output.tau_ref_nm = target_torque;
    if (!DMMotorBuildMitTorqueCommand(&effort_output, motor->mit_limit.torque_max,
                                      &actuator_command, &normalized_output)) {
      memset(&actuator_command, 0, sizeof(actuator_command));
      memset(&normalized_output, 0, sizeof(normalized_output));
    }
    target_torque = actuator_command.mit_torque_nm;

  pack_and_send:
    motor_send_mailbox.angle_cmd_raw = float_to_uint(
        target_angle, motor->mit_limit.angle_min, motor->mit_limit.angle_max, 16);
    motor_send_mailbox.velocity_cmd_raw =
        float_to_uint(target_velocity, motor->mit_limit.omega_min,
                      motor->mit_limit.omega_max, 12);
    motor_send_mailbox.torque_cmd_raw =
        float_to_uint(target_torque, motor->mit_limit.torque_min,
                      motor->mit_limit.torque_max, 12);
    motor_send_mailbox.kp_cmd_raw = float_to_uint(
        target_kp, motor->mit_limit.kp_min, motor->mit_limit.kp_max, 12);
    motor_send_mailbox.kd_cmd_raw = float_to_uint(
        target_kd, motor->mit_limit.kd_min, motor->mit_limit.kd_max, 12);

    if (motor->stop_flag == MOTOR_STOP) {
      motor_send_mailbox.torque_cmd_raw =
          float_to_uint(0, motor->mit_limit.torque_min,
                        motor->mit_limit.torque_max, 12);
      motor_send_mailbox.velocity_cmd_raw =
          float_to_uint(0, motor->mit_limit.omega_min,
                        motor->mit_limit.omega_max, 12);
    }

    motor->motor_can_instance->tx_buff[0] =
        (uint8_t)(motor_send_mailbox.angle_cmd_raw >> 8);
    motor->motor_can_instance->tx_buff[1] =
        (uint8_t)(motor_send_mailbox.angle_cmd_raw);
    motor->motor_can_instance->tx_buff[2] =
        (uint8_t)(motor_send_mailbox.velocity_cmd_raw >> 4);
    motor->motor_can_instance->tx_buff[3] =
        (uint8_t)(((motor_send_mailbox.velocity_cmd_raw & 0xF) << 4) |
                  (motor_send_mailbox.kp_cmd_raw >> 8));
    motor->motor_can_instance->tx_buff[4] =
        (uint8_t)(motor_send_mailbox.kp_cmd_raw);
    motor->motor_can_instance->tx_buff[5] =
        (uint8_t)(motor_send_mailbox.kd_cmd_raw >> 4);
    motor->motor_can_instance->tx_buff[6] =
        (uint8_t)(((motor_send_mailbox.kd_cmd_raw & 0xF) << 4) |
                  (motor_send_mailbox.torque_cmd_raw >> 8));
    motor->motor_can_instance->tx_buff[7] =
        (uint8_t)(motor_send_mailbox.torque_cmd_raw);

    CANTransmit(motor->motor_can_instance, 1);
    osDelay(1);
  }
}

void DMMotorControlInit() {
  if (!idx) {
    return;
  }
  for (size_t i = 0; i < idx; i++) {
    osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
    dm_task_handle[i] =
        osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
  }
}
