#include "shoot.h"
#include "robot_def.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "dji_motor.h"
#include "general_def.h"
#include "message_center.h"
#include "controllers/domain/shoot_effort_controller.h"
#include <math.h>

static DJIMotorInstance *friction_bottom, *friction_top_left,
    *friction_top_right, *loader;

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data;

static float hibernate_time = 0.0f;
static float dead_time = 0.0f;
static float shoot_time = 0.0f;
static float shoot_speed = 0.0f;
static float loader_target_speed = 0.0f;

volatile float friction_bottom_target_speed_12mps = 24595.0f;
volatile float friction_top_left_target_speed_12mps = 25325.0f;
volatile float friction_top_right_target_speed_12mps = 25325.0f;

#define M3508_TAU_PER_RAW_CURRENT_CMD ((0.3f * 20.0f) / 16384.0f)
#define M3508_RAW_CMD_TO_TAU_REF(raw_cmd)                                      \
  ((raw_cmd)*M3508_TAU_PER_RAW_CURRENT_CMD)

#define LOADER_REVERSE_SPEED_DPS 5000.0f
#define LOADER_REVERSE_DEAD_TIME_MS 200.0f
#define LOADER_REVERSE_MAX_TIME_MS                                             \
  ((ONE_BULLET_DELTA_ANGLE / LOADER_REVERSE_SPEED_DPS) * 1000.0f + 200.0f)

typedef enum {
  LOADER_JAM_STATE_NORMAL = 0,
  LOADER_JAM_STATE_SUSPECTED,
  LOADER_JAM_STATE_CONFIRMED,
  LOADER_JAM_STATE_PROCESSING,
} Loader_Jam_State_e;

#define M3508_CURRENT_RAW_PER_AMP 819.2f
#define LOADER_JAM_CURRENT_THRESHOLD_RAW                                       \
  ((int16_t)(18.25f * M3508_CURRENT_RAW_PER_AMP))
#define LOADER_JAM_SPEED_THRESHOLD_DPS 180.0f
#define LOADER_JAM_CONFIRM_TIME_MS 325.0f
#define LOADER_JAM_PROCESS_TIME_MS 850.0f
#define LOADER_JAM_RECOVERY_OUTPUT_ANGLE_DEG 20.0f
#define LOADER_JAM_RECOVERY_DELTA_ANGLE                                        \
  (LOADER_JAM_RECOVERY_OUTPUT_ANGLE_DEG * REDUCTION_RATIO_LOADER)
#define LOADER_JAM_TARGET_TOLERANCE_ANGLE 120.0f

static uint8_t loader_reverse_active = 0U;
static float loader_reverse_start_angle = 0.0f;
static float loader_reverse_start_time_ms = 0.0f;
static float loader_reverse_cooldown_until = 0.0f;
static Loader_Jam_State_e loader_jam_state = LOADER_JAM_STATE_NORMAL;
static float loader_jam_state_since_ms = 0.0f;
static float loader_jam_process_start_ms = 0.0f;
static float loader_jam_process_target_angle = 0.0f;
static uint16_t loader_jam_recovery_count = 0U;
static uint8_t loader_discrete_shot_active = 0U;
static loader_mode_e loader_discrete_shot_mode = LOAD_STOP;
static float loader_discrete_shot_target_angle = 0.0f;

static uint8_t IsLoaderFireIntent(loader_mode_e load_mode) {
  return (load_mode == LOAD_1_BULLET || load_mode == LOAD_3_BULLET ||
          load_mode == LOAD_BURSTFIRE)
             ? 1U
             : 0U;
}

static float GetLoaderDiscreteShotDeltaAngle(loader_mode_e load_mode) {
  return (load_mode == LOAD_3_BULLET) ? (3.0f * ONE_BULLET_DELTA_ANGLE)
                                      : ONE_BULLET_DELTA_ANGLE;
}

static void ClearLoaderDiscreteShot(void) {
  loader_discrete_shot_active = 0U;
  loader_discrete_shot_mode = LOAD_STOP;
  loader_discrete_shot_target_angle = 0.0f;
}

static uint8_t IsLoaderDiscreteShotFinished(void) {
  const float angle_error =
      fabsf(loader_discrete_shot_target_angle - loader->measure.total_angle);
  const float abs_speed = fabsf(loader->measure.speed_aps);
  return (angle_error <= LOADER_JAM_TARGET_TOLERANCE_ANGLE &&
          abs_speed <= LOADER_JAM_SPEED_THRESHOLD_DPS)
             ? 1U
             : 0U;
}

static void SetLoaderJamState(Loader_Jam_State_e new_state, float now_ms) {
  if (loader_jam_state != new_state) {
    loader_jam_state = new_state;
    loader_jam_state_since_ms = now_ms;
  }
}

static void ResetLoaderJamState(float now_ms, uint8_t clear_recovery_count) {
  loader_jam_state = LOADER_JAM_STATE_NORMAL;
  loader_jam_state_since_ms = now_ms;
  loader_jam_process_start_ms = 0.0f;
  loader_jam_process_target_angle = 0.0f;
  if (clear_recovery_count) {
    loader_jam_recovery_count = 0U;
  }
}

static uint8_t IsLoaderJamConditionMet(void) {
  const float abs_current = fabsf((float)loader->measure.real_current);
  const float abs_speed = fabsf(loader->measure.speed_aps);
  return (abs_current >= (float)LOADER_JAM_CURRENT_THRESHOLD_RAW &&
          abs_speed <= LOADER_JAM_SPEED_THRESHOLD_DPS)
             ? 1U
             : 0U;
}

static void UpdateShootFeedback(void) {
  shoot_feedback_data.loader_jam_state = (uint8_t)loader_jam_state;
  shoot_feedback_data.loader_jam_active =
      (loader_jam_state == LOADER_JAM_STATE_PROCESSING) ? 1U : 0U;
  shoot_feedback_data.loader_jam_recovery_count = loader_jam_recovery_count;
  shoot_feedback_data.loader_real_current = loader->measure.real_current;
  shoot_feedback_data.loader_speed_aps = loader->measure.speed_aps;
  shoot_feedback_data.loader_total_angle = loader->measure.total_angle;
}

static void PublishShootFeedback(void) {
  UpdateShootFeedback();
  PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}

static uint8_t ApplyShootFrictionTarget(DJIMotorInstance *motor,
                                        float target_speed_aps) {
  Controller_Effort_Output_s effort = {0};

  if (motor == NULL) {
    return 0U;
  }

  if (!ShootFrictionCalculateEffort(motor, target_speed_aps, &effort)) {
    return 0U;
  }

  DJIMotorSetEffort(motor, &effort);
  return 1U;
}

static uint8_t ApplyLoaderAngleTarget(float target_angle_deg) {
  Controller_Effort_Output_s effort = {0};

  if (loader == NULL) {
    return 0U;
  }

  if (!ShootLoaderCalculateAngleEffort(loader, target_angle_deg, &effort)) {
    return 0U;
  }

  DJIMotorSetEffort(loader, &effort);
  return 1U;
}

static uint8_t ApplyLoaderSpeedTarget(float target_speed_aps) {
  Controller_Effort_Output_s effort = {0};

  if (loader == NULL) {
    return 0U;
  }

  if (!ShootLoaderCalculateSpeedEffort(loader, target_speed_aps, &effort)) {
    return 0U;
  }

  DJIMotorSetEffort(loader, &effort);
  return 1U;
}

static uint8_t TriggerLoaderJamRecovery(float now_ms) {
  DJIMotorEnable(loader);
  loader_jam_process_start_ms = now_ms;
  loader_jam_process_target_angle =
      loader->measure.total_angle - LOADER_JAM_RECOVERY_DELTA_ANGLE;
  ApplyLoaderAngleTarget(loader_jam_process_target_angle);
  loader_jam_recovery_count++;
  SetLoaderJamState(LOADER_JAM_STATE_PROCESSING, now_ms);
  LOGWARNING("[shoot] Loader jam confirmed, auto recovery triggered.");
  return 1U;
}

static uint8_t ProcessManualLoaderReverse(float now_ms) {
  ResetLoaderJamState(now_ms, 0U);

  if (now_ms < loader_reverse_cooldown_until) {
    DJIMotorStop(loader);
    return 1U;
  }

  DJIMotorEnable(loader);
  if (!loader_reverse_active) {
    loader_reverse_active = 1U;
    loader_reverse_start_angle = loader->measure.total_angle;
    loader_reverse_start_time_ms = now_ms;
    ApplyLoaderSpeedTarget(-LOADER_REVERSE_SPEED_DPS);
  } else {
    if ((now_ms - loader_reverse_start_time_ms) > LOADER_REVERSE_MAX_TIME_MS) {
      loader_reverse_active = 0U;
      loader_reverse_start_time_ms = 0.0f;
      DJIMotorStop(loader);
      loader_reverse_cooldown_until = now_ms + LOADER_REVERSE_DEAD_TIME_MS;
      return 1U;
    }

    if (fabsf(loader->measure.total_angle - loader_reverse_start_angle) >=
        ONE_BULLET_DELTA_ANGLE) {
      loader_reverse_active = 0U;
      loader_reverse_start_time_ms = 0.0f;
      DJIMotorStop(loader);
      loader_reverse_cooldown_until = now_ms + LOADER_REVERSE_DEAD_TIME_MS;
    } else {
      ApplyLoaderSpeedTarget(-LOADER_REVERSE_SPEED_DPS);
    }
  }

  return 1U;
}

static uint8_t ProcessLoaderJamFSM(float now_ms, uint8_t fire_intent) {
  const float abs_current = fabsf((float)loader->measure.real_current);

  if (!fire_intent && loader_jam_state != LOADER_JAM_STATE_PROCESSING) {
    ResetLoaderJamState(now_ms, 0U);
    return 0U;
  }

  switch (loader_jam_state) {
  case LOADER_JAM_STATE_NORMAL:
    if (fire_intent && IsLoaderJamConditionMet()) {
      SetLoaderJamState(LOADER_JAM_STATE_SUSPECTED, now_ms);
    }
    return 0U;

  case LOADER_JAM_STATE_SUSPECTED:
    if (!fire_intent || abs_current < (float)LOADER_JAM_CURRENT_THRESHOLD_RAW) {
      ResetLoaderJamState(now_ms, 0U);
      return 0U;
    }

    if ((now_ms - loader_jam_state_since_ms) >= LOADER_JAM_CONFIRM_TIME_MS) {
      SetLoaderJamState(LOADER_JAM_STATE_CONFIRMED, now_ms);
      return TriggerLoaderJamRecovery(now_ms);
    }
    return 0U;

  case LOADER_JAM_STATE_CONFIRMED:
    return TriggerLoaderJamRecovery(now_ms);

  case LOADER_JAM_STATE_PROCESSING:
    DJIMotorEnable(loader);
    ApplyLoaderAngleTarget(loader_jam_process_target_angle);

    if ((now_ms - loader_jam_process_start_ms) >= LOADER_JAM_PROCESS_TIME_MS) {
      hibernate_time = now_ms;
      dead_time = LOADER_JAM_PROCESS_TIME_MS;
      ResetLoaderJamState(now_ms, 0U);
    }
    return 1U;

  default:
    ResetLoaderJamState(now_ms, 0U);
    return 0U;
  }
}

void ShootInit() {
  Motor_Init_Config_s friction_config = {
      .can_init_config =
          {
              .can_handle = &hcan1,
          },
      .controller_param_init_config =
          {
              .speed_PID =
                  {
                      .kp = M3508_RAW_CMD_TO_TAU_REF(4.0f),
                      .ki = 0.0f,
                      .kd = M3508_RAW_CMD_TO_TAU_REF(0.08f),
                      .Derivative_LPF_RC = 0.008f,
                      .Output_LPF_RC = 0.022f,
                      .Improve = PID_DerivativeFilter | PID_OutputFilter,
                      .MaxOut = M3508_RAW_CMD_TO_TAU_REF(16384.0f),
                  },
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,
              .outer_loop_type = SPEED_LOOP,
              .close_loop_type = SPEED_LOOP,
              .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
          },
      .motor_type = M3508,
  };

  friction_config.can_init_config.tx_id = 1;
  friction_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE;
  friction_bottom = DJIMotorInit(&friction_config);

  friction_config.can_init_config.tx_id = 2;
  friction_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE;
  friction_top_right = DJIMotorInit(&friction_config);

  friction_config.can_init_config.tx_id = 3;
  friction_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_NORMAL;
  friction_top_left = DJIMotorInit(&friction_config);

  Motor_Init_Config_s loader_config = {
      .can_init_config =
          {
              .can_handle = &hcan2,
              .tx_id = 3,
          },
      .controller_param_init_config =
          {
              .angle_PID =
                  {
                      .kp = 17.5f,
                      .ki = 0.0f,
                      .kd = 0.05f,
                      .Improve = PID_Integral_Limit,
                      .MaxOut = 6000.0f,
                  },
              .speed_PID =
                  {
                      .kp = M3508_RAW_CMD_TO_TAU_REF(1.75f),
                      .ki = M3508_RAW_CMD_TO_TAU_REF(1.51f),
                      .kd = 0.0f,
                      .Improve = PID_Integral_Limit | PID_OutputFilter,
                      .IntegralLimit = M3508_RAW_CMD_TO_TAU_REF(7500.0f),
                      .Output_LPF_RC = 0.010f,
                      .MaxOut = M3508_RAW_CMD_TO_TAU_REF(16384.0f),
                  },
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,
              .outer_loop_type = ANGLE_LOOP,
              .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
              .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
          },
      .motor_type = M3508,
  };
  loader = DJIMotorInit(&loader_config);

  ResetLoaderJamState(DWT_GetTimeline_ms(), 1U);

  shoot_pub = RegisterPublisher("shoot_feed", sizeof(Shoot_Upload_Data_s));
  shoot_sub = RegisterSubscriber("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

void ShootTask() {
  const float now_ms = DWT_GetTimeline_ms();
  loader_mode_e requested_load_mode = LOAD_STOP;
  loader_mode_e effective_load_mode = LOAD_STOP;
  const float friction_bottom_target_speed = friction_bottom_target_speed_12mps;
  const float friction_top_left_target_speed = friction_top_left_target_speed_12mps;
  const float friction_top_right_target_speed =
      friction_top_right_target_speed_12mps;
  uint8_t loader_control_taken = 0U;

  SubGetMessage(shoot_sub, &shoot_cmd_recv);

  if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
    DJIMotorStop(friction_bottom);
    DJIMotorStop(friction_top_left);
    DJIMotorStop(friction_top_right);
    DJIMotorStop(loader);

    hibernate_time = 0.0f;
    dead_time = 0.0f;
    shoot_time = 0.0f;
    shoot_speed = 0.0f;
    loader_target_speed = 0.0f;
    loader_reverse_active = 0U;
    loader_reverse_start_time_ms = 0.0f;
    loader_reverse_cooldown_until = 0.0f;
    ClearLoaderDiscreteShot();
    ResetLoaderJamState(now_ms, 0U);

    PublishShootFeedback();
    return;
  }

  requested_load_mode = shoot_cmd_recv.load_mode;
  if (loader_discrete_shot_active &&
      (requested_load_mode == LOAD_REVERSE ||
       requested_load_mode == LOAD_BURSTFIRE)) {
    ClearLoaderDiscreteShot();
  }

  effective_load_mode =
      loader_discrete_shot_active ? loader_discrete_shot_mode
                                  : requested_load_mode;

  const uint8_t manual_reverse = (requested_load_mode == LOAD_REVERSE) ? 1U : 0U;
  const uint8_t fire_intent = IsLoaderFireIntent(effective_load_mode);
  const uint8_t deadtime_blocked = (effective_load_mode != LOAD_STOP) &&
                                   !manual_reverse &&
                                   (hibernate_time + dead_time > now_ms);

  DJIMotorEnable(friction_bottom);
  DJIMotorEnable(friction_top_left);
  DJIMotorEnable(friction_top_right);
  DJIMotorEnable(loader);

  if (!manual_reverse) {
    loader_reverse_active = 0U;
    loader_reverse_start_time_ms = 0.0f;
  }

  if (manual_reverse) {
    loader_control_taken = ProcessManualLoaderReverse(now_ms);
  } else {
    loader_control_taken = ProcessLoaderJamFSM(now_ms, fire_intent);
  }

  if (!loader_control_taken && deadtime_blocked) {
    PublishShootFeedback();
    return;
  }

  if (!loader_control_taken) {
    switch (effective_load_mode) {
    case LOAD_STOP:
      DJIMotorStop(loader);
      ClearLoaderDiscreteShot();
      ResetLoaderJamState(now_ms, 0U);
      break;

    case LOAD_1_BULLET:
    case LOAD_3_BULLET:
      if (!loader_discrete_shot_active) {
        loader_discrete_shot_active = 1U;
        loader_discrete_shot_mode = effective_load_mode;
        loader_discrete_shot_target_angle =
            loader->measure.total_angle +
            GetLoaderDiscreteShotDeltaAngle(effective_load_mode);
        hibernate_time = now_ms;
        shoot_time = hibernate_time;
        shoot_speed = shoot_cmd_recv.shoot_rate;
        dead_time =
            (shoot_cmd_recv.shoot_rate > 0.0f)
                ? (GetLoaderDiscreteShotDeltaAngle(effective_load_mode) /
                   ONE_BULLET_DELTA_ANGLE) *
                      (1000.0f / shoot_cmd_recv.shoot_rate)
                : ((effective_load_mode == LOAD_3_BULLET) ? 300.0f : 1000.0f);
      }
      ApplyLoaderAngleTarget(loader_discrete_shot_target_angle);
      if (IsLoaderDiscreteShotFinished()) {
        ClearLoaderDiscreteShot();
        DJIMotorStop(loader);
      }
      break;

    case LOAD_BURSTFIRE:
      ClearLoaderDiscreteShot();
      loader_target_speed =
          (shoot_cmd_recv.shoot_rate > 0.0f)
              ? (shoot_cmd_recv.shoot_rate * 360.0f * REDUCTION_RATIO_LOADER /
                 (float)NUM_PER_CIRCLE)
              : (360.0f * REDUCTION_RATIO_LOADER / (float)NUM_PER_CIRCLE);
      shoot_speed = shoot_cmd_recv.shoot_rate;
      shoot_time = now_ms;
      ApplyLoaderSpeedTarget(loader_target_speed);
      break;

    case LOAD_REVERSE:
      break;

    default:
      LOGERROR("[shoot] Invalid load_mode=%d, fallback to LOAD_STOP.",
               (int)effective_load_mode);
      shoot_cmd_recv.load_mode = LOAD_STOP;
      hibernate_time = 0.0f;
      dead_time = 0.0f;
      shoot_time = 0.0f;
      shoot_speed = 0.0f;
      loader_target_speed = 0.0f;
      loader_reverse_active = 0U;
      loader_reverse_start_time_ms = 0.0f;
      loader_reverse_cooldown_until = 0.0f;
      ClearLoaderDiscreteShot();
      ResetLoaderJamState(now_ms, 0U);
      DJIMotorStop(loader);
      break;
    }
  }

  if (!fire_intent && !manual_reverse &&
      loader_jam_state != LOADER_JAM_STATE_PROCESSING) {
    loader_target_speed = 0.0f;
    shoot_time = 0.0f;
    shoot_speed = 0.0f;
  }

  if (shoot_cmd_recv.friction_mode == FRICTION_ON) {
    ApplyShootFrictionTarget(friction_bottom, friction_bottom_target_speed);
    ApplyShootFrictionTarget(friction_top_left, friction_top_left_target_speed);
    ApplyShootFrictionTarget(friction_top_right, friction_top_right_target_speed);
  } else {
    ApplyShootFrictionTarget(friction_bottom, 0.0f);
    ApplyShootFrictionTarget(friction_top_left, 0.0f);
    ApplyShootFrictionTarget(friction_top_right, 0.0f);
  }

  PublishShootFeedback();
}
