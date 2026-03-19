#include "shoot.h"
#include "robot_def.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "dji_motor.h"
#include "general_def.h"
#include "message_center.h"
#include <math.h>

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
// 三摩擦轮（倒三角分布）：底点(bottom) + 顶边左/右(top_left/top_right)
// 约定：底点ID=1，右轮ID=2，左轮ID=3
static DJIMotorInstance *friction_bottom, *friction_top_left,
    *friction_top_right, *loader; // 拨盘电机
// ===========================================================

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;
// 射频前馈执行态缓存：用于停射当周期清零，避免残留速度指令引起二次触发过冲
static float shoot_time = 0.0f, shoot_speed = 0.0f;
static float loader_target_speed = 0.0f;

// 12m/s 固定弹速对应的三摩擦轮目标转速，保留为 volatile 便于 Ozone 在线调参
volatile float friction_bottom_target_speed_12mps = 58000.0f;
volatile float friction_top_left_target_speed_12mps = 58000.0f;
volatile float friction_top_right_target_speed_12mps = 58000.0f;

// 拨盘反转控制（用于卡弹恢复）：反转固定角度（按“一发弹丸角度”计算）
// 说明：这里不做卡弹检测，仅提供LOAD_REVERSE动作；检测逻辑由上层决定何时进入该模式
#define LOADER_REVERSE_SPEED_DPS                                               \
  5000.0f // 反转目标速度（单位：度/秒），需联调标定
#define LOADER_REVERSE_DEAD_TIME_MS                                            \
  200.0f // 反转完成后的最小间隔（防止持续触发反复反转）
// 反转动作最大持续时间（ms）
// 目的：卡死/堵转时避免无限期输出反转指令导致过热或保护
// 经验值：按“转过一发角度”理论时间 + 裕量
#define LOADER_REVERSE_MAX_TIME_MS                                             \
  ((ONE_BULLET_DELTA_ANGLE / LOADER_REVERSE_SPEED_DPS) * 1000.0f + 200.0f)
static uint8_t loader_reverse_active = 0;       // 反转动作是否进行中
static float loader_reverse_start_angle = 0;    // 反转起始角度（电机轴总角度）
static float loader_reverse_start_time_ms = 0;  // 反转起始时间（ms）
static float loader_reverse_cooldown_until = 0; // 反转冷却截止时间（ms）

typedef enum {
  LOADER_JAM_STATE_NORMAL = 0, // 常规
  LOADER_JAM_STATE_SUSPECTED,  // 嫌疑
  LOADER_JAM_STATE_CONFIRMED,  // 确认
  LOADER_JAM_STATE_PROCESSING, // 处理
} Loader_Jam_State_e;

#define M3508_CURRENT_RAW_PER_AMP 819.2f
#define LOADER_JAM_CURRENT_THRESHOLD_RAW                                       \
  ((int16_t)(16.0f * M3508_CURRENT_RAW_PER_AMP))
#define LOADER_JAM_SPEED_THRESHOLD_DPS 180.0f
#define LOADER_JAM_CONFIRM_TIME_MS 500.0f
#define LOADER_JAM_PROCESS_TIME_MS 800.0f
#define LOADER_JAM_RECOVERY_OUTPUT_ANGLE_DEG 15.0f
#define LOADER_JAM_RECOVERY_DELTA_ANGLE                                        \
  (LOADER_JAM_RECOVERY_OUTPUT_ANGLE_DEG * REDUCTION_RATIO_LOADER)
#define LOADER_JAM_TARGET_TOLERANCE_ANGLE 120.0f

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

static uint8_t TriggerLoaderJamRecovery(float now_ms) {
  DJIMotorEnable(loader);
  DJIMotorOuterLoop(loader, ANGLE_LOOP);
  loader_jam_process_start_ms = now_ms;
  loader_jam_process_target_angle =
      loader->measure.total_angle - LOADER_JAM_RECOVERY_DELTA_ANGLE;
  DJIMotorSetRef(loader, loader_jam_process_target_angle);
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
  DJIMotorOuterLoop(loader, SPEED_LOOP);
  if (!loader_reverse_active) {
    loader_reverse_active = 1U;
    loader_reverse_start_angle = loader->measure.total_angle;
    loader_reverse_start_time_ms = now_ms;
    DJIMotorSetRef(loader, -LOADER_REVERSE_SPEED_DPS);
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
      DJIMotorSetRef(loader, -LOADER_REVERSE_SPEED_DPS);
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
    DJIMotorOuterLoop(loader, ANGLE_LOOP);
    DJIMotorSetRef(loader, loader_jam_process_target_angle);

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

void ShootInit() // 已适配三摩擦轮发射机构（倒三角布局）
{
  Motor_Init_Config_s friction_config = {
      .can_init_config =
          {
              .can_handle = &hcan1, // 统一使用CAN1
          },
      .controller_param_init_config =
          {
              .speed_PID =
                  {
                      .kp = 4.0f,
                      .ki = 0.0f, // 纯P控制：不启用积分
                      .kd = 0.08f,
                      .Derivative_LPF_RC = 0.008f,
                      // 输出滤波：一阶低通时间常数（单位：秒），500Hz控制频率下推荐
                      // 0.01s
                      .Output_LPF_RC = 0.022f,
                      .Improve = PID_DerivativeFilter | PID_OutputFilter,
                      .MaxOut = 16384,
                  },
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,

              .outer_loop_type = SPEED_LOOP,
              .close_loop_type = SPEED_LOOP,
              .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // ID1默认正转
          },
      .motor_type = M3508};

  // ID1: 底点(bottom)
  friction_config.can_init_config.tx_id = 1;
  friction_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE;
  friction_bottom = DJIMotorInit(&friction_config);

  // ID2: 顶边右(top_right)
  friction_config.can_init_config.tx_id = 2;
  friction_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE;
  friction_top_right = DJIMotorInit(&friction_config);

  // ID3: 顶边左(top_left)
  friction_config.can_init_config.tx_id = 3;
  friction_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_NORMAL;
  friction_top_left = DJIMotorInit(&friction_config);

  // 拨盘电机
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
                      // 位置环：输出到速度环的目标速度（单位：度/秒）
                      // ⚠️ 注意：误差=3060°（考虑了减速比），不是60°
                      .kp = 17.5f, // P项（误差3060°→输出约5049°/s，接近MaxOut）
                      .ki = 0.0f,  // I项消除稳态误差
                      .kd = 0.05f,
                      .Improve = PID_Integral_Limit,
                      .MaxOut = 6000, // 最大目标速度6000°/s
                  },
              .speed_PID =
                  {
                      // 无电流环模式：速度环直接输出CAN控制指令（等效电流/力矩类控制量）
                      // 输出单位：控制指令（范围约-16384到+16384）
                      .kp = 1.75f, // 速度环比例（开环输出需要更大的Kp）
                      .ki = 1.51f, // 速度环积分（消除稳态误差）
                      .kd = 0.0f,
                      // 输出滤波：对速度环输出做一阶低通，抑制齿隙/负载突变引起的力矩抖动
                      // 建议从 0.01s 起步（约 16Hz
                      // 截止），再根据响应/抖动情况微调
                      .Improve = PID_Integral_Limit | PID_OutputFilter,
                      .IntegralLimit = 7500, // 积分限幅
                      .Output_LPF_RC = 0.010f,
                      .MaxOut = 16384, // 输出限幅
                  },
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,

              .outer_loop_type = ANGLE_LOOP, // 初始外环：角度环（单发模式默认）
              .close_loop_type =
                  SPEED_LOOP |
                  ANGLE_LOOP, // 启用速度环+角度环（双环串级，无电流环）
              .motor_reverse_flag =
                  MOTOR_DIRECTION_NORMAL, // ⚠️ 方向：REVERSE会导致正反馈震荡！
          },
      .motor_type = M3508 // 英雄使用m3508
  };
  loader = DJIMotorInit(&loader_config);

  ResetLoaderJamState(DWT_GetTimeline_ms(), 1U);

  shoot_pub = RegisterPublisher("shoot_feed", sizeof(Shoot_Upload_Data_s));
  shoot_sub = RegisterSubscriber("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask() {
  const float now_ms = DWT_GetTimeline_ms();
  loader_mode_e requested_load_mode = LOAD_STOP;
  loader_mode_e effective_load_mode = LOAD_STOP;
  // 从cmd获取控制数据
  SubGetMessage(shoot_sub, &shoot_cmd_recv);

  // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
  if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
    DJIMotorStop(friction_bottom);
    DJIMotorStop(friction_top_left);
    DJIMotorStop(friction_top_right);
    DJIMotorStop(loader);

    // ⚠️ 不要直接return！继续清理状态
    // 清除不应期
    hibernate_time = 0;
    dead_time = 0;
    shoot_time = 0.0f;
    shoot_speed = 0.0f;
    loader_target_speed = 0.0f;
    loader_reverse_active = 0;
    loader_reverse_start_time_ms = 0.0f;
    loader_reverse_cooldown_until = 0.0f;
    ClearLoaderDiscreteShot();
    ResetLoaderJamState(now_ms, 0U);

    PublishShootFeedback();
    return; // 紧急停止后直接返回，不再执行后续逻辑
  }

  requested_load_mode = shoot_cmd_recv.load_mode;
  if (loader_discrete_shot_active &&
      (requested_load_mode == LOAD_REVERSE ||
       requested_load_mode == LOAD_BURSTFIRE)) {
    ClearLoaderDiscreteShot();
  }

  effective_load_mode =
      loader_discrete_shot_active ? loader_discrete_shot_mode : requested_load_mode;

  const uint8_t manual_reverse = (requested_load_mode == LOAD_REVERSE) ? 1U : 0U;
  const uint8_t fire_intent = IsLoaderFireIntent(effective_load_mode);
  const uint8_t deadtime_blocked = (effective_load_mode != LOAD_STOP) &&
                                   !manual_reverse &&
                                   (hibernate_time + dead_time > now_ms);
  uint8_t loader_control_taken = 0U;

  // 发射模块使能由各个load_mode自行控制（STOP会停止，BURSTFIRE会使能）
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
    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (effective_load_mode) {
    case LOAD_STOP:
      // 直接停止电机，同时重置PID状态（见DJIMotorStop实现）
      DJIMotorStop(loader);
      ClearLoaderDiscreteShot();
      ResetLoaderJamState(now_ms, 0U);
      break;

    case LOAD_1_BULLET:                      // 激活能量机关/干扰对方用,英雄用.
    case LOAD_3_BULLET:
      if (!loader_discrete_shot_active) {
        loader_discrete_shot_active = 1U;
        loader_discrete_shot_mode = effective_load_mode;
        loader_discrete_shot_target_angle =
            loader->measure.total_angle +
            GetLoaderDiscreteShotDeltaAngle(effective_load_mode);
        hibernate_time = now_ms; // 记录触发指令的时间
        shoot_time = hibernate_time;
        shoot_speed = shoot_cmd_recv.shoot_rate;
        dead_time =
            (shoot_cmd_recv.shoot_rate > 0.0f)
                ? (GetLoaderDiscreteShotDeltaAngle(effective_load_mode) /
                   ONE_BULLET_DELTA_ANGLE) *
                      (1000.0f / shoot_cmd_recv.shoot_rate)
                : ((effective_load_mode == LOAD_3_BULLET) ? 300.0f : 1000.0f);
      }
      DJIMotorEnable(loader);                // 从STOP切换过来时需要使能
      DJIMotorOuterLoop(loader, ANGLE_LOOP); // 切换到角度环
      DJIMotorSetRef(loader, loader_discrete_shot_target_angle);
      if (IsLoaderDiscreteShotFinished()) {
        ClearLoaderDiscreteShot();
        DJIMotorStop(loader);
      }
      break;

    case LOAD_BURSTFIRE:
      ClearLoaderDiscreteShot();
      DJIMotorEnable(loader);                // 从STOP切换过来时需要重新使能
      DJIMotorOuterLoop(loader, SPEED_LOOP); // 必须切换到速度环

      loader_target_speed =
          (shoot_cmd_recv.shoot_rate > 0.0f)
              ? (shoot_cmd_recv.shoot_rate * 360.0f * REDUCTION_RATIO_LOADER /
                 (float)NUM_PER_CIRCLE)
              : (360.0f * REDUCTION_RATIO_LOADER / (float)NUM_PER_CIRCLE);
      shoot_speed = shoot_cmd_recv.shoot_rate;
      shoot_time = now_ms;
      DJIMotorSetRef(loader, loader_target_speed);
      break;

    case LOAD_REVERSE:
      // 已在前面的人工兜底路径处理，不应到达这里
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

  // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
  if (shoot_cmd_recv.friction_mode == FRICTION_ON) {
    // ⭐ 使能摩擦轮电机
    DJIMotorEnable(friction_bottom);
    DJIMotorEnable(friction_top_left);
    DJIMotorEnable(friction_top_right);

    // 根据目标弹速分档设置目标速度（保持原有逻辑），并给出方向
    // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
    switch (shoot_cmd_recv.bullet_speed) {
    case SMALL_AMU_12:
    default:
      DJIMotorSetRef(friction_bottom, friction_bottom_target_speed_12mps);
      DJIMotorSetRef(friction_top_left, friction_top_left_target_speed_12mps);
      DJIMotorSetRef(friction_top_right, friction_top_right_target_speed_12mps);
      break;
    }
  } else // 关闭摩擦轮
  {
    // 停止电机（这会设置stop_flag，清除PID状态，电机立即停转）
    DJIMotorSetRef(friction_bottom, 0);
    DJIMotorSetRef(friction_top_left, 0);
    DJIMotorSetRef(friction_top_right, 0);
  }

  PublishShootFeedback();
}
