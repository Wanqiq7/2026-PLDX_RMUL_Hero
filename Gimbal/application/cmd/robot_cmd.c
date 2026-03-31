// app
#include "robot_cmd.h"
#include "robot_def.h"
// module
#include "bmi088.h"
#include "dji_motor.h"
#include "general_def.h"
#include "heat_gate_model.h"
#include "ins_task.h"
#include "message_center.h"
#include "remote_control.h"
#include "user_lib.h"
#include "vision_comm.h"
#include "vtm_input/vtm_input.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include "arm_math_compat.h"
#include "main.h"
#include <math.h>
#include <string.h>

#define DEG_TO_RAD 0.01745329251994329576f

/* ============================================================
 * 遥控器输入滤波器设计说明
 * ============================================================
 *
 * 1. 滤波器类型：一阶低通滤波器（IIR）
 *    公式：y[n] = α·x[n] + (1-α)·y[n-1]
 *
 * 2. 截止频率计算公式：
 *    fc = -fs·ln(1-α)/(2π)，其中 fs = 200Hz（控制频率）
 *
 * 3. 参数选择原则：
 *    - 遥控器杆量：α=0.90，fc≈84Hz（快速响应）
 *    - 云台控制：α=0.95/0.93，fc≈99/85Hz（平衡精度与响应）
 *    - 键盘控制：α=0.80，fc≈62Hz（平滑阶跃）
 *    - 鼠标控制：α=0.85/0.80，fc≈63/51Hz（瞄准稳定）
 *
 * 4. 总带宽分析：
 *    串联滤波器后的系统总带宽约17-20Hz，满足机器人控制需求
 * ============================================================ */

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE                                                        \
  (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
// Pitch轴角度限位保护宏（与陀螺仪角度同单位，弧度制）
#define LIMIT_PITCH_RAD(angle_rad)                                             \
  do {                                                                         \
    const float max_rad = PITCH_MAX_ANGLE * DEG_TO_RAD;                        \
    const float min_rad = PITCH_MIN_ANGLE * DEG_TO_RAD;                        \
    if ((angle_rad) > max_rad)                                                 \
      (angle_rad) = max_rad;                                                   \
    if ((angle_rad) < min_rad)                                                 \
      (angle_rad) = min_rad;                                                   \
  } while (0)

// 键鼠功率档位（通过 chassis_speed_buff 透传给底盘功率控制）
static const int CHASSIS_POWER_ECO_PERCENT = 80;        // 经济档：80%功率
static const int CHASSIS_POWER_LIMIT_PERCENT = 100;     // 额定档：100%功率
static const int CHASSIS_POWER_OVERDRIVE_PERCENT = 120; // 超功率档：120%功率

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_fast_comm;
static CANCommInstance *cmd_can_state_comm;
static CANCommInstance *cmd_can_ui_comm;
static CANCommInstance *cmd_can_event_comm;
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s
    chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s
    chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
static Chassis_Ctrl_Fast_Pkt_s chassis_ctrl_fast_send;
static Chassis_Ctrl_State_Pkt_s chassis_ctrl_state_send;
static Chassis_Ctrl_UI_Pkt_s chassis_ctrl_ui_send;
static Chassis_Ctrl_Event_Pkt_s chassis_ctrl_event_send;
static Chassis_Feed_Fast_Pkt_s chassis_feed_fast_recv;
static Chassis_Feed_State_Pkt_s chassis_feed_state_recv;

static RC_ctrl_t *rc_data;               // 遥控器数据,初始化时返回
static VTM_Input_Data_s *vtm_input_data; // 官方图传输入数据

// 视觉控制消息(通过message center与vision应用通信)
static Publisher_t *vision_cmd_pub;           // 视觉控制指令发布者
static Subscriber_t *vision_data_sub;         // 视觉处理数据订阅者
static Vision_Ctrl_Cmd_s vision_cmd_send;     // 发送给视觉应用的控制指令
static Vision_Upload_Data_s vision_data_recv; // 从视觉应用接收的处理数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

typedef enum {
  CMD_INPUT_SOURCE_NONE = 0,
  CMD_INPUT_SOURCE_RC,
  CMD_INPUT_SOURCE_VTM,
} Cmd_Input_Source_e;

typedef struct {
  struct {
    int16_t rocker_l_;
    int16_t rocker_l1;
    int16_t rocker_r_;
    int16_t rocker_r1;
    uint8_t switch_left;
    uint8_t switch_right;
  } rc;
  struct {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
    uint8_t press_m;
  } mouse;
  Key_t key[3];
  uint8_t key_count[3][16];
  uint8_t pause_pressed;
  uint8_t trigger_pressed;
  uint8_t online;
  Cmd_Input_Source_e source;
} Cmd_Input_Data_s;

static Cmd_Input_Data_s cmd_input_active;
static Cmd_Input_Source_e cmd_input_source = CMD_INPUT_SOURCE_NONE;

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
// 键盘控制
static float keyboard_vx_cmd_planned = 0.0f;
static float keyboard_vy_cmd_planned = 0.0f;
static int keyboard_power_mode_selected = 100;
static loader_mode_e keyboard_load_mode_selected = LOAD_STOP;
static loader_mode_e ui_loader_mode_selected = LOAD_STOP;
static chassis_mode_e keyboard_chassis_mode_selected =
    CHASSIS_FOLLOW_GIMBAL_YAW; // 键鼠模式默认底盘跟随云台

// 失联降级状态机与门控参数
typedef enum {
  CMD_STATE_NORMAL = 0,
  CMD_STATE_DEGRADED_VISION,
  CMD_STATE_DEGRADED_REFEREE,
  CMD_STATE_DEGRADED_LINK,
} Cmd_Degrade_State_e;

static Cmd_Degrade_State_e cmd_degrade_state = CMD_STATE_DEGRADED_LINK;
static Cmd_Degrade_State_e cmd_degrade_candidate = CMD_STATE_DEGRADED_LINK;
static uint32_t cmd_state_candidate_since_ms = 0;
static uint32_t cmd_last_telemetry_ms = 0;
static uint32_t cmd_last_can_warn_ms = 0;
static uint32_t cmd_last_ff_update_ms = 0;
static uint8_t cmd_force_safe_fire = 0;
static float cmd_rest_heat_filtered = 0.0f;
static float cmd_shoot_rate_ff = 1.0f;
static Heat_Gate_State_s cmd_heat_gate;
static uint16_t cmd_last_barrel_heat = 0U;
static uint16_t cmd_last_barrel_heat_limit = 0U;
static uint16_t cmd_last_barrel_cooling_value = 0U;
static uint8_t cmd_last_rest_heat_raw = 0U;
static uint8_t cmd_heat_snapshot_valid = 0U;
static loader_mode_e cmd_last_discrete_gate_mode = LOAD_STOP;
static uint8_t cmd_last_discrete_gate_allow = 0U;
static uint8_t cmd_burst_gate_active = 0U;
static uint32_t cmd_burst_gate_last_ms = 0U;
static uint8_t cmd_auto_fire_shot_pulse = 0U;
static uint32_t cmd_auto_fire_pulse_start_ms = 0U;
static uint32_t cmd_auto_fire_next_shot_ms = 0U;
static float cmd_last_required_heat = 0.0f;
static uint8_t cmd_last_gate_allow = 0U;
static loader_mode_e cmd_last_gate_mode = LOAD_STOP;
static uint8_t cmd_vtm_pause_latched = 0U;
static uint8_t cmd_vtm_pause_prev = 0U;
static uint8_t cmd_ui_refresh_ctrl_prev = 0U;
static uint32_t cmd_next_fast_can_send_ms = 0U;
static uint32_t cmd_next_state_can_send_ms = 0U;
static uint32_t cmd_next_ui_can_send_ms = 0U;
static uint16_t cmd_last_event_seq_sent = 0U;
static const uint32_t CMD_CAN_FAST_PERIOD_MS = 10U;  // 100Hz
static const uint32_t CMD_CAN_STATE_PERIOD_MS = 50U; // 20Hz
static const uint32_t CMD_CAN_UI_PERIOD_MS = 100U;   // 10Hz
static const uint32_t CMD_CAN_FAST_PHASE_MS = 0U;
static const uint32_t CMD_CAN_STATE_PHASE_MS = 3U;
static const uint32_t CMD_CAN_UI_PHASE_MS = 6U;

static const uint32_t CMD_DEGRADE_ENTER_HOLD_MS = 80U;
static const uint32_t CMD_DEGRADE_EXIT_HOLD_MS = 300U;
static const uint32_t CMD_GATE_TELEMETRY_PERIOD_MS = 500U;
static const uint32_t CMD_FF_UPDATE_PERIOD_MS = 100U;
// 单发/三发采用脉冲触发。为避免 cmd 与 shoot 任务相位错开导致单帧脉冲丢失，
// 这里将触发命令短暂保持数十毫秒，让 shoot 任务至少能消费一次。
static const uint32_t CMD_MOUSE_FIRE_PULSE_HOLD_MS = 30U;
static const float CMD_MIN_REST_HEAT_TO_FIRE = HEAT_PER_SHOT_D;
static const float CMD_SAFE_SHOOT_RATE = SHOOT_RATE_SAFE;

static uint8_t IsChassisLinkOnline(void) {
#ifdef GIMBAL_BOARD
  return (cmd_can_fast_comm != NULL) ? CANCommIsOnline(cmd_can_fast_comm) : 0U;
#else
  return 1U;
#endif
}

static void ResetChassisFetchDataSafe(void) {
  chassis_fetch_data.referee_online = 0;
  chassis_fetch_data.rest_heat = 0;
  chassis_fetch_data.bullet_speed = SHOOT_FIXED_BULLET_SPEED;
  chassis_fetch_data.chassis_power_limit = 0;
  chassis_fetch_data.barrel_heat = 0;
  chassis_fetch_data.barrel_heat_limit = 0;
  chassis_fetch_data.barrel_cooling_value = 0U;
  chassis_fetch_data.bullet_speed_limit = 0.0f;
}

static void ResetCmdInputActive(void) {
  memset(&cmd_input_active, 0, sizeof(cmd_input_active));
  cmd_input_source = CMD_INPUT_SOURCE_NONE;
}

static void LoadCmdInputFromRC(Cmd_Input_Data_s *input_data) {
  if (input_data == NULL) {
    return;
  }

  memset(input_data, 0, sizeof(*input_data));
  input_data->online = RemoteControlIsOnline() ? 1U : 0U;
  input_data->source =
      input_data->online ? CMD_INPUT_SOURCE_RC : CMD_INPUT_SOURCE_NONE;

  if (!input_data->online || rc_data == NULL) {
    return;
  }

  input_data->rc.rocker_l_ = rc_data[TEMP].rc.rocker_l_;
  input_data->rc.rocker_l1 = rc_data[TEMP].rc.rocker_l1;
  input_data->rc.rocker_r_ = rc_data[TEMP].rc.rocker_r_;
  input_data->rc.rocker_r1 = rc_data[TEMP].rc.rocker_r1;
  input_data->rc.switch_left = rc_data[TEMP].rc.switch_left;
  input_data->rc.switch_right = rc_data[TEMP].rc.switch_right;
  input_data->mouse.x = rc_data[TEMP].mouse.x;
  input_data->mouse.y = rc_data[TEMP].mouse.y;
  input_data->mouse.z = rc_data[TEMP].mouse.z;
  input_data->mouse.press_l = rc_data[TEMP].mouse.press_l;
  input_data->mouse.press_r = rc_data[TEMP].mouse.press_r;
  memcpy(input_data->key, rc_data[TEMP].key, sizeof(input_data->key));
  memcpy(input_data->key_count, rc_data[TEMP].key_count,
         sizeof(input_data->key_count));
}

static void LoadCmdInputFromVTM(Cmd_Input_Data_s *input_data) {
  if (input_data == NULL) {
    return;
  }

  memset(input_data, 0, sizeof(*input_data));
  input_data->online = VTMInputIsOnline() ? 1U : 0U;
  input_data->source =
      input_data->online ? CMD_INPUT_SOURCE_VTM : CMD_INPUT_SOURCE_NONE;

  if (!input_data->online || vtm_input_data == NULL) {
    return;
  }

  input_data->rc.rocker_l_ = vtm_input_data->rc.rocker_l_;
  input_data->rc.rocker_l1 = vtm_input_data->rc.rocker_l1;
  input_data->rc.rocker_r_ = vtm_input_data->rc.rocker_r_;
  input_data->rc.rocker_r1 = vtm_input_data->rc.rocker_r1;
  input_data->rc.switch_left = vtm_input_data->rc.switch_left;
  input_data->rc.switch_right = vtm_input_data->rc.switch_right;
  input_data->mouse.x = vtm_input_data->mouse.x;
  input_data->mouse.y = vtm_input_data->mouse.y;
  input_data->mouse.z = vtm_input_data->mouse.z;
  input_data->mouse.press_l = vtm_input_data->mouse.press_l;
  input_data->mouse.press_r = vtm_input_data->mouse.press_r;
  input_data->mouse.press_m = vtm_input_data->mouse.press_m;
  input_data->pause_pressed = vtm_input_data->rc.pause_pressed;
  input_data->trigger_pressed = vtm_input_data->rc.trigger_pressed;
  memcpy(input_data->key, vtm_input_data->key, sizeof(input_data->key));
  memcpy(input_data->key_count, vtm_input_data->key_count,
         sizeof(input_data->key_count));
}

static void SelectActiveControlInput(void) {
  Cmd_Input_Data_s rc_input;
  Cmd_Input_Data_s vtm_input;

  LoadCmdInputFromRC(&rc_input);
  LoadCmdInputFromVTM(&vtm_input);

  // 输入源仲裁：在 RC 与图传同时在线时，优先采用 RC。
  if (rc_input.online) {
    cmd_input_active = rc_input;
  } else if (vtm_input.online) {
    cmd_input_active = vtm_input;
  } else {
    ResetCmdInputActive();
    return;
  }

  cmd_input_source = cmd_input_active.source;
}

static void UpdateUIRefreshRequest(void) {
  const uint8_t ctrl_pressed =
      (cmd_input_active.online && cmd_input_active.key[KEY_PRESS].ctrl) ? 1U : 0U;

  if (ctrl_pressed && !cmd_ui_refresh_ctrl_prev) {
    chassis_cmd_send.ui_refresh_request_seq++;
    LOGINFO("[cmd] ctrl edge, request UI refresh seq=%u",
            (unsigned int)chassis_cmd_send.ui_refresh_request_seq);
  }

  cmd_ui_refresh_ctrl_prev = ctrl_pressed;
}

static uint8_t IsShootLoadFireIntent(loader_mode_e load_mode) {
  return (load_mode == LOAD_1_BULLET || load_mode == LOAD_3_BULLET ||
          load_mode == LOAD_BURSTFIRE)
             ? 1U
             : 0U;
}

static float GetRequiredHeatForLoadMode(loader_mode_e load_mode) {
  switch (load_mode) {
  case LOAD_3_BULLET:
    return 3.0f * HEAT_PER_SHOT_D;
  case LOAD_1_BULLET:
  case LOAD_BURSTFIRE:
    return HEAT_PER_SHOT_D;
  default:
    return 0.0f;
  }
}

static void ResetManualHeatGateState(uint32_t now_ms) {
  cmd_last_discrete_gate_mode = LOAD_STOP;
  cmd_last_discrete_gate_allow = 0U;
  cmd_burst_gate_active = 0U;
  cmd_burst_gate_last_ms = now_ms;
  cmd_last_required_heat = 0.0f;
  cmd_last_gate_allow = 0U;
  cmd_last_gate_mode = LOAD_STOP;
}

static float GetRawRestHeatFloat(void) {
  if (chassis_fetch_data.barrel_heat_limit > 0U) {
    return HeatGateComputeRestHeat(chassis_fetch_data.barrel_heat_limit,
                                   chassis_fetch_data.barrel_heat);
  }
  return (float)chassis_fetch_data.rest_heat;
}

static uint8_t HasHeatSnapshotUpdated(void) {
  if (!cmd_heat_snapshot_valid) {
    return 1U;
  }

  return (cmd_last_barrel_heat != chassis_fetch_data.barrel_heat ||
          cmd_last_barrel_heat_limit != chassis_fetch_data.barrel_heat_limit ||
          cmd_last_barrel_cooling_value !=
              chassis_fetch_data.barrel_cooling_value ||
          cmd_last_rest_heat_raw != chassis_fetch_data.rest_heat)
             ? 1U
             : 0U;
}

static void SyncHeatGateState(uint32_t now_ms, uint8_t referee_online) {
  if (!referee_online) {
    HeatGateReset(&cmd_heat_gate, 0.0f, 0.0f, 0.0f, now_ms);
    cmd_heat_snapshot_valid = 0U;
    cmd_rest_heat_filtered = 0.0f;
    return;
  }

  {
    const uint8_t raw_updated = HasHeatSnapshotUpdated();
    HeatGateUpdateRaw(&cmd_heat_gate, GetRawRestHeatFloat(),
                      (float)chassis_fetch_data.barrel_cooling_value,
                      (float)chassis_fetch_data.barrel_heat_limit, raw_updated,
                      now_ms);

    cmd_last_barrel_heat = chassis_fetch_data.barrel_heat;
    cmd_last_barrel_heat_limit = chassis_fetch_data.barrel_heat_limit;
    cmd_last_barrel_cooling_value = chassis_fetch_data.barrel_cooling_value;
    cmd_last_rest_heat_raw = chassis_fetch_data.rest_heat;
    cmd_heat_snapshot_valid = 1U;
    cmd_rest_heat_filtered = cmd_heat_gate.predicted_rest_heat;
  }
}

static void UpdateShootRateFeedforward100ms(void) {
  const uint32_t now_ms = (uint32_t)DWT_GetTimeline_ms();
  const uint8_t link_online = IsChassisLinkOnline();
  const uint8_t referee_online =
      (link_online && chassis_fetch_data.referee_online) ? 1U : 0U;

  if (cmd_last_ff_update_ms == 0U ||
      (now_ms - cmd_last_ff_update_ms) >= CMD_FF_UPDATE_PERIOD_MS) {
    cmd_last_ff_update_ms = now_ms;

    if (referee_online) {
      float m = GetRawRestHeatFloat();
      if (m <= 0.0f && chassis_fetch_data.barrel_heat_limit >
                           chassis_fetch_data.barrel_heat) {
        m = (float)(chassis_fetch_data.barrel_heat_limit -
                    chassis_fetch_data.barrel_heat);
      }

      const float a = (float)chassis_fetch_data.barrel_cooling_value;
      // c_ff=(10m-a)/(d*n)+a/d，其中 n=10*t_target
      const float n = 10.0f * FEEDFORWARD_T_TARGET_S;
      const float denominator = HEAT_PER_SHOT_D * n;

      if (denominator > 0.0f) {
        const float c_ff =
            ((10.0f * m - a) / denominator) + (a / HEAT_PER_SHOT_D);
        if (isfinite(c_ff)) {
          cmd_shoot_rate_ff =
              float_constrain(c_ff, SHOOT_RATE_MIN, SHOOT_RATE_MAX);
        } else {
          cmd_shoot_rate_ff = CMD_SAFE_SHOOT_RATE;
        }
      } else {
        cmd_shoot_rate_ff = CMD_SAFE_SHOOT_RATE;
      }
    } else {
      cmd_shoot_rate_ff = CMD_SAFE_SHOOT_RATE;
    }
  }

  shoot_cmd_send.shoot_rate = cmd_shoot_rate_ff;
}

static void UpdateAutoFireGate(void) {
  const uint32_t now_ms = (uint32_t)DWT_GetTimeline_ms();
  const uint8_t link_online = IsChassisLinkOnline();
  const uint8_t referee_online =
      (link_online && chassis_fetch_data.referee_online) ? 1U : 0U;
  const uint8_t vision_online = VisionIsOnline() ? 1U : 0U;
  const uint8_t autoaim_mode =
      (vision_cmd_send.vision_mode == VISION_MODE_AUTO_AIM) ? 1U : 0U;
  const loader_mode_e requested_load_mode = shoot_cmd_send.load_mode;
  const float raw_rest_heat = GetRawRestHeatFloat();
  const uint8_t manual_fire_request = IsShootLoadFireIntent(requested_load_mode);
  float gate_rest_heat = 0.0f;
  float required_heat = 0.0f;
  uint8_t manual_fire_ok = 0U;
  uint8_t auto_fire_permission = 0U;
  uint8_t auto_fire_request_active = 0U;
  uint8_t auto_fire_shot_allow = 0U;

  SyncHeatGateState(now_ms, referee_online);
  if (cmd_auto_fire_shot_pulse &&
      (now_ms - cmd_auto_fire_pulse_start_ms) >= CMD_MOUSE_FIRE_PULSE_HOLD_MS) {
    cmd_auto_fire_shot_pulse = 0U;
  }
  gate_rest_heat = referee_online ? cmd_heat_gate.predicted_rest_heat : 0.0f;
  cmd_rest_heat_filtered = gate_rest_heat;

  Cmd_Degrade_State_e next_candidate = CMD_STATE_NORMAL;
  if (!link_online) {
    next_candidate = CMD_STATE_DEGRADED_LINK;
  } else if (!referee_online) {
    next_candidate = CMD_STATE_DEGRADED_REFEREE;
  } else if (autoaim_mode && !vision_online) {
    next_candidate = CMD_STATE_DEGRADED_VISION;
  }

  if (next_candidate != cmd_degrade_candidate) {
    cmd_degrade_candidate = next_candidate;
    cmd_state_candidate_since_ms = now_ms;
  }

  uint32_t hold_ms = (cmd_degrade_candidate == CMD_STATE_NORMAL)
                         ? CMD_DEGRADE_EXIT_HOLD_MS
                         : CMD_DEGRADE_ENTER_HOLD_MS;
  if (cmd_degrade_state != cmd_degrade_candidate &&
      (now_ms - cmd_state_candidate_since_ms) >= hold_ms) {
    cmd_degrade_state = cmd_degrade_candidate;
  }

  cmd_force_safe_fire = (cmd_degrade_state == CMD_STATE_NORMAL) ? 0U : 1U;

  shoot_cmd_send.bullet_speed = SHOOT_FIXED_BULLET_SPEED;
  shoot_cmd_send.rest_heat =
      (uint8_t)float_constrain(cmd_rest_heat_filtered, 0.0f, 255.0f);

  uint8_t operator_auto_fire_request =
      (autoaim_mode && vision_cmd_send.allow_auto_fire) ? 1U : 0U;
  if (requested_load_mode != LOAD_BURSTFIRE) {
    cmd_burst_gate_active = 0U;
    cmd_burst_gate_last_ms = now_ms;
  }
  if (requested_load_mode != LOAD_1_BULLET &&
      requested_load_mode != LOAD_3_BULLET) {
    cmd_last_discrete_gate_mode = LOAD_STOP;
    cmd_last_discrete_gate_allow = 0U;
  }

  // 手动发射不再受热量门控限制：允许操作员持续驱动拨弹盘。
  // 自动开火仍沿用 HeatGateTryReserve() 进行热量预约与限流。
  if (manual_fire_request) {
    manual_fire_ok = 1U;
    required_heat = 0.0f;
    cmd_last_discrete_gate_mode = requested_load_mode;
    cmd_last_discrete_gate_allow = 1U;
    cmd_burst_gate_active = 0U;
    cmd_burst_gate_last_ms = now_ms;
  }
  cmd_rest_heat_filtered = cmd_heat_gate.predicted_rest_heat;
  gate_rest_heat = cmd_rest_heat_filtered;

  const uint8_t heat_ok = (gate_rest_heat >= CMD_MIN_REST_HEAT_TO_FIRE) ? 1U : 0U;
  auto_fire_permission =
      (operator_auto_fire_request && vision_data_recv.vision_valid &&
       vision_data_recv.target_locked && vision_data_recv.vision_takeover &&
       link_online && !cmd_force_safe_fire && referee_online && heat_ok)
          ? 1U
          : 0U;
  auto_fire_request_active =
      (auto_fire_permission && vision_data_recv.should_fire) ? 1U : 0U;

  if (!auto_fire_permission) {
    cmd_auto_fire_shot_pulse = 0U;
  }

  if (auto_fire_request_active && !cmd_auto_fire_shot_pulse) {
    const uint32_t auto_fire_interval_ms =
        (shoot_cmd_send.shoot_rate > 0.0f)
            ? (uint32_t)(1000.0f / shoot_cmd_send.shoot_rate)
            : 0U;
    const uint32_t auto_fire_safe_interval_ms =
        auto_fire_interval_ms + CMD_MOUSE_FIRE_PULSE_HOLD_MS;
    const uint8_t auto_fire_due =
        (cmd_auto_fire_next_shot_ms == 0U ||
         now_ms >= cmd_auto_fire_next_shot_ms)
            ? 1U
            : 0U;

    if (auto_fire_due) {
      required_heat = GetRequiredHeatForLoadMode(LOAD_1_BULLET);
      auto_fire_shot_allow =
          HeatGateTryReserve(&cmd_heat_gate, required_heat, now_ms);
      if (auto_fire_shot_allow) {
        cmd_auto_fire_shot_pulse = 1U;
        cmd_auto_fire_pulse_start_ms = now_ms;
        cmd_auto_fire_next_shot_ms = now_ms + auto_fire_safe_interval_ms;
      }
    }
  }

  if (cmd_force_safe_fire) {
    vision_cmd_send.allow_auto_fire = 0U;
    shoot_cmd_send.shoot_rate = CMD_SAFE_SHOOT_RATE;
    if (autoaim_mode) {
      shoot_cmd_send.load_mode = LOAD_STOP;
    }
  } else {
    vision_cmd_send.allow_auto_fire = auto_fire_permission;
    if (autoaim_mode && !auto_fire_permission) {
      shoot_cmd_send.load_mode = LOAD_STOP;
    }
  }

  if (manual_fire_request && !manual_fire_ok) {
    shoot_cmd_send.load_mode = LOAD_STOP;
  }

  shoot_cmd_send.rest_heat =
      (uint8_t)float_constrain(cmd_rest_heat_filtered, 0.0f, 255.0f);
  cmd_last_required_heat =
      manual_fire_request
          ? required_heat
          : (operator_auto_fire_request ? CMD_MIN_REST_HEAT_TO_FIRE : 0.0f);
  cmd_last_gate_allow =
      manual_fire_request ? manual_fire_ok : cmd_auto_fire_shot_pulse;
  cmd_last_gate_mode =
      manual_fire_request
          ? requested_load_mode
          : (operator_auto_fire_request ? LOAD_1_BULLET : requested_load_mode);

  if ((now_ms - cmd_last_telemetry_ms) >= CMD_GATE_TELEMETRY_PERIOD_MS) {
    char req_heat_str[16];
    char raw_heat_str[16];
    char pred_heat_str[16];
    Float2Str(req_heat_str, cmd_last_required_heat);
    Float2Str(raw_heat_str, raw_rest_heat);
    Float2Str(pred_heat_str, cmd_rest_heat_filtered);
    LOGINFO("[cmd-gate] state=%d link=%u referee=%u vision=%u "
            "req_mode=%d allow=%u req_heat=%s raw=%s pred=%s cool=%u "
            "auto=%u",
            (int)cmd_degrade_state, link_online, referee_online, vision_online,
            (int)cmd_last_gate_mode, cmd_last_gate_allow, req_heat_str,
            raw_heat_str, pred_heat_str, chassis_fetch_data.barrel_cooling_value,
            vision_cmd_send.allow_auto_fire);
    cmd_last_telemetry_ms = now_ms;
  }
}

static void UpdateChassisFetchDataFromCan(void) {
#ifdef GIMBAL_BOARD
  if (cmd_can_fast_comm != NULL && CANCommIsOnline(cmd_can_fast_comm)) {
    memcpy(&chassis_feed_fast_recv, CANCommGet(cmd_can_fast_comm),
           sizeof(chassis_feed_fast_recv));
    chassis_fetch_data.referee_online = chassis_feed_fast_recv.referee_online;
    chassis_fetch_data.rest_heat = chassis_feed_fast_recv.rest_heat;
    chassis_fetch_data.barrel_heat = chassis_feed_fast_recv.barrel_heat;
    chassis_fetch_data.barrel_heat_limit = chassis_feed_fast_recv.barrel_heat_limit;
    chassis_fetch_data.barrel_cooling_value =
        chassis_feed_fast_recv.barrel_cooling_value;
    chassis_fetch_data.bullet_speed_limit =
        chassis_feed_fast_recv.bullet_speed_limit;
  } else {
    const uint32_t now_ms = (uint32_t)DWT_GetTimeline_ms();
    ResetChassisFetchDataSafe();
    if ((now_ms - cmd_last_can_warn_ms) >= 500U) {
      LOGWARNING("[cmd] Chassis CAN link offline, fallback to safe defaults.");
      cmd_last_can_warn_ms = now_ms;
    }
    return;
  }

  if (cmd_can_state_comm != NULL && CANCommIsOnline(cmd_can_state_comm)) {
    memcpy(&chassis_feed_state_recv, CANCommGet(cmd_can_state_comm),
           sizeof(chassis_feed_state_recv));
    chassis_fetch_data.bullet_speed = chassis_feed_state_recv.bullet_speed;
    chassis_fetch_data.chassis_power_limit =
        chassis_feed_state_recv.chassis_power_limit;
  } else {
    chassis_fetch_data.bullet_speed = SHOOT_FIXED_BULLET_SPEED;
    chassis_fetch_data.chassis_power_limit = 0U;
  }
#endif
}

static void BuildChassisCanPackets(void) {
  chassis_ctrl_fast_send.vx = chassis_cmd_send.vx;
  chassis_ctrl_fast_send.vy = chassis_cmd_send.vy;
  chassis_ctrl_fast_send.wz = chassis_cmd_send.wz;
  chassis_ctrl_fast_send.offset_angle = chassis_cmd_send.offset_angle;
  chassis_ctrl_fast_send.near_center_error = chassis_cmd_send.near_center_error;
  chassis_ctrl_fast_send.chassis_mode = chassis_cmd_send.chassis_mode;

  chassis_ctrl_state_send.chassis_speed_buff = chassis_cmd_send.chassis_speed_buff;

  chassis_ctrl_ui_send.ui_friction_on = chassis_cmd_send.ui_friction_on;
  chassis_ctrl_ui_send.ui_autoaim_enabled = chassis_cmd_send.ui_autoaim_enabled;
  chassis_ctrl_ui_send.ui_fire_allow = chassis_cmd_send.ui_fire_allow;
  chassis_ctrl_ui_send.ui_stuck_active = chassis_cmd_send.ui_stuck_active;
  chassis_ctrl_ui_send.ui_loader_mode = chassis_cmd_send.ui_loader_mode;

  chassis_ctrl_event_send.ui_refresh_request_seq =
      chassis_cmd_send.ui_refresh_request_seq;
}

static void SendChassisCommandCanIfDue(void) {
#ifdef GIMBAL_BOARD
  const uint32_t now_ms = (uint32_t)DWT_GetTimeline_ms();

  if (cmd_can_fast_comm == NULL || cmd_can_state_comm == NULL ||
      cmd_can_ui_comm == NULL || cmd_can_event_comm == NULL) {
    return;
  }

  BuildChassisCanPackets();

  if (now_ms >= cmd_next_fast_can_send_ms) {
    CANCommSend(cmd_can_fast_comm, (void *)&chassis_ctrl_fast_send);
    do {
      cmd_next_fast_can_send_ms += CMD_CAN_FAST_PERIOD_MS;
    } while (cmd_next_fast_can_send_ms <= now_ms);
  }

  if (now_ms >= cmd_next_state_can_send_ms) {
    CANCommSend(cmd_can_state_comm, (void *)&chassis_ctrl_state_send);
    do {
      cmd_next_state_can_send_ms += CMD_CAN_STATE_PERIOD_MS;
    } while (cmd_next_state_can_send_ms <= now_ms);
  }

  if (now_ms >= cmd_next_ui_can_send_ms) {
    CANCommSend(cmd_can_ui_comm, (void *)&chassis_ctrl_ui_send);
    do {
      cmd_next_ui_can_send_ms += CMD_CAN_UI_PERIOD_MS;
    } while (cmd_next_ui_can_send_ms <= now_ms);
  }

  if (cmd_last_event_seq_sent != chassis_ctrl_event_send.ui_refresh_request_seq) {
    cmd_last_event_seq_sent = chassis_ctrl_event_send.ui_refresh_request_seq;
    CANCommSend(cmd_can_event_comm, (void *)&chassis_ctrl_event_send);
  }
#endif
}

static void ApplyRobotStopOutputs(void) {
  robot_state = ROBOT_STOP;
  cmd_force_safe_fire = 1U;

  gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
  gimbal_cmd_send.vision_yaw_direct = 0;
  gimbal_cmd_send.vision_pitch_direct = 0;
  gimbal_cmd_send.vision_yaw_current = 0.0f;
  gimbal_cmd_send.vision_pitch_ref = 0.0f;

  chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
  chassis_cmd_send.vx = 0.0f;
  chassis_cmd_send.vy = 0.0f;
  chassis_cmd_send.wz = 0.0f;
  chassis_cmd_send.offset_angle = 0.0f;
  chassis_cmd_send.near_center_error = 0.0f;
  chassis_cmd_send.ui_friction_on = 0U;
  chassis_cmd_send.ui_autoaim_enabled = 0U;
  chassis_cmd_send.ui_fire_allow = 0U;
  chassis_cmd_send.ui_stuck_active = 0U;
  chassis_cmd_send.ui_loader_mode = (uint8_t)LOAD_STOP;

  shoot_cmd_send.shoot_mode = SHOOT_OFF;
  shoot_cmd_send.friction_mode = FRICTION_OFF;
  shoot_cmd_send.load_mode = LOAD_STOP;
  shoot_cmd_send.shoot_rate = CMD_SAFE_SHOOT_RATE;
  shoot_cmd_send.rest_heat = 0U;

  vision_cmd_send.vision_mode = VISION_MODE_OFF;
  vision_cmd_send.allow_auto_fire = 0U;
  cmd_auto_fire_shot_pulse = 0U;
  cmd_auto_fire_pulse_start_ms = 0U;
  cmd_auto_fire_next_shot_ms = 0U;
  cmd_rest_heat_filtered = 0.0f;
  HeatGateReset(&cmd_heat_gate, 0.0f, 0.0f, 0.0f,
                (uint32_t)DWT_GetTimeline_ms());
  ResetManualHeatGateState((uint32_t)DWT_GetTimeline_ms());
  cmd_heat_snapshot_valid = 0U;
}

static void UpdateRobotSafetyState(void) {
  uint8_t safe_hold_active = 0U;
  const uint8_t input_online = cmd_input_active.online;
  const uint8_t vtm_pause_toggle =
      (cmd_input_source == CMD_INPUT_SOURCE_VTM && cmd_input_active.pause_pressed &&
       !cmd_vtm_pause_prev)
          ? 1U
          : 0U;

  if (vtm_pause_toggle) {
    cmd_vtm_pause_latched = cmd_vtm_pause_latched ? 0U : 1U;
  }

  if (cmd_input_source == CMD_INPUT_SOURCE_VTM) {
    safe_hold_active = cmd_vtm_pause_latched;
  }

  if (!input_online) {
    if (robot_state != ROBOT_STOP) {
      LOGWARNING("[CMD] control input offline, force robot to STOP.");
    }
    robot_state = ROBOT_STOP;
    cmd_vtm_pause_latched = 0U;
  } else if (safe_hold_active) {
    if (robot_state != ROBOT_STOP) {
      LOGWARNING("[CMD] safe hold active, force robot to STOP.");
    }
    robot_state = ROBOT_STOP;
  } else if (cmd_input_source == CMD_INPUT_SOURCE_VTM) {
    if (robot_state != ROBOT_READY) {
      robot_state = ROBOT_READY;
      LOGINFO("[CMD] VTM input online, robot enters READY.");
    }
  } else if (cmd_input_source == CMD_INPUT_SOURCE_RC) {
    if (robot_state != ROBOT_READY) {
      robot_state = ROBOT_READY;
      LOGINFO("[CMD] RC input online, robot enters READY.");
    }
  }

  cmd_vtm_pause_prev =
      (cmd_input_source == CMD_INPUT_SOURCE_VTM && cmd_input_active.pause_pressed)
          ? 1U
          : 0U;
}

static void PublishImageTelemetrySummary(void) {
  const loader_mode_e ui_load_mode =
      (vision_cmd_send.vision_mode == VISION_MODE_AUTO_AIM)
          ? LOAD_1_BULLET
          : ui_loader_mode_selected;
  const float ui_required_heat = GetRequiredHeatForLoadMode(ui_load_mode);
  chassis_cmd_send.ui_friction_on =
      (shoot_cmd_send.friction_mode == FRICTION_ON) ? 1U : 0U;
  chassis_cmd_send.ui_autoaim_enabled =
      (vision_cmd_send.vision_mode == VISION_MODE_AUTO_AIM) ? 1U : 0U;
  chassis_cmd_send.ui_fire_allow =
      (shoot_cmd_send.shoot_mode == SHOOT_ON &&
       shoot_cmd_send.friction_mode == FRICTION_ON && !cmd_force_safe_fire &&
       (cmd_rest_heat_filtered >= ui_required_heat) && (ui_required_heat > 0.0f) &&
       (shoot_fetch_data.loader_jam_state == 0U))
          ? 1U
          : 0U;
  // UI 跨板通道仅传最小摘要：卡弹流程状态 + 发射模式。
  // 热量预测/融合值只保留在云台控制层使用，不进入底盘 UI 显示链。
  chassis_cmd_send.ui_stuck_active =
      (shoot_fetch_data.loader_jam_state != 0U) ? 1U : 0U;
  chassis_cmd_send.ui_loader_mode = (uint8_t)ui_loader_mode_selected;
}

void RobotCMDInit() {
  rc_data = RemoteControlInit(
      &huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
  vtm_input_data = VTMInputInit(&huart6);

  // 注意: 视觉CAN通信初始化已移至 VisionAppInit() (vision.c)

  // 视觉控制消息注册
  vision_cmd_pub = RegisterPublisher("vision_cmd", sizeof(Vision_Ctrl_Cmd_s));
  vision_data_sub =
      RegisterSubscriber("vision_data", sizeof(Vision_Upload_Data_s));

  gimbal_cmd_pub = RegisterPublisher("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
  gimbal_feed_sub =
      RegisterSubscriber("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
  shoot_cmd_pub = RegisterPublisher("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
  shoot_feed_sub =
      RegisterSubscriber("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
  chassis_cmd_pub =
      RegisterPublisher("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
  chassis_feed_sub =
      RegisterSubscriber("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
  CANComm_Init_Config_s fast_comm_conf = {
      .can_config =
          {
              .can_handle = &hcan2,
              .tx_id = 0x312,
              .rx_id = 0x311,
          },
      .recv_data_len = sizeof(Chassis_Feed_Fast_Pkt_s),
      .send_data_len = sizeof(Chassis_Ctrl_Fast_Pkt_s),
  };
  CANComm_Init_Config_s state_comm_conf = {
      .can_config =
          {
              .can_handle = &hcan2,
              .tx_id = 0x322,
              .rx_id = 0x321,
          },
      .recv_data_len = sizeof(Chassis_Feed_State_Pkt_s),
      .send_data_len = sizeof(Chassis_Ctrl_State_Pkt_s),
  };
  CANComm_Init_Config_s ui_comm_conf = {
      .can_config =
          {
              .can_handle = &hcan2,
              .tx_id = 0x332,
              .rx_id = 0x331,
          },
      .recv_data_len = 0U,
      .send_data_len = sizeof(Chassis_Ctrl_UI_Pkt_s),
  };
  CANComm_Init_Config_s event_comm_conf = {
      .can_config =
          {
              .can_handle = &hcan2,
              .tx_id = 0x342,
              .rx_id = 0x341,
          },
      .recv_data_len = 0U,
      .send_data_len = sizeof(Chassis_Ctrl_Event_Pkt_s),
  };
  cmd_can_fast_comm = CANCommInit(&fast_comm_conf);
  cmd_can_state_comm = CANCommInit(&state_comm_conf);
  cmd_can_ui_comm = CANCommInit(&ui_comm_conf);
  cmd_can_event_comm = CANCommInit(&event_comm_conf);
#endif // GIMBAL_BOARD

  gimbal_cmd_send.yaw = 0.0f;   // Yaw初始化为0弧度
  gimbal_cmd_send.pitch = 0.0f; // Pitch初始化为0弧度

  // 初始化发射模块控制指令
  shoot_cmd_send.shoot_mode = SHOOT_OFF;                  // 默认安全停射
  shoot_cmd_send.friction_mode = FRICTION_OFF;            // 摩擦轮默认关闭
  shoot_cmd_send.load_mode = LOAD_STOP;                   // 拨盘默认停止
  shoot_cmd_send.bullet_speed = SHOOT_FIXED_BULLET_SPEED; // 固定弹速 12m/s
  shoot_cmd_send.shoot_rate = SHOOT_RATE_SAFE;            // 默认安全射频
  shoot_cmd_send.rest_heat = 0U;

  // 初始化视觉控制指令
  vision_cmd_send.vision_mode = VISION_MODE_OFF; // 默认关闭视觉控制
  vision_cmd_send.allow_auto_fire = 0;           // 默认禁止自动射击
  vision_cmd_send.bullet_speed_limit = 0.0f;     // 默认弹速为0，严格透传当前值
  vision_cmd_send.manual_yaw_offset = 0.0f;      // 无手动微调
  vision_cmd_send.manual_pitch_offset = 0.0f;

  // 初始化底盘回传摘要
  chassis_cmd_send.chassis_speed_buff = CHASSIS_POWER_LIMIT_PERCENT;
  chassis_cmd_send.ui_friction_on = 0;
  chassis_cmd_send.ui_autoaim_enabled = 0;
  chassis_cmd_send.ui_fire_allow = 0;
  chassis_cmd_send.ui_stuck_active = 0;
  chassis_cmd_send.ui_loader_mode = (uint8_t)LOAD_STOP;
  chassis_cmd_send.ui_refresh_request_seq = 0U;
  ResetChassisFetchDataSafe();

  cmd_degrade_state = CMD_STATE_DEGRADED_LINK;
  cmd_degrade_candidate = CMD_STATE_DEGRADED_LINK;
  cmd_state_candidate_since_ms = (uint32_t)DWT_GetTimeline_ms();
  cmd_last_telemetry_ms = 0;
  cmd_last_ff_update_ms = 0;
  cmd_last_can_warn_ms = 0;
  cmd_force_safe_fire = 1U;
  cmd_auto_fire_shot_pulse = 0U;
  cmd_auto_fire_pulse_start_ms = 0U;
  cmd_auto_fire_next_shot_ms = 0U;
  cmd_rest_heat_filtered = 0.0f;
  cmd_shoot_rate_ff = CMD_SAFE_SHOOT_RATE;
  HeatGateInit(&cmd_heat_gate, HEAT_PER_SHOT_D);
  ResetManualHeatGateState((uint32_t)DWT_GetTimeline_ms());
  cmd_heat_snapshot_valid = 0U;
  cmd_last_barrel_heat = 0U;
  cmd_last_barrel_heat_limit = 0U;
  cmd_last_barrel_cooling_value = 0U;
  cmd_last_rest_heat_raw = 0U;
  cmd_vtm_pause_latched = 0U;
  cmd_vtm_pause_prev = 0U;
  cmd_ui_refresh_ctrl_prev = 0U;
  {
    const uint32_t now_ms = (uint32_t)DWT_GetTimeline_ms();
    cmd_next_fast_can_send_ms = now_ms + CMD_CAN_FAST_PHASE_MS;
    cmd_next_state_can_send_ms = now_ms + CMD_CAN_STATE_PHASE_MS;
    cmd_next_ui_can_send_ms = now_ms + CMD_CAN_UI_PHASE_MS;
  }
  cmd_last_event_seq_sent = 0U;
  keyboard_power_mode_selected = CHASSIS_POWER_LIMIT_PERCENT;
  keyboard_load_mode_selected = LOAD_STOP;
  ui_loader_mode_selected = LOAD_STOP;
  keyboard_chassis_mode_selected = CHASSIS_FOLLOW_GIMBAL_YAW;
  ResetCmdInputActive();

  robot_state = ROBOT_STOP; // 上电默认停机，控制链路在线后自动进入READY
  ApplyRobotStopOutputs();
}

/**
 * @brief 快速角度归一化到[-180, 180]（内联优化）
 * @note 输入范围假设在[-360, 360]内（单圈角度差），最多需要一次调整
 */
static inline float NormalizeAngleFast(float angle) {
  if (angle > 180.0f)
    return angle - 360.0f;
  if (angle <= -180.0f)
    return angle + 360.0f;
  return angle;
}

/**
 * @brief 计算云台-底盘角度关系（高性能优化版本）
 *        同时计算 offset_angle（坐标变换用）和 near_center_error（跟随控制用）
 *
 * @note 优化要点：
 *       1. 统一角度归一化逻辑，消除条件编译分支
 *       2. 利用数学对称性减少重复计算
 *       3. 使用内联函数和条件表达式优化指令级性能
 */
static void CalcGimbalChassisAngle(void) {
  // ============================================================
  // Part 1: 计算 offset_angle（统一归一化逻辑）
  // ============================================================
  float yaw_angle = gimbal_fetch_data.yaw_motor_single_round_angle;
  float raw_diff = yaw_angle - YAW_ALIGN_ANGLE;

  // 快速归一化到 [-180, 180]
  // 输入范围：yaw_angle ∈ [0, 360], YAW_ALIGN_ANGLE ∈ [0, 360]
  // 差值范围：raw_diff ∈ [-360, 360]，最多需要一次调整
  raw_diff = NormalizeAngleFast(raw_diff);
  chassis_cmd_send.offset_angle = raw_diff;

  // ============================================================
  // Part 2: 就近回中（Flip 逻辑优化）
  // ============================================================
#if CHASSIS_FOLLOW_ALLOW_FLIP
  static uint8_t flip_state = 0;

  // 1. 根据当前翻转状态计算误差
  //    flip_state = 0: target = 0°,   error = raw_diff
  //    flip_state = 1: target = 180°, error = NormalizeAngleFast(raw_diff -
  //    180°)
  float error = raw_diff;
  if (flip_state) {
    error = NormalizeAngleFast(raw_diff - 180.0f);
  }

  // 2. 检查是否需要切换翻转状态（迟滞逻辑）
  //    当误差绝对值超过阈值时，说明底盘"背对"云台，触发翻转
  if (fabsf(error) > CHASSIS_FOLLOW_FLIP_THRESHOLD) {
    flip_state = !flip_state;

    // 翻转后利用数学对称性直接计算新误差：
    // error_new = error_old ± 180° (归一化)
    // 若 error > 0: error_new = error - 180
    // 若 error < 0: error_new = error + 180
    error = (error > 0.0f) ? (error - 180.0f) : (error + 180.0f);
  }
#else
  float error = chassis_cmd_send.offset_angle;
#endif

  // ============================================================
  // Part 3: 限幅（优化为 if-else if 结构，便于编译器生成 VMIN/VMAX 指令）
  // ============================================================
  if (error > CHASSIS_FOLLOW_MAX_ERR)
    error = CHASSIS_FOLLOW_MAX_ERR;
  else if (error < -CHASSIS_FOLLOW_MAX_ERR)
    error = -CHASSIS_FOLLOW_MAX_ERR;

  chassis_cmd_send.near_center_error = error;
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet() {
  // 遥控器模式固定为额定功率档，避免与键鼠档位残留互相干扰
  chassis_cmd_send.chassis_speed_buff = CHASSIS_POWER_LIMIT_PERCENT;
  // 左侧开关状态为[下],底盘跟随云台
  if (switch_is_down(cmd_input_active.rc.switch_left)) {
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    // gimbal_cmd_send.gimbal_mode = GIMBAL_SYS_ID_CHIRP; #系统辨识开关
  }
  // 左侧开关状态为[中],底盘小陀螺
  else if (switch_is_mid(cmd_input_active.rc.switch_left)) {
    chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
  }
  // 左侧开关上档已在 RobotCMDTask 中切入键鼠模式，这里对异常值保守回落到跟随云台。
  else {
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
  }
  // 底盘改为左摇杆
  static float vx_filtered = 0.0f;
  static float vy_filtered = 0.0f;

  float vy_norm = (float)cmd_input_active.rc.rocker_l1 / 660.0f;
  float vx_norm = (float)cmd_input_active.rc.rocker_l_ / 660.0f;

  vx_filtered = LowPassFilter_Float(vx_norm, 0.90f, &vx_filtered);
  vy_filtered = LowPassFilter_Float(vy_norm, 0.90f, &vy_filtered);

  chassis_cmd_send.vx = vx_filtered;
  chassis_cmd_send.vy = vy_filtered;

  // 云台改为右摇杆，添加低通滤波
  static float yaw_increment_filtered = 0.0f;   // 保存上一次的滤波输出
  static float pitch_increment_filtered = 0.0f; // 保存上一次的滤波输出

  // 计算原始增量，右摇杆归一化到 [-1, 1]
  float yaw_norm = (float)cmd_input_active.rc.rocker_r_ / 660.0f; // 转为弧度
  float pitch_norm = (float)cmd_input_active.rc.rocker_r1 / 660.0f;

  // 角度增量：Yaw 转弧度，Pitch 保持角度制，后续增益由使用者调整
  float yaw_increment = yaw_norm / 180.0f * M_PI; // 相当于 1° 的弧度
  float pitch_increment =
      pitch_norm / 180.0f * M_PI; // 归一化刻度直接视为弧度增量

  yaw_increment =
      LowPassFilter_Float(yaw_increment, 0.95f, &yaw_increment_filtered);
  pitch_increment =
      LowPassFilter_Float(pitch_increment, 0.93f, &pitch_increment_filtered);

  // 更新目标角度
  gimbal_cmd_send.yaw += yaw_increment;     // Yaw: 弧度制
  gimbal_cmd_send.pitch += pitch_increment; // Pitch: 弧度制
  LIMIT_PITCH_RAD(gimbal_cmd_send.pitch);   // Pitch 角度限位

  // 发射参数 + 视觉模式联动
  // 右侧开关仅在纯遥控模式下控制手动射击链路。
  // [下档]: 正常遥控云台，摩擦轮关闭
  // [中档]: 正常遥控云台，摩擦轮开启
  // [上档]: 打开摩擦轮并持续单发
  if (switch_is_down(cmd_input_active.rc.switch_right)) {
    // 下档：正常遥控云台，关闭摩擦轮
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    vision_cmd_send.vision_mode = VISION_MODE_OFF; // 关闭视觉
    vision_cmd_send.allow_auto_fire = 0;
    ui_loader_mode_selected = LOAD_STOP;
  } else if (switch_is_mid(cmd_input_active.rc.switch_right)) {
    // 中档：正常遥控云台，开启摩擦轮
    shoot_cmd_send.friction_mode = FRICTION_ON;
    shoot_cmd_send.load_mode = LOAD_STOP;
    vision_cmd_send.vision_mode = VISION_MODE_OFF; // 关闭视觉
    vision_cmd_send.allow_auto_fire = 0;
    ui_loader_mode_selected = LOAD_STOP;
  } else {
    // 上档：打开摩擦轮并持续单发，由 shoot 任务按 LOAD_1_BULLET 连续消费。
    shoot_cmd_send.friction_mode = FRICTION_ON;
    shoot_cmd_send.load_mode = LOAD_1_BULLET;
    vision_cmd_send.vision_mode = VISION_MODE_OFF; // 关闭视觉
    vision_cmd_send.allow_auto_fire = 0;
    ui_loader_mode_selected = LOAD_1_BULLET;
  }
}

static void MouseKeySet() {
  float target_vx, target_vy;
  static uint32_t keyboard_ramp_dwt_cnt = 0U;
  static uint8_t keyboard_ramp_dt_synced = 0U;
  static loader_mode_e mouse_pulse_load_mode = LOAD_STOP;
  static uint32_t mouse_pulse_start_ms = 0U;
  float dt = 0.0f;

  /* 键鼠斜坡规划使用真实控制周期，避免手写固定dt导致参数与实际调度失配。
   * 首次调用只同步时间戳，不使用异常大的首帧dt。
   * 同时对dt做上下限保护，避免断点/卡顿后一次性跳变过大。 */
  if (!keyboard_ramp_dt_synced) {
    DWT_GetDeltaT(&keyboard_ramp_dwt_cnt);
    keyboard_ramp_dt_synced = 1U;
    dt = 0.005f;
  } else {
    dt = DWT_GetDeltaT(&keyboard_ramp_dwt_cnt);
    dt = float_constrain(dt, 0.001f, 0.02f);
  }

  // 添加底盘键盘控制滤波静态变量
  static float vx_kb_filtered = 0.0f;
  static float vy_kb_filtered = 0.0f;

  // 键鼠模式下发送给底盘的 vx/vy 必须与遥控器模式保持同一量纲：归一化到 [-1, 1]
  // 后续由 Chassis 侧统一乘以 max_linear_speed 转换为实际速度
  /* 当前底盘坐标语义下:
   * vx 对应左右平移，vy 对应前后运动。
   * 因此键盘映射需要与常见 WASD 语义对齐：
   * W/S -> 前进/后退 -> vy 正/负
   * A/D -> 左移/右移 -> vx 负/正 */
  target_vx = (float)cmd_input_active.key[KEY_PRESS].d -
              (float)cmd_input_active.key[KEY_PRESS].a;
  target_vy = (float)cmd_input_active.key[KEY_PRESS].w -
              (float)cmd_input_active.key[KEY_PRESS].s;

  // ✅ 为键盘控制添加低通滤波（α=0.8，截止频率约62Hz）
  // 原因：键盘输入是阶跃信号，需要滤波避免突变
  vx_kb_filtered = LowPassFilter_Float(target_vx, 0.80f, &vx_kb_filtered);
  vy_kb_filtered = LowPassFilter_Float(target_vy, 0.80f, &vy_kb_filtered);
  // ✅ 注意：使用滤波后的值作为斜坡规划的输入，减少冲击
  keyboard_vx_cmd_planned =
      SoftRamp(vx_kb_filtered, keyboard_vx_cmd_planned, 0, KEYBOARD_RAMP_ACCEL,
               KEYBOARD_RAMP_DECEL, KEYBOARD_RAMP_BRAKE_DECEL, dt);
  keyboard_vy_cmd_planned =
      SoftRamp(vy_kb_filtered, keyboard_vy_cmd_planned, 0, KEYBOARD_RAMP_ACCEL,
               KEYBOARD_RAMP_DECEL, KEYBOARD_RAMP_BRAKE_DECEL, dt);
  chassis_cmd_send.vx = keyboard_vx_cmd_planned;
  chassis_cmd_send.vy = keyboard_vy_cmd_planned;

  // ✅ 添加鼠标控制滤波静态变量
  static float mouse_yaw_increment = 0.0f;
  static float mouse_pitch_increment = 0.0f;

  // 计算原始鼠标增量
  // Yaw: 角度 → 弧度（LQR需要弧度制）
  float raw_mouse_yaw = (float)cmd_input_active.mouse.x / 660.0f;
  // Pitch: 鼠标输入同样归一化为弧度刻度
  float raw_mouse_pitch = (float)cmd_input_active.mouse.y / 660.0f;

  // ✅ 为鼠标控制添加低通滤波
  // 鼠标DPI通常在400-1600之间，会产生高频噪声，需要滤波
  // Yaw: α=0.85，截止频率约62.8Hz，保持瞄准精度
  // Pitch: α=0.80，截止频率约51Hz，避免垂直抖动
  mouse_yaw_increment =
      LowPassFilter_Float(raw_mouse_yaw, 0.85f, &mouse_yaw_increment);
  mouse_pitch_increment =
      LowPassFilter_Float(raw_mouse_pitch, 0.80f, &mouse_pitch_increment);

  gimbal_cmd_send.yaw += mouse_yaw_increment;
  gimbal_cmd_send.pitch += mouse_pitch_increment;
  LIMIT_PITCH_RAD(gimbal_cmd_send.pitch); // Pitch 角度限位

  // 键鼠路径固定使用单一 12m/s 弹速，不再通过键位切换档位
  shoot_cmd_send.bullet_speed = SHOOT_FIXED_BULLET_SPEED;
  // 发射模式改为直达选择：
  // Z = 1_BULLET, X = 3_BULLET, C = BURSTFIRE
  // 采用“按一下锁定当前模式”的语义，避免循环切换导致误操作。
  static uint8_t load_key_count_z_last = 0U;
  static uint8_t load_key_count_x_last = 0U;
  static uint8_t load_key_count_c_last = 0U;
  if (cmd_input_active.key_count[KEY_PRESS][Key_Z] != load_key_count_z_last) {
    keyboard_load_mode_selected = LOAD_1_BULLET;
    load_key_count_z_last = cmd_input_active.key_count[KEY_PRESS][Key_Z];
  }
  if (cmd_input_active.key_count[KEY_PRESS][Key_X] != load_key_count_x_last) {
    keyboard_load_mode_selected = LOAD_3_BULLET;
    load_key_count_x_last = cmd_input_active.key_count[KEY_PRESS][Key_X];
  }
  if (cmd_input_active.key_count[KEY_PRESS][Key_C] != load_key_count_c_last) {
    keyboard_load_mode_selected = LOAD_BURSTFIRE;
    load_key_count_c_last = cmd_input_active.key_count[KEY_PRESS][Key_C];
  }
  ui_loader_mode_selected = keyboard_load_mode_selected;
  // Z/X/C 仅用于选择左键发射模式，不应在切换时立即触发拨弹。
  shoot_cmd_send.load_mode = LOAD_STOP;
  switch (cmd_input_active.key_count[KEY_PRESS][Key_E] % 2) // E键开关摩擦轮
  {
  case 0:
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    break;
  default:
    shoot_cmd_send.friction_mode = FRICTION_ON;
    break;
  }
  // 键鼠模式功率档改为直达选择：
  // R = 上档(120%), F = 中档(100%), V = 下档(80%)
  // 采用“按一下锁定当前档位”的语义，避免循环切换导致误操作。
  static uint8_t power_key_count_r_last = 0U;
  static uint8_t power_key_count_f_last = 0U;
  static uint8_t power_key_count_v_last = 0U;
  if (cmd_input_active.key_count[KEY_PRESS][Key_R] != power_key_count_r_last) {
    keyboard_power_mode_selected = CHASSIS_POWER_OVERDRIVE_PERCENT;
    power_key_count_r_last = cmd_input_active.key_count[KEY_PRESS][Key_R];
  }
  if (cmd_input_active.key_count[KEY_PRESS][Key_F] != power_key_count_f_last) {
    keyboard_power_mode_selected = CHASSIS_POWER_LIMIT_PERCENT;
    power_key_count_f_last = cmd_input_active.key_count[KEY_PRESS][Key_F];
  }
  if (cmd_input_active.key_count[KEY_PRESS][Key_V] != power_key_count_v_last) {
    keyboard_power_mode_selected = CHASSIS_POWER_ECO_PERCENT;
    power_key_count_v_last = cmd_input_active.key_count[KEY_PRESS][Key_V];
  }
  chassis_cmd_send.chassis_speed_buff = keyboard_power_mode_selected;

  // Q/B/G 直达选择底盘模式，按一下锁定当前模式。
  // Q = 不跟随，B = 跟随云台，G = 小陀螺。
  static uint8_t chassis_mode_key_count_q_last = 0U;
  static uint8_t chassis_mode_key_count_b_last = 0U;
  static uint8_t chassis_mode_key_count_g_last = 0U;
  if (cmd_input_active.key_count[KEY_PRESS][Key_Q] !=
      chassis_mode_key_count_q_last) {
    keyboard_chassis_mode_selected = CHASSIS_NO_FOLLOW;
    chassis_mode_key_count_q_last = cmd_input_active.key_count[KEY_PRESS][Key_Q];
  }
  if (cmd_input_active.key_count[KEY_PRESS][Key_B] !=
      chassis_mode_key_count_b_last) {
    keyboard_chassis_mode_selected = CHASSIS_FOLLOW_GIMBAL_YAW;
    chassis_mode_key_count_b_last = cmd_input_active.key_count[KEY_PRESS][Key_B];
  }
  if (cmd_input_active.key_count[KEY_PRESS][Key_G] !=
      chassis_mode_key_count_g_last) {
    keyboard_chassis_mode_selected = CHASSIS_ROTATE;
    chassis_mode_key_count_g_last = cmd_input_active.key_count[KEY_PRESS][Key_G];
  }

  // 键鼠模式下控制逻辑与遥控器拨杆解耦，底盘模式由 Q/B/G 三态锁存决定。
  chassis_cmd_send.chassis_mode = keyboard_chassis_mode_selected;
  gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
  vision_cmd_send.vision_mode = VISION_MODE_OFF;
  vision_cmd_send.allow_auto_fire = 0;

  // 鼠标右键长按自瞄：按下接管，松开释放
  // 右键优先级高于左键，进入自瞄后由视觉决定拨弹时机
  const uint8_t mouse_left_pressed = cmd_input_active.mouse.press_l ? 1U : 0U;
  static uint8_t mouse_left_pressed_last = 0U;
  const uint8_t mouse_left_rising =
      (mouse_left_pressed && !mouse_left_pressed_last) ? 1U : 0U;
  const uint32_t now_ms = (uint32_t)DWT_GetTimeline_ms();
  mouse_left_pressed_last = mouse_left_pressed;

  if (cmd_input_active.mouse.press_r) {
    mouse_pulse_load_mode = LOAD_STOP;
    vision_cmd_send.vision_mode = VISION_MODE_AUTO_AIM;
    vision_cmd_send.allow_auto_fire = 1; // 自瞄时由视觉控制是否发射
    gimbal_cmd_send.gimbal_mode = GIMBAL_AUTOAIM_MODE;
    shoot_cmd_send.load_mode = LOAD_STOP; // 先停止拨弹，等待视觉should_fire接管
  }
  // 鼠标左键按当前已选模式发射（需摩擦轮开启）
  // 单发/三发采用“按下触发一次”，连发采用“按住持续发射”。
  else if (mouse_left_pressed && shoot_cmd_send.friction_mode == FRICTION_ON) {
    if (keyboard_load_mode_selected == LOAD_BURSTFIRE) {
      mouse_pulse_load_mode = LOAD_STOP;
      shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    } else if (mouse_left_rising) {
      mouse_pulse_load_mode = keyboard_load_mode_selected;
      mouse_pulse_start_ms = now_ms;
      shoot_cmd_send.load_mode = mouse_pulse_load_mode;
    } else if (mouse_pulse_load_mode != LOAD_STOP &&
               (now_ms - mouse_pulse_start_ms) < CMD_MOUSE_FIRE_PULSE_HOLD_MS) {
      shoot_cmd_send.load_mode = mouse_pulse_load_mode;
    } else {
      mouse_pulse_load_mode = LOAD_STOP;
    }
  } else {
    mouse_pulse_load_mode = LOAD_STOP;
  }

  switch (cmd_input_active.key[KEY_PRESS]
              .shift) // 待添加 按shift允许超功率 消耗缓冲能量
  {
  case 1:

    break;

  default:

    break;
  }
}

static void VTMControlSet() {
  const uint8_t mode_switch = cmd_input_active.rc.switch_left;

  /* VT03/VT13 只有一个模式开关，这里按单拨杆语义做控制分流:
   * C(下): 摇杆手动，底盘不跟随云台，摩擦轮关闭
   * N(中): 摇杆手动，底盘跟随云台，摩擦轮开启
   * S(上): 键鼠控制，进入图传键鼠/自瞄路径 */
  if (switch_is_up(mode_switch)) {
    MouseKeySet();
    return;
  }

  chassis_cmd_send.chassis_speed_buff = CHASSIS_POWER_LIMIT_PERCENT;

  if (switch_is_down(mode_switch)) {
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
  } else {
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    shoot_cmd_send.friction_mode = FRICTION_ON;
  }

  static float vx_filtered = 0.0f;
  static float vy_filtered = 0.0f;
  static float yaw_increment_filtered = 0.0f;
  static float pitch_increment_filtered = 0.0f;

  float vy_norm = (float)cmd_input_active.rc.rocker_l1 / 660.0f;
  float vx_norm = (float)cmd_input_active.rc.rocker_l_ / 660.0f;
  float yaw_norm = (float)cmd_input_active.rc.rocker_r_ / 660.0f;
  float pitch_norm = (float)cmd_input_active.rc.rocker_r1 / 660.0f;

  vx_filtered = LowPassFilter_Float(vx_norm, 0.90f, &vx_filtered);
  vy_filtered = LowPassFilter_Float(vy_norm, 0.90f, &vy_filtered);
  chassis_cmd_send.vx = vx_filtered;
  chassis_cmd_send.vy = vy_filtered;

  float yaw_increment = yaw_norm / 180.0f * M_PI;
  float pitch_increment = pitch_norm / 180.0f * M_PI;

  yaw_increment =
      LowPassFilter_Float(yaw_increment, 0.95f, &yaw_increment_filtered);
  pitch_increment =
      LowPassFilter_Float(pitch_increment, 0.93f, &pitch_increment_filtered);

  gimbal_cmd_send.yaw += yaw_increment;
  gimbal_cmd_send.pitch += pitch_increment;
  LIMIT_PITCH_RAD(gimbal_cmd_send.pitch);

  shoot_cmd_send.load_mode = LOAD_STOP;
  vision_cmd_send.vision_mode = VISION_MODE_OFF;
  vision_cmd_send.allow_auto_fire = 0;
  ui_loader_mode_selected = LOAD_STOP;
}

static void VisionControlSet() {
  static uint8_t vision_autoaim_last = 0U;
  const uint8_t autoaim_mode =
      (vision_cmd_send.vision_mode == VISION_MODE_AUTO_AIM) ? 1U : 0U;
  const uint8_t vision_takeover =
      (autoaim_mode && vision_data_recv.vision_takeover) ? 1U : 0U;

  // 默认清除视觉直接控制标志
  gimbal_cmd_send.vision_yaw_direct = 0;
  gimbal_cmd_send.vision_pitch_direct = 0;

  // 退出自瞄或视觉未接管时，立即清除视觉触发的拨弹命令
  if (!autoaim_mode) {
    if (vision_autoaim_last && !cmd_input_active.mouse.press_l) {
      shoot_cmd_send.load_mode = LOAD_STOP;
    }
    vision_autoaim_last = 0U;
    return;
  }

  if (!vision_takeover) {
    shoot_cmd_send.load_mode = LOAD_STOP;
    vision_autoaim_last = 1U;
    return;
  }

  // 传递视觉控制器计算好的控制量给云台
  gimbal_cmd_send.vision_yaw_direct = 1;
  gimbal_cmd_send.vision_yaw_current = vision_data_recv.yaw_current_cmd;
  gimbal_cmd_send.vision_pitch_direct = 1;
  gimbal_cmd_send.vision_pitch_ref = vision_data_recv.pitch_ref_limited;

  // 注意：不在 cmd 层覆盖云台角度目标，避免在“视觉丢目标/无目标”时
  //      云台目标角从视觉单圈角跳回导致突跳。
  //      云台自瞄接管在 gimbal 层实现（Yaw 视觉双环，Pitch 参考限速）。

  // 视觉驱动发射（仅在开启自动开火时生效）
  if (cmd_auto_fire_shot_pulse) {
    shoot_cmd_send.load_mode = LOAD_1_BULLET;
  } else {
    shoot_cmd_send.load_mode = LOAD_STOP;
  }

  vision_autoaim_last = 1U;
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo
 * 后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler(void) {
  UpdateRobotSafetyState();

  if (robot_state != ROBOT_READY) {
    ApplyRobotStopOutputs();
    return;
  }

  shoot_cmd_send.shoot_mode = SHOOT_ON;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
  RemoteControlProcess();
  VTMInputProcess();

  // BMI088Acquire(bmi088_test,&bmi088_data) ;
  // 从其他应用获取回传数据
#ifdef ONE_BOARD
  SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
  UpdateChassisFetchDataFromCan();
#endif // GIMBAL_BOARD
  SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
  SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
  SubGetMessage(vision_data_sub, &vision_data_recv); // 获取视觉处理数据
  SelectActiveControlInput();
  UpdateUIRefreshRequest();
  vision_cmd_send.bullet_speed_limit =
      chassis_fetch_data.bullet_speed_limit;

  // 最高优先级：先同步安全状态，再决定是否继续执行控制逻辑
  EmergencyHandler();
  if (robot_state != ROBOT_READY) {

#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    SendChassisCommandCanIfDue();
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    PubPushMessage(vision_cmd_pub, (void *)&vision_cmd_send);
    return;
  }

  // 计算云台-底盘角度关系（offset_angle 和 near_center_error）
  CalcGimbalChassisAngle();

  if (cmd_input_source == CMD_INPUT_SOURCE_VTM) {
    VTMControlSet();
  } else if (switch_is_up(cmd_input_active.rc.switch_left)) {
    MouseKeySet();
  } else {
    RemoteControlSet();
  }
  // 执行裁判/链路门控与降级策略
  UpdateShootRateFeedforward100ms();
  UpdateAutoFireGate();

  // 视觉数据融合（当视觉有效时覆盖云台控制；可选自动开火）
  VisionControlSet();
  PublishImageTelemetrySummary();

  // 推送消息,双板通信,视觉通信等
  // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
  PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
  SendChassisCommandCanIfDue();
#endif // GIMBAL_BOARD
  PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
  PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
  PubPushMessage(vision_cmd_pub, (void *)&vision_cmd_send); // 发布视觉控制指令
}
