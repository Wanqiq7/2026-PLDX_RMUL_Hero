#include "chassis_can_link.h"
#include "bsp_dwt.h"
#include "can_comm.h"
#include <string.h>

static CANCommInstance *chassis_can_fast_comm = NULL;
static CANCommInstance *chassis_can_state_comm = NULL;
static CANCommInstance *chassis_can_ui_comm = NULL;
static CANCommInstance *chassis_can_event_comm = NULL;

static Chassis_Ctrl_Fast_Pkt_s chassis_ctrl_fast_recv;
static Chassis_Ctrl_State_Pkt_s chassis_ctrl_state_recv;
static Chassis_Ctrl_UI_Pkt_s chassis_ctrl_ui_recv;
static Chassis_Ctrl_Event_Pkt_s chassis_ctrl_event_recv;
static Chassis_Feed_Fast_Pkt_s chassis_feed_fast_send;
static Chassis_Feed_State_Pkt_s chassis_feed_state_send;

static uint32_t chassis_next_fast_can_send_ms = 0U;
static uint32_t chassis_next_state_can_send_ms = 0U;
static const uint32_t CHASSIS_CAN_FAST_PERIOD_MS = 10U;
static const uint32_t CHASSIS_CAN_STATE_PERIOD_MS = 50U;
static const uint32_t CHASSIS_CAN_FAST_PHASE_MS = 5U;
static const uint32_t CHASSIS_CAN_STATE_PHASE_MS = 8U;

static void BuildChassisFeedbackCanPackets(
    const Chassis_Upload_Data_s *feedback_data) {
  chassis_feed_fast_send.referee_online = feedback_data->referee_online;
  chassis_feed_fast_send.rest_heat = feedback_data->rest_heat;
  chassis_feed_fast_send.barrel_heat = feedback_data->barrel_heat;
  chassis_feed_fast_send.barrel_heat_limit = feedback_data->barrel_heat_limit;
  chassis_feed_fast_send.barrel_cooling_value =
      feedback_data->barrel_cooling_value;
  chassis_feed_fast_send.bullet_speed_limit = feedback_data->bullet_speed_limit;
  chassis_feed_fast_send.real_wz = feedback_data->real_wz;
  chassis_feed_fast_send.chassis_safety_status = feedback_data->chassis_safety_status;

  chassis_feed_state_send.bullet_speed = feedback_data->bullet_speed;
  chassis_feed_state_send.chassis_power_limit =
      feedback_data->chassis_power_limit;
}

static void UpdateChassisCommandFromCan(Chassis_Ctrl_Cmd_s *cmd) {
  if (chassis_can_fast_comm != NULL && CANCommIsOnline(chassis_can_fast_comm)) {
    memcpy(&chassis_ctrl_fast_recv, CANCommGet(chassis_can_fast_comm),
           sizeof(chassis_ctrl_fast_recv));
    cmd->vx = chassis_ctrl_fast_recv.vx;
    cmd->vy = chassis_ctrl_fast_recv.vy;
    cmd->wz = chassis_ctrl_fast_recv.wz;
    cmd->offset_angle = chassis_ctrl_fast_recv.offset_angle;
    cmd->near_center_error = chassis_ctrl_fast_recv.near_center_error;
    cmd->chassis_mode = chassis_ctrl_fast_recv.chassis_mode;
  } else {
    memset(cmd, 0, sizeof(*cmd));
    return;
  }

  if (chassis_can_state_comm != NULL && CANCommIsOnline(chassis_can_state_comm)) {
    memcpy(&chassis_ctrl_state_recv, CANCommGet(chassis_can_state_comm),
           sizeof(chassis_ctrl_state_recv));
    cmd->chassis_speed_buff = chassis_ctrl_state_recv.chassis_speed_buff;
  } else {
    cmd->chassis_speed_buff = 100;
  }

  if (chassis_can_ui_comm != NULL && CANCommIsOnline(chassis_can_ui_comm)) {
    memcpy(&chassis_ctrl_ui_recv, CANCommGet(chassis_can_ui_comm),
           sizeof(chassis_ctrl_ui_recv));
    cmd->ui_friction_on = chassis_ctrl_ui_recv.ui_friction_on;
    cmd->ui_autoaim_enabled = chassis_ctrl_ui_recv.ui_autoaim_enabled;
    cmd->ui_fire_allow = chassis_ctrl_ui_recv.ui_fire_allow;
    cmd->ui_stuck_active = chassis_ctrl_ui_recv.ui_stuck_active;
    cmd->ui_loader_mode = chassis_ctrl_ui_recv.ui_loader_mode;
  } else {
    cmd->ui_friction_on = 0U;
    cmd->ui_autoaim_enabled = 0U;
    cmd->ui_fire_allow = 0U;
    cmd->ui_stuck_active = 0U;
    cmd->ui_loader_mode = (uint8_t)LOAD_STOP;
  }

  if (chassis_can_event_comm != NULL) {
    memcpy(&chassis_ctrl_event_recv, CANCommGet(chassis_can_event_comm),
           sizeof(chassis_ctrl_event_recv));
    cmd->ui_refresh_request_seq = chassis_ctrl_event_recv.ui_refresh_request_seq;
  }
}

void ChassisCanLinkInit(CAN_HandleTypeDef *can_handle, uint32_t now_ms) {
  CANComm_Init_Config_s fast_comm_conf = {
      .can_config =
          {
              .can_handle = can_handle,
              .tx_id = 0x311,
              .rx_id = 0x312,
          },
      .recv_data_len = sizeof(Chassis_Ctrl_Fast_Pkt_s),
      .send_data_len = sizeof(Chassis_Feed_Fast_Pkt_s),
      .daemon_count = 30,
  };
  CANComm_Init_Config_s state_comm_conf = {
      .can_config =
          {
              .can_handle = can_handle,
              .tx_id = 0x321,
              .rx_id = 0x322,
          },
      .recv_data_len = sizeof(Chassis_Ctrl_State_Pkt_s),
      .send_data_len = sizeof(Chassis_Feed_State_Pkt_s),
      .daemon_count = 30,
  };
  CANComm_Init_Config_s ui_comm_conf = {
      .can_config =
          {
              .can_handle = can_handle,
              .tx_id = 0x331,
              .rx_id = 0x332,
          },
      .recv_data_len = sizeof(Chassis_Ctrl_UI_Pkt_s),
      .send_data_len = 0U,
      .daemon_count = 30,
  };
  CANComm_Init_Config_s event_comm_conf = {
      .can_config =
          {
              .can_handle = can_handle,
              .tx_id = 0x341,
              .rx_id = 0x342,
          },
      .recv_data_len = sizeof(Chassis_Ctrl_Event_Pkt_s),
      .send_data_len = 0U,
      .daemon_count = 30,
  };

  chassis_can_fast_comm = CANCommInit(&fast_comm_conf);
  chassis_can_state_comm = CANCommInit(&state_comm_conf);
  chassis_can_ui_comm = CANCommInit(&ui_comm_conf);
  chassis_can_event_comm = CANCommInit(&event_comm_conf);
  chassis_next_fast_can_send_ms = now_ms + CHASSIS_CAN_FAST_PHASE_MS;
  chassis_next_state_can_send_ms = now_ms + CHASSIS_CAN_STATE_PHASE_MS;
}

uint8_t ChassisCanLinkUpdateCommand(Chassis_Ctrl_Cmd_s *cmd) {
  UpdateChassisCommandFromCan(cmd);
  return (chassis_can_fast_comm != NULL) ? CANCommIsOnline(chassis_can_fast_comm)
                                         : 0U;
}

static void SendChassisFeedbackCanIfDue(
    const Chassis_Upload_Data_s *feedback) {
  const uint32_t now_ms = (uint32_t)DWT_GetTimeline_ms();

  if (chassis_can_fast_comm == NULL || chassis_can_state_comm == NULL) {
    return;
  }

  BuildChassisFeedbackCanPackets(feedback);

  if (now_ms >= chassis_next_fast_can_send_ms) {
    CANCommSend(chassis_can_fast_comm, (void *)&chassis_feed_fast_send);
    do {
      chassis_next_fast_can_send_ms += CHASSIS_CAN_FAST_PERIOD_MS;
    } while (chassis_next_fast_can_send_ms <= now_ms);
  }

  if (now_ms >= chassis_next_state_can_send_ms) {
    CANCommSend(chassis_can_state_comm, (void *)&chassis_feed_state_send);
    do {
      chassis_next_state_can_send_ms += CHASSIS_CAN_STATE_PERIOD_MS;
    } while (chassis_next_state_can_send_ms <= now_ms);
  }
}

void ChassisCanLinkSendFeedbackIfDue(const Chassis_Upload_Data_s *feedback) {
  SendChassisFeedbackCanIfDue(feedback);
}
