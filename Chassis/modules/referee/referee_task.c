#include "referee_task.h"

#include "cmsis_os.h"

#include "referee_generated_ui.h"

static Referee_UI_Generated_State_t *interactive_data = NULL;
static referee_info_t *referee_recv_info = NULL;
static uint16_t last_ui_refresh_request_seq = 0U;

uint8_t UI_Seq = 0U;

static void DeterminRobotID(void) {
  if (referee_recv_info == NULL) {
    return;
  }

  referee_recv_info->referee_id.Robot_Color =
      (referee_recv_info->GameRobotState.robot_id > 7U) ? Robot_Blue
                                                        : Robot_Red;
  referee_recv_info->referee_id.Robot_ID =
      referee_recv_info->GameRobotState.robot_id;
  referee_recv_info->referee_id.Cilent_ID =
      (uint16_t)(0x0100U + referee_recv_info->referee_id.Robot_ID);
  referee_recv_info->referee_id.Receiver_Robot_ID = 0U;
}

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle,
                           Referee_UI_Generated_State_t *ui_data) {
  referee_recv_info = RefereeInit(referee_usart_handle);
  interactive_data = ui_data;

  if (referee_recv_info != NULL) {
    referee_recv_info->init_flag = 1U;
  }

  return referee_recv_info;
}

void MyUIInit(void) {
  if (referee_recv_info == NULL || interactive_data == NULL ||
      !referee_recv_info->init_flag) {
    vTaskDelete(NULL);
    return;
  }

  while (referee_recv_info->GameRobotState.robot_id == 0U) {
    RefereeProcess();
    osDelay(10U);
  }

  DeterminRobotID();
  RefereeGeneratedUIInit(&referee_recv_info->referee_id);
  last_ui_refresh_request_seq = interactive_data->refresh_request_seq;
}

void UITask(void) {
  if (referee_recv_info == NULL || interactive_data == NULL ||
      !referee_recv_info->init_flag) {
    return;
  }

  RefereeProcess();

  if (interactive_data->refresh_request_seq != last_ui_refresh_request_seq) {
    last_ui_refresh_request_seq = interactive_data->refresh_request_seq;
    DeterminRobotID();
    RefereeGeneratedUIInit(&referee_recv_info->referee_id);
  }

  RefereeGeneratedUIUpdate(interactive_data);
}
