#ifndef REFEREE_H
#define REFEREE_H

#include "rm_referee.h"
#include "robot_def.h"

typedef enum {
  REFEREE_UI_POWER_MILD = 0,
  REFEREE_UI_POWER_NOR,
  REFEREE_UI_POWER_WILD,
} referee_ui_power_mode_e;

typedef struct {
  uint8_t chassis_online;
  chassis_mode_e chassis_mode;
  uint8_t autoaim_on;
  uint8_t fri_on;
  uint8_t fire_allow;
  uint8_t cap_online;
  uint8_t stuck_active;
  uint8_t shoot_mode_display;
  referee_ui_power_mode_e power_mode;
  uint16_t heat_value;
  uint16_t fire_allowance_count;
  uint16_t cap_energy;
  uint16_t refresh_request_seq;
} Referee_UI_Generated_State_t;

/**
 * @brief 初始化裁判系统交互任务（UI 和多机通信）
 *
 */
referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle,
                           Referee_UI_Generated_State_t *ui_data);

/**
 * @brief 裁判系统后台解析任务
 *
 * @note 仅负责推进 RefereeProcess()，避免 UI 更新阻塞裁判数据接收。
 */
void RefereeBackgroundTask(void);

/**
 * @brief 在 referee task 之前调用，添加在 freertos.c 中
 *
 */
void MyUIInit(void);

/**
 * @brief 裁判系统交互任务（UI 和多机通信）
 *
 */
void UITask(void);

#endif // REFEREE_H
