#ifndef REFEREE_H
#define REFEREE_H

#include "rm_referee.h"

/**
 * @brief 初始化裁判系统交互任务(UI和多机通信)
 *
 */
referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data);

/**
 * @brief 在referee task之前调用,添加在freertos.c中
 * 
 */
void MyUIInit();

/**
 * @brief 裁判系统交互任务(UI和多机通信)
 *
 */
void UITask();

/**
 * @brief 查询裁判UI任务是否完成初始化
 * @return 1: 已就绪, 0: 未就绪
 */
uint8_t RefereeTaskIsReady(void);

#endif // REFEREE_H
