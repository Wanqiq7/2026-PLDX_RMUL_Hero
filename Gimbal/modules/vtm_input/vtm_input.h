#ifndef VTM_INPUT_H
#define VTM_INPUT_H

#include <stdint.h>

#include "main.h"
#include "remote_control.h"
#include "usart.h"

#define VTM_INPUT_FRAME_SIZE 21u
#define VTM_INPUT_HEADER_0 0xA9u
#define VTM_INPUT_HEADER_1 0x53u

typedef struct {
  int16_t rocker_l_;
  int16_t rocker_l1;
  int16_t rocker_r_;
  int16_t rocker_r1;
  int16_t dial;
  uint8_t switch_left;
  uint8_t switch_right;
  uint8_t pause_pressed;
  uint8_t custom_button_left;
  uint8_t custom_button_right;
  uint8_t trigger_pressed;
} VTM_Input_RC_s;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
  uint8_t press_l;
  uint8_t press_r;
  uint8_t press_m;
} VTM_Input_Mouse_s;

typedef struct {
  VTM_Input_RC_s rc;
  VTM_Input_Mouse_s mouse;
  Key_t key[3];
  uint8_t key_count[3][16];
  uint16_t last_rx_size;
  uint16_t keyboard_mask_raw;
  uint16_t keyboard_mask_internal;
  uint16_t last_crc_expect;
  uint16_t last_crc_recv_le;
  uint16_t last_crc_recv_be;
  uint32_t frame_seq;
  uint32_t short_frame_count;
  uint32_t crc_error_count;
  uint32_t header_error_count;
  uint8_t last_bad_frame[VTM_INPUT_FRAME_SIZE];
} VTM_Input_Data_s;

/**
 * @brief 初始化官方图传输入模块
 *
 * @param usart_handle 连接图传发送端 UART 的串口句柄
 * @return VTM_Input_Data_s* 最新输入快照指针
 */
VTM_Input_Data_s *VTMInputInit(UART_HandleTypeDef *usart_handle);

/**
 * @brief 获取官方图传输入模块在线状态
 *
 * @return uint8_t 1:在线 0:离线
 */
uint8_t VTMInputIsOnline(void);

/**
 * @brief 获取最新官方图传输入快照
 *
 * @return const VTM_Input_Data_s* 输入快照指针
 */
const VTM_Input_Data_s *VTMInputGetData(void);

/**
 * @brief 推进官方图传输入串口显式读取流程
 *
 * @note 需要在任务上下文中周期调用,避免在 UART ISR 中直接做帧校验与键位解析
 */
void VTMInputProcess(void);

#endif
