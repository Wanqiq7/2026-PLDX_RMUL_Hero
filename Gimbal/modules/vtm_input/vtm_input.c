#include "vtm_input.h"

#include <stdlib.h>
#include <string.h>

#include "bsp_log.h"
#include "bsp_usart.h"
#include "daemon.h"

#define VTM_INPUT_TEMP 0u
#define VTM_INPUT_LAST 1u

static VTM_Input_Data_s vtm_input_data[2];
static USARTInstance *vtm_input_usart_instance;
static DaemonInstance *vtm_input_daemon_instance;
static uint8_t vtm_input_init_flag = 0u;

typedef struct
{
  uint8_t read_pending;
  uint8_t frame_buffer[VTM_INPUT_FRAME_SIZE];
  volatile USART_Polling_Status_e polling_status;
  USART_Operation_s polling_operation;
} VTM_Input_Rx_Context_s;

static VTM_Input_Rx_Context_s vtm_input_rx_context;

static const uint16_t vtm_crc16_tab[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/**
 * @brief 计算图传官方控制帧使用的 CRC16
 *
 * @param data 输入数据
 * @param length 数据长度
 * @return uint16_t CRC16 结果
 */
static uint16_t VTMInputCRC16CCITTFalse(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFFu;

  if (data == NULL) {
    return crc;
  }

  for (uint16_t i = 0; i < length; ++i) {
    crc = (uint16_t)((crc >> 8) ^
                     vtm_crc16_tab[(crc ^ (uint16_t)data[i]) & 0x00FFu]);
  }

  return crc;
}

/**
 * @brief 将图传三挡模式开关映射为工程内部遥控开关语义
 *
 * @param raw_switch 图传原始模式开关值，C=0/N=1/S=2
 * @return uint8_t 工程内部 RC_SW_* 值
 */
static uint8_t VTMInputMapModeSwitch(uint8_t raw_switch) {
  switch (raw_switch) {
  case 0u:
    return RC_SW_DOWN;
  case 1u:
    return RC_SW_MID;
  case 2u:
    return RC_SW_UP;
  default:
    return RC_SW_MID;
  }
}

/**
 * @brief 将图传 11bit 模拟量转换为工程内部[-660,660]通道值
 *
 * @param raw_value 图传原始 11bit 数值
 * @return int16_t 归一化后的通道值
 */
static int16_t VTMInputNormalizeChannel(uint16_t raw_value) {
  int16_t channel_value = (int16_t)raw_value - (int16_t)RC_CH_VALUE_OFFSET;

  if (channel_value > 660 || channel_value < -660) {
    return 0;
  }

  return channel_value;
}

/**
 * @brief 将图传官方键位顺序重映射到工程内部 Key_t 顺序
 *
 * 图传顺序: W S A D Shift Ctrl Q E R F G Z X C V B
 * 工程顺序: W S A D Q E Shift Ctrl R F G Z X C V B
 *
 * @param raw_mask 图传原始键盘掩码
 * @return uint16_t 工程内部键盘掩码
 */
static uint16_t VTMInputRemapKeyboardMask(uint16_t raw_mask) {
  uint16_t internal_mask = 0u;

  internal_mask |= raw_mask & 0x000Fu;
  internal_mask |= ((raw_mask >> 6) & 0x1u) << Key_Q;
  internal_mask |= ((raw_mask >> 7) & 0x1u) << Key_E;
  internal_mask |= ((raw_mask >> 4) & 0x1u) << Key_Shift;
  internal_mask |= ((raw_mask >> 5) & 0x1u) << Key_Ctrl;
  internal_mask |= raw_mask & 0xFF00u;

  return internal_mask;
}

/**
 * @brief 将 16bit 键盘掩码同步到 Key_t 结构体
 *
 * @param key_state 键盘状态结构体
 * @param key_mask 工程内部键盘位图
 */
static void VTMInputDecodeKeyMask(Key_t *key_state, uint16_t key_mask) {
  memset(key_state, 0, sizeof(Key_t));
  key_state->w = (key_mask >> Key_W) & 0x1u;
  key_state->s = (key_mask >> Key_S) & 0x1u;
  key_state->a = (key_mask >> Key_A) & 0x1u;
  key_state->d = (key_mask >> Key_D) & 0x1u;
  key_state->q = (key_mask >> Key_Q) & 0x1u;
  key_state->e = (key_mask >> Key_E) & 0x1u;
  key_state->shift = (key_mask >> Key_Shift) & 0x1u;
  key_state->ctrl = (key_mask >> Key_Ctrl) & 0x1u;
  key_state->r = (key_mask >> Key_R) & 0x1u;
  key_state->f = (key_mask >> Key_F) & 0x1u;
  key_state->g = (key_mask >> Key_G) & 0x1u;
  key_state->z = (key_mask >> Key_Z) & 0x1u;
  key_state->x = (key_mask >> Key_X) & 0x1u;
  key_state->c = (key_mask >> Key_C) & 0x1u;
  key_state->v = (key_mask >> Key_V) & 0x1u;
  key_state->b = (key_mask >> Key_B) & 0x1u;
}

/**
 * @brief 将 Key_t 结构体转换回 16bit 掩码
 *
 * @param key_state 键盘状态结构体
 * @return uint16_t 键盘位图
 */
static uint16_t VTMInputEncodeKeyMask(const Key_t *key_state) {
  uint16_t key_mask = 0u;

  key_mask |= (uint16_t)(key_state->w & 0x1u) << Key_W;
  key_mask |= (uint16_t)(key_state->s & 0x1u) << Key_S;
  key_mask |= (uint16_t)(key_state->a & 0x1u) << Key_A;
  key_mask |= (uint16_t)(key_state->d & 0x1u) << Key_D;
  key_mask |= (uint16_t)(key_state->q & 0x1u) << Key_Q;
  key_mask |= (uint16_t)(key_state->e & 0x1u) << Key_E;
  key_mask |= (uint16_t)(key_state->shift & 0x1u) << Key_Shift;
  key_mask |= (uint16_t)(key_state->ctrl & 0x1u) << Key_Ctrl;
  key_mask |= (uint16_t)(key_state->r & 0x1u) << Key_R;
  key_mask |= (uint16_t)(key_state->f & 0x1u) << Key_F;
  key_mask |= (uint16_t)(key_state->g & 0x1u) << Key_G;
  key_mask |= (uint16_t)(key_state->z & 0x1u) << Key_Z;
  key_mask |= (uint16_t)(key_state->x & 0x1u) << Key_X;
  key_mask |= (uint16_t)(key_state->c & 0x1u) << Key_C;
  key_mask |= (uint16_t)(key_state->v & 0x1u) << Key_V;
  key_mask |= (uint16_t)(key_state->b & 0x1u) << Key_B;

  return key_mask;
}

/**
 * @brief 解析一帧官方图传 21 字节控制帧
 *
 * @param frame 原始接收缓冲区
 */
static void VTMInputParseFrame(const uint8_t *frame) {
  VTM_Input_Data_s *curr = &vtm_input_data[VTM_INPUT_TEMP];
  const VTM_Input_Data_s *last = &vtm_input_data[VTM_INPUT_LAST];
  const uint8_t *payload = frame + 2;
  const uint16_t crc_expect = VTMInputCRC16CCITTFalse(frame, 19u);
  const uint16_t crc_recv_le = (uint16_t)frame[19] | ((uint16_t)frame[20] << 8);
  const uint16_t crc_recv_be = (uint16_t)frame[20] | ((uint16_t)frame[19] << 8);
  const uint16_t key_now =
      VTMInputRemapKeyboardMask((uint16_t)payload[15] | ((uint16_t)payload[16] << 8));
  const uint16_t key_last = VTMInputEncodeKeyMask(&last->key[KEY_PRESS]);
  const uint16_t key_last_with_ctrl =
      VTMInputEncodeKeyMask(&last->key[KEY_PRESS_WITH_CTRL]);
  const uint16_t key_last_with_shift =
      VTMInputEncodeKeyMask(&last->key[KEY_PRESS_WITH_SHIFT]);
  uint16_t key_with_ctrl = 0u;
  uint16_t key_with_shift = 0u;

  if (frame == NULL) {
    return;
  }

  curr->last_crc_expect = crc_expect;
  curr->last_crc_recv_le = crc_recv_le;
  curr->last_crc_recv_be = crc_recv_be;

  if (frame[0] != VTM_INPUT_HEADER_0 || frame[1] != VTM_INPUT_HEADER_1) {
    curr->header_error_count++;
    memcpy(curr->last_bad_frame, frame, VTM_INPUT_FRAME_SIZE);
    return;
  }

  /* 文档没有明确说明 CRC 的字节序，这里兼容大小端两种落地方式。 */
  if (crc_expect != crc_recv_le && crc_expect != crc_recv_be) {
    curr->crc_error_count++;
    memcpy(curr->last_bad_frame, frame, VTM_INPUT_FRAME_SIZE);
    return;
  }

  curr->rc.rocker_r_ =
      VTMInputNormalizeChannel((uint16_t)(payload[0] | (payload[1] << 8)) & 0x07FFu);
  curr->rc.rocker_r1 =
      VTMInputNormalizeChannel(
          (uint16_t)(((payload[1] >> 3) | (payload[2] << 5)) & 0x07FFu));
  /* 说明书定义:
   * Channel 2 = 左摇杆竖直
   * Channel 3 = 左摇杆水平
   * 工程内部字段 rocker_l_ / rocker_l1 分别表示左水平 / 左竖直。 */
  curr->rc.rocker_l1 =
      VTMInputNormalizeChannel(
          (uint16_t)(((payload[2] >> 6) | (payload[3] << 2) |
                      (payload[4] << 10)) &
                     0x07FFu));
  curr->rc.rocker_l_ =
      VTMInputNormalizeChannel(
          (uint16_t)(((payload[4] >> 1) | (payload[5] << 7)) & 0x07FFu));
  curr->rc.switch_left = VTMInputMapModeSwitch((payload[5] >> 4) & 0x03u);
  /* VT03/VT13 只有一个模式开关，不能伪装成工程里的双拨杆。
   * 右开关占位符固定为安全态，避免上层误把它当成功能拨杆。 */
  curr->rc.switch_right = RC_SW_DOWN;
  curr->rc.pause_pressed = (payload[5] >> 6) & 0x01u;
  curr->rc.custom_button_left = (payload[5] >> 7) & 0x01u;
  curr->rc.custom_button_right = payload[6] & 0x01u;
  curr->rc.dial =
      VTMInputNormalizeChannel((uint16_t)(((payload[6] >> 1) | (payload[7] << 7)) &
                                          0x07FFu));
  curr->rc.trigger_pressed = (payload[7] >> 4) & 0x01u;

  curr->mouse.x = (int16_t)((uint16_t)payload[8] | ((uint16_t)payload[9] << 8));
  curr->mouse.y = (int16_t)((uint16_t)payload[10] | ((uint16_t)payload[11] << 8));
  curr->mouse.z = (int16_t)((uint16_t)payload[12] | ((uint16_t)payload[13] << 8));
  curr->mouse.press_l = (payload[14] & 0x03u) ? 1u : 0u;
  curr->mouse.press_r = ((payload[14] >> 2) & 0x03u) ? 1u : 0u;
  curr->mouse.press_m = ((payload[14] >> 4) & 0x03u) ? 1u : 0u;

  curr->keyboard_mask_raw =
      (uint16_t)payload[15] | ((uint16_t)payload[16] << 8);
  curr->keyboard_mask_internal = key_now;

  VTMInputDecodeKeyMask(&curr->key[KEY_PRESS], key_now);
  if (curr->key[KEY_PRESS].ctrl) {
    VTMInputDecodeKeyMask(&curr->key[KEY_PRESS_WITH_CTRL], key_now);
  } else {
    memset(&curr->key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
  }
  if (curr->key[KEY_PRESS].shift) {
    VTMInputDecodeKeyMask(&curr->key[KEY_PRESS_WITH_SHIFT], key_now);
  } else {
    memset(&curr->key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));
  }

  key_with_ctrl = curr->key[KEY_PRESS].ctrl ? key_now : 0u;
  key_with_shift = curr->key[KEY_PRESS].shift ? key_now : 0u;

  for (uint16_t i = 0u, bit_mask = 0x1u; i < 16u; ++i, bit_mask <<= 1u) {
    if (i == Key_Shift || i == Key_Ctrl) {
      continue;
    }

    if ((key_now & bit_mask) && !(key_last & bit_mask) &&
        !(key_with_ctrl & bit_mask) && !(key_with_shift & bit_mask)) {
      curr->key_count[KEY_PRESS][i]++;
    }

    if ((key_with_ctrl & bit_mask) && !(key_last_with_ctrl & bit_mask)) {
      curr->key_count[KEY_PRESS_WITH_CTRL][i]++;
    }

    if ((key_with_shift & bit_mask) && !(key_last_with_shift & bit_mask)) {
      curr->key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }
  }

  curr->frame_seq++;
  memcpy(&vtm_input_data[VTM_INPUT_LAST], curr, sizeof(VTM_Input_Data_s));
  DaemonReload(vtm_input_daemon_instance);
}

static void VTMInputResetRxContext(void) {
  memset(&vtm_input_rx_context, 0, sizeof(vtm_input_rx_context));
  vtm_input_rx_context.polling_status = USART_POLLING_READY;
}

/**
 * @brief 图传输入离线回调
 *
 * @param id 未使用
 */
static void VTMInputLostCallback(void *id) {
  (void)id;
  memset(vtm_input_data, 0, sizeof(vtm_input_data));
  VTMInputResetRxContext();

  if (vtm_input_usart_instance != NULL) {
    USARTServiceInit(vtm_input_usart_instance);
  }

  LOGWARNING("[vtm_input] official video transmission input lost");
}

VTM_Input_Data_s *VTMInputInit(UART_HandleTypeDef *usart_handle) {
  USART_Init_Config_s usart_config;
  Daemon_Init_Config_s daemon_config;

  if (vtm_input_init_flag) {
    return &vtm_input_data[VTM_INPUT_TEMP];
  }

  memset(vtm_input_data, 0, sizeof(vtm_input_data));
  memset(&usart_config, 0, sizeof(usart_config));
  memset(&daemon_config, 0, sizeof(daemon_config));

  usart_config.module_callback = NULL;
  usart_config.usart_handle = usart_handle;
  usart_config.recv_buff_size = 0U;
  usart_config.rx_fifo_size = 128U;
  usart_config.tx_fifo_size = 64U;
  usart_config.tx_queue_depth = 4U;
  vtm_input_usart_instance = USARTRegister(&usart_config);
  VTMInputResetRxContext();

  daemon_config.reload_count = 10u;
  daemon_config.callback = VTMInputLostCallback;
  daemon_config.owner_id = NULL;
  vtm_input_daemon_instance = DaemonRegister(&daemon_config);

  vtm_input_init_flag = 1u;
  return &vtm_input_data[VTM_INPUT_TEMP];
}

uint8_t VTMInputIsOnline(void) {
  if (!vtm_input_init_flag || vtm_input_daemon_instance == NULL) {
    return 0u;
  }

  return DaemonIsOnline(vtm_input_daemon_instance);
}

const VTM_Input_Data_s *VTMInputGetData(void) {
  if (!vtm_input_init_flag) {
    return NULL;
  }

  return &vtm_input_data[VTM_INPUT_TEMP];
}

void VTMInputProcess(void) {
  USART_Status_e status;

  if (!vtm_input_init_flag || vtm_input_usart_instance == NULL) {
    return;
  }

  if (vtm_input_rx_context.read_pending == 0u) {
    USARTOperationInitPolling(&vtm_input_rx_context.polling_operation,
                              &vtm_input_rx_context.polling_status);
    status = USARTRead(vtm_input_usart_instance, vtm_input_rx_context.frame_buffer,
                       VTM_INPUT_FRAME_SIZE, &vtm_input_rx_context.polling_operation,
                       0U);
    if (status == USART_STATUS_OK) {
      vtm_input_rx_context.polling_status = USART_POLLING_DONE;
      vtm_input_rx_context.read_pending = 1u;
    } else if (status == USART_STATUS_PENDING) {
      vtm_input_rx_context.read_pending = 1u;
    } else {
      vtm_input_rx_context.read_pending = 0u;
      vtm_input_rx_context.polling_status = USART_POLLING_ERROR;
    }
  }

  if (vtm_input_rx_context.read_pending != 0u &&
      vtm_input_rx_context.polling_status == USART_POLLING_DONE) {
    vtm_input_data[VTM_INPUT_TEMP].last_rx_size = VTM_INPUT_FRAME_SIZE;
    VTMInputParseFrame(vtm_input_rx_context.frame_buffer);
    vtm_input_rx_context.read_pending = 0u;
    vtm_input_rx_context.polling_status = USART_POLLING_READY;
  } else if (vtm_input_rx_context.polling_status == USART_POLLING_ERROR) {
    VTMInputResetRxContext();
  }
}
