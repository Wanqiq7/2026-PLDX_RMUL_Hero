/**
 * @file vision_comm.c
 * @brief 视觉通信模块实现（USB VCP / CAN）
 * @note Module层：负责与视觉上位机的通信协议封装
 */

#include "vision_comm.h"
#include "bsp_can.h"
#include "bsp_log.h"
#include "daemon.h"
#include <string.h>

/* -------------------------私有常量定义------------------------- */
#define VISION_HEAD0 'S'
#define VISION_HEAD1 'P'
#define VISION_RX_BUF_LEN 128

/**
 * @brief CAN 视觉链路参数（模块内部配置）
 * @note 对齐 sp_vision_25-main 的 `io/cboard.cpp`：
 *       - 四元数帧：VISION_CANID_QUATERNION（默认 0x100）
 *       - 弹速/模式帧：VISION_CANID_BULLET_SPEED（默认 0x101）
 *       - 指令帧：VISION_CANID_COMMAND（默认 0x0FF）
 */
#define VISION_CANID_QUATERNION 0x100
#define VISION_CANID_BULLET_SPEED 0x101
#define VISION_CANID_COMMAND 0x0FF
#define VISION_CAN_BULLET_FRAME_DIV 10  // 弹速帧分频：1kHz->100Hz

// 与 sp_vision_25-main/tools/crc.cpp 保持一致的 CRC16(Modbus) 查表与算法
// 使用 __attribute__((section(".rodata"))) 确保放入Flash只读段
static const uint16_t vision_crc16_table[256] __attribute__((section(".rodata"))) = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48,
    0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108,
    0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb,
    0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
    0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e,
    0xfae7, 0xc87c, 0xd9f5, 0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e,
    0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd,
    0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285,
    0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44,
    0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 0x6306, 0x728f, 0x4014,
    0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
    0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3,
    0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862,
    0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e,
    0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1,
    0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483,
    0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50,
    0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7,
    0x6e6e, 0x5cf5, 0x4d7c, 0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1,
    0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72,
    0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e,
    0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf,
    0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 0xf78f, 0xe606, 0xd49d,
    0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
    0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

static uint16_t VisionCRC16(const uint8_t *data, uint32_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    uint8_t byte = *data++;
    uint8_t i = (crc ^ byte) & 0x00FF;
    crc = (crc >> 8) ^ vision_crc16_table[i];
  }
  return crc;
}

/* -------------------------静态变量定义------------------------- */
static Vision_Link_Type_e vision_link_type = VISION_LINK_VCP; // 运行时链路类型
static DaemonInstance *vision_daemon = NULL;
static Vision_Recv_s vision_recv_data;

// VCP 模式变量
static Gimbal_To_Vision_t vision_tx_frame = {
    .head = {VISION_HEAD0, VISION_HEAD1},
};
static uint8_t *vision_usb_rx_ptr = NULL;
static uint8_t vision_rx_fifo[VISION_RX_BUF_LEN];
static uint16_t vision_rx_head = 0;
static uint16_t vision_rx_len = 0;

// CAN 模式变量
static CANInstance *vision_can_ins = NULL;
static uint16_t vision_can_bullet_div = 0;
static int16_t vision_can_last_bullet_speed = 0;
static uint8_t vision_can_last_mode = 0;
static uint16_t vision_can_last_bullet_count = 0;

static uint8_t vision_initialized = 0;
static uint8_t vision_mode_cached = 0;
static float vision_bullet_speed_cached = 0.0f;
static uint16_t vision_bullet_count_cached = 0;

/* -------------------------私有函数声明------------------------- */
static void VisionUSBRxCallback(uint16_t len);
static void VisionOfflineCallback(void *owner);
static void VisionTryParse(void);
static void VisionHandlePacket(const Vision_To_Gimbal_t *pkt);
static void VisionCANRxCallback(CANInstance *_instance);

Vision_Recv_s *VisionInit(Vision_Init_Config_s *config) {
  if (config == NULL) {
    LOGERROR("[vision_comm] VisionInit: Invalid config!");
    return NULL;
  }

  // 保存链路类型
  vision_link_type = config->link_type;
  memset(&vision_recv_data, 0, sizeof(Vision_Recv_s));

  Daemon_Init_Config_s daemon_config = {
      .reload_count = config->reload_count ? config->reload_count : 10,
      .callback = VisionOfflineCallback,
      .owner_id = NULL,
  };
  vision_daemon = DaemonRegister(&daemon_config);
  if (vision_daemon == NULL) {
    LOGERROR("[vision_comm] VisionInit: Daemon register failed!");
    return NULL;
  }

  if (vision_link_type == VISION_LINK_VCP) {
    // USB VCP 模式初始化
    USB_Init_Config_s usb_conf = {
        .tx_cbk = NULL,
        .rx_cbk = VisionUSBRxCallback,
    };
    vision_usb_rx_ptr = USBInit(usb_conf);
    if (vision_usb_rx_ptr == NULL) {
      LOGERROR("[vision_comm] VisionInit: USB init failed!");
      return NULL;
    }
    vision_initialized = 1;
    LOGINFO("[vision_comm] USB VCP link initialized (reload=%u).",
            daemon_config.reload_count);
  } else if (vision_link_type == VISION_LINK_CAN) {
    // CAN 模式初始化
    CAN_HandleTypeDef *can_handle = (config->can_bus == 2) ? &hcan2 : &hcan1;
    CAN_Init_Config_s can_conf = {
        .can_handle = can_handle,
        .tx_id = VISION_CANID_QUATERNION,
        .rx_id = VISION_CANID_COMMAND,
        .can_module_callback = VisionCANRxCallback,
        .id = NULL,
    };
    vision_can_ins = CANRegister(&can_conf);
    if (vision_can_ins == NULL) {
      LOGERROR("[vision_comm] VisionInit: CAN register failed!");
      return NULL;
    }
    vision_initialized = 1;
    LOGINFO("[vision_comm] CAN link initialized (bus=%u, q=0x%03X, bs=0x%03X, "
            "cmd=0x%03X, reload=%u).",
            (unsigned)config->can_bus, (unsigned)VISION_CANID_QUATERNION,
            (unsigned)VISION_CANID_BULLET_SPEED, (unsigned)VISION_CANID_COMMAND,
            (unsigned)daemon_config.reload_count);
  } else {
    LOGERROR("[vision_comm] VisionInit: Invalid link type!");
    return NULL;
  }

  return &vision_recv_data;
}

uint8_t VisionIsOnline(void) {
  if (vision_daemon == NULL) {
    return 0;
  }
  return DaemonIsOnline(vision_daemon);
}

void VisionUpdateTxAux(uint8_t mode, float bullet_speed,
                       uint16_t bullet_count) {
  vision_mode_cached = mode;
  vision_bullet_speed_cached = bullet_speed;
  vision_bullet_count_cached = bullet_count;
}

void VisionSendIMUPacket(const float q[4], float yaw, float yaw_vel,
                         float pitch, float pitch_vel) {
  if (!vision_initialized) {
    return;
  }

  if (vision_link_type == VISION_LINK_VCP) {
    // VCP 模式发送
    memcpy(vision_tx_frame.q, q, sizeof(vision_tx_frame.q));
    vision_tx_frame.mode = vision_mode_cached;
    vision_tx_frame.yaw = yaw;
    vision_tx_frame.yaw_vel = yaw_vel;
    vision_tx_frame.pitch = pitch;
    vision_tx_frame.pitch_vel = pitch_vel;
    vision_tx_frame.bullet_speed = vision_bullet_speed_cached;
    vision_tx_frame.bullet_count = vision_bullet_count_cached;
    vision_tx_frame.crc16 =
        VisionCRC16((uint8_t *)&vision_tx_frame,
                    sizeof(vision_tx_frame) - sizeof(vision_tx_frame.crc16));
    USBTransmit((uint8_t *)&vision_tx_frame, sizeof(vision_tx_frame));
  } else if (vision_link_type == VISION_LINK_CAN) {
    // CAN 模式发送
    if (vision_can_ins == NULL) {
      return;
    }

    // 发送四元数：按 sp_vision_25 的 CBoard 协议，CAN 载荷为 x y z w（int16，缩放 1e4）
    int16_t qx = (int16_t)(q[1] * 1e4f);
    int16_t qy = (int16_t)(q[2] * 1e4f);
    int16_t qz = (int16_t)(q[3] * 1e4f);
    int16_t qw = (int16_t)(q[0] * 1e4f);

    vision_can_ins->txconf.StdId = VISION_CANID_QUATERNION;
    CANSetDLC(vision_can_ins, 8);
    vision_can_ins->tx_buff[0] = (uint8_t)((qx >> 8) & 0xFF);
    vision_can_ins->tx_buff[1] = (uint8_t)(qx & 0xFF);
    vision_can_ins->tx_buff[2] = (uint8_t)((qy >> 8) & 0xFF);
    vision_can_ins->tx_buff[3] = (uint8_t)(qy & 0xFF);
    vision_can_ins->tx_buff[4] = (uint8_t)((qz >> 8) & 0xFF);
    vision_can_ins->tx_buff[5] = (uint8_t)(qz & 0xFF);
    vision_can_ins->tx_buff[6] = (uint8_t)((qw >> 8) & 0xFF);
    vision_can_ins->tx_buff[7] = (uint8_t)(qw & 0xFF);
    (void)CANTransmit(vision_can_ins, 0.0f);

    // 弹速/模式帧降频发送
    int16_t bullet_speed_raw = (int16_t)(vision_bullet_speed_cached);
    uint8_t need_send_bullet = 0;
    if ((vision_can_bullet_div++ % VISION_CAN_BULLET_FRAME_DIV) == 0) {
      need_send_bullet = 1;
    }
    if (bullet_speed_raw != vision_can_last_bullet_speed ||
        vision_mode_cached != vision_can_last_mode ||
        vision_bullet_count_cached != vision_can_last_bullet_count) {
      need_send_bullet = 1;
    }

    if (need_send_bullet) {
      vision_can_last_bullet_speed = bullet_speed_raw;
      vision_can_last_mode = vision_mode_cached;
      vision_can_last_bullet_count = vision_bullet_count_cached;

      vision_can_ins->txconf.StdId = VISION_CANID_BULLET_SPEED;
      CANSetDLC(vision_can_ins, 8);
      vision_can_ins->tx_buff[0] = (uint8_t)((bullet_speed_raw >> 8) & 0xFF);
      vision_can_ins->tx_buff[1] = (uint8_t)(bullet_speed_raw & 0xFF);
      vision_can_ins->tx_buff[2] = vision_mode_cached;
      vision_can_ins->tx_buff[3] = 0;
      vision_can_ins->tx_buff[4] = 0;
      vision_can_ins->tx_buff[5] = 0;
      vision_can_ins->tx_buff[6] = 0;
      vision_can_ins->tx_buff[7] = 0;
      (void)CANTransmit(vision_can_ins, 0.0f);
    }

    (void)yaw;
    (void)yaw_vel;
    (void)pitch;
    (void)pitch_vel;
  }
}

/* -------------------------VCP 模式回调函数------------------------- */
static void VisionUSBRxCallback(uint16_t len) {
  if (len == 0 || vision_usb_rx_ptr == NULL) {
    return;
  }

  if (len > VISION_RX_BUF_LEN) {
    return;
  }

  if (vision_rx_len + len > VISION_RX_BUF_LEN) {
    vision_rx_head = 0;
    vision_rx_len = 0;
  }

  for (uint16_t i = 0; i < len; ++i) {
    uint16_t tail = (vision_rx_head + vision_rx_len) % VISION_RX_BUF_LEN;
    vision_rx_fifo[tail] = vision_usb_rx_ptr[i];
    vision_rx_len++;
  }

  VisionTryParse();
}

static void VisionTryParse(void) {
  while (vision_rx_len >= sizeof(Vision_To_Gimbal_t)) {
    uint8_t head0 = vision_rx_fifo[vision_rx_head];
    uint8_t head1 = vision_rx_fifo[(vision_rx_head + 1) % VISION_RX_BUF_LEN];

    if (head0 != VISION_HEAD0 || head1 != VISION_HEAD1) {
      vision_rx_head = (vision_rx_head + 1) % VISION_RX_BUF_LEN;
      vision_rx_len--;
      continue;
    }

    if (vision_rx_len < sizeof(Vision_To_Gimbal_t)) {
      return;
    }

    Vision_To_Gimbal_t packet;
    uint8_t *dst = (uint8_t *)&packet;
    for (uint16_t i = 0; i < sizeof(Vision_To_Gimbal_t); ++i) {
      dst[i] = vision_rx_fifo[(vision_rx_head + i) % VISION_RX_BUF_LEN];
    }

    uint16_t crc =
        VisionCRC16((uint8_t *)&packet, sizeof(packet) - sizeof(packet.crc16));
    if (crc != packet.crc16) {
      vision_rx_head = (vision_rx_head + 1) % VISION_RX_BUF_LEN;
      vision_rx_len--;
      continue;
    }

    VisionHandlePacket(&packet);

    vision_rx_head =
        (vision_rx_head + sizeof(Vision_To_Gimbal_t)) % VISION_RX_BUF_LEN;
    vision_rx_len -= sizeof(Vision_To_Gimbal_t);
  }
}

static void VisionHandlePacket(const Vision_To_Gimbal_t *pkt) {
  vision_recv_data.yaw = pkt->yaw;
  vision_recv_data.pitch = pkt->pitch;
  vision_recv_data.yaw_vel = pkt->yaw_vel;
  vision_recv_data.yaw_acc = pkt->yaw_acc;
  vision_recv_data.pitch_vel = pkt->pitch_vel;
  vision_recv_data.pitch_acc = pkt->pitch_acc;
  vision_recv_data.target_type = NO_TARGET_NUM;

  switch (pkt->mode) {
  case 2:
    vision_recv_data.fire_mode = AUTO_FIRE;
    vision_recv_data.target_state = READY_TO_FIRE;
    break;
  case 1:
    vision_recv_data.fire_mode = AUTO_AIM;
    vision_recv_data.target_state = TARGET_CONVERGING;
    break;
  default:
    vision_recv_data.fire_mode = NO_FIRE;
    vision_recv_data.target_state = NO_TARGET;
    break;
  }

  if (vision_daemon != NULL) {
    DaemonReload(vision_daemon);
  }
}

static void VisionOfflineCallback(void *owner) {
  (void)owner;
  memset(&vision_recv_data, 0, sizeof(Vision_Recv_s));
  vision_recv_data.target_state = NO_TARGET;
  vision_recv_data.fire_mode = NO_FIRE;
  vision_recv_data.yaw_vel = 0.0f;
  vision_recv_data.yaw_acc = 0.0f;
  vision_recv_data.pitch_vel = 0.0f;
  vision_recv_data.pitch_acc = 0.0f;
  if (vision_rx_len > 0) {
    vision_rx_head = 0;
    vision_rx_len = 0;
  }
  LOGWARNING("[vision_comm] Vision module offline!");
}

/* -------------------------CAN 模式回调函数------------------------- */
static void VisionCANRxCallback(CANInstance *_instance) {
  if (_instance == NULL) {
    return;
  }
  if (_instance->rx_len != 8) {
    return;
  }

  // 对齐 sp_vision_25 的
  // io/cboard.cpp：control/shoot/yaw/pitch/horizon_distance（int16，缩放 1e4）
  uint8_t control = _instance->rx_buff[0];
  uint8_t shoot = _instance->rx_buff[1];
  int16_t yaw_raw =
      (int16_t)((_instance->rx_buff[2] << 8) | _instance->rx_buff[3]);
  int16_t pitch_raw =
      (int16_t)((_instance->rx_buff[4] << 8) | _instance->rx_buff[5]);
  (void)_instance->rx_buff[6];
  (void)_instance->rx_buff[7];

  vision_recv_data.yaw = (float)yaw_raw / 1e4f;
  vision_recv_data.pitch = (float)pitch_raw / 1e4f;
  vision_recv_data.yaw_vel = 0.0f;
  vision_recv_data.yaw_acc = 0.0f;
  vision_recv_data.pitch_vel = 0.0f;
  vision_recv_data.pitch_acc = 0.0f;
  vision_recv_data.target_type = NO_TARGET_NUM;

  if (!control) {
    vision_recv_data.fire_mode = NO_FIRE;
    vision_recv_data.target_state = NO_TARGET;
  } else if (!shoot) {
    vision_recv_data.fire_mode = AUTO_AIM;
    vision_recv_data.target_state = TARGET_CONVERGING;
  } else {
    vision_recv_data.fire_mode = AUTO_FIRE;
    vision_recv_data.target_state = READY_TO_FIRE;
  }

  if (vision_daemon != NULL) {
    DaemonReload(vision_daemon);
  }
}
