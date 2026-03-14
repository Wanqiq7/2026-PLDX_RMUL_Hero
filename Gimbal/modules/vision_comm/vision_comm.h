/**
 * @file vision_comm.h
 * @brief 视觉通信模块接口（USB VCP / CAN）
 * @note Module层：负责与视觉上位机的通信协议封装
 *       - 支持 USB VCP 和 CAN 两种链路（通过 VISION_LINK_TYPE 宏选择）
 *       - 协议对齐 sp_vision_25-main
 */

#ifndef VISION_COMM_H
#define VISION_COMM_H

#include "bsp_usb.h"
#include <stdint.h>

/* -------------------------链路类型定义------------------------- */
typedef enum {
  VISION_LINK_VCP = 0, // USB 虚拟串口
  VISION_LINK_CAN = 1, // CAN 总线
} Vision_Link_Type_e;

/* -------------------------初始化配置------------------------- */
typedef struct {
  Vision_Link_Type_e link_type; // 通信链路类型
  uint8_t can_bus;              // CAN 总线编号 (1 或 2，仅 CAN 模式有效)
  uint16_t reload_count;        // 离线检测超时计数 (默认10 = 100ms @ 10ms daemon周期)
} Vision_Init_Config_s;

/**
 * @brief 开火模式枚举
 */
typedef enum {
    NO_FIRE   = 0,  // 不开火
    AUTO_FIRE = 1,  // 自动开火（视觉已锁定）
    AUTO_AIM  = 2,  // 仅瞄准（视觉跟踪中）
} Fire_Mode_e;

/**
 * @brief 目标状态枚举
 */
typedef enum {
    NO_TARGET         = 0,  // 无目标
    TARGET_CONVERGING = 1,  // 目标收敛中
    READY_TO_FIRE     = 2,  // 可开火
} Target_State_e;

/**
 * @brief 目标类型枚举
 */
typedef enum {
    NO_TARGET_NUM = 0,  // 无目标
    HERO1         = 1,  // 英雄
    ENGINEER2     = 2,  // 工程
    INFANTRY3     = 3,  // 步兵3
    INFANTRY4     = 4,  // 步兵4
    INFANTRY5     = 5,  // 步兵5
    OUTPOST       = 6,  // 前哨站
    SENTRY        = 7,  // 哨兵
    BASE          = 8,  // 基地
} Target_Type_e;

typedef struct {
  Fire_Mode_e fire_mode;
  Target_State_e target_state;
  Target_Type_e target_type;

  float pitch;
  float yaw;
  float pitch_vel;
  float pitch_acc;
  float yaw_vel;
  float yaw_acc;
} Vision_Recv_s;

typedef struct __attribute__((packed)) {
  uint8_t head[2];
  uint8_t mode; // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];   // wxyz
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
  uint16_t crc16;
} Gimbal_To_Vision_t;

typedef struct __attribute__((packed)) {
  uint8_t head[2];
  uint8_t mode; // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
} Vision_To_Gimbal_t;

_Static_assert(sizeof(Gimbal_To_Vision_t) == 43,
               "Gimbal_To_Vision_t size mismatch");
_Static_assert(sizeof(Vision_To_Gimbal_t) == 29,
               "Vision_To_Gimbal_t size mismatch");

/* 保留枚举定义（robot_def.h中使用） */
typedef enum {
  COLOR_NONE = 0,
  COLOR_BLUE = 1,
  COLOR_RED = 2,
} Enemy_Color_e;

typedef enum {
  BULLET_SPEED_NONE = 0,
  SMALL_AMU_12 = 12,
  BIG_AMU_10 = 10,
  SMALL_AMU_15 = 17,
  BIG_AMU_16 = 16,
  SMALL_AMU_18 = 18,
  SMALL_AMU_30 = 30,
} Bullet_Speed_e;

Vision_Recv_s *VisionInit(Vision_Init_Config_s *config);
uint8_t VisionIsOnline(void);
void VisionUpdateTxAux(uint8_t mode, float bullet_speed, uint16_t bullet_count);
void VisionSendIMUPacket(const float q[4], float yaw, float yaw_vel,
                         float pitch, float pitch_vel);

#endif // !VISION_COMM_H
