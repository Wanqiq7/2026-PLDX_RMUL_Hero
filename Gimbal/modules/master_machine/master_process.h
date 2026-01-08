#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usb.h"
#include <stdint.h>

typedef struct
{
  uint16_t reload_count;  // 离线检测超时计数 (默认10 = 100ms @ 10ms daemon周期)
} Vision_Init_Config_s;

typedef enum
{
  NO_FIRE = 0,
  AUTO_FIRE = 1,
  AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
  NO_TARGET = 0,
  TARGET_CONVERGING = 1,
  READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
  NO_TARGET_NUM = 0,
  HERO1 = 1,
  ENGINEER2 = 2,
  INFANTRY3 = 3,
  INFANTRY4 = 4,
  INFANTRY5 = 5,
  OUTPOST = 6,
  SENTRY = 7,
  BASE = 8
} Target_Type_e;

typedef struct
{
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

typedef struct __attribute__((packed))
{
  uint8_t head[2];
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
  uint16_t crc16;
} Gimbal_To_Vision_t;

typedef struct __attribute__((packed))
{
  uint8_t head[2];
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
} Vision_To_Gimbal_t;

_Static_assert(sizeof(Gimbal_To_Vision_t) == 43, "Gimbal_To_Vision_t size mismatch");
_Static_assert(sizeof(Vision_To_Gimbal_t) == 29, "Vision_To_Gimbal_t size mismatch");

/* 保留枚举定义（robot_def.h中使用） */
typedef enum
{
  COLOR_NONE = 0,
  COLOR_BLUE = 1,
  COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
  BULLET_SPEED_NONE = 0,
  BIG_AMU_10 = 10,
  SMALL_AMU_15 = 15,
  BIG_AMU_16 = 16,
  SMALL_AMU_18 = 18,
  SMALL_AMU_30 = 30,
} Bullet_Speed_e;

Vision_Recv_s *VisionInit(Vision_Init_Config_s *config);
uint8_t VisionIsOnline(void);
void VisionUpdateTxAux(uint8_t mode, float bullet_speed, uint16_t bullet_count);
void VisionSendIMUPacket(
  const float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel);

#endif  // !MASTER_PROCESS_H
