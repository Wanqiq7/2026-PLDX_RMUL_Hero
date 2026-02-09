# vision_comm

视觉通信模块（Module层）

<p align='right'>Updated: 2026-01-15</p>

## 总览

本模块负责与视觉上位机的通信协议封装，支持两种链路：
- **USB VCP**：虚拟串口，适用于调试和高带宽场景
- **CAN**：适用于与其他设备共总线的场景

协议对齐 `sp_vision_25-main` 项目。

## 链路选择

在 `robot_def.h` 中通过宏定义选择：

```c
#define VISION_LINK_VCP 0
#define VISION_LINK_CAN 1
#define VISION_LINK_TYPE VISION_LINK_VCP  // 或 VISION_LINK_CAN
```

## 代码结构

```
modules/vision_comm/
├── vision_comm.c    # 通信协议实现
└── vision_comm.h    # 接口定义和数据结构
```

## 类型定义

### 接收数据结构

```c
typedef enum { NO_FIRE = 0, AUTO_FIRE = 1, AUTO_AIM = 2 } Fire_Mode_e;

typedef enum {
  NO_TARGET = 0,
  TARGET_CONVERGING = 1,
  READY_TO_FIRE = 2
} Target_State_e;

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
```

### 发送数据结构（USB VCP）

```c
typedef struct __attribute__((packed)) {
  uint8_t head[2];      // 'S', 'P'
  uint8_t mode;         // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];           // 四元数 wxyz
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
  uint16_t crc16;
} Gimbal_To_Vision_t;
```

## 外部接口

```c
// 初始化视觉通信模块
Vision_Recv_s *VisionInit(Vision_Init_Config_s *config);

// 检查视觉是否在线
uint8_t VisionIsOnline(void);

// 更新发送辅助数据（模式、弹速、弹丸计数）
void VisionUpdateTxAux(uint8_t mode, float bullet_speed, uint16_t bullet_count);

// 发送IMU数据包（由INS_Task 1kHz调用）
void VisionSendIMUPacket(const float q[4], float yaw, float yaw_vel,
                         float pitch, float pitch_vel);
```

## CAN 链路参数

```c
#define VISION_CAN_BUS 1                    // CAN1 或 CAN2
#define VISION_CANID_QUATERNION 0x100       // 四元数帧ID
#define VISION_CANID_BULLET_SPEED 0x101     // 弹速/模式帧ID
#define VISION_CANID_COMMAND 0x0FF          // 视觉指令帧ID（接收）
#define VISION_CAN_BULLET_FRAME_DIV 10      // 弹速帧分频（1kHz -> 100Hz）
```

## 使用示例

```c
// 初始化
Vision_Init_Config_s config = { .reload_count = 10 };  // 100ms超时
Vision_Recv_s *vision_data = VisionInit(&config);

// 在任务中检查状态
if (VisionIsOnline()) {
    float yaw = vision_data->yaw;
    float pitch = vision_data->pitch;
    // 处理视觉数据...
}

// 更新发送参数（由vision应用调用）
VisionUpdateTxAux(1, 30.0f, bullet_count);  // 自瞄模式，30m/s弹速
```

## 架构说明

本模块属于 **Module层**，仅负责通信协议封装，不包含业务逻辑。

视觉控制算法（Yaw双环、Pitch限速等）位于 **Application层** 的 `application/vision/vision.c` 中。

