/**
 * @file vision.c
 * @brief 视觉控制应用模块实现
 * @note 遵循项目三层架构,处理视觉数据并通过消息中心发布控制建议
 */

#include "vision.h"
#include "bsp_log.h"
#include "master_process.h"
#include "message_center.h"
#include "robot_def.h"

/* 私有变量 */
static Vision_Recv_s *vision_recv_data; // 来自 master_process 的视觉原始数据

static Publisher_t *vision_pub;     // 发布处理后的视觉数据
static Subscriber_t *vision_sub;    // 订阅来自 cmd 的视觉控制指令
static Subscriber_t *shoot_cmd_sub; // 订阅发射控制指令

static Vision_Ctrl_Cmd_s vision_cmd_recv;       // 接收的控制指令
static Vision_Upload_Data_s vision_upload_data; // 发布的处理数据
static Shoot_Ctrl_Cmd_s shoot_cmd_cache; // 发射参数缓存（用于弹速同步给视觉）

static uint8_t vision_last_online_state = 0; // 上一次在线状态(用于状态变化日志)
static uint16_t vision_bullet_count = 0; // 弹丸计数（占位，后续接入拨弹反馈）

/* 私有函数声明 */
static void ProcessAutoAim(void);
static void ProcessEnergyHit(void);
static void ProcessManualAim(void);

static uint8_t VisionModeToProtocol(vision_mode_e mode) {
  // 能量机关沿用小符路径，先统一映射避免重复case
  if (mode == VISION_MODE_ENERGY_HIT) {
    mode = VISION_MODE_SMALL_BUFF;
  }
  switch (mode) {
  case VISION_MODE_AUTO_AIM:
    return 1;
  case VISION_MODE_SMALL_BUFF:
    return 2; // 小符
  case VISION_MODE_BIG_BUFF:
    return 3; // 大符
  case VISION_MODE_MANUAL_AIM:
  case VISION_MODE_OFF:
  default:
    return 0;
  }
}

static float ConvertBulletSpeed(Bullet_Speed_e speed) {
  switch (speed) {
  case BIG_AMU_10:
  case SMALL_AMU_15:
  case BIG_AMU_16:
  case SMALL_AMU_18:
  case SMALL_AMU_30:
    return (float)speed;
  case BULLET_SPEED_NONE:
  default:
    return 0.0f;
  }
}

/**
 * @brief 视觉控制应用初始化
 */
void VisionAppInit(void) {
  // 初始化视觉通信模块（虚拟串口，发送IMU+状态）
  Vision_Init_Config_s vision_config = {
      .reload_count = 10, // 100ms超时
  };
  vision_recv_data = VisionInit(&vision_config);

  // 注册消息中心发布者和订阅者
  vision_pub = PubRegister("vision_data", sizeof(Vision_Upload_Data_s));
  vision_sub = SubRegister("vision_cmd", sizeof(Vision_Ctrl_Cmd_s));
  shoot_cmd_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

  // 初始化上传数据
  vision_upload_data.vision_valid = 0;
  vision_upload_data.target_locked = 0;
  vision_upload_data.yaw = 0.0f;
  vision_upload_data.pitch = 0.0f;
  vision_upload_data.should_fire = 0;
  vision_upload_data.yaw_vel_ff = 0.0f;
  vision_upload_data.yaw_acc_ff = 0.0f;
  vision_upload_data.pitch_vel_ff = 0.0f;
  vision_upload_data.pitch_acc_ff = 0.0f;

  LOGINFO("[vision] Vision application initialized.");
}

/**
 * @brief 视觉控制任务
 */
void VisionAppTask(void) {
  // 获取来自 robot_cmd 的控制指令
  SubGetMessage(vision_sub, &vision_cmd_recv);
  SubGetMessage(shoot_cmd_sub, &shoot_cmd_cache);

  // 更新发送给视觉的模式与弹速（姿态由 INS_Task 1kHz 推送）
  VisionUpdateTxAux(VisionModeToProtocol(vision_cmd_recv.vision_mode),
                    ConvertBulletSpeed(shoot_cmd_cache.bullet_speed),
                    vision_bullet_count);

  // 检查视觉在线状态
  uint8_t vision_online = VisionIsOnline();
  if (!vision_online) {
    // 状态变化时记录日志
    if (vision_last_online_state != vision_online) {
      LOGWARNING("[vision] Vision offline, data invalid.");
      vision_last_online_state = vision_online;
    }
    // 视觉离线,清空数据
    vision_upload_data.vision_valid = 0;
    vision_upload_data.target_locked = 0;
    vision_upload_data.should_fire = 0;
    vision_upload_data.yaw_vel_ff = 0.0f;
    vision_upload_data.yaw_acc_ff = 0.0f;
    vision_upload_data.pitch_vel_ff = 0.0f;
    vision_upload_data.pitch_acc_ff = 0.0f;
    PubPushMessage(vision_pub, &vision_upload_data);
    return;
  } else {
    // 视觉重新上线
    if (vision_last_online_state != vision_online) {
      LOGINFO("[vision] Vision online, ready for processing.");
      vision_last_online_state = vision_online;
    }
  }

  // 根据视觉模式处理数据
  // 能量机关沿用小符路径，先统一映射避免重复case
  if (vision_cmd_recv.vision_mode == VISION_MODE_ENERGY_HIT) {
    vision_cmd_recv.vision_mode = VISION_MODE_SMALL_BUFF;
  }
  switch (vision_cmd_recv.vision_mode) {
  case VISION_MODE_AUTO_AIM:
    ProcessAutoAim();
    break;
  case VISION_MODE_SMALL_BUFF:
  case VISION_MODE_BIG_BUFF:
    ProcessEnergyHit();
    break;
  case VISION_MODE_MANUAL_AIM:
    ProcessManualAim();
    break;
  case VISION_MODE_OFF:
  default:
    // 视觉关闭,清空数据
    vision_upload_data.vision_valid = 0;
    vision_upload_data.target_locked = 0;
    vision_upload_data.should_fire = 0;
    break;
  }

  // 发布处理后的数据
  PubPushMessage(vision_pub, &vision_upload_data);
}

/**
 * @brief 自动瞄准模式处理
 */
static void ProcessAutoAim(void) {
  // 检查视觉是否检测到目标
  if (vision_recv_data->fire_mode == NO_FIRE) {
    vision_upload_data.vision_valid = 0;
    vision_upload_data.target_locked = 0;
    vision_upload_data.should_fire = 0;
    vision_upload_data.yaw_vel_ff = 0.0f;
    vision_upload_data.yaw_acc_ff = 0.0f;
    vision_upload_data.pitch_vel_ff = 0.0f;
    vision_upload_data.pitch_acc_ff = 0.0f;
    return;
  }

  // 视觉数据有效
  vision_upload_data.vision_valid = 1;

  // 设置目标角度(可加入手动微调偏移)
  vision_upload_data.yaw =
      vision_recv_data->yaw + vision_cmd_recv.manual_yaw_offset;
  vision_upload_data.pitch =
      vision_recv_data->pitch + vision_cmd_recv.manual_pitch_offset;
  vision_upload_data.yaw_vel_ff = vision_recv_data->yaw_vel;
  vision_upload_data.yaw_acc_ff = vision_recv_data->yaw_acc;
  vision_upload_data.pitch_vel_ff = vision_recv_data->pitch_vel;
  vision_upload_data.pitch_acc_ff = vision_recv_data->pitch_acc;

  // 根据目标状态设置锁定和射击标志
  switch (vision_recv_data->target_state) {
  case READY_TO_FIRE:
    vision_upload_data.target_locked = 1;
    vision_upload_data.should_fire = vision_cmd_recv.allow_auto_fire;
    break;
  case TARGET_CONVERGING:
    vision_upload_data.target_locked = 0;
    vision_upload_data.should_fire = 0;
    break;
  case NO_TARGET:
  default:
    vision_upload_data.target_locked = 0;
    vision_upload_data.should_fire = 0;
    break;
  }
}

/**
 * @brief 能量机关模式处理(预留扩展)
 */
static void ProcessEnergyHit(void) {
  // TODO: 实现大/小能量机关预测打击逻辑
  vision_upload_data.vision_valid = 0;
  vision_upload_data.target_locked = 0;
  vision_upload_data.should_fire = 0;
  vision_upload_data.yaw_vel_ff = 0.0f;
  vision_upload_data.yaw_acc_ff = 0.0f;
  vision_upload_data.pitch_vel_ff = 0.0f;
  vision_upload_data.pitch_acc_ff = 0.0f;
}

/**
 * @brief 手动辅助瞄准模式处理(预留扩展)
 */
static void ProcessManualAim(void) {
  // TODO: 实现视觉辅助但保留手动微调能力
  vision_upload_data.vision_valid = 0;
  vision_upload_data.target_locked = 0;
  vision_upload_data.should_fire = 0;
  vision_upload_data.yaw_vel_ff = 0.0f;
  vision_upload_data.yaw_acc_ff = 0.0f;
  vision_upload_data.pitch_vel_ff = 0.0f;
  vision_upload_data.pitch_acc_ff = 0.0f;
}
