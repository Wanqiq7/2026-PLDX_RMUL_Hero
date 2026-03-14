/**
 * @file vision.c
 * @brief 视觉控制应用模块实现
 * @note 遵循项目三层架构,处理视觉数据并通过消息中心发布控制建议
 *
 * 架构说明：
 * - 本模块集成视觉控制算法（Yaw双环、Pitch限速），不再拆分为独立文件
 * - Yaw：位置环(角度误差→角速度参考) + 速度环(角速度误差→电流指令)
 * - Pitch：限速/限幅参考生成，避免目标突跳
 */

#include "vision.h"
#include "bsp_log.h"
#include "ins_task.h"
#include "message_center.h"
#include "robot_def.h"
#include "vision_comm.h"
#include "vision_control.h"

/* ==================== 常量定义 ==================== */
static volatile VisionCtrlParams_s vision_params = {
    .yaw_pos_kp = 10.5f, // 默认保守值，需实物调参
    .yaw_pos_ki = 0.0f,
    .yaw_rate_kp = 20000.0f,
    .yaw_rate_ki = 2000.0f,
    .yaw_rate_max = 6.0f,
    .yaw_current_max = 16384.0f,
    .pitch_rate_max = 4.0f,
    .pitch_max_angle = 38.0f,  // Pitch轴最大角度限位 [deg]
    .pitch_min_angle = -18.5f, // Pitch轴最小角度限位 [deg]
    // 前馈参数（默认关闭，需实物调参后启用）
    .yaw_vel_ff = 0.0f,   // Yaw速度前馈系数
    .pitch_vel_ff = 0.0f, // Pitch速度前馈系数
};

/* ==================== 私有变量 ==================== */
// 视觉通信相关
static Vision_Recv_s *vision_recv_data; // 来自 vision_comm 的视觉原始数据

// 消息中心相关
static Publisher_t *vision_pub;       // 发布处理后的视觉数据
static Subscriber_t *vision_sub;      // 订阅来自 cmd 的视觉控制指令
static Subscriber_t *gimbal_feed_sub; // 订阅云台反馈（获取IMU数据）

static Vision_Ctrl_Cmd_s vision_cmd_recv;       // 接收的控制指令
static Vision_Upload_Data_s vision_upload_data; // 发布的处理数据
static Gimbal_Upload_Data_s gimbal_feed_recv;   // 云台反馈数据（含IMU）

// 状态变量
static uint8_t vision_last_online_state = 0;  // 上一次在线状态
static uint16_t vision_bullet_count = 0;      // 弹丸计数
static uint8_t vision_takeover_last = 0;      // 上一次视觉接管状态
static uint8_t vision_link_ready = 0;         // 视觉链路初始化完成标志
static uint8_t vision_init_failed_logged = 0; // 初始化失败日志保护

// 控制器状态
static VisionCtrlState_s vision_ctrl_state;

/* ==================== 私有函数声明 ==================== */
// 模式处理
static void ProcessAutoAim(void);
static void ProcessEnergyHit(void);
static void ProcessManualAim(void);
static void ClearVisionOutput(void);

// 控制算法
static void VisionControllerInit(void);
static void VisionControllerReset(float pitch_feedback_rad);

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

/**
 * @brief 视觉控制应用初始化
 */
void VisionAppInit(void) {
  // 初始化视觉通信模块
  // link_type: VISION_LINK_VCP (USB) 或 VISION_LINK_CAN (CAN总线)
  // can_bus: 1 或 2，仅 CAN 模式有效
  Vision_Init_Config_s vision_config = {
      .link_type = (Vision_Link_Type_e)VISION_LINK_TYPE, // 运行时跟随编译配置
      .can_bus = 1,                                     // CAN1（仅 CAN 模式有效）
      .reload_count = 10,                               // 100ms 超时
  };
  vision_recv_data = VisionInit(&vision_config);
  if (vision_recv_data == NULL) {
    vision_link_ready = 0;
    LOGERROR("[vision] Vision link init failed, running without vision.");
  } else {
    vision_link_ready = 1;
  }

  // 初始化视觉控制器
  VisionControllerInit();

  // 注册消息中心发布者和订阅者
  vision_pub = RegisterPublisher("vision_data", sizeof(Vision_Upload_Data_s));
  if (vision_pub == NULL) {
    LOGERROR("[vision] Failed to register vision_data publisher!");
  }

  vision_sub = RegisterSubscriber("vision_cmd", sizeof(Vision_Ctrl_Cmd_s));
  if (vision_sub == NULL) {
    LOGERROR("[vision] Failed to register vision_cmd subscriber!");
  }

  gimbal_feed_sub =
      RegisterSubscriber("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
  if (gimbal_feed_sub == NULL) {
    LOGERROR("[vision] Failed to register gimbal_feed subscriber!");
  }

  // 初始化上传数据
  ClearVisionOutput();

  LOGINFO("[vision] Vision application initialized.");
}

/**
 * @brief 视觉控制任务
 */
void VisionAppTask(void) {
  // 获取来自 robot_cmd 的控制指令和云台反馈
  SubGetMessage(vision_sub, &vision_cmd_recv);
  SubGetMessage(gimbal_feed_sub, &gimbal_feed_recv);

  if (!vision_link_ready) {
    if (!vision_init_failed_logged) {
      LOGWARNING("[vision] Vision link unavailable, outputs disabled.");
      vision_init_failed_logged = 1;
    }
    ClearVisionOutput();
    PubPushMessage(vision_pub, &vision_upload_data);
    return;
  }

  // 更新发送给视觉的模式与弹速（姿态由 INS_Task 1kHz 推送）
  VisionUpdateTxAux(VisionModeToProtocol(vision_cmd_recv.vision_mode),
                    12.0f,
                    vision_bullet_count);

  // 检查视觉在线状态
  uint8_t vision_online = VisionIsOnline();
  if (!vision_online) {
    // 状态变化时记录日志
    if (vision_last_online_state != vision_online) {
      LOGWARNING("[vision] Vision offline, data invalid.");
      vision_last_online_state = vision_online;
    }
    // 视觉离线,清空数据并重置控制器
    ClearVisionOutput();
    if (vision_takeover_last) {
      VisionControllerReset(gimbal_feed_recv.gimbal_imu_data.Pitch_rad);
      vision_takeover_last = 0;
    }
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
    ClearVisionOutput();
    if (vision_takeover_last) {
      VisionControllerReset(gimbal_feed_recv.gimbal_imu_data.Pitch_rad);
      vision_takeover_last = 0;
    }
    break;
  }

  // 发布处理后的数据
  PubPushMessage(vision_pub, &vision_upload_data);
}

/**
 * @brief 自动瞄准模式处理
 * @note 集成vision_controller进行Yaw双环和Pitch限速控制
 */
static void ProcessAutoAim(void) {
  // 检查视觉是否检测到目标
  if (vision_recv_data->fire_mode == NO_FIRE) {
    // 无目标，清空输出并重置控制器
    ClearVisionOutput();
    if (vision_takeover_last) {
      VisionControllerReset(gimbal_feed_recv.gimbal_imu_data.Pitch_rad);
      vision_takeover_last = 0;
    }
    return;
  }

  // 视觉数据有效，准备接管
  vision_upload_data.vision_valid = 1;
  vision_upload_data.vision_takeover = 1;

  // 处理视觉接管状态切换
  if (!vision_takeover_last) {
    // 刚进入视觉接管，重置控制器
    VisionControllerReset(gimbal_feed_recv.gimbal_imu_data.Pitch_rad);
    LOGINFO("[vision] Vision takeover started.");
  }
  vision_takeover_last = 1;

  // 计算目标角度（加入手动微调偏移）
  const float yaw_target =
      vision_recv_data->yaw + vision_cmd_recv.manual_yaw_offset;
  const float pitch_target =
      vision_recv_data->pitch + vision_cmd_recv.manual_pitch_offset;

  // 保存原始目标（用于调试）
  vision_upload_data.yaw = yaw_target;
  vision_upload_data.pitch = pitch_target;

  VisionCtrlInput_s ctrl_input = {
      .yaw_target_rad = yaw_target,
      .pitch_target_rad = pitch_target,
      .yaw_angle_rad = gimbal_feed_recv.gimbal_imu_data.YawAngle_rad,
      .yaw_gyro_rad_s = gimbal_feed_recv.gimbal_imu_data.Gyro[2],
      .pitch_feedback_rad = gimbal_feed_recv.gimbal_imu_data.Pitch_rad,
      // 视觉提供的目标速度（用于前馈）
      .target_yaw_vel = vision_recv_data->yaw_vel,
      .target_pitch_vel = vision_recv_data->pitch_vel,
  };
  VisionCtrlOutput_s ctrl_output = {0};
  VisionCtrlStep(&vision_ctrl_state, (const VisionCtrlParams_s *)&vision_params,
                 &ctrl_input, &ctrl_output);

  vision_upload_data.yaw_current_cmd = ctrl_output.yaw_current_cmd;
  // Pitch限速：输出限速后的目标角度（注意符号，与gimbal.c保持一致）
  vision_upload_data.pitch_ref_limited = -ctrl_output.pitch_ref_limited;

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
  ClearVisionOutput();
  if (vision_takeover_last) {
    VisionControllerReset(gimbal_feed_recv.gimbal_imu_data.Pitch_rad);
    vision_takeover_last = 0;
  }
}

/**
 * @brief 手动辅助瞄准模式处理(预留扩展)
 */
static void ProcessManualAim(void) {
  // TODO: 实现视觉辅助但保留手动微调能力
  ClearVisionOutput();
  if (vision_takeover_last) {
    VisionControllerReset(gimbal_feed_recv.gimbal_imu_data.Pitch_rad);
    vision_takeover_last = 0;
  }
}

/**
 * @brief 清空视觉输出数据
 */
static void ClearVisionOutput(void) {
  vision_upload_data.vision_valid = 0;
  vision_upload_data.target_locked = 0;
  vision_upload_data.should_fire = 0;
  vision_upload_data.vision_takeover = 0;
  vision_upload_data.yaw_current_cmd = 0.0f;
  vision_upload_data.pitch_ref_limited = 0.0f;
  vision_upload_data.yaw = 0.0f;
  vision_upload_data.pitch = 0.0f;
}

/* ==========================================================================
 * 视觉控制算法实现（原vision_controller.c）
 * ========================================================================== */

/**
 * @brief 角度归一化到[-π, π]
 */
static void VisionControllerInit(void) { VisionCtrlInit(&vision_ctrl_state); }

static void VisionControllerReset(float pitch_feedback_rad) {
  VisionCtrlReset(&vision_ctrl_state,
                  (const VisionCtrlParams_s *)&vision_params,
                  pitch_feedback_rad);
}
