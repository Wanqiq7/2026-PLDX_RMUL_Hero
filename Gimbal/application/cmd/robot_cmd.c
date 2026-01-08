// app
#include "robot_cmd.h"
#include "robot_def.h"
// module
#include "bmi088.h"
#include "dji_motor.h"
#include "general_def.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "remote_control.h"
#include "user_lib.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include "arm_math.h"
#include <math.h>

#define DEG_TO_RAD 0.01745329251994329576f

/* ============================================================
 * 遥控器输入滤波器设计说明
 * ============================================================
 *
 * 1. 滤波器类型：一阶低通滤波器（IIR）
 *    公式：y[n] = α·x[n] + (1-α)·y[n-1]
 *
 * 2. 截止频率计算公式：
 *    fc = -fs·ln(1-α)/(2π)，其中 fs = 200Hz（控制频率）
 *
 * 3. 参数选择原则：
 *    - 遥控器杆量：α=0.90，fc≈84Hz（快速响应）
 *    - 云台控制：α=0.95/0.93，fc≈99/85Hz（平衡精度与响应）
 *    - 键盘控制：α=0.80，fc≈62Hz（平滑阶跃）
 *    - 鼠标控制：α=0.85/0.80，fc≈63/51Hz（瞄准稳定）
 *
 * 4. 总带宽分析：
 *    串联滤波器后的系统总带宽约17-20Hz，满足机器人控制需求
 * ============================================================ */

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE                                                        \
  (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
// Pitch轴角度限位保护宏（与陀螺仪角度同单位，弧度制）
#define LIMIT_PITCH_RAD(angle_rad)                                             \
  do {                                                                         \
    const float max_rad = PITCH_MAX_ANGLE * DEG_TO_RAD;                        \
    const float min_rad = PITCH_MIN_ANGLE * DEG_TO_RAD;                        \
    if ((angle_rad) > max_rad)                                                 \
      (angle_rad) = max_rad;                                                   \
    if ((angle_rad) < min_rad)                                                 \
      (angle_rad) = min_rad;                                                   \
  } while (0)

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s
    chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s
    chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回

// 视觉控制消息(通过message center与vision应用通信)
static Publisher_t *vision_cmd_pub;           // 视觉控制指令发布者
static Subscriber_t *vision_data_sub;         // 视觉处理数据订阅者
static Vision_Ctrl_Cmd_s vision_cmd_send;     // 发送给视觉应用的控制指令
static Vision_Upload_Data_s vision_data_recv; // 从视觉应用接收的处理数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
// 键盘控制
static float keyboard_vx_cmd_planned = 0.0f;
static float keyboard_vy_cmd_planned = 0.0f;
void RobotCMDInit() {
  rc_data = RemoteControlInit(
      &huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个

  // 注意: 视觉CAN通信初始化已移至 VisionAppInit() (vision.c)

  // 视觉控制消息注册
  vision_cmd_pub = PubRegister("vision_cmd", sizeof(Vision_Ctrl_Cmd_s));
  vision_data_sub = SubRegister("vision_data", sizeof(Vision_Upload_Data_s));

  gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
  gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
  shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
  shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
  chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
  chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
  CANComm_Init_Config_s comm_conf = {
      .can_config =
          {
              .can_handle = &hcan2,
              .tx_id = 0x312,
              .rx_id = 0x311,
          },
      .recv_data_len = sizeof(Chassis_Upload_Data_s),
      .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
  };
  cmd_can_comm = CANCommInit(&comm_conf);
#endif                          // GIMBAL_BOARD
  gimbal_cmd_send.yaw = 0.0f;   // Yaw初始化为0弧度
  gimbal_cmd_send.pitch = 0.0f; // Pitch初始化为0弧度
  gimbal_cmd_send.yaw_vel_ff = 0.0f;
  gimbal_cmd_send.yaw_acc_ff = 0.0f;
  gimbal_cmd_send.pitch_vel_ff = 0.0f;
  gimbal_cmd_send.pitch_acc_ff = 0.0f;

  // ⭐ 初始化发射模块控制指令
  shoot_cmd_send.shoot_mode = SHOOT_ON;        // 发射模块使能
  shoot_cmd_send.friction_mode = FRICTION_OFF; // 摩擦轮默认关闭
  shoot_cmd_send.load_mode = LOAD_STOP;        // 拨盘默认停止
  shoot_cmd_send.bullet_speed = SMALL_AMU_15;  // 默认弹速（会被switch覆盖）
  shoot_cmd_send.shoot_rate = 1.0f;            // 默认射频1发/秒（英雄用）
  shoot_cmd_send.lid_mode = LID_CLOSE;         // 默认弹舱盖关闭

  // ⭐ 初始化视觉控制指令
  vision_cmd_send.vision_mode = VISION_MODE_OFF; // 默认关闭视觉控制
  vision_cmd_send.allow_auto_fire = 0;           // 默认禁止自动射击
  vision_cmd_send.manual_yaw_offset = 0.0f;      // 无手动微调
  vision_cmd_send.manual_pitch_offset = 0.0f;

  robot_state =
      ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 快速角度归一化到[-180, 180]（内联优化）
 * @note 输入范围假设在[-360, 360]内（单圈角度差），最多需要一次调整
 */
static inline float NormalizeAngleFast(float angle) {
  if (angle > 180.0f)
    return angle - 360.0f;
  if (angle <= -180.0f)
    return angle + 360.0f;
  return angle;
}

/**
 * @brief 计算云台-底盘角度关系（高性能优化版本）
 *        同时计算 offset_angle（坐标变换用）和 near_center_error（跟随控制用）
 *
 * @note 优化要点：
 *       1. 统一角度归一化逻辑，消除条件编译分支
 *       2. 利用数学对称性减少重复计算
 *       3. 使用内联函数和条件表达式优化指令级性能
 */
static void CalcGimbalChassisAngle(void) {
  // ============================================================
  // Part 1: 计算 offset_angle（统一归一化逻辑）
  // ============================================================
  float yaw_angle = gimbal_fetch_data.yaw_motor_single_round_angle;
  float raw_diff = yaw_angle - YAW_ALIGN_ANGLE;

  // 快速归一化到 [-180, 180]
  // 输入范围：yaw_angle ∈ [0, 360], YAW_ALIGN_ANGLE ∈ [0, 360]
  // 差值范围：raw_diff ∈ [-360, 360]，最多需要一次调整
  raw_diff = NormalizeAngleFast(raw_diff);
  chassis_cmd_send.offset_angle = raw_diff;

  // ============================================================
  // Part 2: 就近回中（Flip 逻辑优化）
  // ============================================================
#if CHASSIS_FOLLOW_ALLOW_FLIP
  static uint8_t flip_state = 0;

  // 1. 根据当前翻转状态计算误差
  //    flip_state = 0: target = 0°,   error = raw_diff
  //    flip_state = 1: target = 180°, error = NormalizeAngleFast(raw_diff -
  //    180°)
  float error = raw_diff;
  if (flip_state) {
    error = NormalizeAngleFast(raw_diff - 180.0f);
  }

  // 2. 检查是否需要切换翻转状态（迟滞逻辑）
  //    当误差绝对值超过阈值时，说明底盘"背对"云台，触发翻转
  if (fabsf(error) > CHASSIS_FOLLOW_FLIP_THRESHOLD) {
    flip_state = !flip_state;

    // 翻转后利用数学对称性直接计算新误差：
    // error_new = error_old ± 180° (归一化)
    // 若 error > 0: error_new = error - 180
    // 若 error < 0: error_new = error + 180
    error = (error > 0.0f) ? (error - 180.0f) : (error + 180.0f);
  }
#else
  float error = chassis_cmd_send.offset_angle;
#endif

  // ============================================================
  // Part 3: 限幅（优化为 if-else if 结构，便于编译器生成 VMIN/VMAX 指令）
  // ============================================================
  if (error > CHASSIS_FOLLOW_MAX_ERR)
    error = CHASSIS_FOLLOW_MAX_ERR;
  else if (error < -CHASSIS_FOLLOW_MAX_ERR)
    error = -CHASSIS_FOLLOW_MAX_ERR;

  chassis_cmd_send.near_center_error = error;
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet() {
  // 左侧开关状态为[中],底盘跟随云台
  if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    // gimbal_cmd_send.gimbal_mode = GIMBAL_SYS_ID_CHIRP;
  }
  // 左侧开关状态为[下],或视觉未识别到目标,纯遥控器拨杆控制
  else if (switch_is_down(
               rc_data[TEMP].rc.switch_left)) // 纯遥控器：底盘不跟随云台
  {
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW; // 允许全向移动，wz=0
    gimbal_cmd_send.gimbal_mode = GIMBAL_FREE_MODE;
  }
  // 左侧开关状态为[上],底盘小陀螺
  else if (switch_is_up(rc_data[TEMP].rc.switch_left)) {
    chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
  }
  // 底盘改为左摇杆
  static float vx_filtered = 0.0f;
  static float vy_filtered = 0.0f;

  float vy_norm = (float)rc_data[TEMP].rc.rocker_l1 / 660.0f;
  float vx_norm = (float)rc_data[TEMP].rc.rocker_l_ / 660.0f;

  // ✅ 优化的低通滤波参数
  // 原参数α=0.85，截止频率约62.8Hz（200Hz采样）
  // 新参数α=0.90，截止频率约83.9Hz，提升响应速度同时过滤噪声
  // 数学计算：fc = -fs·ln(1-α)/(2π)，其中fs=200Hz
  vx_filtered = LowPassFilter_Float(vx_norm, 0.90f, &vx_filtered);
  vy_filtered = LowPassFilter_Float(vy_norm, 0.90f, &vy_filtered);

  chassis_cmd_send.vx = vx_filtered;
  chassis_cmd_send.vy = vy_filtered;

  // 云台改为右摇杆，添加低通滤波
  static float yaw_increment_filtered = 0.0f;   // 保存上一次的滤波输出
  static float pitch_increment_filtered = 0.0f; // 保存上一次的滤波输出

  // 计算原始增量，右摇杆归一化到 [-1, 1]
  float yaw_norm = (float)rc_data[TEMP].rc.rocker_r_ / 660.0f; // 转为弧度
  float pitch_norm = (float)rc_data[TEMP].rc.rocker_r1 / 660.0f;

  // 角度增量：Yaw 转弧度，Pitch 保持角度制，后续增益由使用者调整
  float yaw_increment = yaw_norm / 180.0f * M_PI; // 相当于 1° 的弧度
  float pitch_increment =
      pitch_norm / 180.0f * M_PI; // 归一化刻度直接视为弧度增量

  // ✅ 优化的云台控制滤波参数
  // Yaw轴：原参数α=0.98，截止频率约156Hz（响应过慢）
  // 新参数α=0.95，截止频率约98.9Hz，提升响应速度
  // Pitch轴：保持α=0.93，截止频率约85.3Hz，避免高频抖动
  // 数学依据：截止频率 fc = -200·ln(1-α)/(2π)
  yaw_increment =
      LowPassFilter_Float(yaw_increment, 0.95f, &yaw_increment_filtered);
  pitch_increment =
      LowPassFilter_Float(pitch_increment, 0.93f, &pitch_increment_filtered);

  // 更新目标角度
  gimbal_cmd_send.yaw += yaw_increment;     // Yaw: 弧度制
  gimbal_cmd_send.pitch += pitch_increment; // Pitch: 弧度制
  LIMIT_PITCH_RAD(gimbal_cmd_send.pitch);   // Pitch 角度限位

  // 发射参数 + 视觉模式联动
  // 右侧开关控制发射机构 + 视觉模式 - 三档互斥逻辑
  // [下档]: 全部停止（摩擦轮停止，拨盘停止，视觉关闭）
  // [中档]: 仅摩擦轮转动（摩擦轮开启，拨盘停止，视觉关闭）
  // [上档]: 自瞄模式（摩擦轮开启，单发，自瞄启用，允许自动射击）
  if (switch_is_down(rc_data[TEMP].rc.switch_right)) {
    // 下档：全部停止
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    vision_cmd_send.vision_mode = VISION_MODE_OFF; // 关闭视觉
    vision_cmd_send.allow_auto_fire = 0;
  } else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) {
    // 中档：仅摩擦轮转动
    shoot_cmd_send.friction_mode = FRICTION_ON;
    shoot_cmd_send.load_mode = LOAD_STOP;
    vision_cmd_send.vision_mode = VISION_MODE_OFF; // 关闭视觉
    vision_cmd_send.allow_auto_fire = 0;
  } else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    // 上档：自瞄模式（英雄机器人标准配置）
    shoot_cmd_send.friction_mode = FRICTION_ON;
    // shoot_cmd_send.load_mode = LOAD_STOP; // ⭐ 停止拨盘，等待视觉驱动
    shoot_cmd_send.shoot_rate = 1.0f; // 1Hz射频（每秒1发，不应期1000ms）
    // vision_cmd_send.vision_mode = VISION_MODE_AUTO_AIM; // ⭐ 启用自瞄
    // vision_cmd_send.allow_auto_fire = 1;                // ⭐
    // 授权视觉控制射击
    //  如果需要连发测试速度环，改为：
    shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
  }
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet() {
  float target_vx, target_vy;
  float dt = 0.005f; // 5ms

  // 添加底盘键盘控制滤波静态变量
  static float vx_kb_filtered = 0.0f;
  static float vy_kb_filtered = 0.0f;

  // 系数待测
  target_vx =
      rc_data[TEMP].key[KEY_PRESS].w * CHASSIS_KB_MAX_SPEED_X -
      rc_data[TEMP].key[KEY_PRESS].s * CHASSIS_KB_MAX_SPEED_X; // 系数待测
  target_vy = rc_data[TEMP].key[KEY_PRESS].s * CHASSIS_KB_MAX_SPEED_Y -
              rc_data[TEMP].key[KEY_PRESS].d * CHASSIS_KB_MAX_SPEED_Y;

  // ✅ 为键盘控制添加低通滤波（α=0.8，截止频率约62Hz）
  // 原因：键盘输入是阶跃信号，需要滤波避免突变
  vx_kb_filtered = LowPassFilter_Float(target_vx, 0.80f, &vx_kb_filtered);
  vy_kb_filtered = LowPassFilter_Float(target_vy, 0.80f, &vy_kb_filtered);
  // ✅ 注意：使用滤波后的值作为斜坡规划的输入，减少冲击
  keyboard_vx_cmd_planned =
      SoftRamp(vx_kb_filtered, keyboard_vx_cmd_planned, 0, KEYBOARD_RAMP_ACCEL,
               KEYBOARD_RAMP_DECEL, KEYBOARD_RAMP_BRAKE_DECEL, dt);
  keyboard_vy_cmd_planned =
      SoftRamp(vy_kb_filtered, keyboard_vy_cmd_planned, 0, KEYBOARD_RAMP_ACCEL,
               KEYBOARD_RAMP_DECEL, KEYBOARD_RAMP_BRAKE_DECEL, dt);
  chassis_cmd_send.vx = keyboard_vx_cmd_planned;
  chassis_cmd_send.vy = keyboard_vy_cmd_planned;

  // ✅ 添加鼠标控制滤波静态变量
  static float mouse_yaw_increment = 0.0f;
  static float mouse_pitch_increment = 0.0f;

  // 计算原始鼠标增量
  // Yaw: 角度 → 弧度（LQR需要弧度制）
  float raw_mouse_yaw = (float)rc_data[TEMP].mouse.x / 660.0f;
  // Pitch: 鼠标输入同样归一化为弧度刻度
  float raw_mouse_pitch = (float)rc_data[TEMP].mouse.y / 660.0f;

  // ✅ 为鼠标控制添加低通滤波
  // 鼠标DPI通常在400-1600之间，会产生高频噪声，需要滤波
  // Yaw: α=0.85，截止频率约62.8Hz，保持瞄准精度
  // Pitch: α=0.80，截止频率约51Hz，避免垂直抖动
  mouse_yaw_increment =
      LowPassFilter_Float(raw_mouse_yaw, 0.85f, &mouse_yaw_increment);
  mouse_pitch_increment =
      LowPassFilter_Float(raw_mouse_pitch, 0.80f, &mouse_pitch_increment);

  gimbal_cmd_send.yaw += mouse_yaw_increment;
  gimbal_cmd_send.pitch += mouse_pitch_increment;
  LIMIT_PITCH_RAD(gimbal_cmd_send.pitch); // 添加pitch角度限位保护

  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
  {
  case 0:
    shoot_cmd_send.bullet_speed = 15;
    break;
  case 1:
    shoot_cmd_send.bullet_speed = 18;
    break;
  default:
    shoot_cmd_send.bullet_speed = 30;
    break;
  }
  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4) // E键设置发射模式
  {
  case 0:
    shoot_cmd_send.load_mode = LOAD_STOP;
    break;
  case 1:
    shoot_cmd_send.load_mode = LOAD_1_BULLET;
    break;
  case 2:
    shoot_cmd_send.load_mode = LOAD_3_BULLET;
    break;
  default:
    shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    break;
  }
  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
  {
  case 0:
    shoot_cmd_send.lid_mode = LID_OPEN;
    break;
  default:
    shoot_cmd_send.lid_mode = LID_CLOSE;
    break;
  }
  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
  {
  case 0:
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    break;
  default:
    shoot_cmd_send.friction_mode = FRICTION_ON;
    break;
  }
  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
  {
  case 0:
    chassis_cmd_send.chassis_speed_buff = 40;
    break;
  case 1:
    chassis_cmd_send.chassis_speed_buff = 60;
    break;
  case 2:
    chassis_cmd_send.chassis_speed_buff = 80;
    break;
  default:
    chassis_cmd_send.chassis_speed_buff = 100;
    break;
  }
  switch (rc_data[TEMP]
              .key[KEY_PRESS]
              .shift) // 待添加 按shift允许超功率 消耗缓冲能量
  {
  case 1:

    break;

  default:

    break;
  }
}

/**
 * @brief 视觉数据融合，当视觉有效时覆盖云台和发射控制
 *
 */
static void VisionControlSet() {
  // 仅当视觉模式启用且视觉数据有效时才接管控制
  if (vision_cmd_send.vision_mode != VISION_MODE_OFF &&
      vision_data_recv.vision_valid) {
    // 使用视觉数据覆盖云台目标角度（完全接管云台控制）
    gimbal_cmd_send.yaw = vision_data_recv.yaw;
    gimbal_cmd_send.pitch = vision_data_recv.pitch;
    gimbal_cmd_send.yaw_vel_ff =
        vision_data_recv.yaw_vel_ff; // 仅透传，不在控制环使用
    gimbal_cmd_send.yaw_acc_ff = vision_data_recv.yaw_acc_ff;
    gimbal_cmd_send.pitch_vel_ff = vision_data_recv.pitch_vel_ff;
    gimbal_cmd_send.pitch_acc_ff = vision_data_recv.pitch_acc_ff;
    LIMIT_PITCH_RAD(gimbal_cmd_send.pitch);

    // ⭐ 视觉驱动发射：根据 should_fire 决定是否触发拨弹
    if (vision_data_recv.should_fire) {
      shoot_cmd_send.load_mode = LOAD_BURSTFIRE; // 视觉确认可以射击，触发连射
      // friction_mode 已在 RemoteControlSet() 上档中设置为 ON，无需重复
    } else {
      shoot_cmd_send.load_mode = LOAD_STOP; // 视觉未确认，停止拨盘
    }
  }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo
 * 后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler() {
  // 最高优先级：检测到离线或手动急停信号(开关在最下方)，则强制进入停止状态
  if (!RemoteControlIsOnline()) {
    robot_state = ROBOT_STOP; // 设置状态为停止

    // 发送所有模块的停止指令
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
  }
  // 仅当不满足急停条件时，才检查是否可以进入就绪状态
  else {
    // 这里可以根据需要设置进入 READY 的条件，
    // 例如，当中位或上位时进入 READY 状态
    // 为了安全，通常需要一个明确的动作来“解锁”机器人
    // 只有当机器人从STOP状态恢复时，才设置一次READY
    if (robot_state == ROBOT_STOP) {
      robot_state = ROBOT_READY;
      shoot_cmd_send.shoot_mode = SHOOT_ON; // 恢复发射模块
      LOGINFO("[CMD] Robot reinstated, ready to operate.");
    }
  }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
  // BMI088Acquire(bmi088_test,&bmi088_data) ;
  // 从其他应用获取回传数据
#ifdef ONE_BOARD
  SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
  chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
  SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
  SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
  SubGetMessage(vision_data_sub, &vision_data_recv); // 获取视觉处理数据

  // 计算云台-底盘角度关系（offset_angle 和 near_center_error）
  CalcGimbalChassisAngle();

  // 控制逻辑优先级（从低到高）：
  // 1️⃣ 遥控器基础控制（提供默认控制量）
  RemoteControlSet();

  // 2️⃣ 视觉数据融合（当视觉有效时覆盖云台和发射控制）
  VisionControlSet();

  // 3️⃣ 紧急停止处理（最高优先级，强制停止所有模块）
  EmergencyHandler();

  // 推送消息,双板通信,视觉通信等
  // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
  PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
  CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
  PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
  PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
  PubPushMessage(vision_cmd_pub, (void *)&vision_cmd_send); // 发布视觉控制指令
}
