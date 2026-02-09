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

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE                                                        \
  (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE                                                    \
  (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

// Pitch轴角度限位保护宏
#define LIMIT_PITCH_ANGLE(angle)                                               \
  do {                                                                         \
    if ((angle) > PITCH_MAX_ANGLE)                                             \
      (angle) = PITCH_MAX_ANGLE;                                               \
    if ((angle) < PITCH_MIN_ANGLE)                                             \
      (angle) = PITCH_MIN_ANGLE;                                               \
  } while (0)

static const float RC_CMD_MAX_LINEAR_SPEED = 2.0f;  // m/s
static const float RC_CMD_MAX_ANGULAR_SPEED = 2.5f; // rad/s

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

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
// static Vision_Send_s vision_send_data;  // 视觉发送数据

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
  // BMI088_Init_Config_s bmi088_config = {
  //     .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
  //     .work_mode = BMI088_BLOCK_TRIGGER_MODE,
  //     .spi_acc_config = {
  //         .spi_handle = &hspi1,
  //         .GPIOx = GPIOA,
  //         .cs_pin = GPIO_PIN_4,
  //         .spi_work_mode = SPI_DMA_MODE,
  //     },
  //     .acc_int_config = {
  //         .GPIOx = GPIOC,
  //         .GPIO_Pin = GPIO_PIN_4,
  //         .exti_mode = GPIO_EXTI_MODE_RISING,
  //     },
  //     .spi_gyro_config = {
  //         .spi_handle = &hspi1,
  //         .GPIOx = GPIOB,
  //         .cs_pin = GPIO_PIN_0,
  //         .spi_work_mode = SPI_DMA_MODE,
  //     },
  //     .gyro_int_config = {
  //         .GPIO_Pin = GPIO_PIN_5,
  //         .GPIOx = GPIOC,
  //         .exti_mode = GPIO_EXTI_MODE_RISING,
  //     },
  //     .heat_pwm_config = {
  //         .htim = &htim10,
  //         .channel = TIM_CHANNEL_1,
  //         .period = 1,
  //     },
  //     .heat_pid_config = {
  //         .Kp = 0.5,
  //         .Ki = 0,
  //         .Kd = 0,
  //         .DeadBand = 0.1,
  //         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
  //         PID_Derivative_On_Measurement, .IntegralLimit = 100, .MaxOut = 100,
  //     },
  // };
  // bmi088_test = BMI088Register(&bmi088_config);
  rc_data = RemoteControlInit(
      &huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
  vision_recv_data =
      VisionInit(&huart6); // 视觉通信串口,由原来框架的US1改为适配现车的US6

  gimbal_cmd_pub = RegisterPublisher("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
  gimbal_feed_sub = RegisterSubscriber("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
  shoot_cmd_pub = RegisterPublisher("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
  shoot_feed_sub = RegisterSubscriber("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
  chassis_cmd_pub = RegisterPublisher("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
  chassis_feed_sub = RegisterSubscriber("chassis_feed", sizeof(Chassis_Upload_Data_s));
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
#endif // GIMBAL_BOARD
  gimbal_cmd_send.pitch = 0;

  robot_state =
      ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
/**
 * @brief 就近回中误差计算,支持车头翻转
 *        根据当前offset_angle和翻转状态,计算最优控制误差和微分项
 *        计算结果存储在chassis_cmd_send的near_center_error中
 *
 * @param offset_angle 云台相对底盘的偏差角度(-180~180度)
 */
/**
 * @brief 角度归一化到[-180, 180]区间,使用arm_math优化
 * @param angle 输入角度
 * @return float 归一化后的角度
 */
static inline float NormalizeAngle(float angle) {
  // 使用fmodf进行模运算,比if-else更高效
  angle = fmodf(angle + 180.0f, 360.0f);
  if (angle < 0.0f) {
    angle += 360.0f;
  }
  return angle - 180.0f;
}

static void CalcNearCenterError(float offset_angle) {
  static uint8_t flip_state = 0; // 车头翻转状态: 0-正向, 1-反向(180度)
  float target_angle = 0.0f;     // 目标角度
  float error = 0.0f;
  float abs_error = 0.0f; // 误差绝对值

#if CHASSIS_FOLLOW_ALLOW_FLIP
  // 根据翻转状态确定目标角度
  target_angle = flip_state ? 180.0f : 0.0f;

  // 计算误差并归一化到[-180, 180],使用优化的归一化函数
  error = NormalizeAngle(offset_angle - target_angle);

  // 使用arm_math库的绝对值函数(内联优化)
  abs_error = fabsf(error);

  // 翻转状态切换逻辑:当误差超过阈值时考虑翻转
  if (abs_error > CHASSIS_FOLLOW_FLIP_THRESHOLD) {
    flip_state = !flip_state; // 切换翻转状态
    // 重新计算误差
    target_angle = flip_state ? 180.0f : 0.0f;
    error = NormalizeAngle(offset_angle - target_angle);
  }
#else
  // 不允许翻转,直接使用offset_angle作为误差
  error = offset_angle;
#endif

  // 误差限幅,使用arm_math的float_constrain功能
  error =
      float_constrain(error, -CHASSIS_FOLLOW_MAX_ERR, CHASSIS_FOLLOW_MAX_ERR);

  // 存储就近回中误差,供底盘PID控制器使用
  chassis_cmd_send.near_center_error = error;
}

static void CalcOffsetAngle() {
  // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
  static float angle;
  angle = gimbal_fetch_data
              .yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                // 如果大于180度
  if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
  else if (angle > 180.0f + YAW_ALIGN_ANGLE)
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
  else
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
  if (angle > YAW_ALIGN_ANGLE)
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
  else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
  else
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet() {
  // 左侧开关状态为[中],底盘跟随云台
  if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
    chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    // gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    gimbal_cmd_send.gimbal_mode = GIMBAL_SYS_ID_CHIRP;
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
  /*******************************************************************************************************
   */
  // 底盘改为左摇杆
  float vx_norm = (float)rc_data[TEMP].rc.rocker_l_ / 660.0f;
  float vy_norm = (float)rc_data[TEMP].rc.rocker_l1 / 660.0f;

  static float vx_filtered = 0.0f;
  static float vy_filtered = 0.0f;

  vx_filtered = LowPassFilter_Float(vx_norm, 0.90f, &vx_filtered);
  vy_filtered = LowPassFilter_Float(vy_norm, 0.90f, &vy_filtered);

  chassis_cmd_send.vx = vx_filtered;
  chassis_cmd_send.vy = vy_filtered;
  /*********************************************************************************************************
   */
  // 云台改为右摇杆，添加低通滤波
  static float yaw_increment_filtered = 0.0f;   // 保存上一次的滤波输出
  static float pitch_increment_filtered = 0.0f; // 保存上一次的滤波输出

  // 计算原始增量
  float yaw_increment = 0.001f * (float)rc_data[TEMP].rc.rocker_r_;
  float pitch_increment = 0.0005f * (float)rc_data[TEMP].rc.rocker_r1;

  // 低通滤波，K=0.15对应约5Hz截止频率（在200Hz采样下）
  yaw_increment =
      LowPassFilter_Float(yaw_increment, 0.18f, &yaw_increment_filtered);
  pitch_increment =
      LowPassFilter_Float(pitch_increment, 0.85f, &pitch_increment_filtered);

  // 更新目标角度
  // gimbal_cmd_send.yaw += yaw_increment;
  gimbal_cmd_send.pitch += pitch_increment;
  LIMIT_PITCH_ANGLE(gimbal_cmd_send.pitch); // 添加pitch角度限位保护
  /*********************************************************************************************************
   */
  // 发射参数
  // 右侧开关状态[下],无操作
  if (switch_is_down(rc_data[TEMP].rc.switch_right)) {
    /* chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE; */
  }
  // 右侧开关状态[中],摩擦轮开始转动
  if (switch_is_mid(rc_data[TEMP].rc.switch_right))
    shoot_cmd_send.friction_mode = FRICTION_ON;
  else
    shoot_cmd_send.friction_mode = FRICTION_OFF;
  // 右侧开关状态[上],开始发弹
  if (switch_is_up(rc_data[TEMP].rc.switch_right))
    shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
  else
    shoot_cmd_send.load_mode = LOAD_STOP;
  // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
  /* shoot_cmd_send.shoot_rate = 8; */
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet() {
  float target_vx, target_vy;
  float dt = 0.005f; // 5ms
  // 系数待测
  target_vx =
      rc_data[TEMP].key[KEY_PRESS].w * CHASSIS_KB_MAX_SPEED_X -
      rc_data[TEMP].key[KEY_PRESS].s * CHASSIS_KB_MAX_SPEED_X; // 系数待测
  target_vy = rc_data[TEMP].key[KEY_PRESS].s * CHASSIS_KB_MAX_SPEED_Y -
              rc_data[TEMP].key[KEY_PRESS].d * CHASSIS_KB_MAX_SPEED_Y;
  keyboard_vx_cmd_planned =
      SoftRamp(target_vx, keyboard_vx_cmd_planned, 0, KEYBOARD_RAMP_ACCEL,
               KEYBOARD_RAMP_DECEL, KEYBOARD_RAMP_BRAKE_DECEL, dt);
  keyboard_vy_cmd_planned =
      SoftRamp(target_vy, keyboard_vy_cmd_planned, 0, KEYBOARD_RAMP_ACCEL,
               KEYBOARD_RAMP_DECEL, KEYBOARD_RAMP_BRAKE_DECEL, dt);
  chassis_cmd_send.vx = keyboard_vx_cmd_planned;
  chassis_cmd_send.vy = keyboard_vy_cmd_planned;

  gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
  gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;
  LIMIT_PITCH_ANGLE(gimbal_cmd_send.pitch); // 添加pitch角度限位保护

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

  // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
  CalcOffsetAngle();
  // 计算就近回中误差(支持车头翻转优化)
  CalcNearCenterError(chassis_cmd_send.offset_angle);
  // 遥控器控制
  RemoteControlSet();
  EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

  // 设置视觉发送数据,还需增加加速度和角速度数据
  // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

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
}
