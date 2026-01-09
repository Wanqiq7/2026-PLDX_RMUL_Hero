#include "chassis.h"
#include "VOFA.h"
#include "arm_math.h"
#include "bsp_dwt.h"
#include "bsp_usart.h"
#include "controller.h"
#include "dji_motor.h"
#include "general_def.h"
#include "math.h"
#include "message_center.h"
#include "power_controller.h"
#include "referee_UI.h"
#include "referee_task.h"
#include "robot_def.h"
#include "super_cap.h"
#include "sysid_task.h"
#include "user_lib.h"

/* ============================================================
 * 系统辨识控制开关（测试时改为1，完成后改为0）
 * ============================================================ */
#define ENABLE_CHASSIS_SYSID 0 // 0-关闭，1-启动辨识
#define SYSID_TARGET_MOTOR 0   // 0-左前lf, 1-右前rf, 2-左后lb, 3-右后rb

#if ENABLE_CHASSIS_SYSID
static Publisher_t *sysid_macro_pub = NULL;
static uint8_t sysid_macro_done = 0;

/**
 * @brief 系统辨识宏开关触发函数
 * @note 当ENABLE_CHASSIS_SYSID=1时，上电3秒后自动启动辨识
 *       辨识运行20秒后自动停止，电机完全停止
 *       测试完成后，将宏改为0，重新编译即可恢复正常
 *
 * 使用说明：
 *   1. 架起机器人（轮子悬空）
 *   2. 修改上面的宏：ENABLE_CHASSIS_SYSID = 1
 *   3. 选择目标电机：SYSID_TARGET_MOTOR = 0~3
 *   4. 编译烧录
 *   5. 配置Ozone记录变量（见文档）
 *   6. 上电等待3秒 → 自动启动辨识（持续20秒）
 *   7. 20秒后电机自动停止
 *   8. Ozone导出CSV，运行MATLAB分析
 *   9. 测试完成后，改回：ENABLE_CHASSIS_SYSID = 0
 */
static void ChassisSystemIDSwitch() {
  static uint8_t sysid_stop_sent = 0;

  // 第一阶段：启动辨识（上电3秒后）
  if (!sysid_macro_done && DWT_GetTimeline_s() > 3.0f) {
    // 首次调用，注册发布者
    if (sysid_macro_pub == NULL) {
      sysid_macro_pub =
          PubRegister("chassis_sysid_cmd", sizeof(Chassis_SysID_Ctrl_Cmd_s));
    }

    // 发布启动指令
    Chassis_SysID_Ctrl_Cmd_s sysid_cmd = {
        .enable = 1,
        .target_motor = SYSID_TARGET_MOTOR,
    };
    PubPushMessage(sysid_macro_pub, &sysid_cmd);

    sysid_macro_done = 1; // 标记已触发
  }

  // 第二阶段：确保停止（上电25秒后，辨识应该已完成）
  if (sysid_macro_done && !sysid_stop_sent && DWT_GetTimeline_s() > 25.0f) {
    // 发送停止指令（确保辨识任务退出）
    Chassis_SysID_Ctrl_Cmd_s sysid_cmd = {
        .enable = 0,
        .target_motor = SYSID_TARGET_MOTOR,
    };
    PubPushMessage(sysid_macro_pub, &sysid_cmd);

    sysid_stop_sent = 1; // 标记已发送停止指令
    // 注意：电机停止由sysid_task.c中的逻辑处理
  }
}
#endif
/* ============================================================ */

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)   // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f) // 半轮距

/* 底盘运行时配置实例 - const 优化，编译时优化 */
static const Chassis_Runtime_Config_t chassis_config = {
    .rc =
        {
            .max_linear_speed = 4.0f, // m/s - 最大线速度
            .max_angular_speed = 6.0f // rad/s - 最大角速度
        },
    .force =
        {
            .torque_feedforward_coeff = 0.75f, // 扭矩前馈系数(比例系数)
            .friction_threshold_omega = 2.0f,  // rad/s - 摩擦补偿速度阈值
            .wheel_speed_feedback_coeff =
                0.02f, // A·s/rad - 轮速反馈系数（降低增益减少振荡）
            .omega_error_lpf_alpha =
                0.85f,              // 角速度误差滤波系数（增强滤波抑制噪声）
            .omega_threshold = 4.0f // rad/s - 过零保护角速度阈值（扩大死区）
        },
    .kinematics = {
        .velocity_lpf_alpha = 0.85f, // 速度估算滤波系数
        .speed_deadband = 25.0f,     // 速度死区阈值 (deg/s)
        .follow_lpf_alpha = 0.95f,   // 跟随模式滤波系数
        .rotate_speed = 3.0f         // 小陀螺模式旋转速度 (rad/s)
    }};

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体
 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static referee_info_t *referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t
    ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap; // 超级电容

#if POWER_CONTROLLER_ENABLE
static USARTInstance *vofa_usart = NULL;
static uint8_t vofa_tx_buf[32];
static VofaJustFloatSender_s vofa_sender;

static uint8_t VofaUsartIsReady(void *ctx) {
  USARTInstance *ins = (USARTInstance *)ctx;
  if (ins == NULL || ins->usart_handle == NULL) {
    return 0;
  }

  return (ins->usart_handle->gState == HAL_UART_STATE_READY) ? 1 : 0;
}

static void VofaUsartSend(void *ctx, uint8_t *buf, uint16_t len,
                          VofaTransferMode_e mode) {
  USART_TRANSFER_MODE bsp_mode = USART_TRANSFER_DMA;
  switch (mode) {
  case VOFA_TRANSFER_BLOCKING:
    bsp_mode = USART_TRANSFER_BLOCKING;
    break;
  case VOFA_TRANSFER_IT:
    bsp_mode = USART_TRANSFER_IT;
    break;
  case VOFA_TRANSFER_DMA:
  default:
    bsp_mode = USART_TRANSFER_DMA;
    break;
  }

  USARTSend((USARTInstance *)ctx, buf, len, bsp_mode);
}
#endif
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb,
    *motor_rb; // left right forward back

static PIDInstance chassis_follow_pid; // 底盘跟随云台PID控制器
static float last_follow_wz = 0.0f; // 记录上一次跟随模式的wz输出，用于一阶滤波

/* 功率控制相关变量已移至独立的power_controller模块 */

/* ----------------力控策略相关变量---------------- */
// 力控LQR控制器（替换PID实现更优控制）
// 力控PID控制器（速度→力/扭矩）
static PIDInstance chassis_force_x_pid; // X方向速度PID（输出：力 N）
static PIDInstance chassis_force_y_pid; // Y方向速度PID（输出：力 N）
static PIDInstance chassis_torque_pid;  // 旋转速度PID（输出：扭矩 N·m）

// 速度估算相关变量
static float chassis_estimated_vx = 0.0f; // 底盘估算速度X (m/s)
static float chassis_estimated_vy = 0.0f; // 底盘估算速度Y (m/s)
static float chassis_estimated_wz = 0.0f; // 底盘估算角速度 (rad/s)

// 力控中间变量
static float force_x = 0.0f;   // X方向控制力 (N)
static float force_y = 0.0f;   // Y方向控制力 (N)
static float torque_z = 0.0f;  // Z轴控制扭矩 (N·m)
static float wheel_force[4];   // 各轮分配的力 (N)
static float wheel_current[4]; // 各轮电流指令 (A)

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy; // 将云台系的速度投影到底盘坐标系

// 力控 + 速度内环：麦轮正运动学解算的目标轮速（rad/s）
static float target_wheel_omega[4]; // 各轮目标角速度，用于速度内环反馈

void ChassisInit() {
  // 四个轮子的参数一样,改tx_id和反转标志位即可
  Motor_Init_Config_s chassis_motor_config = {
      .can_init_config = {.can_handle = &hcan1},
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,
              .outer_loop_type = OPEN_LOOP,
              .close_loop_type = OPEN_LOOP,
              .feedforward_flag = FEEDFORWARD_NONE,
          },
      .motor_type = M3508,
  };

  chassis_motor_config.controller_param_init_config.speed_feedforward_ptr =
      NULL;
  // 右前轮（RF）- 电机ID=1
  chassis_motor_config.can_init_config.tx_id = 1;
  chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE;
  motor_rf = DJIMotorInit(&chassis_motor_config);

  // 左前轮（LF）- 电机ID=2
  chassis_motor_config.can_init_config.tx_id = 2;
  chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE;
  motor_lf = DJIMotorInit(&chassis_motor_config);

  // 左后轮（LB）- 电机ID=3
  chassis_motor_config.can_init_config.tx_id = 3;
  chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE;
  motor_lb = DJIMotorInit(&chassis_motor_config);

  // 右后轮（RB）- 电机ID=4
  chassis_motor_config.can_init_config.tx_id = 4;
  chassis_motor_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE;
  motor_rb = DJIMotorInit(&chassis_motor_config);

  referee_data = UITaskInit(&huart6, &ui_data); // 裁判系统初始化,会同时初始化UI

  SuperCap_Init_Config_s cap_conf = {
      .can_config = {
          .can_handle = &hcan1,
          .tx_id = 0x061, // 超级电容控制器实际接收的 CAN ID（修复）
          .rx_id = 0x051, // 超级电容控制器实际发送的 CAN ID（修复）
      }};
  cap = SuperCapInit(&cap_conf); // 超级电容初始化

#if POWER_CONTROLLER_ENABLE
  // VOFA+ JustFloat：用于将超级电容实时功率发送到上位机
  // 注意：USARTRegister() 会启动 DMA 接收，因此 recv_buff_size 不能为 0
  // 串口选择原则：避免与遥控器/裁判系统/视觉等已占用串口冲突
  USART_Init_Config_s vofa_usart_conf = {
      .recv_buff_size = 1,
      .usart_handle = &huart1,
      .module_callback = NULL,
  };
  vofa_usart = USARTRegister(&vofa_usart_conf);

  vofa_sender.ctx = vofa_usart;
  vofa_sender.is_ready = VofaUsartIsReady;
  vofa_sender.send = VofaUsartSend;
  vofa_sender.tx_buf = vofa_tx_buf;
  vofa_sender.tx_buf_len = (uint16_t)sizeof(vofa_tx_buf);
  vofa_sender.max_channels = 1;
  vofa_sender.transfer_mode = VOFA_TRANSFER_DMA;
#endif

  // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
  Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

  CANComm_Init_Config_s comm_conf = {
      .can_config =
          {
              .can_handle = &hcan2,
              .tx_id = 0x311,
              .rx_id = 0x312,
          },
      .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
      .send_data_len = sizeof(Chassis_Upload_Data_s),
  };
  chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                        // CHASSIS_BOARD

  // 功率控制器初始化（独立模块）
  // ⚠️ 所有量纲统一到输出轴侧（与功率模型k1、k2参数匹配）
  PowerControllerConfig_t power_config = {
      .k1_init = 0.22f,      // 转速损耗系数初始值（输出轴量纲）
      .k2_init = 1.2f,       // 力矩损耗系数初始值（输出轴量纲）
      .k3 = 5.2f,            // 静态功率损耗
      .rls_lambda = 0.9999f, // RLS遗忘因子
      // 输出轴转矩常数 = 0.3 Nm/A
      .torque_constant = M3508_TORQUE_CONSTANT,
      .current_scale = 20.0f / 16384.0f, // CAN指令值转电流系数
  };
  PowerControllerInit(&power_config);

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
  chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
  chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD

  // 底盘跟随云台PID控制器初始化（统一力控：输出角速度）
  // 注意：输出单位为角速度（rad/s），后续通过力控PID转换为扭矩
  PID_Init_Config_s follow_pid_config = {
      .Kp = 0.45f, // 单位：(rad/s)/度（角度误差→角速度）
      .Ki = 0.0f,  // 积分增益（可选开启）
      .Kd = 0.04f, // 微分增益
      // 微分低通滤波时间常数RC（秒）
      // RobotTask周期约2ms(500Hz)，取微分截止频率fc≈20Hz：
      // RC = 1/(2πfc) ≈ 1/(2π·20) ≈ 0.00796s
      .Derivative_LPF_RC = 0.008f,
      .IntegralLimit = 0.0f, // 积分限幅：1 rad/s
      .MaxOut = 5.5f,        // 最大输出：5 rad/s（底盘旋转角速度）
      .DeadBand = 1.58f,     // 死区：0.25度（静止时更稳定）
      .Improve = PID_Derivative_On_Measurement | PID_DerivativeFilter,
  };
  PIDInit(&chassis_follow_pid, &follow_pid_config);

  /* ----------------力控策略PID初始化---------------- */
  // X方向力控PID初始化 (输出单位: N)
  PID_Init_Config_s force_x_pid_config = {
      .Kp = 200.0f,                // 比例增益 [N/(m/s)]
      .Ki = 0.0f,                  // 积分增益 [N/(m·s)]
      .Kd = 0.0f,                  // 微分增益 [N·s/m]（抑制超调）
      .Derivative_LPF_RC = 0.12f,  // 微分低通滤波
      .IntegralLimit = 100.0f,     // 积分限幅 [N]
      .MaxOut = MAX_CONTROL_FORCE, // 最大输出力 [N]
      .DeadBand = 0.01f,           // 死区 [m/s]
      .Improve = PID_Integral_Limit,
  };
  PIDInit(&chassis_force_x_pid, &force_x_pid_config);

  // Y方向力控PID初始化 (输出单位: N)
  PID_Init_Config_s force_y_pid_config = {
      .Kp = 200.0f,
      .Ki = 0.0f,
      .Kd = 0.0f,
      .Derivative_LPF_RC = 0.02f,
      .IntegralLimit = 100.0f,
      .MaxOut = MAX_CONTROL_FORCE,
      .DeadBand = 0.01f,
      .Improve = PID_Integral_Limit,
  };
  PIDInit(&chassis_force_y_pid, &force_y_pid_config);

  // 旋转扭矩PID初始化 (输出单位: N·m) - 优化参数减少振荡
  PID_Init_Config_s torque_pid_config = {
      .Kp = 8.0f, // 比例增益 [N·m/(rad/s)] - 从2.5降至1.5减少过冲
      .Ki = 0.0f, // 积分增益 [N·m/rad]
      .Kd = 0.1f, // 微分增益 [N·m·s/rad] - 增加阻尼抑制振荡
      .MaxOut = 6.0f,
      .DeadBand = 0.04f, // 死区 [rad/s] - 修复：从0.15增至0.3，减少回位后震荡
      .Output_LPF_RC = 0.00087f,
      .Improve = PID_OutputFilter | PID_Derivative_On_Measurement,
  };
  PIDInit(&chassis_torque_pid, &torque_pid_config);

  /* ----------------底盘系统辨识初始化---------------- */
  // 初始化底盘轮速系统辨识任务（用于LQR参数标定）
  Chassis_SysIDTaskInit(motor_lf, motor_rf, motor_lb, motor_rb);
}

// 计算每个轮子到旋转中心的距离（单位：m）
// 使用勾股定理: r = sqrt(x^2 + y^2)
// WHEEL_BASE和TRACK_WIDTH在robot_def.h中已经是m，无需转换
#define LF_CENTER                                                              \
  sqrtf(powf(HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X, 2.0f) +                \
        powf(HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y, 2.0f))
#define RF_CENTER                                                              \
  sqrtf(powf(HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X, 2.0f) +                \
        powf(HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y, 2.0f))
#define LB_CENTER                                                              \
  sqrtf(powf(HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X, 2.0f) +                \
        powf(HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y, 2.0f))
#define RB_CENTER                                                              \
  sqrtf(powf(HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X, 2.0f) +                \
        powf(HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y, 2.0f))

/* ==================================================== */
/* ----------------力控策略核心函数---------------- */
/* ==================================================== */

/**
 * @brief 麦轮正运动学解算 - 计算各轮目标角速度（用于速度内环反馈）
 * @note  参考robowalker的Kinematics_Inverse_Resolution思想
 *        从底盘速度(vx,vy,wz)解算出各轮的目标角速度
 *        这是力控+速度内环的关键：提供轮速目标值用于动态补偿
 */
static void MecanumKinematicsCalculate() {
  // 麦轮正运动学：从底盘速度解算各轮线速度
  // v_wheel = v_chassis + ω × r_vec
  // 对于45度麦轮，与力分配公式对应

  float vx = chassis_vx;
  float vy = chassis_vy;
  float wz = chassis_cmd_recv.wz;

  // 计算各轮线速度（m/s）- 必须与力分配公式的符号一致
  // ⚠️ 数组顺序必须与 motors[] = {RF, LF, LB, RB} 一致（572行）
  float wheel_linear_vel[4];
  wheel_linear_vel[0] = (-vx + vy) + wz * RF_CENTER; // 右前 (RF)
  wheel_linear_vel[1] = (-vx - vy) + wz * LF_CENTER; // 左前 (LF)
  wheel_linear_vel[2] = (vx - vy) + wz * LB_CENTER;  // 左后 (LB)
  wheel_linear_vel[3] = (vx + vy) + wz * RB_CENTER;  // 右后 (RB)

  // 转换为电机角速度（rad/s）
  // 关键：线速度 → 轮子角速度 → 电机角速度（考虑减速比）
  for (int i = 0; i < 4; i++) {
    // 轮子角速度 = 线速度 / 轮子半径
    float wheel_omega = wheel_linear_vel[i] / RADIUS_WHEEL;
    // 电机角速度 = 轮子角速度 × 减速比
    target_wheel_omega[i] = wheel_omega * REDUCTION_RATIO_WHEEL;
  }
}

/**
 * @brief 估算底盘当前速度（优化版）
 * @detail 使用一阶低通滤波平滑速度估算
 *
 * 算法流程：
 * 1. 读取四个电机角速度 → 转换为线速度
 * 2. 应用麦轮逆运动学公式计算底盘速度
 * 3. 一阶低通滤波消除编码器噪声
 *
 * 公式说明：
 *   vx = (-vlf - vrf + vlb + vrb) / 4
 *   vy = (-vlf + vrf - vlb + vrb) / 4
 *   wz = (vlf + vrf + vlb + vrb) / (4×r)
 *   其中 r = sqrt( (L/2)² + (W/2)² )
 */
static void EstimateChassisVelocity(void) {
  /* ===== 第一步：电机角速度 → 轮子线速度 ===== */
  // 转换公式：线速度 = 角速度(rad/s) × 半径
  // 注意：所有电机均为REVERSE配置，驱动层会取反，应用层需要再取反回来
  const float speed_to_linear =
      DEGREE_2_RAD * RADIUS_WHEEL / REDUCTION_RATIO_WHEEL;
  float rf_linear_vel = -motor_rf->measure.speed_aps * speed_to_linear;
  float lf_linear_vel = -motor_lf->measure.speed_aps * speed_to_linear;
  float lb_linear_vel = -motor_lb->measure.speed_aps * speed_to_linear;
  float rb_linear_vel = -motor_rb->measure.speed_aps * speed_to_linear;

  /* ===== 第二步：麦轮逆运动学解算 ===== */
  // 计算旋转半径：从底盘中心到轮子的距离（预计算避免重复计算）
  static const float rotation_radius = sqrtf(
      HALF_WHEEL_BASE * HALF_WHEEL_BASE + HALF_TRACK_WIDTH * HALF_TRACK_WIDTH);

  // 应用逆运动学公式计算瞬时速度
  const float inv_4 = 0.25f; // 预计算1/4，避免除法
  float instant_vx =
      (-lf_linear_vel - rf_linear_vel + lb_linear_vel + rb_linear_vel) * inv_4;
  float instant_vy =
      (-lf_linear_vel + rf_linear_vel - lb_linear_vel + rb_linear_vel) * inv_4;
  float instant_wz =
      (lf_linear_vel + rf_linear_vel + lb_linear_vel + rb_linear_vel) /
      (4.0f * rotation_radius);

  /* ===== 第三步：低通滤波平滑噪声 ===== */
  // 使用一阶低通滤波：y[n] = α×x[n] + (1-α)×y[n-1]
  // α = 0.85 对应截止频率约30.9Hz（200Hz采样）
  static float filtered_vx = 0.0f; // 静态变量避免重复初始化
  static float filtered_vy = 0.0f;
  static float filtered_wz = 0.0f;

  chassis_estimated_vx = LowPassFilter_Float(
      instant_vx, chassis_config.kinematics.velocity_lpf_alpha, &filtered_vx);
  chassis_estimated_vy = LowPassFilter_Float(
      instant_vy, chassis_config.kinematics.velocity_lpf_alpha, &filtered_vy);
  chassis_estimated_wz = LowPassFilter_Float(
      instant_wz, chassis_config.kinematics.velocity_lpf_alpha, &filtered_wz);
}

/**
 * @brief 计算目标扭矩前馈补偿
 * @param target_wz 目标角速度 (rad/s)
 * @return 前馈扭矩 (N·m)
 * @detail 前馈原理：预估维持角速度所需的基础扭矩
 */
static inline float CalculateTorqueFeedforward(float target_wz) {
  return chassis_config.force.torque_feedforward_coeff * target_wz;
}

/**
 * @brief 速度闭环控制 → 力/扭矩输出（重构版）
 *
 * 控制链路：
 * 目标速度 → PID控制器 → 需求力/扭矩 → 前馈补偿 → 最终输出
 */
static void VelocityToForceControl(void) {
  /* ===== 步骤1：更新速度估算 ===== */
  EstimateChassisVelocity();

  /* ===== 步骤2：获取目标速度 ===== */
  float target_vx = chassis_vx;
  float target_vy = chassis_vy;
  float target_wz = chassis_cmd_recv.wz; // rad/s

  /* ===== 步骤3：PID速度控制 → 力/扭矩 ===== */
  // PID输出：根据速度误差计算所需的合力(N)和扭矩(N·m)
  force_x = PIDCalculate(&chassis_force_x_pid, chassis_estimated_vx, target_vx);
  force_y = PIDCalculate(&chassis_force_y_pid, chassis_estimated_vy, target_vy);
  float torque_feedback =
      PIDCalculate(&chassis_torque_pid, chassis_estimated_wz, target_wz);

  /* ===== 步骤4：前馈补偿 ===== */
  // 前馈减少PID负担，提高响应速度
  float torque_feedforward = CalculateTorqueFeedforward(target_wz);

  /* ===== 步骤5：合成最终输出 ===== */
  torque_z = torque_feedback + torque_feedforward;

  /* ===== 步骤6：限幅保护 ===== */
  force_x = float_constrain(force_x, -MAX_CONTROL_FORCE, MAX_CONTROL_FORCE);
  force_y = float_constrain(force_y, -MAX_CONTROL_FORCE, MAX_CONTROL_FORCE);
  torque_z = float_constrain(torque_z, -MAX_CONTROL_TORQUE, MAX_CONTROL_TORQUE);
}
/**
 * @brief 力的动力学逆解算（力控核心环节2）
 * @note  将底盘合力分配到各个轮子，使用独立轮距力臂（修复振荡问题）
 *       修复：使用各轮独立力臂而非统一rotation_radius，与正解算保持一致
 */
static void ForceDynamicsInverseResolution() {
  // RF (0)
  wheel_force[0] = (-force_x + force_y) / 4.0f + torque_z / (4.0f * RF_CENTER);
  // LF (1)
  wheel_force[1] = (-force_x - force_y) / 4.0f + torque_z / (4.0f * LF_CENTER);
  // LB (2)
  wheel_force[2] = (force_x - force_y) / 4.0f + torque_z / (4.0f * LB_CENTER);
  // RB (3)
  wheel_force[3] = (force_x + force_y) / 4.0f + torque_z / (4.0f * RB_CENTER);
}

/**
 * @brief 计算速度反馈补偿电流
 * @param target_omega 目标角速度 (rad/s)
 * @param actual_omega 实际角速度 (rad/s)
 * @param motor_index 电机索引
 * @return 补偿电流 (A)
 */
static float CalculateSpeedFeedback(float target_omega, float actual_omega,
                                    uint8_t motor_index) {
  // 计算角速度误差
  float omega_error = target_omega - actual_omega;

  // 二级滤波减少噪声放大
  static float filtered_omega_error[4] = {0.0f}; // 每个轮子独立滤波状态
  omega_error = LowPassFilter_Float(omega_error,
                                    chassis_config.force.omega_error_lpf_alpha,
                                    &filtered_omega_error[motor_index]);

  // 过零保护：低速时减少补偿避免震荡
  // 修复：改为"或"逻辑，只要任一个接近零就不补偿（防止滤波器滞后导致的方向变化误判）
  if (fabsf(target_omega) < chassis_config.force.omega_threshold ||
      fabsf(actual_omega) < chassis_config.force.omega_threshold) {
    return 0.0f;
  }

  // 方向变化时使用弱补偿（注意：此时已经排除了低速场景）
  if (target_omega * actual_omega < 0.0f) {
    return 0.1f * chassis_config.force.wheel_speed_feedback_coeff * omega_error;
  }

  // 正常速度反馈
  return chassis_config.force.wheel_speed_feedback_coeff * omega_error;
}

/**
 * @brief 计算摩擦补偿电流 (修复版)
 * @param target_omega 目标角速度 (rad/s)
 * @return 摩擦补偿电流 (A)
 * @detail 修复了零速时输出非零电流的问题，使用tanh作为软开关
 */
static inline float CalculateFrictionCompensation(float target_omega) {
  // 1. 计算归一化速度
  // friction_threshold_omega 决定了线性区的大小（S曲线的陡峭程度）
  float normalized_speed =
      target_omega / chassis_config.force.friction_threshold_omega;

  // 2. 计算软开关信号 (-1.0 ~ 1.0)
  // 当 target_omega = 0 时，tanhf(0) = 0，彻底切断电流输出
  float soft_sign = tanhf(normalized_speed);

  /* --- 核心修复逻辑 --- */

  // 方案 A：简单库伦摩擦补偿（推荐大多数情况使用）
  // 直接用 soft_sign 乘以你需要补偿的摩擦力大小。
  // 这里的 FRICTION_DYNAMIC_CURRENT 应设定为“让轮子维持最低速转动所需的电流”
  return soft_sign * FRICTION_DYNAMIC_CURRENT;

  /* 方案 B：如果你确实需要区分静摩擦和动摩擦（Stribeck模型简化版）
   * 物理现象：启动瞬间摩擦力大(Static)，动起来后摩擦力变小(Dynamic)
   * 只有在 FRICTION_STATIC_CURRENT 明显大于 FRICTION_DYNAMIC_CURRENT
   * 时才需要用这个
   */
  /*
  float stribeck_effect = (FRICTION_STATIC_CURRENT - FRICTION_DYNAMIC_CURRENT) *
  expf(-(normalized_speed * normalized_speed)); return soft_sign *
  (FRICTION_DYNAMIC_CURRENT + stribeck_effect);
  */
}

/**
 * @brief 力→电流转换（重构版）
 *
 * 处理流程：
 * 1. 基础力→电流转换
 * 2. 速度反馈补偿
 * 3. 摩擦补偿
 * 4. 限幅保护
 */
static void ForceToCurrentConversion(void) {
  // 电机数组（不能使用static const，因为motor_lf等是运行时初始化的）
  DJIMotorInstance *motors[4] = {motor_rf, motor_lf, motor_lb, motor_rb};

  // 预计算转换系数，避免重复计算
  static const float force_to_current = RADIUS_WHEEL / M3508_TORQUE_CONSTANT;

  for (int i = 0; i < 4; i++) {
    /* ===== 步骤1：基础力→电流转换 ===== */
    // 公式：电流 = 力 × 半径 / 转矩常数
    float base_current = wheel_force[i] * force_to_current;

    /* ===== 步骤2：获取实际角速度 ===== */
    // 电机数组顺序：{RF, LF, LB, RB}
    // 电机方向配置：RF=REVERSE, LF=REVERSE, LB=REVERSE, RB=REVERSE
    // REVERSE电机：驱动层已取反，应用层需要再取反回来
    float actual_omega_raw = motors[i]->measure.speed_aps * DEGREE_2_RAD;
    float actual_omega;
    // 所有电机都是REVERSE，统一取反
    actual_omega = -actual_omega_raw;

    /* ===== 步骤3：速度反馈补偿 ===== */
    float speed_feedback =
        CalculateSpeedFeedback(target_wheel_omega[i], actual_omega, i);

    /* ===== 步骤4：摩擦补偿 ===== */
    float friction_comp = CalculateFrictionCompensation(target_wheel_omega[i]);

    /* ===== 步骤5：合成总电流 ===== */
    wheel_current[i] = base_current + speed_feedback + friction_comp;

    /* ===== 步骤6：限幅保护 ===== */
    wheel_current[i] = float_constrain(wheel_current[i], -MAX_WHEEL_CURRENT,
                                       MAX_WHEEL_CURRENT);
  }
}

/* 机器人底盘控制核心任务 */
void ChassisTask() {
#if ENABLE_CHASSIS_SYSID
  // 系统辨识宏开关触发（ENABLE_CHASSIS_SYSID=1时有效）
  ChassisSystemIDSwitch();
#endif

  // 检查系统辨识是否激活
  if (Chassis_SysIDIsActive()) {
    // 系统辨识激活时，完全由辨识任务控制电机
    // 底盘控制任务不干预，直接返回
    return;
  }

  // 后续增加没收到消息的处理(双板的情况)
  // 获取新的控制信息
#ifdef ONE_BOARD
  SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
  chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

  // === 应用遥控器速度增益 ===
  // 从Gimbal接收到的vx/vy是归一化值(-1.0~1.0)
  // 需要乘以增益转换为实际速度(m/s)
  chassis_cmd_recv.vx *= chassis_config.rc.max_linear_speed;
  chassis_cmd_recv.vy *= chassis_config.rc.max_linear_speed;
  // 注意: wz(角速度)由底盘根据模式自动设定，不需要在这里处理

  if (chassis_cmd_recv.chassis_mode ==
      CHASSIS_ZERO_FORCE) { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
    DJIMotorStop(motor_lf);
    DJIMotorStop(motor_rf);
    DJIMotorStop(motor_lb);
    DJIMotorStop(motor_rb);
  } else { // 正常工作
    DJIMotorEnable(motor_lf);
    DJIMotorEnable(motor_rf);
    DJIMotorEnable(motor_lb);
    DJIMotorEnable(motor_rb);
  }

  // 根据控制模式设定旋转速度
  switch (chassis_cmd_recv.chassis_mode) {
  case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
    chassis_cmd_recv.wz = 0;
    last_follow_wz = 0; // 重置滤波状态
    break;

  case CHASSIS_FOLLOW_GIMBAL_YAW: { // 跟随云台,统一力控链路
    // PID输出目标角速度（rad/s）
    // 统一进入力控PID转换为扭矩
    // 输入：角度误差（度）
    // 输出：目标角速度（rad/s）
    // 修复：交换PID参数顺序，使云台右偏时底盘顺时针旋转
    float follow_angular_vel = -PIDCalculate(
        &chassis_follow_pid, chassis_cmd_recv.near_center_error, 0.0f);

    // 直接输出目标角速度（取消一阶滤波）
    chassis_cmd_recv.wz = follow_angular_vel;
    last_follow_wz = chassis_cmd_recv.wz;
    // wz统一为角速度（rad/s），后续通过chassis_torque_pid转换为扭矩
    break;
  }

  case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
    chassis_cmd_recv.wz = chassis_config.kinematics.rotate_speed; // rad/s
    last_follow_wz = chassis_cmd_recv.wz; // 保存当前wz,便于切换后平滑过渡
    break;

  default:
    break;
  }

  // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
  // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
  static float sin_theta, cos_theta;
  cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
  sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
  chassis_vx =
      chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
  chassis_vy =
      chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

  /* ================== 力控策略控制流程 ================== */
  // 参考robowalker的力控思想，实现从速度到电流的完整链路

  // 0. 正运动学解算：计算各轮目标角速度（用于速度内环）
  //    robowalker的关键：提供轮速目标值用于动态补偿
  MecanumKinematicsCalculate();

  // 1. 速度闭环 → 力/扭矩 (力控核心环节1)
  //    速度控制器输出的不是速度参考值，而是需要的合力和合扭矩
  //    统一处理：所有模式的wz（角速度）都通过chassis_torque_pid转换为扭矩
  VelocityToForceControl();

  // 2. 力的动力学逆解算 (力控核心环节2)
  //    将底盘合力分配到各个轮子
  ForceDynamicsInverseResolution();

  // 3. 力→电流转换 + 速度内环 + 摩擦补偿 (力控核心环节3)
  //    robowalker核心：base_current + speed_feedback + friction_comp
  ForceToCurrentConversion();

#if POWER_CONTROLLER_ENABLE
  // 4. 功率限制（必须在发送电机指令前执行）
  // 4.1 获取超级电容数据
  SuperCap_Rx_Data_s cap_rx_data = SuperCapGetData(cap);

  // 4.1.1 通过 VOFA 上位机实时发送超级电容反馈功率（W）
  // 说明：VOFA 走 DMA/IT
  // 时不建议在高频控制环里“每次都发”，否则会打断上一次发送或占满带宽
  static float last_vofa_send_s = 0.0f;
  float now_s = DWT_GetTimeline_s();
  if ((now_s - last_vofa_send_s) >= 0.05f) { // 20Hz
    float vofa_values[1] = {cap_rx_data.chassis_power};
    (void)VofaJustFloatSend(&vofa_sender, vofa_values, 1);
    last_vofa_send_s = now_s;
  }

  // 4.2 更新功率控制器数据（暂时无裁判系统，使用超级电容数据）
  // 手动设定功率限制（根据实际测试调整，单位：W）
  float manual_power_limit = 95.0f;

  // 使用超级电容测量的数据
  float cap_energy_buffer = (float)cap_rx_data.cap_energy; // 电容能量 (0-255)
  float measured_power = cap_rx_data.chassis_power;        // 底盘功率 (W)

  PowerUpdateRefereeData(manual_power_limit, cap_energy_buffer, measured_power);

  // 4.3 更新超级电容在线状态
  // 在线判断：CAN有数据 且 错误码bit7=0（输出未关闭）
  uint8_t cap_online =
      (cap->can_ins->rx_len > 0 && (cap_rx_data.error_code & 0x80) == 0) ? 1
                                                                         : 0;
  PowerUpdateCapData(cap_rx_data.cap_energy, cap_online);

  // 4.4 更新电机反馈数据（用于RLS参数辨识）
  // ⚠️ 量纲统一到输出轴侧（与功率模型k1、k2参数匹配）
  // ⚠️ 参考系统一到"轮子侧"（与wheel_current一致）
  // ⚠️ 数组顺序必须与 motors[] = {RF, LF, LB, RB} 一致（556行）
  // 电机方向配置：RF=REVERSE, LF=REVERSE, LB=REVERSE, RB=REVERSE
  //    所有电机都是REVERSE，需要统一取反
  //    确保pid_output和motor_speeds参考系一致，避免"符号毁灭"导致功率预测失准
  // 输出轴角速度 = 转子角速度 / 减速比（REVERSE电机需加负号）
  float motor_speeds[4] = {
      -motor_rf->measure.speed_aps * DEGREE_2_RAD /
          REDUCTION_RATIO_WHEEL, // RF: REVERSE
      -motor_lf->measure.speed_aps * DEGREE_2_RAD /
          REDUCTION_RATIO_WHEEL, // LF: REVERSE
      -motor_lb->measure.speed_aps * DEGREE_2_RAD /
          REDUCTION_RATIO_WHEEL, // LB: REVERSE
      -motor_rb->measure.speed_aps * DEGREE_2_RAD /
          REDUCTION_RATIO_WHEEL, // RB: REVERSE
  };

  // 使用电机实测电流反馈计算转矩（而非控制指令）
  // 这对RLS参数辨识至关重要：必须使用真实消耗的电流，而非期望指令
  // real_current范围：-16384~16384，需乘以M3508_CMD_TO_CURRENT_COEFF转换为安培
  // ⚠️ 使用输出轴转矩常数（与motor_speeds的输出轴角速度匹配）
  // ⚠️ real_current也需要根据电机方向配置取反，与motor_speeds保持一致
  const float TORQUE_CONSTANT = M3508_TORQUE_CONSTANT; // 0.3 Nm/A（输出轴）
  float motor_torques[4] = {
      -motor_rf->measure.real_current * M3508_CMD_TO_CURRENT_COEFF *
          TORQUE_CONSTANT, // RF: REVERSE
      -motor_lf->measure.real_current * M3508_CMD_TO_CURRENT_COEFF *
          TORQUE_CONSTANT, // LF: REVERSE
      -motor_lb->measure.real_current * M3508_CMD_TO_CURRENT_COEFF *
          TORQUE_CONSTANT, // LB: REVERSE
      -motor_rb->measure.real_current * M3508_CMD_TO_CURRENT_COEFF *
          TORQUE_CONSTANT, // RB: REVERSE
  };
  PowerUpdateMotorFeedback(motor_speeds, motor_torques);

  // 4.5 执行能量环控制和RLS更新
  PowerControllerTask();

  // 4.6 功率限制：对电流进行限幅
  // ⚠️ 量纲说明：
  //   - target_wheel_omega 是目标转子角速度(rad/s)，需转换为输出轴
  //   - motor_speeds 是当前输出轴角速度(rad/s)
  //   - 功率控制器需要统一量纲，都基于输出轴侧
  PowerMotorObj_t motor_objs[4] = {
      {.pid_output = wheel_current[0] / M3508_CMD_TO_CURRENT_COEFF,
       .current_av = motor_speeds[0],
       .target_av = target_wheel_omega[0] / REDUCTION_RATIO_WHEEL,
       .pid_max_output = 16384.0f},
      {.pid_output = wheel_current[1] / M3508_CMD_TO_CURRENT_COEFF,
       .current_av = motor_speeds[1],
       .target_av = target_wheel_omega[1] / REDUCTION_RATIO_WHEEL,
       .pid_max_output = 16384.0f},
      {.pid_output = wheel_current[2] / M3508_CMD_TO_CURRENT_COEFF,
       .current_av = motor_speeds[2],
       .target_av = target_wheel_omega[2] / REDUCTION_RATIO_WHEEL,
       .pid_max_output = 16384.0f},
      {.pid_output = wheel_current[3] / M3508_CMD_TO_CURRENT_COEFF,
       .current_av = motor_speeds[3],
       .target_av = target_wheel_omega[3] / REDUCTION_RATIO_WHEEL,
       .pid_max_output = 16384.0f},
  };
  float limited_output[4];
  PowerGetLimitedOutput(motor_objs, limited_output);

  // 5. 下发电机电流指令（使用功率限制后的值）
  // ⚠️ 数组顺序：limited_output[] = {RF, LF, LB, RB}（与motors[]一致）
  DJIMotorSetRef(motor_rf, limited_output[0]); // RF
  DJIMotorSetRef(motor_lf, limited_output[1]); // LF
  DJIMotorSetRef(motor_lb, limited_output[2]); // LB
  DJIMotorSetRef(motor_rb, limited_output[3]); // RB
#else
  // 5. 下发电机电流指令（无功率限制）
  //    注意：这里直接使用wheel_current数组，单位为安培(A)
  //    需要转换为CAN指令值：cmd = current / M3508_CMD_TO_CURRENT_COEFF
  // ⚠️ 数组顺序：wheel_current[] = {RF, LF, LB, RB}（与motors[]一致）
  DJIMotorSetRef(motor_rf, wheel_current[0] / M3508_CMD_TO_CURRENT_COEFF); // RF
  DJIMotorSetRef(motor_lf, wheel_current[1] / M3508_CMD_TO_CURRENT_COEFF); // LF
  DJIMotorSetRef(motor_lb, wheel_current[2] / M3508_CMD_TO_CURRENT_COEFF); // LB
  DJIMotorSetRef(motor_rb, wheel_current[3] / M3508_CMD_TO_CURRENT_COEFF); // RB
#endif

  // 5. 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
  // LimitChassisOutput();  // 力控策略中已在ForceToCurrentConversion中完成限幅

  // 6. 根据电机的反馈速度和IMU(如果有)计算真实速度
  // EstimateSpeed();

  // // 获取裁判系统数据
  // 建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
  // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue ,
  // 1:red chassis_feedback_data.enemy_color =
  // referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
  // //
  // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
  // chassis_feedback_data.bullet_speed =
  // referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
  // chassis_feedback_data.rest_heat =
  // referee_data->PowerHeatData.shooter_heat0;

  // 推送反馈消息
#ifdef ONE_BOARD
  PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
  CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}
