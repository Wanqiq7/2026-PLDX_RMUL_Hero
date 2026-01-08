#include "gimbal.h"
#include "bmi088.h"
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "general_def.h"
#include "ins_task.h"
#include "message_center.h"
#include "robot_def.h"
#include "sysid_task.h"
#include <math.h>

/* ==================== 调参比例宏 ==================== */
// 可通过修改以下比例对Yaw轴PID进行等比例放大/缩小
#define YAW_ANGLE_PID_SCALE 50.0f // 角度环缩放系数（例如 10.0f 放大十倍）
#define YAW_SPEED_PID_SCALE 50.0f // 速度环缩放系数（例如 1e7 放大一千万倍）

/* ==================== 系统辨识任务交互 ==================== */
// 系统辨识任务通过消息中心与云台任务交互
static Publisher_t *sysid_pub;  // 系统辨识数据发布者
static Subscriber_t *sysid_sub; // 系统辨识指令订阅者
/* ================================================================== */

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;

// ==================== Ozone调试变量暴露 ====================
// 将电机指针暴露为全局变量，方便调试器访问
// volatile防止编译器优化，去掉static让符号表可见
volatile DJIMotorInstance *debug_yaw_motor = NULL;
volatile DJIMotorInstance *debug_pitch_motor = NULL;
// ===========================================================

// 云台前馈变量：电流/力矩前馈（指针在初始化中绑定到电机控制器）
static float gimbal_yaw_cur_ff = 0.0f;
static float gimbal_pitch_cur_ff = 500.0f; // Pitch轴前馈固定值

/* ==================== 系统辨识相关函数已移至独立任务 ==================== */
// 系统辨识功能现在由独立的 StartSYSIDTASK 任务执行，见 robot_task.h
/* ================================================================== */

static Publisher_t *gimbal_pub;  // 云台应用消息发布者(云台反馈cmd)
static Subscriber_t *gimbal_sub; // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

// static BMI088Instance *bmi088; // 云台IMU
void GimbalInit() {
  /*******************************位置控制初始化*******************************/
  gimba_IMU_data =
      INS_Init(); // IMU先初始化,获取姿态数据指针赋值给yaw电机的其他数据来源
  // YAW - 位置控制配置（角度环+速度环串级）
  Motor_Init_Config_s yaw_config = {
      .can_init_config =
          {
              .can_handle = &hcan2,
              .tx_id = 2,
          },
      .controller_param_init_config =
          {
              .angle_PID =
                  {
                      .Kp = 24.5f,                 // 参考旧代码手控模式
                      .Ki = 0.92f,                 // 参考旧代码手控模式
                      .Kd = 3.2f,                  // 参考旧代码手控模式
                      .Derivative_LPF_RC = 0.027f, // 微分滤波器，防止高频噪声
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                                 PID_DerivativeFilter |
                                 PID_Derivative_On_Measurement,
                      .IntegralLimit =
                          800, // 旧代码I限幅±19999，按比例折算约800
                      .DeadBand = 0.05,
                      .MaxOut = 800, // 提高输出限幅，确保能克服静摩擦
                  },
              .speed_PID =
                  {
                      .Kp =
                          62.5f, // 参考旧代码世界角补偿Kp=21，放大约2.4倍适配GM6020
                      .Ki = 1.5f, // 参考旧代码有效Ki≈0.0095，放大约50倍
                      .Kd = 0.0f, // 速度环一般不用微分，避免噪声放大
                      .Derivative_LPF_RC = 0.002f, // 微分滤波器（虽然Kd=0）
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                                 PID_DerivativeFilter,
                      .IntegralLimit =
                          3000,        // 旧代码I限幅±9999，按比例折算约3000
                      .MaxOut = 20000, // GM6020最大电压指令范围，保持充足裕度
                  },
              .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
              // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
              .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
              .current_feedforward_ptr =
                  &gimbal_yaw_cur_ff, // 开启后可用于一般重力/扰动补偿
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = OTHER_FEED,
              .speed_feedback_source = OTHER_FEED,
              .outer_loop_type = ANGLE_LOOP, // 外环为角度环
              .close_loop_type =
                  SPEED_LOOP | ANGLE_LOOP,             // 同时开启角度环和速度环
              .feedforward_flag = CURRENT_FEEDFORWARD, // 使能电流前馈通道
              .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
          },
      .motor_type = GM6020};
  // PITCH - 位置控制配置（角度环+速度环串级）
  Motor_Init_Config_s pitch_config = {
      .can_init_config =
          {
              .can_handle = &hcan2,
              .tx_id = 5,
          },
      .controller_param_init_config =
          {
              .angle_PID =
                  {
                      .Kp = 12.0f,                 // 大幅降低，消除抖动
                      .Ki = 1.5f,                  // 暂时关闭积分
                      .Kd = 3.2f,                  // 大幅降低微分
                      .Derivative_LPF_RC = 0.015f, // 增强滤波
                      .Improve = PID_DerivativeFilter |
                                 PID_Derivative_On_Measurement |
                                 PID_Integral_Limit | PID_Trapezoid_Intergral,
                      .IntegralLimit = 500,
                      .DeadBand = 0.1,
                      .MaxOut = 800, // 降低输出限制
                  },
              .speed_PID =
                  {
                      .Kp = 50.0f,                // 大幅降低，消除抖动
                      .Ki = 1.5f,                 // 暂时关闭积分，避免振荡
                      .Kd = 0.0f,                 // 先关闭微分，等稳定后再加
                      .Derivative_LPF_RC = 0.02f, // 增强滤波（虽然Kd=0）
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                                 PID_DerivativeFilter |
                                 PID_Derivative_On_Measurement,
                      .IntegralLimit = 1500,
                      .MaxOut = 20000, // 降低输出限制
                  },
              .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
              // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
              .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[0]),
              .current_feedforward_ptr =
                  &gimbal_pitch_cur_ff, // Pitch轴重力补偿注入点
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = OTHER_FEED,
              .speed_feedback_source = OTHER_FEED,
              .outer_loop_type = ANGLE_LOOP, // 外环为角度环
              .close_loop_type =
                  SPEED_LOOP | ANGLE_LOOP,          // 同时开启角度环和速度环
              .feedforward_flag = FEEDFORWARD_NONE, // 使能电流前馈通道
              .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
          },
      .motor_type = M3508,
  };
  /******************************************************************/
  // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
  yaw_motor = DJIMotorInit(&yaw_config);
  pitch_motor = DJIMotorInit(&pitch_config);

  // 绑定调试指针，方便Ozone实时监控
  debug_yaw_motor = yaw_motor;
  debug_pitch_motor = pitch_motor;

  gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
  gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

  // 系统辨识任务已移除（底盘板不需要云台辨识）
  // 底盘系统辨识使用独立的Chassis_SysIDTask
  // sysid_pub = PubRegister("sysid_cmd", sizeof(SysID_Ctrl_Cmd_s));
  // sysid_sub = SubRegister("sysid_feedback", sizeof(SysID_Feedback_s));
  // SysIDTaskInit(yaw_motor, pitch_motor, gimba_IMU_data);
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask() {
  // 获取云台控制数据
  // 后续增加没收到数据的处理
  SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

  // @todo:现在已经不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
  // 根据控制模式进行电机反馈切换和过零,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
  switch (gimbal_cmd_recv.gimbal_mode) {
  // 停止
  case GIMBAL_ZERO_FORCE: {
    DJIMotorStop(yaw_motor);
    DJIMotorStop(pitch_motor);
    // 系统辨识任务已移除
    // static SysID_Ctrl_Cmd_s sysid_stop_cmd1;
    // sysid_stop_cmd1.enable = 0;
    // PubPushMessage(sysid_pub, &sysid_stop_cmd1);
    break;
  }
  // 使用陀螺仪的反饋,底盘根据yaw电机的offset跟随云台或视觉模式采用
  case GIMBAL_GYRO_MODE: { // 后续只保留此模式
    DJIMotorEnable(yaw_motor);
    DJIMotorEnable(pitch_motor);
    DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
    DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
    DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
    DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
    DJIMotorSetRef(
        yaw_motor,
        gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
    DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
    // 系统辨识任务已移除
    // static SysID_Ctrl_Cmd_s sysid_stop_cmd2;
    // sysid_stop_cmd2.enable = 0;
    // PubPushMessage(sysid_pub, &sysid_stop_cmd2);
    break;
  }
  // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调试云台姿态(英雄射箭等)/能量机关
  case GIMBAL_FREE_MODE: { // 后续删除,或加入云台追底盘的跟随模式(响应速度更快)
    DJIMotorEnable(yaw_motor);
    DJIMotorEnable(pitch_motor);
    // 位置控制模式：设置目标角度
    DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
    DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
    DJIMotorSetRef(
        yaw_motor,
        gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好过零和单圈
    DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
    // 系统辨识任务已移除
    // static SysID_Ctrl_Cmd_s sysid_stop_cmd3;
    // sysid_stop_cmd3.enable = 0;
    // PubPushMessage(sysid_pub, &sysid_stop_cmd3);
    break;
  }

  default: {
    // 系统辨识任务已移除
    break;
  }
  }
  // 设置反馈数据,主要是imu和yaw的ecd
  gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;

  gimbal_feedback_data.yaw_motor_single_round_angle =
      yaw_motor->measure.angle_single_round;
  // 推送消息
  PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
