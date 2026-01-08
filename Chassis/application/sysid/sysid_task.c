/**
 * @file sysid_task.c
 * @brief 底盘轮速系统辨识独立任务，以1kHz频率执行方波阶跃测试
 * @note 用于辨识底盘单轮电机的等效惯量和阻尼系数，为LQR力控提供参数
 * @author RoboMaster EC Team
 * @date 2025-01-04
 */

#include "sysid_task.h"
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "general_def.h"
#include "message_center.h"
#include "robot_def.h"
#include <string.h>

/* ==================== 底盘系统辨识方波阶跃信号参数定义 ==================== */
#define CHASSIS_SYS_ID_STEP_AMPLITUDE                                          \
  6000.0f // 阶跃信号幅值（电流指令，CAN值：-16384~16384）
#define CHASSIS_SYS_ID_STEP_INTERVAL 2.0f   // 每个阶跃的持续时间 [s]
#define CHASSIS_SYS_ID_TOTAL_DURATION 20.0f // 总测试持续时间 [s]

// 注意：目标电机在chassis.c中通过消息传递，此宏已废弃
// #define CHASSIS_SYS_ID_TARGET_MOTOR 0

/* ==================== 私有变量 ==================== */
static Publisher_t *sysid_pub;  // 系统辨识反馈发布者
static Subscriber_t *sysid_sub; // 系统辨识指令订阅者

// 底盘四个电机实例指针
static DJIMotorInstance *sysid_motor_lf = NULL;
static DJIMotorInstance *sysid_motor_rf = NULL;
static DJIMotorInstance *sysid_motor_lb = NULL;
static DJIMotorInstance *sysid_motor_rb = NULL;

// 系统辨识数据实例
static Chassis_SysID_Feedback_s sysid_data = {0};

// DWT时间戳，用于精确测量任务实际周期
static uint32_t step_tick_last = 0;

// 任务状态标志
static uint8_t sysid_initialized = 0;
static uint8_t sysid_active = 0;

/* ==================== 私有函数声明 ==================== */
static float Chassis_SystemID_GenerateStepSignal(void);
static void Chassis_SystemID_Reset(void);
static void Chassis_SystemID_ConfigMotors(uint8_t target_motor_index);

/* ==================== 函数实现 ==================== */

/**
 * @brief 底盘系统辨识任务初始化
 * @param motor_lf 左前轮电机实例指针
 * @param motor_rf 右前轮电机实例指针
 * @param motor_lb 左后轮电机实例指针
 * @param motor_rb 右后轮电机实例指针
 */
void Chassis_SysIDTaskInit(DJIMotorInstance *motor_lf,
                           DJIMotorInstance *motor_rf,
                           DJIMotorInstance *motor_lb,
                           DJIMotorInstance *motor_rb) {
  // 保存电机指针
  sysid_motor_lf = motor_lf;
  sysid_motor_rf = motor_rf;
  sysid_motor_lb = motor_lb;
  sysid_motor_rb = motor_rb;

  // 注册消息中心话题
  sysid_pub =
      PubRegister("chassis_sysid_feedback", sizeof(Chassis_SysID_Feedback_s));
  sysid_sub =
      SubRegister("chassis_sysid_cmd", sizeof(Chassis_SysID_Ctrl_Cmd_s));

  // 初始化数据结构
  memset(&sysid_data, 0, sizeof(Chassis_SysID_Feedback_s));

  sysid_initialized = 0;
  sysid_active = 0;
}

/**
 * @brief 生成方波阶跃信号用于底盘轮速电机系统辨识
 * @return float 当前时刻的方波信号值（CAN指令值）
 *
 * @note 本函数使用DWT精确测量实际任务周期，自动适配不同频率的调用
 */
static float Chassis_SystemID_GenerateStepSignal(void) {
  // 1) 使用DWT精确测量实际任务周期
  float dt = DWT_GetDeltaT(&step_tick_last);

  // 异常保护：防止首次调用或异常情况下dt过大
  if (dt <= 0.0f || dt > 0.05f) {
    dt = 0.001f; // 默认1ms，对应1kHz控制频率
    if (dt > 0.05f) {
      DWT_GetDeltaT(&step_tick_last); // 再次调用以同步时间戳
    }
  }

  // 诊断：记录实际dt和计算频率
  sysid_data.call_counter++;
  sysid_data.actual_dt = dt;
  if (dt > 0.0001f) {
    sysid_data.task_freq = 1.0f / dt;
  }

  // 2) 累加实际运行时间
  sysid_data.time_elapsed += dt;

  // 3) 检查是否超过总测试时间
  if (sysid_data.time_elapsed >= CHASSIS_SYS_ID_TOTAL_DURATION) {
    sysid_data.is_finished = 1; // 辨识结束，标记完成
    return 0.0f;
  }

  // 4) 计算当前应处于第几个阶跃周期（整数除法）
  uint32_t current_step_index =
      (uint32_t)(sysid_data.time_elapsed / CHASSIS_SYS_ID_STEP_INTERVAL);

  // 5) 根据周期索引的奇偶性决定输出正向还是反向
  if (current_step_index % 2 == 0) {
    sysid_data.step_state = 0; // 正向
    return CHASSIS_SYS_ID_STEP_AMPLITUDE;
  } else {
    sysid_data.step_state = 1; // 反向
    return -CHASSIS_SYS_ID_STEP_AMPLITUDE;
  }
}

/**
 * @brief 系统辨识数据重置函数
 * @note 在进入阶跃辨识模式时调用，重置辨识数据和DWT时间基
 */
static void Chassis_SystemID_Reset(void) {
  memset(&sysid_data, 0, sizeof(Chassis_SysID_Feedback_s));

  // 预热DWT时间戳，避免首次调用时dt异常大
  DWT_GetDeltaT(&step_tick_last);
}

/**
 * @brief 配置底盘电机为开环/闭环模式
 * @note 将目标辨识电机设为开环，其他电机停止
 * @param target_motor_index 目标电机索引（0-lf, 1-rf, 2-lb, 3-rb）
 */
static void Chassis_SystemID_ConfigMotors(uint8_t target_motor_index) {
  DJIMotorInstance *motors[4] = {sysid_motor_lf, sysid_motor_rf, sysid_motor_lb,
                                 sysid_motor_rb};

  for (int i = 0; i < 4; i++) {
    if (i == target_motor_index) {
      // 目标电机：配置为开环电流控制
      DJIMotorEnable(motors[i]);
      motors[i]->motor_settings.close_loop_type = OPEN_LOOP;
      motors[i]->motor_settings.outer_loop_type = OPEN_LOOP;
      motors[i]->motor_settings.feedforward_flag = FEEDFORWARD_NONE;
    } else {
      // 其他电机：停止
      DJIMotorStop(motors[i]);
    }
  }
}

/**
 * @brief 底盘系统辨识任务主循环
 * @note 在1kHz FreeRTOS任务中调用
 */
void Chassis_SysIDTask(void) {
  // 检查初始化状态
  if (sysid_motor_lf == NULL || sysid_motor_rf == NULL ||
      sysid_motor_lb == NULL || sysid_motor_rb == NULL) {
    return; // 未初始化，直接返回
  }

  // 获取控制指令
  static Chassis_SysID_Ctrl_Cmd_s sysid_cmd;
  SubGetMessage(sysid_sub, &sysid_cmd);

  // 检查是否使能辨识
  if (!sysid_cmd.enable) {
    // 如果之前在运行，现在停止了，需要重置状态
    if (sysid_initialized) {
      sysid_initialized = 0;
      sysid_active = 0;

      // 恢复所有电机为正常控制模式
      DJIMotorInstance *motors[4] = {sysid_motor_lf, sysid_motor_rf,
                                     sysid_motor_lb, sysid_motor_rb};
      for (int i = 0; i < 4; i++) {
        DJIMotorEnable(motors[i]);
        DJIMotorChangeFeed(motors[i], SPEED_LOOP, MOTOR_FEED);
      }
    }
    return; // 未使能，直接返回
  }

  // 首次进入辨识模式：重置数据 + 配置电机
  if (!sysid_initialized) {
    Chassis_SystemID_Reset();
    Chassis_SystemID_ConfigMotors(
        sysid_cmd.target_motor); // 使用消息中的目标电机
    sysid_initialized = 1;
    sysid_active = 1;
  }

  // 如果辨识完成，停止激励信号并停止电机
  if (sysid_data.is_finished) {
    sysid_initialized = 0;
    sysid_active = 0;

    // 停止所有电机（确保完全停止）
    DJIMotorStop(sysid_motor_lf);
    DJIMotorStop(sysid_motor_rf);
    DJIMotorStop(sysid_motor_lb);
    DJIMotorStop(sysid_motor_rb);

    // 发布最终数据
    PubPushMessage(sysid_pub, &sysid_data);

    // 完成后不再执行
    return;
  }

  // 生成方波阶跃信号（电流指令）
  sysid_data.step_input = Chassis_SystemID_GenerateStepSignal();

  // 选择目标电机（使用消息中的target_motor）
  DJIMotorInstance *target_motor = NULL;
  switch (sysid_cmd.target_motor) {
  case 0:
    target_motor = sysid_motor_lf;
    break;
  case 1:
    target_motor = sysid_motor_rf;
    break;
  case 2:
    target_motor = sysid_motor_lb;
    break;
  case 3:
    target_motor = sysid_motor_rb;
    break;
  }

  // 更新目标电机电流指令，采集速度反馈
  if (target_motor != NULL) {
    target_motor->motor_controller.pid_ref = sysid_data.step_input;
    // 采集轮速反馈（单位：rad/s）
    sysid_data.motor_output = target_motor->measure.speed_aps * DEGREE_2_RAD;
  }

  // 发布辨识反馈数据
  PubPushMessage(sysid_pub, &sysid_data);
}

/**
 * @brief 检查底盘系统辨识任务是否激活
 * @return 1-激活，0-未激活
 */
uint8_t Chassis_SysIDIsActive(void) { return sysid_active; }
