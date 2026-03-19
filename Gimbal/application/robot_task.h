/* 注意该文件应只用于任务初始化,只能被 robot.c 包含 */
#pragma once

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

#include "HT04.h"
#include "buzzer.h"
#include "daemon.h"
#include "ins_task.h"
#include "motor_task.h"
#include "referee_task.h"
#include "robot.h"
#include "sysid_task.h"
#include "vision_comm.h"

#include "bsp_log.h"

osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;
osThreadId sysidTaskHandle;

void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartUITASK(void const *argument);
void StartSYSIDTASK(void const *argument);

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit() {
  osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
  insTaskHandle = osThreadCreate(
      osThread(instask),
      NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以 1kHz 的频率执行

  osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 256);
  motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

  osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
  daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

  osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 2048);
  robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

  if (RefereeTaskIsReady()) {
    osThreadDef(uitask, StartUITASK, osPriorityNormal, 0, 512);
    uiTaskHandle = osThreadCreate(osThread(uitask), NULL);
  } else {
    uiTaskHandle = NULL;
    LOGWARNING("[freeRTOS] UI Task skipped: referee not ready");
  }

  osThreadDef(sysidtask, StartSYSIDTASK, osPriorityAboveNormal, 0, 512);
  sysidTaskHandle = osThreadCreate(osThread(sysidtask), NULL);

  HTMotorControlInit(); // 没有注册 HT 电机则不会执行
}

__attribute__((noreturn)) void StartINSTASK(void const *argument) {
  static float ins_start;
  static float ins_dt;
  INS_Init(); // 确保 BMI088 被正确初始化
  LOGINFO("[freeRTOS] INS Task Start");
  for (;;) {
    ins_start = DWT_GetTimeline_ms();
    INS_Task();
    ins_dt = DWT_GetTimeline_ms() - ins_start;
    if (ins_dt > 1) {
      char ins_dt_str[16];
      Float2Str(ins_dt_str, ins_dt);
      LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%s]", ins_dt_str);
    }
    osDelay(1);
  }
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument) {
  static float motor_dt;
  static float motor_start;
  LOGINFO("[freeRTOS] MOTOR Task Start");
  for (;;) {
    motor_start = DWT_GetTimeline_ms();
    MotorControlTask();
    motor_dt = DWT_GetTimeline_ms() - motor_start;
    if (motor_dt > 1) {
      char motor_dt_str[16];
      Float2Str(motor_dt_str, motor_dt);
      LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%s]",
               motor_dt_str);
    }
    osDelay(1);
  }
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument) {
  static float daemon_dt;
  static float daemon_start;
  BuzzerInit();
  LOGINFO("[freeRTOS] Daemon Task Start");
  for (;;) {
    daemon_start = DWT_GetTimeline_ms();
    DaemonTask();
    BuzzerTask();
    daemon_dt = DWT_GetTimeline_ms() - daemon_start;
    if (daemon_dt > 10) {
      char daemon_dt_str[16];
      Float2Str(daemon_dt_str, daemon_dt);
      LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%s]",
               daemon_dt_str);
    }
    osDelay(10);
  }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument) {
  static float robot_dt;
  static float robot_start;
  LOGINFO("[freeRTOS] ROBOT core Task Start");
  for (;;) {
    robot_start = DWT_GetTimeline_ms();
    RobotTask();
    robot_dt = DWT_GetTimeline_ms() - robot_start;
    if (robot_dt > 1) {
      char robot_dt_str[16];
      Float2Str(robot_dt_str, robot_dt);
      LOGERROR("[freeRTOS] ROBOT core Task is being DELAY! dt = [%s]",
               robot_dt_str);
    }
    osDelay(1); // 1kHz: 1ms 周期
  }
}

__attribute__((noreturn)) void StartUITASK(void const *argument) {
  LOGINFO("[freeRTOS] UI Task Start");
  MyUIInit();
  LOGINFO("[freeRTOS] UI Init Done, communication with ref has established");
  for (;;) {
    UITask();   // 每给裁判系统发送一包数据会挂起一次,详见 UITask 的 refereeSend()
    osDelay(1); // 即使没有任何 UI 需要刷新,也挂起一次,防止卡在 UITask 中无法切换
  }
}

__attribute__((noreturn)) void StartSYSIDTASK(void const *argument) {
  static float sysid_start;
  static float sysid_dt;
  LOGINFO("[freeRTOS] System Identification Task Start");

  // 系统辨识任务初始化在 GimbalInit() 中完成,这里等待初始化完成
  osDelay(100); // 等待云台初始化完成

  for (;;) {
    sysid_start = DWT_GetTimeline_ms();
    Gimbal_SysIDTask();
    sysid_dt = DWT_GetTimeline_ms() - sysid_start;
    if (sysid_dt > 1) {
      char sysid_dt_str[16];
      Float2Str(sysid_dt_str, sysid_dt);
      LOGERROR("[freeRTOS] SYSID Task is being DELAY! dt = [%s]",
               sysid_dt_str);
    }
    osDelay(1);
  }
}
