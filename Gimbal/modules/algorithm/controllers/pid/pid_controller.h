/**
 ******************************************************************************
 * @file pid_controller.h
 * @author Wang Hongxi
 * @version V1.1.3
 * @date 2021/7/3
 * @brief PID控制器接口定义
 ******************************************************************************
 */
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "main.h"
#include "utils/math/arm_math_compat.h"
#include "bsp_dwt.h"
#include "memory.h"
#include "stdint.h"
#include "stdlib.h"
#include <math.h>

// PID 优化环节使能标志位
typedef enum {
  PID_IMPROVE_NONE = 0b00000000,
  PID_Integral_Limit = 0b00000001,
  PID_Derivative_On_Measurement = 0b00000010,
  PID_Trapezoid_Intergral = 0b00000100,
  PID_Proportional_On_Measurement = 0b00001000,
  PID_OutputFilter = 0b00010000,
  PID_ChangingIntegrationRate = 0b00100000,
  PID_DerivativeFilter = 0b01000000,
  PID_ErrorHandle = 0b10000000,
} PID_Improvement_e;

typedef enum errorType_e {
  PID_ERROR_NONE = 0x00U,
  PID_MOTOR_BLOCKED_ERROR = 0x01U
} ErrorType_e;

typedef struct {
  uint64_t ERRORCount;
  ErrorType_e ERRORType;
} PID_ErrorHandler_t;

typedef struct {
  float kp;
  float ki;
  float kd;
  float MaxOut;
  float DeadBand;

  PID_Improvement_e Improve;
  float IntegralLimit;
  float CoefA;
  float CoefB;
  float Output_LPF_RC;
  float Derivative_LPF_RC;

  float Measure;
  float Last_Measure;
  float Err;
  float Last_Err;
  float Last_ITerm;

  float Pout;
  float Iout;
  float Dout;
  float ITerm;

  float Output;
  float Last_Output;
  float Last_Dout;

  float Ref;

  uint32_t DWT_CNT;
  float dt;

  PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

typedef struct {
  float kp;
  float ki;
  float kd;
  float MaxOut;
  float DeadBand;

  PID_Improvement_e Improve;
  float IntegralLimit;
  float CoefA;
  float CoefB;
  float Output_LPF_RC;
  float Derivative_LPF_RC;
} PID_Init_Config_s;

void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);
float PIDCalculate(PIDInstance *pid, float measure, float ref);
void PIDReset(PIDInstance *pid);

#endif // PID_CONTROLLER_H
