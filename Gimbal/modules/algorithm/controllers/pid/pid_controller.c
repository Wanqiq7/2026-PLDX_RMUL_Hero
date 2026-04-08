/**
 ******************************************************************************
 * @file pid_controller.c
 * @author Wang Hongxi
 * @author modified by neozng
 * @brief PID控制器实现
 ******************************************************************************
 */
#include "controllers/pid/pid_controller.h"
#include "memory.h"

static void PIDTrapezoidIntegral(PIDInstance *pid) {
  pid->ITerm = pid->ki * ((pid->Err + pid->Last_Err) / 2.0f) * pid->dt;
}

static void PIDChangingIntegrationRate(PIDInstance *pid) {
  if (pid->Err * pid->Iout > 0.0f) {
    if (fabsf(pid->Err) <= pid->CoefB) return;
    if (fabsf(pid->Err) <= (pid->CoefA + pid->CoefB)) {
      pid->ITerm *=
          (pid->CoefA - fabsf(pid->Err) + pid->CoefB) / pid->CoefA;
    } else {
      pid->ITerm = 0.0f;
    }
  }
}

static void PIDIntegralLimit(PIDInstance *pid) {
  const float temp_iout = pid->Iout + pid->ITerm;
  const float temp_output = pid->Pout + pid->Iout + pid->Dout;

  if (fabsf(temp_output) > pid->MaxOut && (pid->Err * pid->Iout > 0.0f)) {
    pid->ITerm = 0.0f;
  }

  if (temp_iout > pid->IntegralLimit) {
    pid->ITerm = 0.0f;
    pid->Iout = pid->IntegralLimit;
  }
  if (temp_iout < -pid->IntegralLimit) {
    pid->ITerm = 0.0f;
    pid->Iout = -pid->IntegralLimit;
  }
}

static void PIDDerivativeOnMeasurement(PIDInstance *pid) {
  pid->Dout = pid->kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

static void PIDDerivativeFilter(PIDInstance *pid) {
  pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
              pid->Last_Dout * pid->Derivative_LPF_RC /
                  (pid->Derivative_LPF_RC + pid->dt);
}

static void PIDOutputFilter(PIDInstance *pid) {
  pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                pid->Last_Output * pid->Output_LPF_RC /
                    (pid->Output_LPF_RC + pid->dt);
}

static void PIDOutputLimit(PIDInstance *pid) {
  if (pid->Output > pid->MaxOut) pid->Output = pid->MaxOut;
  if (pid->Output < -pid->MaxOut) pid->Output = -pid->MaxOut;
}

static void PIDErrorHandle(PIDInstance *pid) {
  if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f) {
    return;
  }

  if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f) {
    pid->ERRORHandler.ERRORCount++;
  } else {
    pid->ERRORHandler.ERRORCount = 0;
  }

  if (pid->ERRORHandler.ERRORCount > 500U) {
    pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
  }
}

void PIDInit(PIDInstance *pid, PID_Init_Config_s *config) {
  memset(pid, 0, sizeof(PIDInstance));
  memcpy(pid, config, sizeof(PID_Init_Config_s));
  DWT_GetDeltaT(&pid->DWT_CNT);
}

void PIDReset(PIDInstance *pid) {
  pid->Measure = 0.0f;
  pid->Last_Measure = 0.0f;
  pid->Err = 0.0f;
  pid->Last_Err = 0.0f;
  pid->Last_ITerm = 0.0f;
  pid->Pout = 0.0f;
  pid->Iout = 0.0f;
  pid->Dout = 0.0f;
  pid->ITerm = 0.0f;
  pid->Output = 0.0f;
  pid->Last_Output = 0.0f;
  pid->Last_Dout = 0.0f;
  pid->Ref = 0.0f;
  pid->dt = 0.0f;
  pid->ERRORHandler.ERRORCount = 0U;
  pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
  DWT_GetDeltaT(&pid->DWT_CNT);
}

float PIDCalculate(PIDInstance *pid, float measure, float ref) {
  if (pid->Improve & PID_ErrorHandle) {
    PIDErrorHandle(pid);
  }

  pid->dt = DWT_GetDeltaT(&pid->DWT_CNT);
  pid->Measure = measure;
  pid->Ref = ref;
  pid->Err = pid->Ref - pid->Measure;

  if (fabsf(pid->Err) > pid->DeadBand) {
    pid->Pout = pid->kp * pid->Err;
    pid->ITerm = pid->ki * pid->Err * pid->dt;
    pid->Dout = pid->kd * (pid->Err - pid->Last_Err) / pid->dt;

    if (pid->Improve & PID_Trapezoid_Intergral) PIDTrapezoidIntegral(pid);
    if (pid->Improve & PID_ChangingIntegrationRate)
      PIDChangingIntegrationRate(pid);
    if (pid->Improve & PID_Derivative_On_Measurement)
      PIDDerivativeOnMeasurement(pid);
    if (pid->Improve & PID_DerivativeFilter) PIDDerivativeFilter(pid);
    if (pid->Improve & PID_Integral_Limit) PIDIntegralLimit(pid);

    pid->Iout += pid->ITerm;
    pid->Output = pid->Pout + pid->Iout + pid->Dout;

    if (pid->Improve & PID_OutputFilter) PIDOutputFilter(pid);

    PIDOutputLimit(pid);
  } else {
    pid->Output = 0.0f;
    pid->ITerm = 0.0f;
  }

  pid->Last_Measure = pid->Measure;
  pid->Last_Output = pid->Output;
  pid->Last_Dout = pid->Dout;
  pid->Last_Err = pid->Err;
  pid->Last_ITerm = pid->ITerm;

  return pid->Output;
}
