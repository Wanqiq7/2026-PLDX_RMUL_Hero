/**
 ******************************************************************************
 * @file	 user_lib.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _USER_LIB_H
#define _USER_LIB_H

#include "arm_math.h"
#include "cmsis_os.h"
#include "main.h"
#include "stdint.h"
#include "stm32f407xx.h"

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

#define msin(x) (arm_sin_f32(x))
#define mcos(x) (arm_cos_f32(x))

typedef arm_matrix_instance_f32 mat;
// 矩阵运算宏定义，使用ARM DSP库加速
// 注意：矩阵存储格式为行优先，索引范围0-31行，0-32列
#define MatAdd arm_mat_add_f32
#define MatSubtract arm_mat_sub_f32
#define MatMultiply arm_mat_mult_f32
#define MatTranspose arm_mat_trans_f32
#define MatInverse arm_mat_inverse_f32
void MatInit(mat *m, uint8_t row, uint8_t col);

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif

#define VAL_LIMIT(val, min, max)                                               \
  do {                                                                         \
    if ((val) <= (min)) {                                                      \
      (val) = (min);                                                           \\\n    } else if ((val) >= (max)) {                                               \
      (val) = (max);                                                           \
    }                                                                          \
  } while (0)

#define ANGLE_LIMIT_360(val, angle)                                            \
  do {                                                                         \
    (val) = (angle) - (int)(angle);                                            \
    (val) += (int)(angle) % 360;                                               \
  } while (0)

#define ANGLE_LIMIT_360_TO_180(val)                                            \
  do {                                                                         \
    if ((val) > 180)                                                           \
      (val) -= 360;                                                            \
  } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

/**
 * @brief
 * 特殊的malloc函数，使用FreeRTOS的内存分配
 * 如果系统未启用FreeRTOS，则使用标准malloc
 *
 * @param size 需要分配的内存大小（字节）
 * @return void*
 */
void *zmalloc(size_t size);

// 快速平方根计算（使用ARM DSP库）
float Sqrt(float x);
// 带限幅的绝对值函数
float abs_limit(float num, float Limit);
// 符号函数，返回数值的符号
float sign(float value);
// 浮点数死区函数
float float_deadband(float Value, float minValue, float maxValue);
// 浮点数约束函数
float float_constrain(float Value, float minValue, float maxValue);
// 整型约束函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
// 循环浮点数约束函数（适用于角度等周期性数值）
float loop_float_constrain(float Input, float minValue, float maxValue);
// 角度格式化函数，将角度限制在 -180 ~ 180 度范围内
float theta_format(float Ang);

int float_rounding(float raw);

float *Norm3d(float *v);

float NormOf3d(float *v);

void Cross3d(float *v1, float *v2, float *res);

float Dot3d(float *v1, float *v2);

float AverageFilter(float new_data, float *buf, uint8_t len);

uint16_t LowPassFilter(uint16_t Out, float K);

int16_t sign_with_deadband(float value, float deadband);

#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

#endif
// 循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue) {
  if (maxValue < minValue) {
    return Input;
  }

  if (Input > maxValue) {
    float len = maxValue - minValue;
    while (Input > maxValue) {
      Input -= len;
    }
  } else if (Input < minValue) {
    float len = maxValue - minValue;
    while (Input < minValue) {
      Input += len;
    }
  }
  return Input;
}

// 将角度限制在 -PI~PI 范围内（弧度制）

// 或者限制在 -180~180 范围内（角度制）
float theta_format(float Ang) {
  return loop_float_constrain(Ang, -180.0f, 180.0f);
}

int float_rounding(float raw) {
  static int integer;
  static float decimal;
  integer = (int)raw;
  decimal = raw - integer;
  if (decimal > 0.5f)
    integer++;
  return integer;
}

// 三维向量归一化
float *Norm3d(float *v) {
  float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  v[0] /= len;
  v[1] /= len;
  v[2] /= len;
  return v;
}

// 三维向量模长
float NormOf3d(float *v) {
  return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 三维向量叉乘：v1 x v2
void Cross3d(float *v1, float *v2, float *res) {
  res[0] = v1[1] * v2[2] - v1[2] * v2[1];
  res[1] = v1[2] * v2[0] - v1[0] * v2[2];
  res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 三维向量点乘
float Dot3d(float *v1, float *v2) {
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// 滑动平均滤波器，新数据加入buffer并计算平均值
// buffer需要提前分配内存，len为buffer长度
float AverageFilter(float new_data, float *buf, uint8_t len) {
  float sum = 0;
  for (uint8_t i = 0; i < len - 1; i++) {
    buf[i] = buf[i + 1];
    sum += buf[i];
  }
  buf[len - 1] = new_data;
  sum += new_data;
  return sum / len;
}

void MatInit(mat *m, uint8_t row, uint8_t col) {
  m->numCols = col;
  m->numRows = row;
  m->pData = (float *)zmalloc(row * col * sizeof(float));
}

uint16_t LowPassFilter(uint16_t Out, float K) {
  // 保存上一次的输出值
  static uint16_t Last_Out = 0;
  // 一阶低通滤波
  Out = K * Out + (1 - K) * Last_Out;
  // 更新上一次输出值
  Last_Out = Out;
  return Out;
}

/**
 * @brief 符号函数 (带死区)
 * @param value 输入值
 * @param deadband 死区范围
 * @return -1, 0, or 1
 * @note 当输入值在[-deadband, deadband]范围内时, 返回0;
 * 否则返回输入值的符号，用于底盘阻力补偿
 */
int16_t sign_with_deadband(float value, float deadband) {
  if (value > deadband)
    return 1;
  else if (value < -deadband)
    return -1;
  else
    return 0;
}

/**
 * @brief 浮点数低通滤波器
 * @param new_value 新的输入值
 * @param K 滤波系数 (0.0~1.0), 越小滤波效果越强
 * @param last_value 上一次的输出值
 * @return 滤波后的输出值
 */
float LowPassFilter_Float(float new_value, float K, float *last_value) {
  float out = K * new_value + (1.0f - K) * (*last_value);
  *last_value = out;
  return out;
}

/**
 * @brief  斜坡轨迹规划器核心函数 (单位: CAN指令值),用于键盘控制
 * @param  target      目标指令值
 * @param  current     当前规划器的输出指令值 (上一周期的结果)
 * @param  accel       加速度 (指令值/秒)
 * @param  decel       减速度 (指令值/秒)
 * @param  brake_decel 反向制动减速度 (指令值/秒)
 * @param  dt          控制周期 (s)
 * @retval             当前周期规划好的输出指令值
 */
static float SoftRamp_CMD(float target, float current, float accel, float decel,
                          float brake_decel, float dt) {
  float ramp_out = current;
  float error = target - current;

  // 判断是加速、减速还是反向制动
  if (target * current >= 0) // 目标值和当前值同号
  {
    if (fabsf(target) > fabsf(current)) // 加速
    {
      ramp_out += accel * dt * sign(error);
    } else // 减速
    {
      ramp_out += decel * dt * sign(error);
    }
  } else // 目标值和当前值异号 (反向制动)
  {
    ramp_out += brake_decel * dt * sign(error);
  }

  // 限制规划值不能超过目标值
  if (sign(error) > 0) {
    ramp_out = float_constrain(ramp_out, current, target);
  } else {
    ramp_out = float_constrain(ramp_out, target, current);
  }

  // 最终再对输出进行一次总范围的限制
  return float_constrain(ramp_out, -16384.0f, 16384.0f);
}

float Sqrt(float x) {
  if (x <= 0.0f)
    return 0.0f;
  float out = 0.0f;
  arm_sqrt_f32(x, &out);
  return out;
}

float float_constrain(float Value, float minValue, float maxValue) {
  if (Value < minValue)
    return minValue;
  if (Value > maxValue)
    return maxValue;
  return Value;
}
