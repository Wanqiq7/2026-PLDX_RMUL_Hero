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
// 鑻ヨ繍绠楅€熷害涓嶅,鍙互浣跨敤q31浠ｆ浛f32,浣嗘槸绮惧害浼氶檷浣�
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
      (val) = (min);                                                           \
    } else if ((val) >= (max)) {                                               \
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
 * 杩斿洖涓€鍧楀共鍑€鐨勫唴锟�?,涓嶈繃浠嶇劧闇€瑕佸己鍒惰浆锟�?涓轰綘闇€瑕佺殑绫诲瀷
 *
 * @param size 鍒嗛厤澶у皬
 * @return void*
 */
void *zmalloc(size_t size);

// 锟斤拷锟劫匡拷锟斤拷
float Sqrt(float x);
// 锟斤拷锟斤拷锟斤拷锟斤拷
float abs_limit(float num, float Limit);
// 锟叫断凤拷锟斤拷位
float sign(float value);
// 锟斤拷锟斤拷锟斤拷锟斤拷
float float_deadband(float Value, float minValue, float maxValue);
// 锟睫凤拷锟斤拷锟斤拷
float float_constrain(float Value, float minValue, float maxValue);
// 锟睫凤拷锟斤拷锟斤拷
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
// 循锟斤拷锟睫凤拷锟斤拷锟斤拷
float loop_float_constrain(float Input, float minValue, float maxValue);
// 锟角讹拷 锟斤拷锟睫凤拷 180 ~ -180
float theta_format(float Ang);

int float_rounding(float raw);

float *Norm3d(float *v);

float NormOf3d(float *v);

void Cross3d(float *v1, float *v2, float *res);

float Dot3d(float *v1, float *v2);

float AverageFilter(float new_data, float *buf, uint8_t len);

uint16_t LowPassFilter(uint16_t Out, float K);

int16_t sign_with_deadband(float value, float deadband);

float LowPassFilter_Float(float new_value, float K, float *last_value);

#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

// 斜坡软规划公共接口
float SoftRamp(float target, float current, float reserved_zero, float accel,
               float decel, float brake_decel, float dt);

/* ==================== 矩阵运算辅助函数 (用于LQR等高级控制)
 * ==================== */

/**
 * @brief 2x2矩阵乘以2维向量
 * @param mat_2x2 2x2矩阵，按行优先存储 [a11, a12, a21, a22]
 * @param vec_2   2维向量 [x1, x2]
 * @param result  结果向量 [y1, y2]
 * @note  计算 result = mat_2x2 * vec_2
 *        用于LQR控制律计算: u = -K*x
 */
void Mat2x2_Mult_Vec2(const float mat_2x2[4], const float vec_2[2],
                      float result[2]);

/**
 * @brief 计算2维向量的内积
 * @param vec1 向量1 [x1, x2]
 * @param vec2 向量2 [y1, y2]
 * @return float 内积结果 = x1*y1 + x2*y2
 * @note  用于状态反馈增益计算
 */
float Vec2_DotProduct(const float vec1[2], const float vec2[2]);

/**
 * @brief 向量归一化 (限制到[-limit, limit])
 * @param vec   输入向量
 * @param n     向量维度
 * @param limit 限制范围
 * @note  用于LQR状态量限幅
 */
void Vec_Constrain(float *vec, uint8_t n, float limit);

/**
 * @brief 角度归一化到 [-π, π]
 * @param angle 输入角度 [rad]
 * @return float 归一化后的角度 [rad]
 * @note  用于LQR角度误差计算，处理角度跳变问题
 */
float AngleNormalize(float angle);

#endif
