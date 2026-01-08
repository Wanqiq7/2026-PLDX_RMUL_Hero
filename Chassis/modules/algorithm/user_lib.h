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
// 閼汇儴绻嶇粻妤呪偓鐔峰娑撳秴顧�,閸欘垯浜掓担璺ㄦ暏q31娴狅絾娴沠32,娴ｅ棙妲哥划鎯у娴兼岸妾锋担锟�
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
 * 鏉╂柨娲栨稉鈧崸妤€鍏遍崙鈧惃鍕敶閿燂拷?,娑撳秷绻冩禒宥囧姧闂団偓鐟曚礁宸遍崚鎯版祮閿燂拷?娑撹桨缍橀棁鈧憰浣烘畱缁鐎�
 *
 * @param size 閸掑棝鍘ゆ径褍鐨�
 * @return void*
 */
void *zmalloc(size_t size);

// 閿熸枻鎷烽敓鍔尅鎷烽敓鏂ゆ嫹
float Sqrt(float x);
// 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
float abs_limit(float num, float Limit);
// 閿熷彨鏂嚖鎷烽敓鏂ゆ嫹浣�
float sign(float value);
// 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
float float_deadband(float Value, float minValue, float maxValue);
// 閿熺潾鍑ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
float float_constrain(float Value, float minValue, float maxValue);
// 閿熺潾鍑ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
// 寰敓鏂ゆ嫹閿熺潾鍑ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
float loop_float_constrain(float Input, float minValue, float maxValue);
// 閿熻璁规嫹 閿熸枻鎷烽敓鐫嚖鎷� 180 ~ -180
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

// 鏂滃潯杞鍒掑叕鍏辨帴鍙�
float SoftRamp(float target, float current, float reserved_zero, float accel,
               float decel, float brake_decel, float dt);

#endif
