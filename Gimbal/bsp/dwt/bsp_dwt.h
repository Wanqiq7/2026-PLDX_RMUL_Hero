/**
 ******************************************************************************
 * @file    bsp_dwt.h
 * @author  Wang Hongxi
 * @author  modified by NeoZng
 * @version V1.2.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _BSP_DWT_H
#define _BSP_DWT_H

#include "main.h"
#include "stdint.h"
#include "bsp_log.h"

typedef struct {
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

/**
 * @brief 该宏用于计算代码段执行时间,单位为秒/s,返回值为 float 类型
 *        首先需要创建一个 float 类型变量用于存储时间间隔
 *        计算得到的时间间隔同时还会通过 RTT 打印到日志终端
 */
#define TIME_ELAPSE(dt, code)                        \
    do                                               \
    {                                                \
        float tstart = DWT_GetTimeline_s();          \
        char dt_str[16];                             \
        code;                                        \
        dt = DWT_GetTimeline_s() - tstart;           \
        Float2Str(dt_str, dt);                       \
        LOGINFO("[DWT] " #dt " = %s s\r\n", dt_str); \
    } while (0)

/**
 * @brief 初始化 DWT,传入参数为 CPU 频率,单位 MHz
 *
 * @param CPU_Freq_mHz c 板为 168MHz,A 板为 180MHz
 */
void DWT_Init(uint32_t CPU_Freq_mHz);

/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s
 *
 * @param cnt_last 上一次调用的时间戳
 * @return float 时间间隔,单位为秒/s
 */
float DWT_GetDeltaT(uint32_t *cnt_last);

/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s,高精度
 *
 * @param cnt_last 上一次调用的时间戳
 * @return double 时间间隔,单位为秒/s
 */
double DWT_GetDeltaT64(uint32_t *cnt_last);

/**
 * @brief 获取当前时间,单位为秒/s,即初始化后的时间
 *
 * @return float 时间
 */
float DWT_GetTimeline_s(void);

/**
 * @brief 获取当前时间,单位为毫秒 ms,即初始化后的时间
 *
 * @return float
 */
float DWT_GetTimeline_ms(void);

/**
 * @brief 获取当前时间,单位为微秒 us,即初始化后的时间
 *
 * @return uint64_t
 */
uint64_t DWT_GetTimeline_us(void);

/**
 * @brief DWT 延时函数,单位为秒/s
 * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
 * @note 禁止在 __disable_irq() 和 __enable_irq() 之间使用 HAL_Delay(),应使用本函数
 *
 * @param Delay 延时时间,单位为秒/s
 */
void DWT_Delay(float Delay);

/**
 * @brief DWT 更新时间轴函数,会被三个 timeline 函数调用
 * @attention 如果长时间不调用 timeline 函数,需要手动调用该函数更新时间轴,
 *            否则 CYCCNT 溢出后定时和时间轴不准确
 */
void DWT_SysTimeUpdate(void);

#endif /* BSP_DWT_H_ */
