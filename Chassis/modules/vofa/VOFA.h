#ifndef VOFA_H
#define VOFA_H

#include <stdint.h>

/**
 * @brief VOFA+ JustFloat 发送模式
 * @note  该枚举不与 BSP 枚举做“数值暗绑定”，在实现层做显式映射以避免维护风险。
 */
typedef enum
{
    VOFA_TRANSFER_BLOCKING = 0,
    VOFA_TRANSFER_IT,
    VOFA_TRANSFER_DMA,
} VofaTransferMode_e;

/**
 * @brief VOFA 发送适配层：让 VOFA 模块不直接依赖 BSP/驱动类型
 *
 * 设计意图：
 * - VOFA 模块只负责 JustFloat 打包，不感知 USARTInstance / HAL / FreeRTOS 等具体实现。
 * - 上层（如 ChassisTask）通过回调把“是否可发送 / 如何发送”注入进来。
 */
typedef uint8_t (*VofaIsReadyFn)(void *ctx);
typedef void (*VofaSendFn)(void *ctx, uint8_t *buf, uint16_t len,
                           VofaTransferMode_e mode);

/**
 * @brief JustFloat 发送器（方案A：薄封装，不做实例池/限频/队列）
 * @note  使用 DMA/IT 时，发送未完成前 tx_buf 不可被覆盖。
 */
typedef struct
{
    void *ctx;                         // 传输层上下文（例如 USARTInstance*）
    VofaIsReadyFn is_ready;            // 判断发送通道是否空闲
    VofaSendFn send;                   // 触发发送
    uint8_t *tx_buf;                  // 发送缓冲区（由 APP 提供）
    uint16_t tx_buf_len;              // 发送缓冲区长度（必须 >= channels*sizeof(float)+4）
    uint8_t max_channels;             // 最大通道数（防御性限制）
    VofaTransferMode_e transfer_mode; // 发送模式（推荐 DMA）
} VofaJustFloatSender_s;

/**
 * @brief 计算 JustFloat 一帧长度（bytes）：float32[channels] + tail(4)
 */
uint16_t VofaGetJustFloatFrameLen(uint8_t channels);

/**
 * @brief 发送一帧 JustFloat 数据：float32[channels] + 帧尾(0x00 0x00 0x80 0x7f)
 * @return 1=已触发发送；0=串口忙/参数非法
 */
uint8_t VofaJustFloatSend(const VofaJustFloatSender_s *sender,
                          const float *values,
                          uint8_t channels);

#endif // VOFA_H
