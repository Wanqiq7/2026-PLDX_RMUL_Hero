#ifndef BSP_USART_H
#define BSP_USART_H

#include <stddef.h>
#include <stdint.h>

#include "main.h"

#define DEVICE_USART_CNT 3     // C板至多分配3个串口
#define USART_RXBUFF_LIMIT 256 // legacy 固定帧兼容缓存上限

typedef struct USARTInstance USARTInstance;
typedef struct USART_Read_Port_s USART_Read_Port_s;
typedef struct USART_Write_Port_s USART_Write_Port_s;

// legacy 模块回调函数,用于兼容当前 fixed-length 模块
// 新模块不应继续依赖该无参回调,而应通过 USARTRead/USARTWrite 使用读写端口
typedef void (*usart_module_callback)(void);

// 新操作完成回调,用于 CALLBACK 模式
typedef void (*usart_operation_callback)(void *user_arg, int status);

/* 发送模式枚举 */
typedef enum
{
    USART_TRANSFER_NONE = 0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

// USART 通用返回状态
typedef enum
{
    USART_STATUS_OK = 0,
    USART_STATUS_PENDING,
    USART_STATUS_BUSY,
    USART_STATUS_FULL,
    USART_STATUS_EMPTY,
    USART_STATUS_TIMEOUT,
    USART_STATUS_FAILED,
    USART_STATUS_INIT_ERR,
    USART_STATUS_INVALID_PARAM,
    USART_STATUS_NOT_SUPPORT,
} USART_Status_e;

// USART 异步操作模式,与 libxr_rw 的 BLOCK/CALLBACK/POLLING 语义对齐
typedef enum
{
    USART_OPERATION_NONE = 0,
    USART_OPERATION_CALLBACK,
    USART_OPERATION_BLOCK,
    USART_OPERATION_POLLING,
} USART_Operation_Type_e;

// POLLING 模式状态
typedef enum
{
    USART_POLLING_READY = 0,
    USART_POLLING_RUNNING,
    USART_POLLING_DONE,
    USART_POLLING_ERROR,
} USART_Polling_Status_e;

// USART 读/写操作描述
// BLOCK 模式的 wait_obj 由 BSP 在实现阶段解释,调用者不应直接跨层访问 HAL/RTOS 内部对象
typedef struct
{
    USART_Operation_Type_e type;
    union
    {
        struct
        {
            usart_operation_callback callback;
            void *user_arg;
        } callback_info;
        struct
        {
            volatile USART_Polling_Status_e *status;
        } polling_info;
        struct
        {
            void *wait_obj;
            uint32_t timeout_ms;
        } block_info;
    } arg;
} USART_Operation_s;

#include "bsp_usart_async.h"

// 串口实例结构体,每个 module 都拥有自己的 USARTInstance
// 兼容策略:
// 1. recv_buff/module_callback 仅服务于 legacy fixed-length 模块
// 2. read_port/write_port 是新接口唯一推荐入口
// 3. owner_id 参考 CAN 的 id/parent pointer 设计,避免长期依赖文件内全局单例指针
struct USARTInstance
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT]; // legacy 固定帧兼容缓存
    uint16_t rx_data_len;                  // legacy 缓存中的有效长度
    uint16_t recv_buff_size;               // legacy 模块期望的固定帧长度
    UART_HandleTypeDef *usart_handle;      // 实例对应的 usart_handle
    usart_module_callback module_callback; // legacy 接收回调
    void *owner_id;                        // 拥有该 USART 实例的 module 指针

    uint16_t rx_fifo_size;   // 新 RX 软件 FIFO 容量
    uint16_t tx_fifo_size;   // 新 TX 数据 FIFO 容量
    uint8_t tx_queue_depth;  // 新 TX 元信息队列深度

    void *driver_context; // BSP 私有驱动上下文,Module 层禁止访问
    USART_Read_Port_s *read_port;   // 新读端口,对齐 BLOCK/CALLBACK/POLLING 机制
    USART_Write_Port_s *write_port; // 新写端口,对齐 BLOCK/CALLBACK/POLLING 机制
};

/* usart 初始化配置结构体 */
typedef struct
{
    uint16_t recv_buff_size;               // legacy 模块接收固定帧长度,无 legacy 需求时可为0
    uint16_t rx_fifo_size;                 // 新 RX FIFO 容量
    uint16_t tx_fifo_size;                 // 新 TX FIFO 容量
    uint8_t tx_queue_depth;                // 新 TX 元信息队列深度
    UART_HandleTypeDef *usart_handle;      // 实例对应的 usart_handle
    usart_module_callback module_callback; // legacy 解析收到数据的回调函数
    void *owner_id;                        // 拥有该 USART 实例的 module 指针
} USART_Init_Config_s;

/**
 * @brief 注册一个串口实例,返回一个串口实例指针
 *
 * @note 会建立新 USART 异步引擎所需的端口与队列,并保留对 legacy fixed-length 模块的兼容
 *
 * @param init_config 传入串口初始化结构体
 */
USARTInstance *USARTRegister(USART_Init_Config_s *init_config);

/**
 * @brief 启动或重建串口服务
 *
 * @note 仅 BSP 内部允许直接触碰 HAL,Module 层只能通过本接口请求重启接收/恢复链路
 *
 * @param _instance 串口实例
 */
void USARTServiceInit(USARTInstance *_instance);

/**
 * @brief legacy 发送包装接口
 *
 * @note 保留现有接口以兼容旧模块,后续内部将映射到 USARTWrite() 与 TX 队列/双缓冲实现
 *
 * @param _instance 串口实例
 * @param send_buf 待发送数据的 buffer
 * @param send_size 待发送数据长度
 * @param mode legacy 传输模式选择
 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size,
               USART_TRANSFER_MODE mode);

/**
 * @brief legacy 查询接口,判断发送链路是否可立即装载新数据
 *
 * @note 该接口保留是为了兼容旧模块,最终推荐使用 USARTWrite() 的状态返回值
 *
 * @param _instance 要判断的串口实例
 * @return uint8_t ready 1, busy 0
 */
uint8_t USARTIsReady(USARTInstance *_instance);

/**
 * @brief 新读接口
 *
 * @note BLOCK 模式禁止在 ISR 中使用; CALLBACK/POLLING 模式可在任务或 ISR 中发起
 *
 * @param _instance 串口实例
 * @param recv_buf 输出缓冲区
 * @param recv_size 期望读取的数据长度
 * @param operation 异步操作描述
 * @param in_isr 当前调用是否位于 ISR 上下文
 * @return USART_Status_e 调用结果
 */
USART_Status_e USARTRead(USARTInstance *_instance, uint8_t *recv_buf, uint16_t recv_size,
                         USART_Operation_s *operation, uint8_t in_isr);

/**
 * @brief 新写接口
 *
 * @note 后续由 TX 数据队列 + 元信息队列 + 双缓冲 DMA 驱动,可完整支持 BLOCK/CALLBACK/POLLING
 *
 * @param _instance 串口实例
 * @param send_buf 待发送数据
 * @param send_size 待发送长度
 * @param operation 异步操作描述
 * @param in_isr 当前调用是否位于 ISR 上下文
 * @return USART_Status_e 调用结果
 */
USART_Status_e USARTWrite(USARTInstance *_instance, const uint8_t *send_buf,
                          uint16_t send_size, USART_Operation_s *operation,
                          uint8_t in_isr);

/**
 * @brief 处理 RX FIFO 中待完成的读请求
 *
 * @note 仅 USART BSP 内部或需要桥接 pending 完成的 module 适配层可以调用
 *
 * @param _instance 串口实例
 * @param in_isr 当前调用是否位于 ISR 上下文
 */
void USARTReadPortProcessPending(USARTInstance *_instance, uint8_t in_isr);

/**
 * @brief 推进 TX 发送状态机
 *
 * @note 仅 USART BSP 内部或兼容层需要在发送完成后继续搬运 pending buffer 时调用
 *
 * @param _instance 串口实例
 * @param in_isr 当前调用是否位于 ISR 上下文
 * @return USART_Status_e 推进结果
 */
USART_Status_e USARTWritePortKick(USARTInstance *_instance, uint8_t in_isr);

#endif
