#ifndef BSP_USART_H
#define BSP_USART_H

#include <stddef.h>
#include <stdint.h>

#include "main.h"

#define DEVICE_USART_CNT 3     // C板至多分配3个串口
#define USART_RXBUFF_LIMIT 512 // legacy 固定帧兼容缓存上限

typedef struct USARTInstance USARTInstance;
typedef struct USART_Read_Port_s USART_Read_Port_s;
typedef struct USART_Write_Port_s USART_Write_Port_s;

// legacy 模块回调函数,用于兼容当前 fixed-length 模块
// Gimbal 旧接口带 size 参数,因此兼容层仍会把实际交付的固定帧长度传入
typedef void (*usart_module_callback)(uint16_t size);

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

    void *driver_context;                    // BSP 私有驱动上下文,Module 层禁止访问
    USART_Read_Port_s *read_port;            // 新读端口
    USART_Write_Port_s *write_port;          // 新写端口
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

USARTInstance *USARTRegister(USART_Init_Config_s *init_config);
void USARTServiceInit(USARTInstance *_instance);
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size,
               USART_TRANSFER_MODE mode);
uint8_t USARTIsReady(USARTInstance *_instance);

USART_Status_e USARTRead(USARTInstance *_instance, uint8_t *recv_buf, uint16_t recv_size,
                         USART_Operation_s *operation, uint8_t in_isr);
USART_Status_e USARTReadAvailable(USARTInstance *_instance, uint8_t *recv_buf,
                                  uint16_t recv_size, uint16_t *actual_size,
                                  uint8_t in_isr);
USART_Status_e USARTWrite(USARTInstance *_instance, const uint8_t *send_buf,
                          uint16_t send_size, USART_Operation_s *operation,
                          uint8_t in_isr);
void USARTReadPortProcessPending(USARTInstance *_instance, uint8_t in_isr);
USART_Status_e USARTWritePortKick(USARTInstance *_instance, uint8_t in_isr);

#endif
