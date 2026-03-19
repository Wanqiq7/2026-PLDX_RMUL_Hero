#ifndef BSP_USART_ASYNC_H
#define BSP_USART_ASYNC_H

#include <stdint.h>

#ifndef BSP_USART_H
#error "Please include bsp_usart.h before bsp_usart_async.h"
#endif

typedef struct
{
    uint8_t *buffer;
    uint16_t capacity;
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t size;
} USART_Byte_Queue_s;

typedef struct
{
    uint16_t data_size;
    USART_Operation_s operation;
} USART_Write_Request_s;

typedef struct
{
    USART_Write_Request_s *buffer;
    uint8_t capacity;
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t size;
} USART_Write_Request_Queue_s;

typedef struct
{
    uint8_t *block[2];
    uint16_t block_size;
    uint8_t active_index;
    uint8_t pending_valid;
    uint16_t active_len;
    uint16_t pending_len;
} USART_Double_Buffer_s;

typedef enum
{
    USART_READ_PORT_IDLE = 0,
    USART_READ_PORT_PENDING,
    USART_READ_PORT_BLOCK_CLAIMED,
    USART_READ_PORT_BLOCK_DETACHED,
    USART_READ_PORT_EVENT,
} USART_Read_Port_Busy_e;

typedef enum
{
    USART_WRITE_PORT_LOCKED = 0,
    USART_WRITE_PORT_BLOCK_WAITING,
    USART_WRITE_PORT_BLOCK_CLAIMED,
    USART_WRITE_PORT_BLOCK_DETACHED,
    USART_WRITE_PORT_IDLE = 0xFF,
} USART_Write_Port_Busy_e;

struct USART_Read_Port_s
{
    USART_Byte_Queue_s queue_data;
    volatile USART_Read_Port_Busy_e busy;
    USART_Status_e block_result;
    uint8_t *pending_recv_buf;
    uint16_t pending_recv_size;
    USART_Operation_s pending_operation;
    void *block_waiter;
};

struct USART_Write_Port_s
{
    USART_Byte_Queue_s queue_data;
    USART_Write_Request_Queue_s queue_info;
    USART_Double_Buffer_s tx_double_buffer;
    volatile USART_Write_Port_Busy_e busy;
    USART_Status_e block_result;
    USART_Write_Request_s active_request;
    USART_Write_Request_s pending_request;
};

void USARTOperationInitCallback(USART_Operation_s *operation,
                                usart_operation_callback callback, void *user_arg);
void USARTOperationInitBlock(USART_Operation_s *operation, uint32_t timeout_ms);
void USARTOperationInitPolling(USART_Operation_s *operation,
                               volatile USART_Polling_Status_e *status);
void USARTOperationInitNone(USART_Operation_s *operation);

USART_Status_e USARTAsyncReadPortInit(USART_Read_Port_s *port, uint16_t fifo_size);
USART_Status_e USARTAsyncWritePortInit(USART_Write_Port_s *port,
                                       uint16_t data_fifo_size,
                                       uint8_t queue_depth,
                                       uint16_t double_buffer_block_size);
void USARTAsyncReadPortReset(USART_Read_Port_s *port);
void USARTAsyncWritePortReset(USART_Write_Port_s *port);
uint16_t USARTAsyncReadPortSize(const USART_Read_Port_s *port);
uint16_t USARTAsyncReadPortEmptySize(const USART_Read_Port_s *port);
uint16_t USARTAsyncWritePortSize(const USART_Write_Port_s *port);
uint16_t USARTAsyncWritePortEmptySize(const USART_Write_Port_s *port);
USART_Status_e USARTAsyncRead(USART_Read_Port_s *port, uint8_t *recv_buf,
                              uint16_t recv_size, USART_Operation_s *operation,
                              uint8_t in_isr);
USART_Status_e USARTAsyncReadPortReadAvailable(USART_Read_Port_s *port,
                                               uint8_t *recv_buf,
                                               uint16_t recv_size,
                                               uint16_t *actual_size,
                                               uint8_t in_isr);
USART_Status_e USARTAsyncReadPortPushBytes(USART_Read_Port_s *port,
                                           const uint8_t *data, uint16_t length,
                                           uint8_t in_isr);
void USARTAsyncReadPortProcessPending(USART_Read_Port_s *port, uint8_t in_isr);
USART_Status_e USARTAsyncWrite(USART_Write_Port_s *port, const uint8_t *send_buf,
                               uint16_t send_size, USART_Operation_s *operation,
                               uint8_t in_isr);
USART_Status_e USARTAsyncWritePortLoadNext(USART_Write_Port_s *port, uint8_t in_isr);
void USARTAsyncWritePortFinishActive(USART_Write_Port_s *port, USART_Status_e status,
                                     uint8_t in_isr);

#endif
