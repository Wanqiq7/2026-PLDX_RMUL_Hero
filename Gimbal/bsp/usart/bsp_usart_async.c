#include "bsp_usart.h"
#include "bsp_usart_async.h"

#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

static UBaseType_t USARTEnterCritical(uint8_t in_isr)
{
    if (in_isr)
    {
        return taskENTER_CRITICAL_FROM_ISR();
    }

    taskENTER_CRITICAL();
    return 0U;
}

static void USARTExitCritical(uint8_t in_isr, UBaseType_t state)
{
    if (in_isr)
    {
        taskEXIT_CRITICAL_FROM_ISR(state);
    }
    else
    {
        taskEXIT_CRITICAL();
    }
}

static void USARTAsyncOperationMarkRunning(USART_Operation_s *operation)
{
    if (operation == NULL)
        return;

    if (operation->type == USART_OPERATION_POLLING &&
        operation->arg.polling_info.status != NULL)
    {
        *operation->arg.polling_info.status = USART_POLLING_RUNNING;
    }
}

static void USARTAsyncOperationFinish(USART_Operation_s *operation, USART_Status_e status,
                                      uint8_t in_isr)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    uint32_t notify_value = (uint32_t)status;

    if (operation == NULL)
        return;

    switch (operation->type)
    {
    case USART_OPERATION_CALLBACK:
        if (operation->arg.callback_info.callback != NULL)
        {
            operation->arg.callback_info.callback(operation->arg.callback_info.user_arg,
                                                  (int)status);
        }
        break;
    case USART_OPERATION_BLOCK:
        if (operation->arg.block_info.wait_obj != NULL)
        {
            if (in_isr)
            {
                xTaskNotifyFromISR((TaskHandle_t)operation->arg.block_info.wait_obj,
                                   notify_value, eSetValueWithOverwrite,
                                   &higher_priority_task_woken);
                portYIELD_FROM_ISR(higher_priority_task_woken);
            }
            else
            {
                xTaskNotify((TaskHandle_t)operation->arg.block_info.wait_obj, notify_value,
                            eSetValueWithOverwrite);
            }
        }
        break;
    case USART_OPERATION_POLLING:
        if (operation->arg.polling_info.status != NULL)
        {
            *operation->arg.polling_info.status =
                (status == USART_STATUS_OK) ? USART_POLLING_DONE : USART_POLLING_ERROR;
        }
        break;
    case USART_OPERATION_NONE:
    default:
        break;
    }
}

static void USARTByteQueueReset(USART_Byte_Queue_s *queue)
{
    if (queue == NULL)
        return;

    queue->head = 0U;
    queue->tail = 0U;
    queue->size = 0U;
}

static USART_Status_e USARTByteQueuePushBatch(USART_Byte_Queue_s *queue,
                                              const uint8_t *data, uint16_t length,
                                              uint8_t in_isr)
{
    UBaseType_t critical_state;

    if (queue == NULL || (data == NULL && length > 0U))
        return USART_STATUS_INVALID_PARAM;

    if (length == 0U)
        return USART_STATUS_OK;

    critical_state = USARTEnterCritical(in_isr);
    if ((uint16_t)(queue->capacity - queue->size) < length)
    {
        USARTExitCritical(in_isr, critical_state);
        return USART_STATUS_FULL;
    }

    for (uint16_t i = 0U; i < length; ++i)
    {
        queue->buffer[queue->head] = data[i];
        queue->head = (uint16_t)((queue->head + 1U) % queue->capacity);
    }
    queue->size = (uint16_t)(queue->size + length);
    USARTExitCritical(in_isr, critical_state);
    return USART_STATUS_OK;
}

static USART_Status_e USARTByteQueuePopBatch(USART_Byte_Queue_s *queue, uint8_t *data,
                                             uint16_t length, uint8_t in_isr)
{
    UBaseType_t critical_state;

    if (queue == NULL || (data == NULL && length > 0U))
        return USART_STATUS_INVALID_PARAM;

    if (length == 0U)
        return USART_STATUS_OK;

    critical_state = USARTEnterCritical(in_isr);
    if (queue->size < length)
    {
        USARTExitCritical(in_isr, critical_state);
        return USART_STATUS_EMPTY;
    }

    for (uint16_t i = 0U; i < length; ++i)
    {
        data[i] = queue->buffer[queue->tail];
        queue->tail = (uint16_t)((queue->tail + 1U) % queue->capacity);
    }
    queue->size = (uint16_t)(queue->size - length);
    USARTExitCritical(in_isr, critical_state);
    return USART_STATUS_OK;
}

static USART_Status_e USARTWriteRequestQueuePush(USART_Write_Request_Queue_s *queue,
                                                 const USART_Write_Request_s *request,
                                                 uint8_t in_isr)
{
    UBaseType_t critical_state;

    if (queue == NULL || request == NULL)
        return USART_STATUS_INVALID_PARAM;

    critical_state = USARTEnterCritical(in_isr);
    if (queue->size >= queue->capacity)
    {
        USARTExitCritical(in_isr, critical_state);
        return USART_STATUS_FULL;
    }

    queue->buffer[queue->head] = *request;
    queue->head = (uint8_t)((queue->head + 1U) % queue->capacity);
    queue->size++;
    USARTExitCritical(in_isr, critical_state);
    return USART_STATUS_OK;
}

static USART_Status_e USARTWriteRequestQueuePop(USART_Write_Request_Queue_s *queue,
                                                USART_Write_Request_s *request,
                                                uint8_t in_isr)
{
    UBaseType_t critical_state;

    if (queue == NULL || request == NULL)
        return USART_STATUS_INVALID_PARAM;

    critical_state = USARTEnterCritical(in_isr);
    if (queue->size == 0U)
    {
        USARTExitCritical(in_isr, critical_state);
        return USART_STATUS_EMPTY;
    }

    *request = queue->buffer[queue->tail];
    queue->tail = (uint8_t)((queue->tail + 1U) % queue->capacity);
    queue->size--;
    USARTExitCritical(in_isr, critical_state);
    return USART_STATUS_OK;
}

static void USARTWriteRequestQueueReset(USART_Write_Request_Queue_s *queue)
{
    if (queue == NULL)
        return;

    queue->head = 0U;
    queue->tail = 0U;
    queue->size = 0U;
}

static USART_Status_e USARTDoubleBufferInit(USART_Double_Buffer_s *buffer,
                                            uint16_t block_size)
{
    if (buffer == NULL || block_size == 0U)
        return USART_STATUS_INVALID_PARAM;

    memset(buffer, 0, sizeof(*buffer));
    buffer->block[0] = (uint8_t *)malloc(block_size);
    buffer->block[1] = (uint8_t *)malloc(block_size);
    if (buffer->block[0] == NULL || buffer->block[1] == NULL)
    {
        free(buffer->block[0]);
        free(buffer->block[1]);
        buffer->block[0] = NULL;
        buffer->block[1] = NULL;
        return USART_STATUS_INIT_ERR;
    }

    buffer->block_size = block_size;
    buffer->active_index = 0U;
    return USART_STATUS_OK;
}

static void USARTDoubleBufferReset(USART_Double_Buffer_s *buffer)
{
    if (buffer == NULL)
        return;

    buffer->active_index = 0U;
    buffer->pending_valid = 0U;
    buffer->active_len = 0U;
    buffer->pending_len = 0U;
    if (buffer->block[0] != NULL)
        memset(buffer->block[0], 0, buffer->block_size);
    if (buffer->block[1] != NULL)
        memset(buffer->block[1], 0, buffer->block_size);
}

static uint8_t *USARTDoubleBufferActive(USART_Double_Buffer_s *buffer)
{
    return buffer->block[buffer->active_index];
}

static uint8_t *USARTDoubleBufferPending(USART_Double_Buffer_s *buffer)
{
    return buffer->block[buffer->active_index ^ 1U];
}

static void USARTDoubleBufferSwitch(USART_Double_Buffer_s *buffer)
{
    if (buffer == NULL || buffer->pending_valid == 0U)
        return;

    buffer->active_index ^= 1U;
    buffer->active_len = buffer->pending_len;
    buffer->pending_len = 0U;
    buffer->pending_valid = 0U;
}

void USARTOperationInitCallback(USART_Operation_s *operation,
                                usart_operation_callback callback, void *user_arg)
{
    if (operation == NULL)
        return;

    memset(operation, 0, sizeof(*operation));
    operation->type = USART_OPERATION_CALLBACK;
    operation->arg.callback_info.callback = callback;
    operation->arg.callback_info.user_arg = user_arg;
}

void USARTOperationInitBlock(USART_Operation_s *operation, uint32_t timeout_ms)
{
    if (operation == NULL)
        return;

    memset(operation, 0, sizeof(*operation));
    operation->type = USART_OPERATION_BLOCK;
    operation->arg.block_info.timeout_ms = timeout_ms;
    operation->arg.block_info.wait_obj = NULL;
}

void USARTOperationInitPolling(USART_Operation_s *operation,
                               volatile USART_Polling_Status_e *status)
{
    if (operation == NULL)
        return;

    memset(operation, 0, sizeof(*operation));
    operation->type = USART_OPERATION_POLLING;
    operation->arg.polling_info.status = status;
    if (status != NULL)
    {
        *status = USART_POLLING_READY;
    }
}

void USARTOperationInitNone(USART_Operation_s *operation)
{
    if (operation == NULL)
        return;

    memset(operation, 0, sizeof(*operation));
    operation->type = USART_OPERATION_NONE;
}

USART_Status_e USARTAsyncReadPortInit(USART_Read_Port_s *port, uint16_t fifo_size)
{
    if (port == NULL || fifo_size == 0U)
        return USART_STATUS_INVALID_PARAM;

    memset(port, 0, sizeof(*port));
    port->queue_data.buffer = (uint8_t *)malloc(fifo_size);
    if (port->queue_data.buffer == NULL)
        return USART_STATUS_INIT_ERR;

    port->queue_data.capacity = fifo_size;
    port->busy = USART_READ_PORT_IDLE;
    port->block_result = USART_STATUS_OK;
    return USART_STATUS_OK;
}

USART_Status_e USARTAsyncWritePortInit(USART_Write_Port_s *port,
                                       uint16_t data_fifo_size,
                                       uint8_t queue_depth,
                                       uint16_t double_buffer_block_size)
{
    USART_Status_e status;

    if (port == NULL || data_fifo_size == 0U || queue_depth == 0U ||
        double_buffer_block_size == 0U)
        return USART_STATUS_INVALID_PARAM;

    memset(port, 0, sizeof(*port));
    port->queue_data.buffer = (uint8_t *)malloc(data_fifo_size);
    port->queue_info.buffer =
        (USART_Write_Request_s *)malloc(queue_depth * sizeof(USART_Write_Request_s));
    if (port->queue_data.buffer == NULL || port->queue_info.buffer == NULL)
    {
        free(port->queue_data.buffer);
        free(port->queue_info.buffer);
        port->queue_data.buffer = NULL;
        port->queue_info.buffer = NULL;
        return USART_STATUS_INIT_ERR;
    }

    port->queue_data.capacity = data_fifo_size;
    port->queue_info.capacity = queue_depth;
    port->busy = USART_WRITE_PORT_IDLE;
    port->block_result = USART_STATUS_OK;

    status = USARTDoubleBufferInit(&port->tx_double_buffer, double_buffer_block_size);
    if (status != USART_STATUS_OK)
    {
        free(port->queue_data.buffer);
        free(port->queue_info.buffer);
        port->queue_data.buffer = NULL;
        port->queue_info.buffer = NULL;
        return status;
    }

    return USART_STATUS_OK;
}

void USARTAsyncReadPortReset(USART_Read_Port_s *port)
{
    if (port == NULL)
        return;

    USARTByteQueueReset(&port->queue_data);
    port->busy = USART_READ_PORT_IDLE;
    port->block_result = USART_STATUS_OK;
    port->pending_recv_buf = NULL;
    port->pending_recv_size = 0U;
    memset(&port->pending_operation, 0, sizeof(port->pending_operation));
}

void USARTAsyncWritePortReset(USART_Write_Port_s *port)
{
    if (port == NULL)
        return;

    USARTByteQueueReset(&port->queue_data);
    USARTWriteRequestQueueReset(&port->queue_info);
    USARTDoubleBufferReset(&port->tx_double_buffer);
    port->busy = USART_WRITE_PORT_IDLE;
    port->block_result = USART_STATUS_OK;
    memset(&port->active_request, 0, sizeof(port->active_request));
    memset(&port->pending_request, 0, sizeof(port->pending_request));
}

uint16_t USARTAsyncReadPortSize(const USART_Read_Port_s *port)
{
    if (port == NULL)
        return 0U;
    return port->queue_data.size;
}

uint16_t USARTAsyncReadPortEmptySize(const USART_Read_Port_s *port)
{
    if (port == NULL)
        return 0U;
    return (uint16_t)(port->queue_data.capacity - port->queue_data.size);
}

uint16_t USARTAsyncWritePortSize(const USART_Write_Port_s *port)
{
    uint16_t size = 0U;

    if (port == NULL)
        return 0U;

    size = port->queue_data.size;
    size = (uint16_t)(size + port->tx_double_buffer.active_len);
    size = (uint16_t)(size + port->tx_double_buffer.pending_len);
    return size;
}

uint16_t USARTAsyncWritePortEmptySize(const USART_Write_Port_s *port)
{
    if (port == NULL)
        return 0U;
    return (uint16_t)(port->queue_data.capacity - port->queue_data.size);
}

USART_Status_e USARTAsyncRead(USART_Read_Port_s *port, uint8_t *recv_buf,
                              uint16_t recv_size, USART_Operation_s *operation,
                              uint8_t in_isr)
{
    USART_Status_e status;
    USART_Operation_s none_operation;
    uint32_t notify_value = 0U;
    TickType_t wait_ticks;

    if (port == NULL || (recv_buf == NULL && recv_size > 0U))
        return USART_STATUS_INVALID_PARAM;

    if (operation == NULL)
    {
        USARTOperationInitNone(&none_operation);
        operation = &none_operation;
    }

    if (recv_size == 0U)
    {
        USARTAsyncOperationFinish(operation, USART_STATUS_OK, in_isr);
        return USART_STATUS_OK;
    }

    status = USARTByteQueuePopBatch(&port->queue_data, recv_buf, recv_size, in_isr);
    if (status == USART_STATUS_OK)
    {
        USARTAsyncOperationFinish(operation, USART_STATUS_OK, in_isr);
        return USART_STATUS_OK;
    }

    if (port->busy != USART_READ_PORT_IDLE && port->busy != USART_READ_PORT_EVENT)
        return USART_STATUS_BUSY;

    port->pending_recv_buf = recv_buf;
    port->pending_recv_size = recv_size;
    port->pending_operation = *operation;
    if (port->pending_operation.type == USART_OPERATION_BLOCK)
    {
        if (in_isr)
            return USART_STATUS_INVALID_PARAM;

        port->pending_operation.arg.block_info.wait_obj = xTaskGetCurrentTaskHandle();
        xTaskNotifyStateClear(NULL);
    }

    USARTAsyncOperationMarkRunning(&port->pending_operation);
    port->busy = USART_READ_PORT_PENDING;

    if (port->pending_operation.type != USART_OPERATION_BLOCK)
        return USART_STATUS_PENDING;

    wait_ticks = (port->pending_operation.arg.block_info.timeout_ms == UINT32_MAX)
                     ? portMAX_DELAY
                     : pdMS_TO_TICKS(port->pending_operation.arg.block_info.timeout_ms);

    if (xTaskNotifyWait(0U, UINT32_MAX, &notify_value, wait_ticks) == pdTRUE)
    {
        return (USART_Status_e)notify_value;
    }

    port->busy = USART_READ_PORT_IDLE;
    return USART_STATUS_TIMEOUT;
}

USART_Status_e USARTAsyncReadPortPushBytes(USART_Read_Port_s *port,
                                           const uint8_t *data, uint16_t length,
                                           uint8_t in_isr)
{
    USART_Status_e status;

    if (port == NULL)
        return USART_STATUS_INVALID_PARAM;

    status = USARTByteQueuePushBatch(&port->queue_data, data, length, in_isr);
    if (status != USART_STATUS_OK)
        return status;

    USARTAsyncReadPortProcessPending(port, in_isr);
    return USART_STATUS_OK;
}

void USARTAsyncReadPortProcessPending(USART_Read_Port_s *port, uint8_t in_isr)
{
    USART_Status_e status;
    USART_Operation_s operation;

    if (port == NULL)
        return;

    if (port->busy != USART_READ_PORT_PENDING)
    {
        if (port->busy == USART_READ_PORT_IDLE && port->queue_data.size > 0U)
            port->busy = USART_READ_PORT_EVENT;
        return;
    }

    if (port->queue_data.size < port->pending_recv_size)
        return;

    status = USARTByteQueuePopBatch(&port->queue_data, port->pending_recv_buf,
                                    port->pending_recv_size, in_isr);
    if (status != USART_STATUS_OK)
        return;

    operation = port->pending_operation;
    port->busy = USART_READ_PORT_IDLE;
    port->pending_recv_buf = NULL;
    port->pending_recv_size = 0U;
    memset(&port->pending_operation, 0, sizeof(port->pending_operation));
    USARTAsyncOperationFinish(&operation, USART_STATUS_OK, in_isr);
}

USART_Status_e USARTAsyncWrite(USART_Write_Port_s *port, const uint8_t *send_buf,
                               uint16_t send_size, USART_Operation_s *operation,
                               uint8_t in_isr)
{
    USART_Write_Request_s request;
    USART_Status_e status;
    USART_Operation_s none_operation;

    if (port == NULL || (send_buf == NULL && send_size > 0U))
        return USART_STATUS_INVALID_PARAM;

    if (operation == NULL)
    {
        USARTOperationInitNone(&none_operation);
        operation = &none_operation;
    }

    if (send_size == 0U)
    {
        USARTAsyncOperationFinish(operation, USART_STATUS_OK, in_isr);
        return USART_STATUS_OK;
    }

    if (USARTAsyncWritePortEmptySize(port) < send_size ||
        port->queue_info.size >= port->queue_info.capacity)
        return USART_STATUS_FULL;

    request.data_size = send_size;
    request.operation = *operation;
    if (request.operation.type == USART_OPERATION_BLOCK)
    {
        if (in_isr)
            return USART_STATUS_INVALID_PARAM;

        request.operation.arg.block_info.wait_obj = xTaskGetCurrentTaskHandle();
        xTaskNotifyStateClear(NULL);
    }
    USARTAsyncOperationMarkRunning(&request.operation);

    status = USARTByteQueuePushBatch(&port->queue_data, send_buf, send_size, in_isr);
    if (status != USART_STATUS_OK)
        return status;

    status = USARTWriteRequestQueuePush(&port->queue_info, &request, in_isr);
    if (status != USART_STATUS_OK)
        return status;

    status = USARTAsyncWritePortLoadNext(port, in_isr);
    return (status == USART_STATUS_OK || status == USART_STATUS_BUSY)
               ? USART_STATUS_PENDING
               : status;
}

USART_Status_e USARTAsyncWritePortLoadNext(USART_Write_Port_s *port, uint8_t in_isr)
{
    USART_Write_Request_s request;
    uint8_t *target_buffer = NULL;
    uint16_t *target_length = NULL;

    if (port == NULL)
        return USART_STATUS_INVALID_PARAM;

    if (port->queue_info.size == 0U)
        return USART_STATUS_PENDING;

    if (port->tx_double_buffer.active_len == 0U)
    {
        target_buffer = USARTDoubleBufferActive(&port->tx_double_buffer);
        target_length = &port->tx_double_buffer.active_len;
    }
    else if (port->tx_double_buffer.pending_valid == 0U)
    {
        target_buffer = USARTDoubleBufferPending(&port->tx_double_buffer);
        target_length = &port->tx_double_buffer.pending_len;
    }
    else
    {
        return USART_STATUS_BUSY;
    }

    if (USARTWriteRequestQueuePop(&port->queue_info, &request, in_isr) != USART_STATUS_OK)
        return USART_STATUS_EMPTY;

    if (USARTByteQueuePopBatch(&port->queue_data, target_buffer, request.data_size,
                               in_isr) != USART_STATUS_OK)
        return USART_STATUS_EMPTY;

    *target_length = request.data_size;
    if (port->tx_double_buffer.active_len == request.data_size &&
        target_buffer == USARTDoubleBufferActive(&port->tx_double_buffer))
    {
        port->active_request = request;
    }
    else
    {
        port->pending_request = request;
        port->tx_double_buffer.pending_valid = 1U;
    }

    return USART_STATUS_OK;
}

void USARTAsyncWritePortFinishActive(USART_Write_Port_s *port, USART_Status_e status,
                                     uint8_t in_isr)
{
    USART_Write_Request_s completed_request;

    if (port == NULL || port->tx_double_buffer.active_len == 0U)
        return;

    completed_request = port->active_request;
    memset(&port->active_request, 0, sizeof(port->active_request));
    USARTAsyncOperationFinish(&completed_request.operation, status, in_isr);

    if (port->tx_double_buffer.pending_valid != 0U)
    {
        USARTDoubleBufferSwitch(&port->tx_double_buffer);
        port->active_request = port->pending_request;
        memset(&port->pending_request, 0, sizeof(port->pending_request));
    }
    else
    {
        port->tx_double_buffer.active_len = 0U;
    }

    (void)USARTAsyncWritePortLoadNext(port, in_isr);
}
