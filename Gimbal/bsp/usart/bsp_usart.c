/**
 * @file bsp_usart.c
 * @author neozng
 * @brief 串口 BSP 实现
 * @version beta
 * @date 2022-11-01
 */
#include "bsp_usart.h"

#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "bsp_log.h"
#include "task.h"

typedef struct
{
    uint8_t *rx_dma_buffer;
    uint16_t rx_dma_size;
    uint16_t rx_last_pos;
    uint8_t tx_dma_active;
} USARTDriverContext;

static uint8_t idx;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

static uint16_t USARTGetDefaultRxFifoSize(uint16_t legacy_frame_size)
{
    if (legacy_frame_size == 0U)
        return 256U;

    if (legacy_frame_size >= 128U)
        return (uint16_t)(legacy_frame_size * 2U);

    return 256U;
}

static uint16_t USARTGetDefaultTxFifoSize(void) { return 256U; }
static uint8_t USARTGetDefaultTxQueueDepth(void) { return 5U; }

static USARTDriverContext *USARTGetDriverContext(USARTInstance *_instance)
{
    return (USARTDriverContext *)_instance->driver_context;
}

static USART_Status_e USARTCreateDriverContext(USARTInstance *_instance)
{
    USARTDriverContext *context;

    if (_instance == NULL)
        return USART_STATUS_INVALID_PARAM;

    context = (USARTDriverContext *)malloc(sizeof(USARTDriverContext));
    if (context == NULL)
        return USART_STATUS_INIT_ERR;

    memset(context, 0, sizeof(*context));
    context->rx_dma_size = _instance->rx_fifo_size;
    context->rx_dma_buffer = (uint8_t *)malloc(context->rx_dma_size);
    if (context->rx_dma_buffer == NULL)
    {
        free(context);
        return USART_STATUS_INIT_ERR;
    }

    memset(context->rx_dma_buffer, 0, context->rx_dma_size);
    _instance->driver_context = context;
    return USART_STATUS_OK;
}

static void USARTLegacyTryDispatchFrames(USARTInstance *_instance, uint8_t in_isr)
{
    USART_Operation_s none_operation;

    if (_instance == NULL || _instance->module_callback == NULL ||
        _instance->recv_buff_size == 0U)
        return;

    while (USARTAsyncReadPortSize(_instance->read_port) >= _instance->recv_buff_size)
    {
        USARTOperationInitNone(&none_operation);
        if (USARTAsyncRead(_instance->read_port, _instance->recv_buff,
                           _instance->recv_buff_size, &none_operation, in_isr) !=
            USART_STATUS_OK)
            return;

        _instance->rx_data_len = _instance->recv_buff_size;
        _instance->module_callback(_instance->recv_buff_size);
    }
}

static USART_Status_e USARTTryStartTxDMA(USARTInstance *_instance)
{
    USARTDriverContext *context;
    uint8_t *active_buffer;
    uint16_t active_length;

    if (_instance == NULL || _instance->write_port == NULL ||
        _instance->usart_handle == NULL)
        return USART_STATUS_INVALID_PARAM;

    context = USARTGetDriverContext(_instance);
    if (context == NULL)
        return USART_STATUS_INIT_ERR;

    if (context->tx_dma_active != 0U)
    {
        if (_instance->write_port->tx_double_buffer.pending_valid == 0U)
        {
            (void)USARTAsyncWritePortLoadNext(_instance->write_port, 0U);
        }
        return USART_STATUS_PENDING;
    }

    if (_instance->write_port->tx_double_buffer.active_len == 0U)
    {
        USART_Status_e load_status =
            USARTAsyncWritePortLoadNext(_instance->write_port, 0U);
        if (load_status != USART_STATUS_OK)
            return load_status;
    }

    active_buffer = _instance->write_port->tx_double_buffer
                        .block[_instance->write_port->tx_double_buffer.active_index];
    active_length = _instance->write_port->tx_double_buffer.active_len;
    if (active_buffer == NULL || active_length == 0U)
        return USART_STATUS_EMPTY;

    if (_instance->usart_handle->gState != HAL_UART_STATE_READY)
        return USART_STATUS_PENDING;

    context->tx_dma_active = 1U;
    if (HAL_UART_Transmit_DMA(_instance->usart_handle, active_buffer, active_length) !=
        HAL_OK)
    {
        context->tx_dma_active = 0U;
        return USART_STATUS_FAILED;
    }

    (void)USARTAsyncWritePortLoadNext(_instance->write_port, 0U);
    return USART_STATUS_OK;
}

void USARTServiceInit(USARTInstance *_instance)
{
    USARTDriverContext *context;

    if (_instance == NULL || _instance->usart_handle == NULL || _instance->read_port == NULL)
        return;

    context = USARTGetDriverContext(_instance);
    if (context == NULL)
        return;

    memset(_instance->recv_buff, 0, sizeof(_instance->recv_buff));
    _instance->rx_data_len = 0U;
    USARTAsyncReadPortReset(_instance->read_port);
    context->rx_last_pos = 0U;
    memset(context->rx_dma_buffer, 0, context->rx_dma_size);

    if (_instance->usart_handle->hdmarx != NULL)
    {
        _instance->usart_handle->hdmarx->Init.Mode = DMA_CIRCULAR;
        /* 与 libxr 保持一致：RX 使用 direct mode，不启用 DMA FIFO。 */
        _instance->usart_handle->hdmarx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        _instance->usart_handle->hdmarx->Init.MemBurst = DMA_MBURST_SINGLE;
        _instance->usart_handle->hdmarx->Init.PeriphBurst = DMA_PBURST_SINGLE;
        HAL_DMA_Init(_instance->usart_handle->hdmarx);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, context->rx_dma_buffer,
                                 context->rx_dma_size);
}

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    USARTInstance *instance;
    USART_Status_e status;

    if (init_config == NULL || init_config->usart_handle == NULL)
        return NULL;

    if (idx >= DEVICE_USART_CNT)
        while (1)
            LOGERROR("[bsp_usart] USART exceed max instance count!");

    for (uint8_t i = 0U; i < idx; i++)
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
            while (1)
                LOGERROR("[bsp_usart] USART instance already registered!");

    instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    if (instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(USARTInstance));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;
    instance->owner_id = init_config->owner_id;
    instance->rx_fifo_size = (init_config->rx_fifo_size != 0U)
                                 ? init_config->rx_fifo_size
                                 : USARTGetDefaultRxFifoSize(init_config->recv_buff_size);
    instance->tx_fifo_size = (init_config->tx_fifo_size != 0U)
                                 ? init_config->tx_fifo_size
                                 : USARTGetDefaultTxFifoSize();
    instance->tx_queue_depth = (init_config->tx_queue_depth != 0U)
                                   ? init_config->tx_queue_depth
                                   : USARTGetDefaultTxQueueDepth();

    instance->read_port = (USART_Read_Port_s *)malloc(sizeof(USART_Read_Port_s));
    instance->write_port = (USART_Write_Port_s *)malloc(sizeof(USART_Write_Port_s));
    if (instance->read_port == NULL || instance->write_port == NULL)
    {
        free(instance->read_port);
        free(instance->write_port);
        free(instance);
        return NULL;
    }

    status = USARTAsyncReadPortInit(instance->read_port, instance->rx_fifo_size);
    if (status != USART_STATUS_OK)
    {
        free(instance->read_port);
        free(instance->write_port);
        free(instance);
        return NULL;
    }

    status = USARTAsyncWritePortInit(instance->write_port, instance->tx_fifo_size,
                                     instance->tx_queue_depth, instance->tx_fifo_size);
    if (status != USART_STATUS_OK)
    {
        free(instance->read_port);
        free(instance->write_port);
        free(instance);
        return NULL;
    }

    status = USARTCreateDriverContext(instance);
    if (status != USART_STATUS_OK)
    {
        free(instance->read_port);
        free(instance->write_port);
        free(instance);
        return NULL;
    }

    usart_instance[idx++] = instance;
    USARTServiceInit(instance);
    return instance;
}

USART_Status_e USARTRead(USARTInstance *_instance, uint8_t *recv_buf,
                         uint16_t recv_size, USART_Operation_s *operation,
                         uint8_t in_isr)
{
    if (_instance == NULL || _instance->read_port == NULL)
        return USART_STATUS_INVALID_PARAM;

    return USARTAsyncRead(_instance->read_port, recv_buf, recv_size, operation, in_isr);
}

USART_Status_e USARTReadAvailable(USARTInstance *_instance, uint8_t *recv_buf,
                                  uint16_t recv_size, uint16_t *actual_size,
                                  uint8_t in_isr)
{
    if (_instance == NULL || _instance->read_port == NULL)
        return USART_STATUS_INVALID_PARAM;

    return USARTAsyncReadPortReadAvailable(_instance->read_port, recv_buf, recv_size,
                                           actual_size, in_isr);
}

USART_Status_e USARTWrite(USARTInstance *_instance, const uint8_t *send_buf,
                          uint16_t send_size, USART_Operation_s *operation,
                          uint8_t in_isr)
{
    USART_Status_e status;
    TickType_t wait_ticks;
    uint32_t notify_value = 0U;

    if (_instance == NULL || _instance->write_port == NULL)
        return USART_STATUS_INVALID_PARAM;

    status = USARTAsyncWrite(_instance->write_port, send_buf, send_size, operation, in_isr);
    if (status != USART_STATUS_PENDING && status != USART_STATUS_OK)
        return status;

    status = USARTWritePortKick(_instance, in_isr);
    if (status == USART_STATUS_FAILED || status == USART_STATUS_INIT_ERR)
        return status;

    if (operation != NULL && operation->type == USART_OPERATION_BLOCK)
    {
        if (in_isr)
            return USART_STATUS_INVALID_PARAM;

        wait_ticks = (operation->arg.block_info.timeout_ms == UINT32_MAX)
                         ? portMAX_DELAY
                         : pdMS_TO_TICKS(operation->arg.block_info.timeout_ms);

        if (xTaskNotifyWait(0U, UINT32_MAX, &notify_value, wait_ticks) == pdTRUE)
            return (USART_Status_e)notify_value;

        return USART_STATUS_TIMEOUT;
    }

    return USART_STATUS_PENDING;
}

void USARTReadPortProcessPending(USARTInstance *_instance, uint8_t in_isr)
{
    if (_instance == NULL || _instance->read_port == NULL)
        return;

    USARTAsyncReadPortProcessPending(_instance->read_port, in_isr);
}

USART_Status_e USARTWritePortKick(USARTInstance *_instance, uint8_t in_isr)
{
    UNUSED(in_isr);
    return USARTTryStartTxDMA(_instance);
}

void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size,
               USART_TRANSFER_MODE mode)
{
    USART_Status_e status;
    USART_Operation_s none_operation;

    if (_instance == NULL || _instance->usart_handle == NULL)
        return;

    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100U);
        break;
    case USART_TRANSFER_IT:
    case USART_TRANSFER_DMA:
        USARTOperationInitNone(&none_operation);
        status = USARTWrite(_instance, send_buf, send_size, &none_operation, 0U);
        if (status != USART_STATUS_PENDING && status != USART_STATUS_OK)
        {
            LOGWARNING("[bsp_usart] legacy send enqueue failed, status [%d]", status);
        }
        break;
    case USART_TRANSFER_NONE:
    default:
        break;
    }
}

uint8_t USARTIsReady(USARTInstance *_instance)
{
    if (_instance == NULL || _instance->write_port == NULL)
        return 0U;

    if (_instance->write_port->queue_info.size >= _instance->write_port->queue_info.capacity)
        return 0U;

    return 1U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    UNUSED(Size);

    for (uint8_t i = 0U; i < idx; ++i)
    {
        USARTInstance *instance = usart_instance[i];
        USARTDriverContext *context;
        uint16_t curr_pos;

        if (huart != instance->usart_handle)
            continue;

        context = USARTGetDriverContext(instance);
        if (context == NULL || huart->hdmarx == NULL)
            return;

        /* 与 libxr 一致：使用 NDTR 反推当前 DMA 写入位置，而不是依赖回调的 Size。 */
        curr_pos =
            (uint16_t)(context->rx_dma_size - __HAL_DMA_GET_COUNTER(huart->hdmarx));

        if (curr_pos != context->rx_last_pos)
        {
            USART_Status_e push_status;

            if (curr_pos > context->rx_last_pos)
            {
                push_status = USARTAsyncReadPortPushBytes(
                    instance->read_port, &context->rx_dma_buffer[context->rx_last_pos],
                    (uint16_t)(curr_pos - context->rx_last_pos), 1U);
            }
            else
            {
                push_status = USARTAsyncReadPortPushBytes(
                    instance->read_port, &context->rx_dma_buffer[context->rx_last_pos],
                    (uint16_t)(context->rx_dma_size - context->rx_last_pos), 1U);
                if (push_status == USART_STATUS_OK && curr_pos > 0U)
                {
                    push_status = USARTAsyncReadPortPushBytes(
                        instance->read_port, &context->rx_dma_buffer[0], curr_pos, 1U);
                }
            }

            if (push_status != USART_STATUS_OK)
            {
                LOGWARNING("[bsp_usart] RX FIFO push failed, status [%d]", push_status);
            }

            context->rx_last_pos = curr_pos;
            USARTLegacyTryDispatchFrames(instance, 1U);
        }
        return;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0U; i < idx; ++i)
    {
        USARTInstance *instance = usart_instance[i];
        USARTDriverContext *context;

        if (huart != instance->usart_handle)
            continue;

        context = USARTGetDriverContext(instance);
        if (context == NULL)
            return;

        context->tx_dma_active = 0U;
        USARTAsyncWritePortFinishActive(instance->write_port, USART_STATUS_OK, 1U);
        (void)USARTWritePortKick(instance, 1U);
        return;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0U; i < idx; ++i)
    {
        USARTInstance *instance = usart_instance[i];
        USARTDriverContext *context;

        if (huart != instance->usart_handle)
            continue;

        context = USARTGetDriverContext(instance);
        if (context != NULL)
        {
            context->tx_dma_active = 0U;
        }
        USARTServiceInit(instance);
        (void)USARTWritePortKick(instance, 1U);
        LOGWARNING("[bsp_usart] USART error callback triggered, instance idx [%d]", i);
        return;
    }
}
