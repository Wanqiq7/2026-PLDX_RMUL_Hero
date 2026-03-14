/**
 * @file rm_referee.c
 * @author kidneygood
 * @brief 裁判系统模块
 * @version 0.1
 * @date 2022-11-18
 */

#include "rm_referee.h"

#include <string.h>

#include "bsp_log.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "crc_ref.h"
#include "daemon.h"
#include "task.h"

#define RE_RX_BUFFER_SIZE 512u // 裁判系统接收缓冲区大小（覆盖0x0310大包全帧）
#define REFEREE_PARSE_STEP_LIMIT 16u
#define REFEREE_MIN_FRAME_LEN (LEN_HEADER + LEN_CMDID + LEN_TAIL)

typedef enum
{
    REFEREE_RX_WAIT_SOF = 0,
    REFEREE_RX_WAIT_HEADER,
    REFEREE_RX_WAIT_BODY,
} Referee_Rx_State_e;

typedef struct
{
    Referee_Rx_State_e state;
    uint16_t expected_frame_len;
    uint16_t pending_read_len;
    uint8_t read_pending;
    uint8_t frame_buffer[RE_RX_BUFFER_SIZE];
    uint8_t read_buffer[RE_RX_BUFFER_SIZE];
    volatile USART_Polling_Status_e polling_status;
    USART_Operation_s polling_operation;
} Referee_Rx_Parser_s;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(RE_RX_BUFFER_SIZE <= USART_RXBUFF_LIMIT,
               "RE_RX_BUFFER_SIZE must not exceed USART_RXBUFF_LIMIT");
_Static_assert(RE_RX_BUFFER_SIZE >= (LEN_HEADER + LEN_CMDID + IMAGE_INTERACTIVE_MAX_DATA_LEN + LEN_TAIL),
               "RE_RX_BUFFER_SIZE must cover max interactive frame");
#endif

static USARTInstance *referee_usart_instance;
static DaemonInstance *referee_daemon;
static referee_info_t referee_info;
static Referee_Rx_Parser_s referee_rx_parser;

static void RefereeResetParser(void)
{
    memset(&referee_rx_parser, 0, sizeof(referee_rx_parser));
    referee_rx_parser.state = REFEREE_RX_WAIT_SOF;
    referee_rx_parser.polling_status = USART_POLLING_READY;
}

static void JudgeParseFrame(const uint8_t *frame, uint16_t frame_len)
{
    uint16_t data_len;

    if (frame == NULL || frame_len < REFEREE_MIN_FRAME_LEN)
        return;

    memcpy(&referee_info.FrameHeader, frame, LEN_HEADER);
    referee_info.FrameHeader.DataLength =
        (uint16_t)frame[DATA_LENGTH] | ((uint16_t)frame[DATA_LENGTH + 1] << 8);
    referee_info.CmdID = (uint16_t)frame[5] | ((uint16_t)frame[6] << 8);
    data_len = referee_info.FrameHeader.DataLength;

    switch (referee_info.CmdID)
    {
    case ID_game_state:
        memcpy(&referee_info.GameState, frame + DATA_Offset, LEN_game_state);
        break;
    case ID_game_result:
        memcpy(&referee_info.GameResult, frame + DATA_Offset, LEN_game_result);
        break;
    case ID_game_robot_survivors:
        memcpy(&referee_info.GameRobotHP, frame + DATA_Offset, LEN_game_robot_HP);
        break;
    case ID_event_data:
        memcpy(&referee_info.EventData, frame + DATA_Offset, LEN_event_data);
        break;
    case ID_supply_projectile_action:
        memcpy(&referee_info.SupplyProjectileAction, frame + DATA_Offset,
               LEN_supply_projectile_action);
        break;
    case ID_referee_warning:
    case ID_dart_info:
        break;
    case ID_game_robot_state:
        memcpy(&referee_info.GameRobotState, frame + DATA_Offset,
               LEN_game_robot_state);
        break;
    case ID_power_heat_data:
        memcpy(&referee_info.PowerHeatData, frame + DATA_Offset,
               LEN_power_heat_data);
        break;
    case ID_game_robot_pos:
        memcpy(&referee_info.GameRobotPos, frame + DATA_Offset, LEN_game_robot_pos);
        break;
    case ID_buff_musk:
        memcpy(&referee_info.BuffMusk, frame + DATA_Offset, LEN_buff_musk);
        break;
    case ID_aerial_robot_energy:
        memcpy(&referee_info.AerialRobotEnergy, frame + DATA_Offset,
               LEN_aerial_robot_energy);
        break;
    case ID_robot_hurt:
        memcpy(&referee_info.RobotHurt, frame + DATA_Offset, LEN_robot_hurt);
        break;
    case ID_shoot_data:
        memcpy(&referee_info.ShootData, frame + DATA_Offset, LEN_shoot_data);
        break;
    case ID_projectile_allowance:
    case ID_rfid_status:
    case ID_dart_client_cmd:
    case ID_ground_robot_position:
    case ID_radar_mark_data:
    case ID_sentry_info:
    case ID_radar_info:
    case ID_student_interactive:
    case ID_map_command:
    case ID_map_robot_data:
    case ID_map_path_data:
    case ID_map_robot_custom_data:
    case ID_custom_info_030A:
    case ID_custom_info_030B:
    case ID_custom_info_030C:
    case ID_custom_info_030D:
    case ID_custom_info_030E:
    case ID_custom_info_030F:
        break;
    case ID_map_interactive_data: // 0x0309
        if (data_len <= IMAGE_INTERACTIVE_MAX_DATA_LEN)
        {
            referee_info.MapInteractiveData.data_len = data_len;
            memset(referee_info.MapInteractiveData.data, 0,
                   sizeof(referee_info.MapInteractiveData.data));
            if (data_len > 0U)
            {
                memcpy(referee_info.MapInteractiveData.data, frame + DATA_Offset, data_len);
            }
            referee_info.MapInteractiveDataValid = 1U;
        }
        break;
    case ID_custom_info_0310: // 0x0310
        if (data_len <= IMAGE_INTERACTIVE_MAX_DATA_LEN)
        {
            referee_info.CustomInfo0310Data.data_len = data_len;
            memset(referee_info.CustomInfo0310Data.data, 0,
                   sizeof(referee_info.CustomInfo0310Data.data));
            if (data_len > 0U)
            {
                memcpy(referee_info.CustomInfo0310Data.data, frame + DATA_Offset, data_len);
            }
            referee_info.CustomInfo0310Valid = 1U;
        }
        break;
    case ID_custom_info_0311: // 0x0311
        if (data_len <= IMAGE_INTERACTIVE_MAX_DATA_LEN)
        {
            referee_info.CustomInfo0311Data.data_len = data_len;
            memset(referee_info.CustomInfo0311Data.data, 0,
                   sizeof(referee_info.CustomInfo0311Data.data));
            if (data_len > 0U)
            {
                memcpy(referee_info.CustomInfo0311Data.data, frame + DATA_Offset, data_len);
            }
            referee_info.CustomInfo0311Valid = 1U;
        }
        break;
    default:
        break;
    }
}

static uint8_t RefereeStartPollingRead(uint16_t read_len)
{
    USART_Status_e status;

    if (referee_usart_instance == NULL || read_len == 0U ||
        read_len > sizeof(referee_rx_parser.read_buffer))
        return 0U;

    USARTOperationInitPolling(&referee_rx_parser.polling_operation,
                              &referee_rx_parser.polling_status);
    referee_rx_parser.pending_read_len = read_len;
    referee_rx_parser.read_pending = 1U;

    status = USARTRead(referee_usart_instance, referee_rx_parser.read_buffer, read_len,
                       &referee_rx_parser.polling_operation, 0U);
    if (status == USART_STATUS_OK)
    {
        referee_rx_parser.polling_status = USART_POLLING_DONE;
        return 1U;
    }

    if (status == USART_STATUS_PENDING)
        return 1U;

    referee_rx_parser.read_pending = 0U;
    referee_rx_parser.pending_read_len = 0U;
    referee_rx_parser.polling_status = USART_POLLING_ERROR;
    return 0U;
}

static void RefereeHandleCompletedRead(void)
{
    uint16_t expected_body_len;

    switch (referee_rx_parser.state)
    {
    case REFEREE_RX_WAIT_SOF:
        if (referee_rx_parser.read_buffer[0] == REFEREE_SOF)
        {
            referee_rx_parser.frame_buffer[0] = REFEREE_SOF;
            referee_rx_parser.state = REFEREE_RX_WAIT_HEADER;
        }
        break;

    case REFEREE_RX_WAIT_HEADER:
        memcpy(&referee_rx_parser.frame_buffer[1], referee_rx_parser.read_buffer,
               LEN_HEADER - 1U);
        if (Verify_CRC8_Check_Sum(referee_rx_parser.frame_buffer, LEN_HEADER) != TRUE)
        {
            referee_rx_parser.state = REFEREE_RX_WAIT_SOF;
            break;
        }

        referee_rx_parser.expected_frame_len =
            (uint16_t)(referee_rx_parser.frame_buffer[DATA_LENGTH] |
                       ((uint16_t)referee_rx_parser.frame_buffer[DATA_LENGTH + 1] << 8));
        referee_rx_parser.expected_frame_len =
            (uint16_t)(referee_rx_parser.expected_frame_len + LEN_HEADER + LEN_CMDID +
                       LEN_TAIL);
        if (referee_rx_parser.expected_frame_len < REFEREE_MIN_FRAME_LEN ||
            referee_rx_parser.expected_frame_len > RE_RX_BUFFER_SIZE)
        {
            referee_rx_parser.state = REFEREE_RX_WAIT_SOF;
            break;
        }

        referee_rx_parser.state = REFEREE_RX_WAIT_BODY;
        break;

    case REFEREE_RX_WAIT_BODY:
        expected_body_len =
            (uint16_t)(referee_rx_parser.expected_frame_len - LEN_HEADER);
        memcpy(&referee_rx_parser.frame_buffer[LEN_HEADER], referee_rx_parser.read_buffer,
               expected_body_len);

        if (Verify_CRC16_Check_Sum(referee_rx_parser.frame_buffer,
                                   referee_rx_parser.expected_frame_len) == TRUE)
        {
            JudgeParseFrame(referee_rx_parser.frame_buffer,
                            referee_rx_parser.expected_frame_len);
        }
        referee_rx_parser.state = REFEREE_RX_WAIT_SOF;
        referee_rx_parser.expected_frame_len = 0U;
        break;

    default:
        referee_rx_parser.state = REFEREE_RX_WAIT_SOF;
        break;
    }

    referee_rx_parser.read_pending = 0U;
    referee_rx_parser.pending_read_len = 0U;
    referee_rx_parser.polling_status = USART_POLLING_READY;
}

void RefereeProcess(void)
{
    uint8_t step = 0U;

    if (referee_usart_instance == NULL)
        return;

    for (step = 0U; step < REFEREE_PARSE_STEP_LIMIT; ++step)
    {
        if (referee_rx_parser.read_pending != 0U)
        {
            if (referee_rx_parser.polling_status == USART_POLLING_DONE)
            {
                DaemonReload(referee_daemon);
                RefereeHandleCompletedRead();
                continue;
            }

            if (referee_rx_parser.polling_status == USART_POLLING_ERROR)
            {
                RefereeResetParser();
            }
            break;
        }

        switch (referee_rx_parser.state)
        {
        case REFEREE_RX_WAIT_SOF:
            if (!RefereeStartPollingRead(1U))
                return;
            break;
        case REFEREE_RX_WAIT_HEADER:
            if (!RefereeStartPollingRead((uint16_t)(LEN_HEADER - 1U)))
                return;
            break;
        case REFEREE_RX_WAIT_BODY:
            if (referee_rx_parser.expected_frame_len <= LEN_HEADER)
            {
                RefereeResetParser();
                return;
            }
            if (!RefereeStartPollingRead(
                    (uint16_t)(referee_rx_parser.expected_frame_len - LEN_HEADER)))
                return;
            break;
        default:
            RefereeResetParser();
            return;
        }

        if (referee_rx_parser.polling_status != USART_POLLING_DONE)
            break;
    }
}

static void RefereeLostCallback(void *arg)
{
    UNUSED(arg);
    RefereeResetParser();
    USARTServiceInit(referee_usart_instance);
    LOGWARNING("[rm_ref] lost referee data");
}

referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
    USART_Init_Config_s conf;
    Daemon_Init_Config_s daemon_conf;

    memset(&conf, 0, sizeof(conf));
    conf.module_callback = NULL;
    conf.usart_handle = referee_usart_handle;
    conf.recv_buff_size = 0U;
    conf.rx_fifo_size = 1024U;
    conf.tx_fifo_size = 512U;
    conf.tx_queue_depth = 5U;
    referee_usart_instance = USARTRegister(&conf);
    RefereeResetParser();

    memset(&daemon_conf, 0, sizeof(daemon_conf));
    daemon_conf.callback = RefereeLostCallback;
    daemon_conf.owner_id = referee_usart_instance;
    daemon_conf.reload_count = 30U;
    referee_daemon = DaemonRegister(&daemon_conf);

    return &referee_info;
}

uint8_t RefereeIsOnline(void)
{
    if (referee_daemon == NULL)
        return 0U;
    return DaemonIsOnline(referee_daemon);
}

static uint8_t RefereeConsumeDataIfValid(uint8_t *valid_flag, const void *src_data,
                                         void *out_data, size_t data_size)
{
    uint8_t consumed = 0U;

    if (valid_flag == NULL)
        return 0U;

    taskENTER_CRITICAL();
    if (*valid_flag != 0U)
    {
        if (out_data != NULL && src_data != NULL && data_size > 0U)
            memcpy(out_data, src_data, data_size);
        *valid_flag = 0U;
        consumed = 1U;
    }
    taskEXIT_CRITICAL();
    return consumed;
}

uint8_t RefereeTryConsumeMapInteractiveData(ext_map_interactive_data_var_t *out_data)
{
    return RefereeConsumeDataIfValid(&referee_info.MapInteractiveDataValid,
                                     &referee_info.MapInteractiveData, out_data,
                                     sizeof(referee_info.MapInteractiveData));
}

uint8_t RefereeTryConsumeCustomInfo0310(ext_custom_info_0310_var_t *out_data)
{
    return RefereeConsumeDataIfValid(&referee_info.CustomInfo0310Valid,
                                     &referee_info.CustomInfo0310Data, out_data,
                                     sizeof(referee_info.CustomInfo0310Data));
}

uint8_t RefereeTryConsumeCustomInfo0311(ext_custom_info_0311_var_t *out_data)
{
    return RefereeConsumeDataIfValid(&referee_info.CustomInfo0311Valid,
                                     &referee_info.CustomInfo0311Data, out_data,
                                     sizeof(referee_info.CustomInfo0311Data));
}

void RefereeClearExtendedDataFlags(void)
{
    taskENTER_CRITICAL();
    referee_info.MapInteractiveDataValid = 0U;
    referee_info.CustomInfo0310Valid = 0U;
    referee_info.CustomInfo0311Valid = 0U;
    referee_info.MapInteractiveData.data_len = 0U;
    referee_info.CustomInfo0310Data.data_len = 0U;
    referee_info.CustomInfo0311Data.data_len = 0U;
    taskEXIT_CRITICAL();
}

void RefereeSend(uint8_t *send, uint16_t tx_len)
{
    USART_Operation_s none_operation;
    USART_Status_e status;

    USARTOperationInitNone(&none_operation);
    status = USARTWrite(referee_usart_instance, send, tx_len, &none_operation, 0U);
    if (status != USART_STATUS_PENDING && status != USART_STATUS_OK)
    {
        LOGWARNING("[rm_ref] enqueue tx failed, status [%d]", status);
    }
    /*
     * 2026 协议下，0x0301 机器人交互数据触发发送频率上限为 30Hz。
     * 这里保留统一的基础节流，避免连续突发发送挤占裁判链路带宽。
     * 35ms 对应约 28.6Hz，低于协议上限并为 DMA/调度留出余量。
     */
    osDelay(35U);
}
