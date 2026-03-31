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

#define RE_RX_BUFFER_SIZE 255u            // 裁判系统最大单帧长度
#define REFEREE_PARSE_STEP_LIMIT 16u      // 单次任务循环内最多推进的解析步数
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

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *referee_daemon;        // 裁判系统守护进程
static referee_info_t referee_info;           // 裁判系统数据
static Referee_Rx_Parser_s referee_rx_parser; // 裁判流式解析状态

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
    case ID_game_state: // 0x0001
        memcpy(&referee_info.GameState, (frame + DATA_Offset), LEN_game_state);
        break;
    case ID_game_result: // 0x0002
        memcpy(&referee_info.GameResult, (frame + DATA_Offset), LEN_game_result);
        break;
    case ID_game_robot_survivors: // 0x0003
        memcpy(&referee_info.GameRobotHP, (frame + DATA_Offset), LEN_game_robot_HP);
        break;
    case ID_event_data: // 0x0101
        memcpy(&referee_info.EventData, (frame + DATA_Offset), LEN_event_data);
        break;
    case ID_supply_projectile_action: // 0x0102
        memcpy(&referee_info.SupplyProjectileAction, (frame + DATA_Offset),
               LEN_supply_projectile_action);
        break;
    case ID_referee_warning: // 0x0104
        break;
    case ID_dart_info: // 0x0105
        break;
    case ID_game_robot_state: // 0x0201
        memcpy(&referee_info.GameRobotState, (frame + DATA_Offset),
               LEN_game_robot_state);
        break;
    case ID_power_heat_data: // 0x0202
        memcpy(&referee_info.PowerHeatData, (frame + DATA_Offset),
               LEN_power_heat_data);
        break;
    case ID_game_robot_pos: // 0x0203
        memcpy(&referee_info.GameRobotPos, (frame + DATA_Offset), LEN_game_robot_pos);
        break;
    case ID_buff_musk: // 0x0204
        memcpy(&referee_info.BuffMusk, (frame + DATA_Offset), LEN_buff_musk);
        break;
    case ID_aerial_robot_energy: // 0x0205
        memcpy(&referee_info.AerialRobotEnergy, (frame + DATA_Offset),
               LEN_aerial_robot_energy);
        break;
    case ID_robot_hurt: // 0x0206
        memcpy(&referee_info.RobotHurt, (frame + DATA_Offset), LEN_robot_hurt);
        break;
    case ID_shoot_data: // 0x0207
        memcpy(&referee_info.ShootData, (frame + DATA_Offset), LEN_shoot_data);
        break;
    case ID_projectile_allowance: // 0x0208
    case ID_rfid_status:          // 0x0209
    case ID_dart_client_cmd:      // 0x020A
    case ID_ground_robot_position:// 0x020B
    case ID_radar_mark_data:      // 0x020C
    case ID_sentry_info:          // 0x020D
    case ID_radar_info:           // 0x020E
        break;
    case ID_student_interactive: // 0x0301
    {
        uint16_t payload_len;

        if (data_len < STUDENT_INTERACTIVE_HEADER_LEN ||
            data_len > STUDENT_INTERACTIVE_MAX_FRAME_DATA_LEN)
            break;

        referee_info.ReceiveData.datahead.data_cmd_id =
            (uint16_t)frame[DATA_Offset] | ((uint16_t)frame[DATA_Offset + 1] << 8);
        referee_info.ReceiveData.datahead.sender_ID =
            (uint16_t)frame[DATA_Offset + 2] | ((uint16_t)frame[DATA_Offset + 3] << 8);
        referee_info.ReceiveData.datahead.receiver_ID =
            (uint16_t)frame[DATA_Offset + 4] | ((uint16_t)frame[DATA_Offset + 5] << 8);

        payload_len = data_len - STUDENT_INTERACTIVE_HEADER_LEN;
        if (payload_len > STUDENT_INTERACTIVE_MAX_DATA_LEN)
            break;

        referee_info.ReceiveData.data_len = payload_len;
        memset(referee_info.ReceiveData.data, 0, sizeof(referee_info.ReceiveData.data));
        if (payload_len > 0U)
        {
            memcpy(referee_info.ReceiveData.data,
                   frame + DATA_Offset + STUDENT_INTERACTIVE_HEADER_LEN, payload_len);
        }
        referee_info.ReceiveDataValid = 1U;
        break;
    }
    case ID_map_command: // 0x0303
        if (data_len == LEN_map_command)
        {
            memcpy(&referee_info.MapCommandData, (frame + DATA_Offset), LEN_map_command);
            referee_info.MapCommandValid = 1U;
        }
        break;
    case ID_map_robot_data: // 0x0305
        if (data_len == LEN_map_robot_data)
        {
            memcpy(&referee_info.MapRobotData, (frame + DATA_Offset),
                   LEN_map_robot_data);
            referee_info.MapRobotDataValid = 1U;
        }
        break;
    case ID_map_path_data: // 0x0307
        if (data_len == LEN_map_path_data)
        {
            memcpy(&referee_info.MapPathData, (frame + DATA_Offset), LEN_map_path_data);
            referee_info.MapPathDataValid = 1U;
        }
        break;
    case ID_map_robot_custom_data: // 0x0308
        if (data_len == LEN_map_robot_custom_data)
        {
            memcpy(&referee_info.MapRobotCustomData, (frame + DATA_Offset),
                   LEN_map_robot_custom_data);
            referee_info.MapRobotCustomDataValid = 1U;
        }
        break;
    case ID_map_interactive_data: // 0x0309
    case ID_custom_info_030A:     // 0x030A
    case ID_custom_info_030B:     // 0x030B
    case ID_custom_info_030C:     // 0x030C
    case ID_custom_info_030D:     // 0x030D
    case ID_custom_info_030E:     // 0x030E
    case ID_custom_info_030F:     // 0x030F
    case ID_custom_info_0310:     // 0x0310
    case ID_custom_info_0311:     // 0x0311
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

// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
    UNUSED(arg);
    RefereeResetParser();
    USARTServiceInit(referee_usart_instance);
    LOGWARNING("[rm_ref] lost referee data");
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
    USART_Init_Config_s conf;

    memset(&conf, 0, sizeof(conf));
    conf.module_callback = NULL; // 使用新 read port 机制,不再走 legacy fixed-length callback
    conf.usart_handle = referee_usart_handle;
    conf.recv_buff_size = 0U;
    conf.rx_fifo_size = 512U;
    conf.tx_fifo_size = 256U;
    conf.tx_queue_depth = 5U;
    referee_usart_instance = USARTRegister(&conf);
    RefereeResetParser();

    {
        Daemon_Init_Config_s daemon_conf = {
            .callback = RefereeLostCallback,
            .owner_id = referee_usart_instance,
            .reload_count = 30, // 0.3s没有收到数据,则认为丢失
        };
        referee_daemon = DaemonRegister(&daemon_conf);
    }

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

uint8_t RefereeTryConsumeReceiveData(robot_interactive_data_var_t *out_data)
{
    return RefereeConsumeDataIfValid(&referee_info.ReceiveDataValid,
                                     &referee_info.ReceiveData, out_data,
                                     sizeof(referee_info.ReceiveData));
}

uint8_t RefereeTryConsumeMapCommand(ext_map_command_t *out_data)
{
    return RefereeConsumeDataIfValid(&referee_info.MapCommandValid,
                                     &referee_info.MapCommandData, out_data,
                                     sizeof(referee_info.MapCommandData));
}

uint8_t RefereeTryConsumeMapRobotData(ext_map_robot_data_t *out_data)
{
    return RefereeConsumeDataIfValid(&referee_info.MapRobotDataValid,
                                     &referee_info.MapRobotData, out_data,
                                     sizeof(referee_info.MapRobotData));
}

uint8_t RefereeTryConsumeMapPathData(ext_map_path_data_t *out_data)
{
    return RefereeConsumeDataIfValid(&referee_info.MapPathDataValid,
                                     &referee_info.MapPathData, out_data,
                                     sizeof(referee_info.MapPathData));
}

uint8_t RefereeTryConsumeMapRobotCustomData(ext_map_robot_custom_data_t *out_data)
{
    return RefereeConsumeDataIfValid(&referee_info.MapRobotCustomDataValid,
                                     &referee_info.MapRobotCustomData, out_data,
                                     sizeof(referee_info.MapRobotCustomData));
}

void RefereeClearExtendedDataFlags(void)
{
    taskENTER_CRITICAL();
    referee_info.ReceiveDataValid = 0U;
    referee_info.MapCommandValid = 0U;
    referee_info.MapRobotDataValid = 0U;
    referee_info.MapPathDataValid = 0U;
    referee_info.MapRobotCustomDataValid = 0U;
    taskEXIT_CRITICAL();
}

/**
 * @brief 裁判系统数据发送函数
 */
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
    osDelay(115U);
}
