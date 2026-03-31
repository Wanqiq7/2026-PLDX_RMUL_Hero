#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

#define REMOTE_CONTROL_FRAME_SIZE 18u // 遥控器接收的buffer大小

// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

typedef struct
{
    uint8_t read_pending;
    uint8_t frame_buffer[REMOTE_CONTROL_FRAME_SIZE];
    volatile USART_Polling_Status_e polling_status;
    USART_Operation_s polling_operation;
} Remote_Control_Rx_Context_s;

static Remote_Control_Rx_Context_s rc_rx_context;

/**
 * @brief 将 16bit 键盘掩码同步到 Key_t 结构体
 *
 * @param key_state 键盘状态结构体
 * @param key_mask DT7/DR16 原始键盘位图
 */
static void DecodeKeyMask(Key_t *key_state, uint16_t key_mask)
{
    memset(key_state, 0, sizeof(Key_t));
    key_state->w = (key_mask >> Key_W) & 0x1u;
    key_state->s = (key_mask >> Key_S) & 0x1u;
    key_state->a = (key_mask >> Key_A) & 0x1u;
    key_state->d = (key_mask >> Key_D) & 0x1u;
    key_state->q = (key_mask >> Key_Q) & 0x1u;
    key_state->e = (key_mask >> Key_E) & 0x1u;
    key_state->shift = (key_mask >> Key_Shift) & 0x1u;
    key_state->ctrl = (key_mask >> Key_Ctrl) & 0x1u;
    key_state->r = (key_mask >> Key_R) & 0x1u;
    key_state->f = (key_mask >> Key_F) & 0x1u;
    key_state->g = (key_mask >> Key_G) & 0x1u;
    key_state->z = (key_mask >> Key_Z) & 0x1u;
    key_state->x = (key_mask >> Key_X) & 0x1u;
    key_state->c = (key_mask >> Key_C) & 0x1u;
    key_state->v = (key_mask >> Key_V) & 0x1u;
    key_state->b = (key_mask >> Key_B) & 0x1u;
}

/**
 * @brief 将 Key_t 结构体转换回 16bit 掩码
 *
 * @param key_state 键盘状态结构体
 * @return uint16_t 键盘位图
 */
static uint16_t EncodeKeyMask(const Key_t *key_state)
{
    uint16_t key_mask = 0u;
    key_mask |= (uint16_t)(key_state->w & 0x1u) << Key_W;
    key_mask |= (uint16_t)(key_state->s & 0x1u) << Key_S;
    key_mask |= (uint16_t)(key_state->a & 0x1u) << Key_A;
    key_mask |= (uint16_t)(key_state->d & 0x1u) << Key_D;
    key_mask |= (uint16_t)(key_state->q & 0x1u) << Key_Q;
    key_mask |= (uint16_t)(key_state->e & 0x1u) << Key_E;
    key_mask |= (uint16_t)(key_state->shift & 0x1u) << Key_Shift;
    key_mask |= (uint16_t)(key_state->ctrl & 0x1u) << Key_Ctrl;
    key_mask |= (uint16_t)(key_state->r & 0x1u) << Key_R;
    key_mask |= (uint16_t)(key_state->f & 0x1u) << Key_F;
    key_mask |= (uint16_t)(key_state->g & 0x1u) << Key_G;
    key_mask |= (uint16_t)(key_state->z & 0x1u) << Key_Z;
    key_mask |= (uint16_t)(key_state->x & 0x1u) << Key_X;
    key_mask |= (uint16_t)(key_state->c & 0x1u) << Key_C;
    key_mask |= (uint16_t)(key_state->v & 0x1u) << Key_V;
    key_mask |= (uint16_t)(key_state->b & 0x1u) << Key_B;
    return key_mask;
}

/**
 * @brief 将 RC 官方键位顺序重映射到工程内部 Key_t 顺序
 *
 * 官方顺序: W S A D Shift Ctrl Q E R F G Z X C V B
 * 工程顺序: W S A D Q E Shift Ctrl R F G Z X C V B
 *
 * @param raw_mask RC 原始键盘掩码
 * @return uint16_t 工程内部键盘位图
 */
static uint16_t RemoteControlRemapKeyboardMask(uint16_t raw_mask)
{
    uint16_t internal_mask = 0u;

    internal_mask |= raw_mask & 0x000Fu;
    internal_mask |= ((raw_mask >> 6) & 0x1u) << Key_Q;
    internal_mask |= ((raw_mask >> 7) & 0x1u) << Key_E;
    internal_mask |= ((raw_mask >> 4) & 0x1u) << Key_Shift;
    internal_mask |= ((raw_mask >> 5) & 0x1u) << Key_Ctrl;
    internal_mask |= raw_mask & 0xFF00u;

    return internal_mask;
}

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 4; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_rc(const uint8_t *sbus_buf)
{
    // 摇杆,直接解算时减去偏置
    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
    rc_ctrl[TEMP].rc.aux_raw16 = (uint16_t)(sbus_buf[16] | (sbus_buf[17] << 8)); // 协议尾部16bit原始值
    RectifyRCjoystick();
    // 开关,0左1右
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // 鼠标解析
    rc_ctrl[TEMP].mouse.x = (int16_t)(sbus_buf[6] | (sbus_buf[7] << 8));   //!< Mouse X axis
    rc_ctrl[TEMP].mouse.y = (int16_t)(sbus_buf[8] | (sbus_buf[9] << 8));   //!< Mouse Y axis
    rc_ctrl[TEMP].mouse.z = (int16_t)(sbus_buf[10] | (sbus_buf[11] << 8)); //!< Mouse Z axis
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?

    // RC 原始键盘位图遵循官方顺序: W,S,A,D,Shift,Ctrl,Q,E,R,F,G,Z,X,C,V,B
    uint16_t key_now =
        RemoteControlRemapKeyboardMask((uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8)));
    uint16_t key_last = EncodeKeyMask(&rc_ctrl[LAST].key[KEY_PRESS]);
    uint16_t key_last_with_ctrl =
        EncodeKeyMask(&rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL]);
    uint16_t key_last_with_shift =
        EncodeKeyMask(&rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT]);
    uint16_t key_with_ctrl;
    uint16_t key_with_shift;

    DecodeKeyMask(&rc_ctrl[TEMP].key[KEY_PRESS], key_now);
    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
    {
        DecodeKeyMask(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], key_now);
    }
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
    {
        DecodeKeyMask(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], key_now);
    }
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS].ctrl ? key_now : 0u;
    key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS].shift ? key_now : 0u;

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == Key_Shift || i == Key_Ctrl) // Shift/Ctrl 为组合键修饰位,不参与普通按键计数
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }
    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断

}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlResetRxContext(void)
{
    memset(&rc_rx_context, 0, sizeof(rc_rx_context));
    rc_rx_context.polling_status = USART_POLLING_READY;
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    RemoteControlResetRxContext();
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    memset(&conf, 0, sizeof(conf));
    conf.module_callback = NULL;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = 0U;
    conf.rx_fifo_size = 128U;
    conf.tx_fifo_size = 64U;
    conf.tx_queue_depth = 4U;
    rc_usart_instance = USARTRegister(&conf);
    RemoteControlResetRxContext();

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}

void RemoteControlProcess(void)
{
    USART_Status_e status;

    if (!rc_init_flag || rc_usart_instance == NULL)
        return;

    if (rc_rx_context.read_pending == 0U)
    {
        USARTOperationInitPolling(&rc_rx_context.polling_operation,
                                  &rc_rx_context.polling_status);
        status = USARTRead(rc_usart_instance, rc_rx_context.frame_buffer,
                           REMOTE_CONTROL_FRAME_SIZE, &rc_rx_context.polling_operation, 0U);
        if (status == USART_STATUS_OK)
        {
            rc_rx_context.polling_status = USART_POLLING_DONE;
            rc_rx_context.read_pending = 1U;
        }
        else if (status == USART_STATUS_PENDING)
        {
            rc_rx_context.read_pending = 1U;
        }
        else
        {
            rc_rx_context.read_pending = 0U;
            rc_rx_context.polling_status = USART_POLLING_ERROR;
        }
    }

    if (rc_rx_context.read_pending != 0U &&
        rc_rx_context.polling_status == USART_POLLING_DONE)
    {
        DaemonReload(rc_daemon_instance);
        sbus_to_rc(rc_rx_context.frame_buffer);
        rc_rx_context.read_pending = 0U;
        rc_rx_context.polling_status = USART_POLLING_READY;
    }
    else if (rc_rx_context.polling_status == USART_POLLING_ERROR)
    {
        RemoteControlResetRxContext();
    }
}
