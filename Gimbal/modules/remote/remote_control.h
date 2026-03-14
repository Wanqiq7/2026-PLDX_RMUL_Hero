/**
 * @file remote_control.h
 * @author DJI 2016
 * @author modified by neozng
 * @brief  遥控器模块定义头文件
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2016 DJI corp
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>
#include "main.h"
#include "usart.h"

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

// 获取按键操作
#define KEY_PRESS 0
#define KEY_STATE 1
#define KEY_PRESS_WITH_CTRL 1
#define KEY_PRESS_WITH_SHIFT 2

// 检查接收值是否出错
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)   // 开关向上时的值
#define RC_SW_MID ((uint16_t)3)  // 开关中间时的值
#define RC_SW_DOWN ((uint16_t)2) // 开关向下时的值
// 三个判断开关状态的宏
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

/* ----------------------- PC Key Definition-------------------------------- */
// 对应 DT7/DR16 手册中的 16bit 键盘位定义
// bit0~bit15 依次为: W S A D Q E Shift Ctrl R F G Z X C V B
#define Key_W 0
#define Key_S 1
#define Key_A 2
#define Key_D 3
#define Key_Q 4
#define Key_E 5
#define Key_Shift 6
#define Key_Ctrl 7
#define Key_R 8
#define Key_F 9
#define Key_G 10
#define Key_Z 11
#define Key_X 12
#define Key_C 13
#define Key_V 14
#define Key_B 15

/* ----------------------- Data Struct ------------------------------------- */
// 键盘状态结构体
// 注意: 该结构仅供上层逻辑读取, 协议解码时请以显式 bitmask 为准。
typedef struct
{
    uint16_t w : 1;
    uint16_t s : 1;
    uint16_t a : 1;
    uint16_t d : 1;
    uint16_t q : 1;
    uint16_t e : 1;
    uint16_t shift : 1;
    uint16_t ctrl : 1;
    uint16_t r : 1;
    uint16_t f : 1;
    uint16_t g : 1;
    uint16_t z : 1;
    uint16_t x : 1;
    uint16_t c : 1;
    uint16_t v : 1;
    uint16_t b : 1;
} Key_t;

// @todo 当前结构体嵌套过深,需要进行优化
typedef struct
{
    struct
    {
        int16_t rocker_l_; // 左水平
        int16_t rocker_l1; // 左竖直
        int16_t rocker_r_; // 右水平
        int16_t rocker_r1; // 右竖直
        uint16_t aux_raw16; // 协议尾部16bit原始值(兼容旧工程附加通道/拨杆反馈)

        uint8_t switch_left;  // 左侧开关
        uint8_t switch_right; // 右侧开关
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    Key_t key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍

    uint8_t key_count[3][16];
} RC_ctrl_t;

/* ------------------------- Internal Data ----------------------------------- */

/**
 * @brief 初始化遥控器,该函数会将遥控器注册到串口
 *
 * @attention 注意分配正确的串口硬件,遥控器在C板上使用USART3
 *
 */
RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle);

/**
 * @brief 检查遥控器是否在线,若尚未初始化也视为离线
 *
 * @return uint8_t 1:在线 0:离线
 */
uint8_t RemoteControlIsOnline();

/**
 * @brief 推进遥控器串口显式读取流程
 *
 * @note 需要在任务上下文中周期调用,避免在 UART ISR 中直接解析 SBUS 数据
 */
void RemoteControlProcess(void);

#endif
