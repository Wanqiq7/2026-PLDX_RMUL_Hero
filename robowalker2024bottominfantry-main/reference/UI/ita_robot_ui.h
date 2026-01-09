/**
 * @file ita_ui.h
 *
 * @author Thhos_htk
 *
 * @brief 裁判系统UI绘制
 * @version 0.1
 * @date 2024-04-15 0.1
 *
 * @copyright USTC-RoboWalker (c) 2024
 *
 */


#ifndef ITA_UI_H
#define ITA_UI_H

/* Includes ------------------------------------------------------------------*/
#include "2_Device/Referee/dvc_referee.h"
#include "1_Middleware/2_Algorithm/FSM/alg_fsm.h"
#include "4_Interaction/UI/ita_ui_message.h"

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief UI绘制状态
 *
 */
enum Enum_UI_Display_Status
{
    UI_Display_Status_DYNAMIC = 0,
    UI_Display_Status_STATIC,
};

class Class_UI;

/**
 * @brief UI图层状态有限自动机
 *
 */
class Class_FSM_UI_Layer : public Class_FSM
{
public:
    Class_UI *UI;

    void Update_UI_Layer();
};

class Class_UI
{
public:
    // 类
    Class_FSM_UI_Layer FSM_UI_Layer;
    Class_UI_Message UI_Message;

    friend class Class_FSM_UI_Layer;

    // 函数
    void Init();

    void TIM_110ms_PeriodElapsedCallback();

    void UI_Reset();

    inline void Set_Current_UI_Layer(uint8_t __Current_UI_Layer);

    inline void Set_Current_UI_Display_Status(Enum_UI_Display_Status __UI_Display_Status);

    inline uint8_t Get_Current_UI_Layer();

    inline Enum_UI_Display_Status Get_Current_UI_Display_Status();

protected:
    // 常量
    uint16_t UI_Middle_X = 980;

    // 变量
    Enum_UI_Display_Status UI_Display_Status = UI_Display_Status_STATIC;

    // 函数
    void Draw_UI_Layer_1();

    void Draw_UI_Layer_2();

    void Draw_UI_Layer_3();

    void Draw_UI_Layer_4();

    void Draw_UI_Layer_5();

    void Draw_UI_Layer_6();

    void Draw_UI_Layer_7();

    void Draw_UI_Layer_8();

    void Draw_UI_Layer_9();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 设置下一个绘制的UI图层
 *
 */
inline void Class_UI::Set_Current_UI_Layer(uint8_t __Current_UI_Layer)
{
    FSM_UI_Layer.Set_Status(__Current_UI_Layer);
}

/**
 * @brief 设置UI显示状态
 *
 */
inline void Class_UI::Set_Current_UI_Display_Status(Enum_UI_Display_Status __UI_Display_Status)
{
    UI_Display_Status = __UI_Display_Status;
}

/**
 * @brief 获取当前即将绘制的UI图层
 *
 */
inline uint8_t Class_UI::Get_Current_UI_Layer()
{
    return FSM_UI_Layer.Get_Now_Status_Serial();
}

/**
 * @brief 获取UI显示状态
 *
 */
inline Enum_UI_Display_Status Class_UI::Get_Current_UI_Display_Status()
{
    return UI_Display_Status;
}

#endif