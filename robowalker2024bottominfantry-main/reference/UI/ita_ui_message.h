/**
 * @file referee_UI.h
 * @author gjc
 * @brief
 * @version 0.1
 * @date 2023-12-06
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#ifndef REFEREE_UI_WARNING_H
#define REFEREE_UI_WARNING_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <cstdio>
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UI警告触发类型
 *
 */
enum Enum_UI_Message_Status
{
    Referee_UI_Message_Status_DISABLE = 0,
    Referee_UI_Message_Status_ENABLE,
};

/**
 * @brief 警告类
 *
 */
class Class_UI_Message
{
public:
    void Init();

    void TIM_110ms_Warning_PeriodElapsedCallback();

    char *Get_Message();

    inline Enum_UI_Message_Status Get_Warning_Status();

protected:
    void Update_Message(const char *Temp_Message, int8_t Temp_Priority);

    Enum_UI_Message_Status Message_Status;
    char *Message_String;
    int8_t Message_Priority;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline Enum_UI_Message_Status Class_UI_Message::Get_Warning_Status()
{
    return (Message_Status);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
