/**
 * @file referee_UI_warning.cpp
 * @author Thhos_htk
 * @brief 在UI上提供字符串信息
 * @version 0.1
 * @date 2024-3-29
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "ita_ui_message.h"
#include "4_Interaction/ita_robot.h"
#include "2_Device/Referee/dvc_referee.h"
#include "2_Device/Sampler/dvc_sampler.h"
#include "2_Device/Serialplot/dvc_serialplot.h"
#include "2_Device/Supercap/RM24_Supercap/dvc_rm24supercap.h"
#include "1_Middleware/1_Driver/BSP/drv_djiboarda.h"
#include "1_Middleware/1_Driver/TIM/drv_tim.h"

/* Private macros ------------------------------------------------------------*/
extern Class_Robot robot;
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 检查警告信息回调函数
 * @note
 *
 * @param
 * @return
 */

void Class_UI_Message::Init()
{
    Message_String = (char *) malloc(30 * sizeof(char));
    Message_Status = Referee_UI_Message_Status_DISABLE;
    Message_Priority = 100;
}

void Class_UI_Message::TIM_110ms_Warning_PeriodElapsedCallback()
{
    char referee_out_message[30];
    Message_Status = Referee_UI_Message_Status_DISABLE;
    if (robot.Chassis_Gyroscope_Mode_Status != Robot_Gyroscope_Type_DISABLE)
    {
        if (fabs(robot.Chassis.Get_Now_Omega() - robot.Chassis.Get_Target_Omega()) > 0.3f * robot.Chassis.Get_Omega_Max())
        {
            Update_Message("    gyro fail leave wall     ", 2);
        }
    }
    if (robot.Referee.Get_Chassis_Power() > (float) robot.Referee.Get_Self_Chassis_Power_Max() * 1.2f)
    {
        Update_Message("          slow speed         ", 2);
    }
    if (robot.Supercap.Get_Supercap_Status() == false)
    {
        if (robot.Supercap.Get_Now_Energy() < 500)
        {
            Update_Message("        charge supercap      ", 3);
        }
    }
    if (robot.Referee.Get_Self_PM01_Booster_Status() == Referee_Data_Status_DISABLE)
    {
        Update_Message("      booster deactivate     ", 3);
    }
    if (robot.Referee.Get_Game_Stage() == Referee_Game_Status_Stage_BATTLE)
    {
        if (robot.Referee.Get_17mm_Remaining() < 20)
        {
            Update_Message("         lack bullet         ", 4);
        }
    }
    if (robot.Referee.Get_Game_Stage() == Referee_Game_Status_Stage_READY || robot.Referee.Get_Game_Stage() == Referee_Game_Status_Stage_15s_SELF_TESTING || robot.Referee.Get_Game_Stage() == Referee_Game_Status_Stage_5S_COUNTDOWN)
    {
        Update_Message("         not in game         ", 4);
    }
    if (robot.Referee.Get_Referee_Trust_Status() == Referee_Data_Status_DISABLE || robot.Referee.Get_Referee_Status() == Referee_Status_DISABLE)
    {
        sprintf(referee_out_message, "    referee out level=%2ld     ", robot.Robot_Level);
        Update_Message(referee_out_message, 2);
    }
    else if (robot.Referee_Not_Trust_Enum > 200)
    {
        Update_Message("      referee unstable       ", 1);
    }
    if (robot.Chassis.AHRS_Chassis->Get_WHEELTEC_Status() == WHEELTEC_AHRS_Status_DISABLE)
    {
        Update_Message("      chassis ahrs fail      ", 2);
    }
    if (robot.Gimbal.AHRS_Gimbal->Get_WIT_Status() == WIT_AHRS_Status_DISABLE)
    {
        Update_Message("      gimbal ahrs fail       ", 1);
    }
    if (robot.Supercap.Get_Supercap_Status() == RM24_Supercap_Status_DISABLE)
    {
        Update_Message("        supercap fail        ", 1);
    }
    if (robot.Manifold.Get_Manifold_Status() == Manifold_Status_DISABLE)
    {
        Update_Message("         vision fail         ", 2);
    }
    if (robot.DR16.Get_DR16_Status() == DR16_Status_DISABLE)
    {
        Update_Message("       controller fail       ", 1);
    }
    if (robot.DR16.Get_Left_Switch() != DR16_Switch_Status_MIDDLE)
    {
        Update_Message("   controller switch wrong   ", 1);
    }
}

void Class_UI_Message::Update_Message(const char *Temp_Message, int8_t Temp_Priority)
{
    if (Temp_Priority >= Message_Priority && Message_Status == Referee_UI_Message_Status_ENABLE)
    {
        return;
    }
    strcpy(Message_String, Temp_Message);
    Message_Priority = Temp_Priority;
    Message_Status = Referee_UI_Message_Status_ENABLE;
}

char *Class_UI_Message::Get_Message()
{
    if (Message_Status == Referee_UI_Message_Status_DISABLE)
    {
        char *temp_char = (char *) malloc(30 * sizeof(char));
        strcpy(temp_char, "                             ");
        return temp_char;
    }
    return (Message_String);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
