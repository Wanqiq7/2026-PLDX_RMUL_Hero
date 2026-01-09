/**
 * @file ita_ui.cpp
 * @author Thhos_htk
 *
 * @brief 裁判系统UI绘制
 * @version 0.1
 * @date 2024-04-15 0.1
 *
 * @copyright USTC-RoboWalker (c) 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "ita_robot_ui.h"
#include "5_Task/tsk_config_and_callback.h"
#include "4_Interaction/ita_robot.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

extern Class_Robot robot;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 图层选择与绘制有限自动机
 *
 */
void Class_FSM_UI_Layer::Update_UI_Layer()
{
    if (UI->Get_Current_UI_Display_Status() == UI_Display_Status_STATIC)
    {
        // 清屏重置静态UI状态
        switch (Now_Status_Serial)
        {
        case (9):
        {
            UI->Draw_UI_Layer_9();
            UI->Set_Current_UI_Layer(8);
            break;
        }
        case (8):
        {
            UI->Draw_UI_Layer_8();
            UI->Set_Current_UI_Layer(7);
            break;
        }
        case (7):
        {
            UI->Draw_UI_Layer_7();
            UI->Set_Current_UI_Layer(6);
            break;
        }
        case (6):
        {
            UI->Draw_UI_Layer_6();
            UI->Set_Current_UI_Layer(5);
            break;
        }
        case (5):
        {
            UI->Draw_UI_Layer_5();
            UI->Set_Current_UI_Layer(4);
            break;
        }
        case (4):
        {
            UI->Draw_UI_Layer_4();
            UI->Set_Current_UI_Layer(3);
            break;
        }
        case (3):
        {
            UI->Draw_UI_Layer_3();
            UI->Set_Current_UI_Layer(7);
            UI->UI_Display_Status = UI_Display_Status_DYNAMIC;
            break;
        }
        default:
        {
            UI->Set_Current_UI_Layer(9);
            break;
        }
        }
    }
    else if (UI->Get_Current_UI_Display_Status() == UI_Display_Status_DYNAMIC)
    {
        // 正常绘制动态UI状态
        switch (Now_Status_Serial)
        {
        case (7):
        {
            UI->Draw_UI_Layer_7();
            UI->Set_Current_UI_Layer(6);
            break;
        }
        case (6):
        {
            UI->Draw_UI_Layer_6();
            UI->Set_Current_UI_Layer(5);
            break;
        }
        case (5):
        {
            UI->Draw_UI_Layer_5();
            UI->Set_Current_UI_Layer(4);
            break;
        }
        case (4):
        {
            UI->Draw_UI_Layer_4();
            UI->Set_Current_UI_Layer(3);
            break;
        }
        case (3):
        {
            UI->Draw_UI_Layer_3();
            UI->Set_Current_UI_Layer(7);
            break;
        }
        default:
        {
            UI->Set_Current_UI_Layer(7);
            break;
        }
        }
    }
}

/**
 * @brief UI初始化
 *
 */
void Class_UI::Init()
{
    FSM_UI_Layer.UI = this;
    FSM_UI_Layer.Init(10, 9);
    UI_Message.Init();
    UI_Display_Status = UI_Display_Status_STATIC;
}

void Class_UI::TIM_110ms_PeriodElapsedCallback()
{
    UI_Message.TIM_110ms_Warning_PeriodElapsedCallback();
    FSM_UI_Layer.Update_UI_Layer();
}

/**
 * @brief 重置整个UI
 *
 */
void Class_UI::UI_Reset()
{
    for (uint8_t layer_num = 0; layer_num < 10; layer_num++)
    {
        for (uint8_t graphic_num = 0; graphic_num < 10; graphic_num++)
        {
            robot.Referee.Set_Referee_UI_Clear(layer_num, graphic_num);
        }
    }
    Set_Current_UI_Display_Status(UI_Display_Status_STATIC);
    Set_Current_UI_Layer(9);
}
/* Private functions --------------------------------------------------------*/

/**
 * @brief 绘制图层9的内容
 * @note 静态，包括准星，倒车雷达
 *
 * @param
 * @return
 */
void Class_UI::Draw_UI_Layer_9()
{
    // 弹道竖线
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_booster_aiming_straight_line = robot.Referee.Set_Referee_UI_Line(9, 0, Referee_Data_Interaction_Graphic_Color_YELLOW, 1, UI_Middle_X, 240, UI_Middle_X, 560);
    // 远距准星线
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_booster_far_line = robot.Referee.Set_Referee_UI_Rectangle(9, 1, Referee_Data_Interaction_Graphic_Color_YELLOW, 1, UI_Middle_X - 20, 500, UI_Middle_X + 20, 500);
    // 近战准星线
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_booster_near_line = robot.Referee.Set_Referee_UI_Rectangle(9, 2, Referee_Data_Interaction_Graphic_Color_YELLOW, 1, UI_Middle_X - 140, 400, UI_Middle_X + 140, 400);
    // 左倒车雷达
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_left_radar = robot.Referee.Set_Referee_UI_Line(8, 5, Referee_Data_Interaction_Graphic_Color_MAIN, 2, 500, 0, 650, 200);
    // 右倒车雷达
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_right_radar = robot.Referee.Set_Referee_UI_Line(8, 6, Referee_Data_Interaction_Graphic_Color_MAIN, 2, 1420, 0, 1270, 200);
    // 发送当前五个图形
    robot.Referee.UART_Send_Interaction_UI_Graphic_5(graphic_booster_aiming_straight_line, graphic_booster_far_line, graphic_booster_near_line, graphic_left_radar, graphic_right_radar);
}

/**
 * @brief 绘制图层8的内容
 * @note 包括电容外框，功率限制外框，枪管指向，摩擦轮转速框，自瞄框，倒车雷达，静态
 *
 * @param
 * @return
 */
void Class_UI::Draw_UI_Layer_8()
{
    // 电容外框
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_capacitor_frame = robot.Referee.Set_Referee_UI_Rectangle(8, 0, Referee_Data_Interaction_Graphic_Color_YELLOW, 1, 760, 175, 1160, 209);
    // 功率限制外框
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_power_limit_frame = robot.Referee.Set_Referee_UI_Rectangle(8, 1, Referee_Data_Interaction_Graphic_Color_YELLOW, 3, 1700, 700, 1850, 780);
    // 枪管指向
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_gimbal_direction = robot.Referee.Set_Referee_UI_Line(8, 2, Referee_Data_Interaction_Graphic_Color_PURPLE, 5, 960, 90, 960, 150);
    // 摩擦轮转速框
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_booster_friction_motor_speed_frame = robot.Referee.Set_Referee_UI_Rectangle(8, 3, Referee_Data_Interaction_Graphic_Color_CYAN, 3, 50, 650, 250, 800);
    // 自瞄框
    Enum_Referee_Data_Interaction_Graphic_Color auto_aiming_frame_color;
    if (robot.Manifold.Get_Manifold_Error() < 10.0f)
    {
        auto_aiming_frame_color = Referee_Data_Interaction_Graphic_Color_WHITE;
    }
    else if (robot.Manifold.Get_Manifold_Error() < 20.0f)
    {
        auto_aiming_frame_color = Referee_Data_Interaction_Graphic_Color_YELLOW;
    }
    else
    {
        auto_aiming_frame_color = Referee_Data_Interaction_Graphic_Color_ORANGE;
    }
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_auto_aiming_frame = robot.Referee.Set_Referee_UI_Rectangle(8, 4, auto_aiming_frame_color, 1, 750, 310, 1240, 670);
    // 发送当前五个图形
    robot.Referee.UART_Send_Interaction_UI_Graphic_5(graphic_capacitor_frame, graphic_power_limit_frame, graphic_gimbal_direction, graphic_booster_friction_motor_speed_frame, graphic_auto_aiming_frame);
}

/**
 * @brief 绘制图层7的内容
 * @note 动态，包括目前功率限制，摩擦轮转速，车体姿态（4直线），射频
 *
 * @param
 * @return
 */
void Class_UI::Draw_UI_Layer_7()
{

    //底盘四点坐标
    //右上为0，逆时针0123
    static uint16_t coordinate_x[4];
    static uint16_t coordinate_y[4];
    coordinate_x[0] = (uint16_t) (40 * cosf(PI / 4.0f - robot.Gimbal.Get_Now_Yaw_Angle()) + 960);
    coordinate_x[1] = (uint16_t) (40 * cosf(3.0f * PI / 4.0f - robot.Gimbal.Get_Now_Yaw_Angle()) + 960);
    coordinate_x[2] = (uint16_t) (40 * cosf(-3.0f * PI / 4.0f - robot.Gimbal.Get_Now_Yaw_Angle()) + 960);
    coordinate_x[3] = (uint16_t) (40 * cosf(-PI / 4.0f - robot.Gimbal.Get_Now_Yaw_Angle()) + 960);
    coordinate_y[0] = (uint16_t) (40 * sinf(PI / 4.0f - robot.Gimbal.Get_Now_Yaw_Angle()) + 90);
    coordinate_y[1] = (uint16_t) (40 * sinf(3.0f * PI / 4.0f - robot.Gimbal.Get_Now_Yaw_Angle()) + 90);
    coordinate_y[2] = (uint16_t) (40 * sinf(-3.0f * PI / 4.0f - robot.Gimbal.Get_Now_Yaw_Angle()) + 90);
    coordinate_y[3] = (uint16_t) (40 * sinf(-PI / 4.0f - robot.Gimbal.Get_Now_Yaw_Angle()) + 90);

    // 当前功率限制
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_power_limit = robot.Referee.Set_Referee_UI_Integer(7, 0, Referee_Data_Interaction_Graphic_Color_PINK, 5, 1720, 750, 40, robot.Referee.Get_Self_Chassis_Power_Max());
    // 摩擦轮转速
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_booster_speed;
    if (robot.Booster.Get_Booster_Control_Type() == Booster_Control_Type_DISABLE)
    {
        graphic_booster_speed = robot.Referee.Set_Referee_UI_Integer(7, 1, Referee_Data_Interaction_Graphic_Color_WHITE, 5, 70, 770, 30, (int) (robot.Booster.Get_Friction_Omega() * 180.0f / PI));
    }
    else
    {
        graphic_booster_speed = robot.Referee.Set_Referee_UI_Integer(7, 1, Referee_Data_Interaction_Graphic_Color_PURPLE, 5, 70, 770, 30, (int) (robot.Booster.Get_Friction_Omega() * 180.0f / PI));
    }
    // 车体姿态
    Enum_Referee_Data_Interaction_Graphic_Color chassis_pos_line_color;
    if (robot.Chassis_Correction_Mode_Status == true)
    {
        chassis_pos_line_color = Referee_Data_Interaction_Graphic_Color_GREEN;
    }
    else
    {
        chassis_pos_line_color = Referee_Data_Interaction_Graphic_Color_BLACK;
    }
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_chassis_pos_1 = robot.Referee.Set_Referee_UI_Line(7, 2, chassis_pos_line_color, 3, coordinate_x[0], coordinate_y[0], coordinate_x[1], coordinate_y[1]);
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_chassis_pos_2 = robot.Referee.Set_Referee_UI_Line(7, 3, chassis_pos_line_color, 3, coordinate_x[1], coordinate_y[1], coordinate_x[2], coordinate_y[2]);
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_chassis_pos_3 = robot.Referee.Set_Referee_UI_Line(7, 4, Referee_Data_Interaction_Graphic_Color_MAIN, 3, coordinate_x[2], coordinate_y[2], coordinate_x[3], coordinate_y[3]);
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_chassis_pos_4 = robot.Referee.Set_Referee_UI_Line(7, 5, chassis_pos_line_color, 3, coordinate_x[3], coordinate_y[3], coordinate_x[0], coordinate_y[0]);
    // 枪管指向
    Enum_Referee_Data_Interaction_Graphic_Color gimbal_direction_color;
    if (robot.Burst_Mode_Status == true)
    {
        gimbal_direction_color = Referee_Data_Interaction_Graphic_Color_PURPLE;
    }
    else
    {
        gimbal_direction_color = Referee_Data_Interaction_Graphic_Color_WHITE;
    }
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_gimbal_direction = robot.Referee.Set_Referee_UI_Line(8, 2, gimbal_direction_color, 5, 960, 90, 960, 150);

    // 发送当前七个图形
    robot.Referee.UART_Send_Interaction_UI_Graphic_7(graphic_power_limit, graphic_booster_speed, graphic_chassis_pos_1, graphic_chassis_pos_2, graphic_chassis_pos_3, graphic_chassis_pos_4, graphic_gimbal_direction);
}

/**
 * @brief 绘制图层6的内容
 * @note 底盘状态
 *
 * @param
 * @return
 */
void Class_UI::Draw_UI_Layer_6()
{
    char str_chassis_state[8];
    if (robot.Supercap_Status)
    {
        if (robot.Chassis_Gyroscope_Mode_Status == Robot_Gyroscope_Type_CLOCKWISE || robot.Chassis_Gyroscope_Mode_Status == Robot_Gyroscope_Type_COUNTERCLOCKWISE)
        {
            memcpy(str_chassis_state, "SGyro", 8);
        }
        else
        {
            memcpy(str_chassis_state, "SNormal", 8);
        }
    }
    else
    {
        if (robot.Chassis_Gyroscope_Mode_Status == Robot_Gyroscope_Type_CLOCKWISE || robot.Chassis_Gyroscope_Mode_Status == Robot_Gyroscope_Type_COUNTERCLOCKWISE)
        {
            memcpy(str_chassis_state, "Gyro", 8);
        }
        else
        {
            memcpy(str_chassis_state, "Normal", 8);
        }
    }
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_chassis_state = robot.Referee.Set_Referee_UI_String(6, 0, Referee_Data_Interaction_Graphic_Color_WHITE, 4, 1680, 600, 40, sizeof(str_chassis_state));
    // 发送当前一个图形
    robot.Referee.UART_Send_Interaction_UI_Graphic_String(graphic_chassis_state, str_chassis_state);
}

/**
 * @brief 绘制图层5的内容
 * @note 超级电容电量，动态
 *
 * @param
 * @return
 */
void Class_UI::Draw_UI_Layer_5()
{
    Enum_Referee_Data_Interaction_Graphic_Color supercap_status_color;
    uint16_t supercap_energy = robot.Supercap.Get_Now_Energy();
    if (robot.Supercap.Get_Supercap_Status() == RM24_Supercap_Status_DISABLE || robot.Supercap_Enable == false)
    {
        supercap_status_color = Referee_Data_Interaction_Graphic_Color_BLACK;
    }
    else if (robot.Supercap_Status == true)
    {
        supercap_status_color = Referee_Data_Interaction_Graphic_Color_MAIN;
    }
    else
    {
        if (supercap_energy > 1000)
        {
            supercap_status_color = Referee_Data_Interaction_Graphic_Color_GREEN;
        }
        else if (supercap_energy > 500)
        {
            supercap_status_color = Referee_Data_Interaction_Graphic_Color_YELLOW;
        }
        else
        {
            supercap_status_color = Referee_Data_Interaction_Graphic_Color_PINK;
        }
    }
    // 超级电容电量条
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_supercap_energy_line = robot.Referee.Set_Referee_UI_Line(5, 0, supercap_status_color, 30, 760, 192, 760 + supercap_energy / 5, 192);
    // 超级电容电量数值
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_supercap_energy = robot.Referee.Set_Referee_UI_Integer(5, 1, supercap_status_color, 3, 1000, 160, 25, supercap_energy);
    // 发送当前两个图形
    robot.Referee.UART_Send_Interaction_UI_Graphic_2(graphic_supercap_energy_line, graphic_supercap_energy);
}

/**
 * @brief 绘制图层3的内容
 * @note 裁判系统调试信息
 *
 * @param
 * @return
 */
void Class_UI::Draw_UI_Layer_3()
{
    Enum_Referee_Data_Interaction_Graphic_Color referee_color;
    uint8_t id_stat;
    uint32_t start_x = 1550;
    uint32_t start_y = 360;
    if (robot.Referee.Get_Self_ID() == Referee_Data_Robots_ID_RED_INFANTRY_3 || robot.Referee.Get_Self_ID() == Referee_Data_Robots_ID_RED_INFANTRY_4 || robot.Referee.Get_Self_ID() == Referee_Data_Robots_ID_RED_INFANTRY_5)
    {
        referee_color = Referee_Data_Interaction_Graphic_Color_PINK;
        id_stat = 1;
    }
    else if (robot.Referee.Get_Self_ID() == Referee_Data_Robots_ID_BLUE_INFANTRY_3 || robot.Referee.Get_Self_ID() == Referee_Data_Robots_ID_BLUE_INFANTRY_4 || robot.Referee.Get_Self_ID() == Referee_Data_Robots_ID_BLUE_INFANTRY_5)
    {
        referee_color = Referee_Data_Interaction_Graphic_Color_CYAN;
        id_stat = 2;
    }
    else
    {
        referee_color = Referee_Data_Interaction_Graphic_Color_BLACK;
        id_stat = 0;
    }
    Struct_Referee_Data_Interaction_Graphic_Config *self_id = robot.Referee.Set_Referee_UI_Integer(3, 0, referee_color, 2, start_x, start_y, 20, id_stat);
    Struct_Referee_Data_Interaction_Graphic_Config *self_level = robot.Referee.Set_Referee_UI_Integer(3, 1, referee_color, 2, start_x + 30, start_y, 20, robot.Referee.Get_Self_Level());
    Struct_Referee_Data_Interaction_Graphic_Config *self_hp = robot.Referee.Set_Referee_UI_Integer(3, 2, referee_color, 2, start_x + 80, start_y, 20, robot.Referee.Get_Self_HP_Max());
    Struct_Referee_Data_Interaction_Graphic_Config *self_power = robot.Referee.Set_Referee_UI_Integer(3, 3, referee_color, 2, start_x + 160, start_y, 20, robot.Referee.Get_Self_Chassis_Power_Max());
    Struct_Referee_Data_Interaction_Graphic_Config *self_heat_max = robot.Referee.Set_Referee_UI_Integer(3, 4, referee_color, 2, start_x + 240, start_y, 20, robot.Referee.Get_Self_Booster_Heat_Max());
    robot.Referee.UART_Send_Interaction_UI_Graphic_5(self_id, self_level, self_hp, self_power, self_heat_max);
}

/**
 * @brief 绘制图层4的内容
 * @note 文本提示信息
 *
 * @param
 * @return
 */
void Class_UI::Draw_UI_Layer_4()
{
    Struct_Referee_Data_Interaction_Graphic_Config *graphic_chassis_state = robot.Referee.Set_Referee_UI_String(4, 0, Referee_Data_Interaction_Graphic_Color_CYAN, 5, 220, 800, 50, strlen(UI_Message.Get_Message()));

    robot.Referee.UART_Send_Interaction_UI_Graphic_String(graphic_chassis_state, UI_Message.Get_Message());
}