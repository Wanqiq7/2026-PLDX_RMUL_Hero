#ifndef RM_REFEREE_H
#define RM_REFEREE_H

#include "usart.h"
#include "referee_protocol.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"

extern uint8_t UI_Seq;

/* -------------------------UI专用枚举定义（Module层）------------------------- */
// 底盘模式（UI显示用）
typedef enum {
  UI_CHASSIS_ZERO_FORCE = 0,    // 电流零输入
  UI_CHASSIS_ROTATE,            // 小陀螺模式
  UI_CHASSIS_NO_FOLLOW,         // 不跟随
  UI_CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式
} ui_chassis_mode_e;

// 云台模式（UI显示用）
typedef enum {
  UI_GIMBAL_ZERO_FORCE = 0, // 电流零输入
  UI_GIMBAL_FREE_MODE,      // 自由模式
  UI_GIMBAL_GYRO_MODE,      // 陀螺仪模式
} ui_gimbal_mode_e;

// 发射模式（UI显示用）
typedef enum {
  UI_SHOOT_OFF = 0,
  UI_SHOOT_ON,
} ui_shoot_mode_e;

// 摩擦轮模式（UI显示用）
typedef enum {
  UI_FRICTION_OFF = 0,
  UI_FRICTION_ON,
} ui_friction_mode_e;

// 功率数据（UI显示用）
typedef struct {
  float chassis_power_mx;
} UI_Chassis_Power_Data_s;

#pragma pack(1)
typedef struct
{
	uint8_t Robot_Color;		// 机器人颜色
	uint16_t Robot_ID;			// 本机器人ID
	uint16_t Cilent_ID;			// 本机器人对应的客户端ID
	uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

// 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
typedef struct
{
	referee_id_t referee_id;

	xFrameHeader FrameHeader; // 接收到的帧头信息
	uint16_t CmdID;
	ext_game_state_t GameState;							   // 0x0001
	ext_game_result_t GameResult;						   // 0x0002
	ext_game_robot_HP_t GameRobotHP;					   // 0x0003
	ext_event_data_t EventData;							   // 0x0101
	ext_supply_projectile_action_t SupplyProjectileAction; // 0x0102
	ext_game_robot_state_t GameRobotState;				   // 0x0201
	ext_power_heat_data_t PowerHeatData;				   // 0x0202
	ext_game_robot_pos_t GameRobotPos;					   // 0x0203
	ext_buff_musk_t BuffMusk;							   // 0x0204
	aerial_robot_energy_t AerialRobotEnergy;			   // 0x0205
	ext_robot_hurt_t RobotHurt;							   // 0x0206
	ext_shoot_data_t ShootData;							   // 0x0207

	// 图传链路扩展命令接收缓存（仅协议层落地）
	ext_map_interactive_data_var_t MapInteractiveData;
	ext_custom_info_0310_var_t CustomInfo0310Data;
	ext_custom_info_0311_var_t CustomInfo0311Data;
	uint8_t MapInteractiveDataValid;
	uint8_t CustomInfo0310Valid;
	uint8_t CustomInfo0311Valid;

	uint8_t init_flag;

} referee_info_t;

// 模式是否切换标志位，0为未切换，1为切换，static定义默认为0
typedef struct
{
	uint32_t chassis_flag : 1;
	uint32_t gimbal_flag : 1;
	uint32_t shoot_flag : 1;
	uint32_t friction_flag : 1;
	uint32_t Power_flag : 1;
} Referee_Interactive_Flag_t;

// 此结构体包含UI绘制与机器人车间通信的需要的其他非裁判系统数据
typedef struct
{
	Referee_Interactive_Flag_t Referee_Interactive_Flag;
	// 为UI绘制以及交互数据所用
	ui_chassis_mode_e chassis_mode;			 // 底盘模式
	ui_gimbal_mode_e gimbal_mode;				 // 云台模式
	ui_shoot_mode_e shoot_mode;				 // 发射模式设置
	ui_friction_mode_e friction_mode;			 // 摩擦轮关闭
	UI_Chassis_Power_Data_s Chassis_Power_Data; // 功率控制

	// 上一次的模式，用于flag判断
	ui_chassis_mode_e chassis_last_mode;
	ui_gimbal_mode_e gimbal_last_mode;
	ui_shoot_mode_e shoot_last_mode;
	ui_friction_mode_e friction_last_mode;
	UI_Chassis_Power_Data_s Chassis_last_Power_Data;

} Referee_Interactive_info_t;

#pragma pack()

/**
 * @brief 裁判系统通信初始化,该函数会初始化裁判系统串口,开启中断
 *
 * @param referee_usart_handle 串口handle,C板一般用串口6
 * @return referee_info_t* 返回裁判系统反馈的数据,包括热量/血量/状态等
 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief UI绘制和交互数据的发送接口，由UI绘制任务和多机通信函数调用
 * @note 内部包含基础节流，当前按 2026 协议 0x0301 上行 30Hz 上限进行保守发送
 *
 * @param send 发送数据首地址
 * @param tx_len 发送长度
 */
void RefereeSend(uint8_t *send, uint16_t tx_len);

/**
 * @brief 获取裁判系统在线状态
 * @return 1在线, 0离线
 */
uint8_t RefereeIsOnline(void);

/**
 * @brief 推进裁判串口流式解析状态机
 *
 * @note 必须在任务上下文中周期调用,避免将复杂协议解析放回 UART ISR
 */
void RefereeProcess(void);

/**
 * @brief 尝试消费0x0309图传链路数据，成功后自动清除有效标志
 * @param out_data 输出数据指针，可为NULL
 * @return 1成功消费, 0无新数据
 */
uint8_t RefereeTryConsumeMapInteractiveData(ext_map_interactive_data_var_t *out_data);

/**
 * @brief 尝试消费0x0310图传链路大包数据，成功后自动清除有效标志
 * @param out_data 输出数据指针，可为NULL
 * @return 1成功消费, 0无新数据
 */
uint8_t RefereeTryConsumeCustomInfo0310(ext_custom_info_0310_var_t *out_data);

/**
 * @brief 尝试消费0x0311图传链路上行数据，成功后自动清除有效标志
 * @param out_data 输出数据指针，可为NULL
 * @return 1成功消费, 0无新数据
 */
uint8_t RefereeTryConsumeCustomInfo0311(ext_custom_info_0311_var_t *out_data);

/**
 * @brief 清空扩展协议有效标志位
 */
void RefereeClearExtendedDataFlags(void);

#endif // !REFEREE_H
