/**
 * @file dji_motor.h
 * @author neozng
 * @brief DJI智能电机头文件
 * @version 0.2
 * @date 2022-11-01
 *
 * @todo  1. 给不同的电机设置不同的低通滤波器惯性系数而不是统一使用宏
          2.
 为M2006和M3508增加开环的零位校准函数,并在初始化时调用(根据用户配置决定是否调用)

 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */

#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include "bsp_can.h"
#include "controller.h"
#include "daemon.h"
#include "motor_def.h"
#include "stdint.h"

#define DJI_MOTOR_CNT 12

/* 滤波系数设置为1的时候即关闭滤波 */
#define DJI_SPEED_SMOOTH_COEF 0.90f   // 最好大于0.85
#define DJI_CURRENT_SMOOTH_COEF 0.90f // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f  // (360/8192),将编码器值转化为角度制

/* DJI电机CAN反馈信息*/
typedef struct {
  uint16_t last_ecd;        // 上一次读取的编码器值
  uint16_t ecd;             // 0-8191,刻度总共有8192格
  float angle_single_round; // 单圈角度
  float speed_aps;          // 角速度,单位为:度/秒
  int16_t real_current;     // 实际电流
  uint8_t temperature;      // 温度 Celsius

  float total_angle;   // 总角度,注意方向
  int32_t total_round; // 总圈数,注意方向
} DJI_Motor_Measure_s;

/**
 * @brief DJI intelligent motor typedef
 *
 */
typedef struct {
  DJI_Motor_Measure_s measure;            // 电机测量值
  Motor_Control_Setting_s motor_settings; // 电机设置
  Motor_Controller_s motor_controller;    // 电机控制器

  CANInstance *motor_can_instance; // 电机CAN实例
  // 分组发送设置
  uint8_t sender_group;
  uint8_t message_num;

  Motor_Physical_Param_s physical_param;
  Motor_Type_e motor_type;        // 电机类型
  Motor_Working_Type_e stop_flag; // 启停标志

  DaemonInstance *daemon;
  uint32_t feed_cnt;
  float dt;
} DJIMotorInstance;

/**
 * @brief
 * 调用此函数注册一个DJI智能电机,需要传递较多的初始化参数,请在application初始化的时候调用此函数
 *        推荐传参时像标准库一样构造initStructure然后传入此函数.
 *        recommend: type xxxinitStructure = {.member1=xx,
 *                                            .member2=xx,
 *                                             ....};
 *        请注意不要在一条总线上挂载过多的电机(超过6个),若一定要这么做,请降低每个电机的反馈频率(设为500Hz),
 *        并减小DJIMotorControl()任务的运行频率.
 *
 * @attention
 * M3508和M2006的反馈报文都是0x200+id,而GM6020的反馈是0x204+id,请注意前两者和后者的id不要冲突.
 *            如果产生冲突,在初始化电机的时候会进入IDcrash_Handler(),可以通过debug来判断是否出现冲突.
 *
 * @param config
 * 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 *
 * @return DJIMotorInstance*
 */
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 兼容接口：给电机设置传统参考值
 *        该接口会写入电机内部的 reference carrier，由闭环逻辑继续解释
 *
 * @param motor 要设置的电机
 * @param ref 设定参考值
 *
 * @note  新的动力执行器主线优先使用 DJIMotorSetEffort()。
 *        SetRef() 仅保留给角度/速度等旧闭环链路与兼容调用方使用。
 */
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);

/**
 * @brief 直接设置电机控制努力量，供扭矩主线或兼容桥使用
 *
 * @param motor 要设置的电机
 * @param effort 统一控制努力量；传入 NULL 时清空直通努力量
 */
void DJIMotorSetEffort(DJIMotorInstance *motor,
                       const Controller_Effort_Output_s *effort);

/**
 * @brief 按当前控制器类型与反馈配置计算统一控制努力量
 *
 * @param motor 电机实例
 * @param ref 参考输入（角度/速度等，由当前外环解释）
 * @param effort 输出的统一控制努力量
 * @return uint8_t 1-成功，0-失败
 */
uint8_t DJIMotorCalculateEffort(DJIMotorInstance *motor, float ref,
                                Controller_Effort_Output_s *effort);

/**
 * @brief 显式 bypass 接口：直接设置电机原始电流参考值
 *
 * @param motor 要设置的电机
 * @param raw_ref 期望发送到CAN的原始电流值
 *
 * @note  该接口仅用于 OPEN_LOOP/raw current 接管场景，
 *        会自动补偿电机层内部的方向处理，不属于扭矩主线默认路径。
 */
void DJIMotorSetRawRef(DJIMotorInstance *motor, float raw_ref);

/**
 * @brief 切换反馈的目标来源,如将角速度和角度的来源换为IMU(小陀螺模式常用)
 *
 * @param motor 要切换反馈数据来源的电机
 * @param loop  要切换反馈数据来源的控制闭环
 * @param type  目标反馈模式
 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop,
                        Feedback_Source_e type);

/**
 * @brief
 * 该函数被motor_task调用运行在rtos上,motor_stask内通过osDelay()确定控制频率
 */
void DJIMotorControl();

/**
 * @brief 停止电机,注意不是将设定值设为零,而是直接给电机发送的电流值置零
 *
 */
void DJIMotorStop(DJIMotorInstance *motor);

/**
 * @brief 启动电机,此时电机会响应设定值
 *        初始化时不需要此函数,因为stop_flag的默认值为0
 *
 */
void DJIMotorEnable(DJIMotorInstance *motor);

/**
 * @brief 修改电机闭环目标(外层闭环)
 *
 * @param motor  要修改的电机实例指针
 * @param outer_loop 外层闭环类型
 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop);

/**
 * @brief 切换电机控制器类型（PID/LQR/SMC）
 *
 * @param motor 要切换控制器的电机实例指针
 * @param controller_type 控制器类型
 *
 * @note  切换到LQR模式前，需要确保：
 *        1. 已在初始化时配置好LQR参数
 *        2. angle_feedback_source 和 speed_feedback_source 已正确设置
 *        3. LQR参数已通过系统辨识获得（否则使用默认值可能不稳定）
 *        切换到SMC模式前，需要确保：
 *        1. 已在初始化时配置好SMC参数和模式
 *        2. 输出单位与目标电机最终控制量一致（例如 DJI 原始电流量）
 */
void DJIMotorChangeController(DJIMotorInstance *motor,
                              Controller_Type_e controller_type);

#endif // !DJI_MOTOR_H
