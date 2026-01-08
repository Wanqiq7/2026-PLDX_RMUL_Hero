/*
 * @Descripttion: 超级电容模块 - 适配 RM2024 超级电容控制器
 * @version: 2.0
 * @Author: Chenfu (原作者), Claude (2025-01-16 修改)
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2025-01-16
 */
#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include "bsp_can.h"

#pragma pack(1)
/* 超级电容反馈数据（底盘接收，对应超级电容控制器 TxData） */
typedef struct
{
    uint8_t error_code;           // 错误码（bit7=1表示输出关闭，其他位表示具体错误）
    float chassis_power;          // 底盘实际功率 (W)
    uint16_t chassis_power_limit; // 底盘功率限制 (W)
    uint8_t cap_energy;           // 电容能量百分比 (0-255，255表示100%)
} SuperCap_Rx_Data_s;

/* 超级电容控制数据（底盘发送，对应超级电容控制器 RxData） */
typedef struct
{
    uint8_t enable_DCDC : 1;           // DCDC 使能标志（1=使能，0=禁用）
    uint8_t system_restart : 1;        // 系统重启请求（1=重启）
    uint8_t resv0 : 6;                 // 保留位
    uint16_t referee_power_limit;      // 裁判系统功率限制 (W)
    uint16_t referee_energy_buffer;    // 裁判系统能量缓冲区 (J)
    uint8_t resv1[3];                  // 保留字节
} SuperCap_Tx_Data_s;
#pragma pack()

/* 超级电容实例 */
typedef struct
{
    CANInstance *can_ins;         // CAN实例
    SuperCap_Rx_Data_s rx_data;   // 接收数据（超级电容反馈）
    SuperCap_Tx_Data_s tx_data;   // 发送数据（底盘控制指令）
} SuperCapInstance;

/* 超级电容初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
} SuperCap_Init_Config_s;

/**
 * @brief 初始化超级电容模块
 *
 * @param supercap_config 超级电容初始化配置
 * @return SuperCapInstance* 超级电容实例指针
 */
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config);

/**
 * @brief 发送超级电容控制指令
 *
 * @param instance 超级电容实例
 * @param enable_DCDC DCDC使能标志（1=使能，0=禁用）
 * @param power_limit 裁判系统功率限制 (W)
 * @param energy_buffer 裁判系统能量缓冲区 (J)
 */
void SuperCapSendControl(SuperCapInstance *instance,
                         uint8_t enable_DCDC,
                         uint16_t power_limit,
                         uint16_t energy_buffer);

/**
 * @brief 获取超级电容反馈数据
 *
 * @param instance 超级电容实例
 * @return SuperCap_Rx_Data_s 超级电容反馈数据（错误码、功率、能量等）
 */
SuperCap_Rx_Data_s SuperCapGetData(SuperCapInstance *instance);

#endif // SUPER_CAP_H
