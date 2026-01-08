/*
 * @Descripttion: 超级电容模块 - 适配 RM2024 超级电容控制器
 * @version: 2.0
 * @Author: Chenfu (原作者), Claude (2025-01-16 修改)
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2025-01-16
 */
#include "super_cap.h"
#include "memory.h"
#include "stdlib.h"
#include "string.h"

static SuperCapInstance *super_cap_instance = NULL; // 可以由app保存此指针

/**
 * @brief 超级电容CAN接收回调函数
 * @note 接收超级电容控制器发送的反馈数据（CAN ID 0x051）
 *       数据格式：{errorCode, chassisPower(float), chassisPowerLimit, capEnergy}
 */
static void SuperCapRxCallback(CANInstance *_instance)
{
    /* 直接内存拷贝，保证数据结构对齐，避免字节序问题 */
    memcpy(&super_cap_instance->rx_data,
           _instance->rx_buff,
           sizeof(SuperCap_Rx_Data_s));
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)
{
    super_cap_instance = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
    memset(super_cap_instance, 0, sizeof(SuperCapInstance));

    supercap_config->can_config.can_module_callback = SuperCapRxCallback;
    super_cap_instance->can_ins = CANRegister(&supercap_config->can_config);
    return super_cap_instance;
}

/**
 * @brief 发送超级电容控制指令
 * @note 发送给超级电容控制器的控制数据（CAN ID 0x061）
 *       数据格式：{enableDCDC, systemRestart, refereePowerLimit, refereeEnergyBuffer}
 *
 * @param instance 超级电容实例
 * @param enable_DCDC DCDC使能标志（1=使能，0=禁用）
 * @param power_limit 裁判系统功率限制 (W)
 * @param energy_buffer 裁判系统能量缓冲区 (J)
 */
void SuperCapSendControl(SuperCapInstance *instance,
                         uint8_t enable_DCDC,
                         uint16_t power_limit,
                         uint16_t energy_buffer)
{
    /* 更新发送数据 */
    instance->tx_data.enable_DCDC = enable_DCDC;
    instance->tx_data.system_restart = 0; // 正常情况下不重启
    instance->tx_data.referee_power_limit = power_limit;
    instance->tx_data.referee_energy_buffer = energy_buffer;

    /* 拷贝到CAN发送缓冲区并发送 */
    memcpy(instance->can_ins->tx_buff,
           &instance->tx_data,
           sizeof(SuperCap_Tx_Data_s));
    CANTransmit(instance->can_ins, 1);
}

/**
 * @brief 获取超级电容反馈数据
 *
 * @param instance 超级电容实例
 * @return SuperCap_Rx_Data_s 超级电容反馈数据（错误码、功率、功率限制、能量百分比）
 */
SuperCap_Rx_Data_s SuperCapGetData(SuperCapInstance *instance)
{
    return instance->rx_data;
}