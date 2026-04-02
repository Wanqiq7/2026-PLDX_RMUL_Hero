#ifndef CHASSIS_CAN_LINK_H
#define CHASSIS_CAN_LINK_H

#include "bsp_can.h"
#include "robot_def.h"
#include <stdint.h>

void ChassisCanLinkInit(CAN_HandleTypeDef *can_handle, uint32_t now_ms);
uint8_t ChassisCanLinkUpdateCommand(Chassis_Ctrl_Cmd_s *cmd);
void ChassisCanLinkSendFeedbackIfDue(const Chassis_Upload_Data_s *feedback);

#endif
