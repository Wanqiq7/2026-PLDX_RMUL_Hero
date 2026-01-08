/**
 * @file vision.h
 * @brief 视觉控制应用模块接口
 * @note 遵循项目三层架构,作为独立应用处理视觉数据并生成控制指令
 */

#ifndef VISION_APP_H
#define VISION_APP_H

/**
 * @brief 视觉控制应用初始化
 * @note 由RobotInit()调用,初始化视觉通信模块和消息中心订阅/发布
 */
void VisionAppInit(void);

/**
 * @brief 视觉控制任务
 * @note 处理视觉数据并生成云台/发射控制建议,通过消息中心发布
 */
void VisionAppTask(void);

#endif // VISION_APP_H
