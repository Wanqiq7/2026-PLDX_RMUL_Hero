#include "gimbal.h"
#include "bmi088.h"
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "general_def.h"
#include "ins_task.h"
#include "message_center.h"
#include "robot_def.h"
#include "sysid_task.h"
#include <math.h>

/* ==================== 调参比例宏 ==================== */
// 可通过修改以下比例对Yaw轴PID进行等比例放大/缩小
#define YAW_ANGLE_PID_SCALE 50.0f // 角度环缩放系数（例如 10.0f 放大十倍）
#define YAW_SPEED_PID_SCALE 50.0f // 速度环缩放系数（例如 1e7 放大一千万倍）

