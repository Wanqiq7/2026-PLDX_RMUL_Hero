#include "shoot.h"
#include "robot_def.h"

#include "bsp_dwt.h"
#include "dji_motor.h"
#include "general_def.h"
#include "message_center.h"
#include <math.h>

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
// 三摩擦轮（倒三角分布）：底点(bottom) + 顶边左/右(top_left/top_right)
// 约定：底点ID=1，右轮ID=2，左轮ID=3
static DJIMotorInstance *friction_bottom, *friction_top_left,
    *friction_top_right, *loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖
// ===========================================================

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

// 拨盘反转控制（用于卡弹恢复）：反转固定角度（按“一发弹丸角度”计算）
// 说明：这里不做卡弹检测，仅提供LOAD_REVERSE动作；检测逻辑由上层决定何时进入该模式
#define LOADER_REVERSE_SPEED_DPS                                               \
  5000.0f // 反转目标速度（单位：度/秒），需联调标定
#define LOADER_REVERSE_DEAD_TIME_MS                                            \
  200.0f // 反转完成后的最小间隔（防止持续触发反复反转）
// 反转动作最大持续时间（ms）
// 目的：卡死/堵转时避免无限期输出反转指令导致过热或保护
// 经验值：按“转过一发角度”理论时间 + 裕量
#define LOADER_REVERSE_MAX_TIME_MS                                             \
  ((ONE_BULLET_DELTA_ANGLE / LOADER_REVERSE_SPEED_DPS) * 1000.0f + 200.0f)
static uint8_t loader_reverse_active = 0;       // 反转动作是否进行中
static float loader_reverse_start_angle = 0;    // 反转起始角度（电机轴总角度）
static float loader_reverse_start_time_ms = 0;  // 反转起始时间（ms）
static float loader_reverse_cooldown_until = 0; // 反转冷却截止时间（ms）

void ShootInit() // 已适配三摩擦轮发射机构（倒三角布局）
{
  Motor_Init_Config_s friction_config = {
      .can_init_config =
          {
              .can_handle = &hcan1, // 统一使用CAN1
          },
      .controller_param_init_config =
          {
              .speed_PID =
                  {
                      .Kp = 4.0f,
                      .Ki = 0.0f, // 纯P控制：不启用积分
                      .Kd = 0.08f,
                      .Derivative_LPF_RC = 0.008f,
                      // 输出滤波：一阶低通时间常数（单位：秒），500Hz控制频率下推荐
                      // 0.01s
                      .Output_LPF_RC = 0.022f,
                      .Improve = PID_DerivativeFilter | PID_OutputFilter,
                      .MaxOut = 16384,
                  },
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,

              .outer_loop_type = SPEED_LOOP,
              .close_loop_type = SPEED_LOOP,
              .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // ID1默认正转
          },
      .motor_type = M3508};

  // ID1: 底点(bottom)
  friction_config.can_init_config.tx_id = 1;
  friction_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE;
  friction_bottom = DJIMotorInit(&friction_config);

  // ID2: 顶边右(top_right)
  friction_config.can_init_config.tx_id = 2;
  friction_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE;
  friction_top_right = DJIMotorInit(&friction_config);

  // ID3: 顶边左(top_left)
  friction_config.can_init_config.tx_id = 3;
  friction_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_NORMAL;
  friction_top_left = DJIMotorInit(&friction_config);

  // 拨盘电机
  Motor_Init_Config_s loader_config = {
      .can_init_config =
          {
              .can_handle = &hcan2,
              .tx_id = 3,
          },
      .controller_param_init_config =
          {
              .angle_PID =
                  {
                      // 位置环：输出到速度环的目标速度（单位：度/秒）
                      // ⚠️ 注意：误差=3060°（考虑了减速比），不是60°
                      .Kp = 17.5f, // P项（误差3060°→输出约5049°/s，接近MaxOut）
                      .Ki = 0.0f,  // I项消除稳态误差
                      .Kd = 0.05f,
                      .Improve = PID_Integral_Limit,
                      .MaxOut = 6000, // 最大目标速度6000°/s
                  },
              .speed_PID =
                  {
                      // 无电流环模式：速度环直接输出CAN控制指令（等效电流/力矩类控制量）
                      // 输出单位：控制指令（范围约-16384到+16384）
                      .Kp = 1.75f, // 速度环比例（开环输出需要更大的Kp）
                      .Ki = 1.51f, // 速度环积分（消除稳态误差）
                      .Kd = 0.0f,
                      // 输出滤波：对速度环输出做一阶低通，抑制齿隙/负载突变引起的力矩抖动
                      // 建议从 0.01s 起步（约 16Hz
                      // 截止），再根据响应/抖动情况微调
                      .Improve = PID_Integral_Limit | PID_OutputFilter,
                      .IntegralLimit = 7500, // 积分限幅
                      .Output_LPF_RC = 0.010f,
                      .MaxOut = 16384, // 输出限幅
                  },
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,

              .outer_loop_type = ANGLE_LOOP, // 初始外环：角度环（单发模式默认）
              .close_loop_type =
                  SPEED_LOOP |
                  ANGLE_LOOP, // 启用速度环+角度环（双环串级，无电流环）
              .motor_reverse_flag =
                  MOTOR_DIRECTION_NORMAL, // ⚠️ 方向：REVERSE会导致正反馈震荡！
          },
      .motor_type = M3508 // 英雄使用m3508
  };
  loader = DJIMotorInit(&loader_config);

  shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
  shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask() {
  // 从cmd获取控制数据
  SubGetMessage(shoot_sub, &shoot_cmd_recv);

  // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
  if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
    DJIMotorStop(friction_bottom);
    DJIMotorStop(friction_top_left);
    DJIMotorStop(friction_top_right);
    DJIMotorStop(loader);

    // ⚠️ 不要直接return！继续清理状态
    // 清除不应期
    hibernate_time = 0;
    dead_time = 0;
    loader_reverse_active = 0;
    loader_reverse_start_time_ms = 0;
    loader_reverse_cooldown_until = 0;

    return; // 紧急停止后直接返回，不再执行后续逻辑
  }

  // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
  // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
  // 不应期保护：防止在持续触发状态下重复设置目标角度，实现间歇性拨弹
  // ⚠️ 注意：LOAD_STOP/LOAD_REVERSE 需要随时响应，因此不受不应期限制
  if ((shoot_cmd_recv.load_mode != LOAD_STOP) &&
      (shoot_cmd_recv.load_mode != LOAD_REVERSE) &&
      (hibernate_time + dead_time > DWT_GetTimeline_ms()))
    return;

  // 发射模块使能由各个load_mode自行控制（STOP会停止，BURSTFIRE会使能）
  if (shoot_cmd_recv.load_mode != LOAD_REVERSE) {
    loader_reverse_active = 0;
    loader_reverse_start_time_ms = 0;
  }

  // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
  switch (shoot_cmd_recv.load_mode) {
  // 停止拨盘
  case LOAD_STOP:
    // 直接停止电机，同时重置PID状态（见DJIMotorStop实现）
    DJIMotorStop(loader);
    break;
  // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
  case LOAD_1_BULLET:                      // 激活能量机关/干扰对方用,英雄用.
    DJIMotorEnable(loader);                // 从STOP切换过来时需要使能
    DJIMotorOuterLoop(loader, ANGLE_LOOP); // 切换到角度环
    // ✅ 基于当前实际角度设定目标，避免停止期间位置漂移导致反转
    DJIMotorSetRef(loader,
                   loader->measure.total_angle +
                       ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
    hibernate_time = DWT_GetTimeline_ms();      // 记录触发指令的时间
    // 根据射频动态计算不应期时间：1Hz=1000ms, 2Hz=500ms, 以此类推
    // 英雄默认1Hz（每秒1发），实现间歇性拨弹
    dead_time = (shoot_cmd_recv.shoot_rate > 0)
                    ? (1000.0f / shoot_cmd_recv.shoot_rate)
                    : 1000;
    break;
  // 三连发,如果不需要后续可能删除
  case LOAD_3_BULLET:
    DJIMotorEnable(loader);                // 从STOP切换过来时需要使能
    DJIMotorOuterLoop(loader, ANGLE_LOOP); // 切换到角度环
    // ✅ 基于当前实际角度设定目标，避免停止期间位置漂移导致反转
    DJIMotorSetRef(loader, loader->measure.total_angle +
                               3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
    hibernate_time = DWT_GetTimeline_ms(); // 记录触发指令的时间
    // 三连发不应期：按射频一致化（3发≈3个单发周期）
    // 例如 shoot_rate=2Hz -> 单发500ms，三连发1500ms
    dead_time = (shoot_cmd_recv.shoot_rate > 0)
                    ? (3.0f * (1000.0f / shoot_cmd_recv.shoot_rate))
                    : 300.0f;
    break;
  // 连发模式 - ⚠️ 调参阶段：纯速度环控制（无电流环）
  case LOAD_BURSTFIRE:
    DJIMotorEnable(loader);                // 从STOP切换过来时需要重新使能
    DJIMotorOuterLoop(loader, SPEED_LOOP); // ⚠️ 必须切换到速度环！

    // 速度环控制：设置目标转速（单位：度/秒）
    // 目标速度（建议与射频一致）：target_speed = shoot_rate × 一发弹丸角度
    // shoot_rate=1Hz 时：target_speed = 1 × 3060 = 3060°/s（电机轴）
    float target_speed =
        (shoot_cmd_recv.shoot_rate > 0)
            ? (shoot_cmd_recv.shoot_rate * ONE_BULLET_DELTA_ANGLE)
            : ONE_BULLET_DELTA_ANGLE;
    DJIMotorSetRef(loader, target_speed);
    break;
  // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
  // 也有可能需要从switch-case中独立出来
  case LOAD_REVERSE:
    // 反转固定角度（一发弹丸角度），用于卡弹恢复动作
    // 说明：若上层持续保持
    // LOAD_REVERSE，本状态机会按“反转->停止->冷却”的周期反复退一发
    float now_ms = DWT_GetTimeline_ms();
    if (now_ms < loader_reverse_cooldown_until) {
      DJIMotorStop(loader);
      break;
    }

    DJIMotorEnable(loader);
    DJIMotorOuterLoop(loader, SPEED_LOOP);
    if (!loader_reverse_active) {
      loader_reverse_active = 1;
      loader_reverse_start_angle = loader->measure.total_angle;
      loader_reverse_start_time_ms = now_ms;
      DJIMotorSetRef(loader, -LOADER_REVERSE_SPEED_DPS);
    } else {
      // 超时保护：卡死/堵转时，避免无限反转
      if ((now_ms - loader_reverse_start_time_ms) >
          LOADER_REVERSE_MAX_TIME_MS) {
        loader_reverse_active = 0;
        loader_reverse_start_time_ms = 0;
        DJIMotorStop(loader);
        loader_reverse_cooldown_until = now_ms + LOADER_REVERSE_DEAD_TIME_MS;
        break;
      }
      // 达到目标角度后停止
      if (fabsf(loader->measure.total_angle - loader_reverse_start_angle) >=
          ONE_BULLET_DELTA_ANGLE) {
        loader_reverse_active = 0;
        loader_reverse_start_time_ms = 0;
        DJIMotorStop(loader);
        loader_reverse_cooldown_until = now_ms + LOADER_REVERSE_DEAD_TIME_MS;
      } else {
        // 保持反转速度指令
        DJIMotorSetRef(loader, -LOADER_REVERSE_SPEED_DPS);
      }
    }
    break;
  default:
    while (1)
      ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
  }

  // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
  if (shoot_cmd_recv.friction_mode == FRICTION_ON) {
    // ⭐ 使能摩擦轮电机
    DJIMotorEnable(friction_bottom);
    DJIMotorEnable(friction_top_left);
    DJIMotorEnable(friction_top_right);

    // 根据目标弹速分档设置目标速度（保持原有逻辑），并给出方向
    // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
    switch (shoot_cmd_recv.bullet_speed) {
    case SMALL_AMU_15:
      DJIMotorSetRef(friction_bottom, 24000); // 15m/s弹速对应转速
      DJIMotorSetRef(friction_top_left, 24000);
      DJIMotorSetRef(friction_top_right, 24000);
      break;
    case SMALL_AMU_18:
      DJIMotorSetRef(friction_bottom, 24000); // 18m/s弹速对应转速
      DJIMotorSetRef(friction_top_left, 24000);
      DJIMotorSetRef(friction_top_right, 24000);
      break;
    case SMALL_AMU_30:
      DJIMotorSetRef(friction_bottom, 24000); // 30m/s弹速对应转速
      DJIMotorSetRef(friction_top_left, 24000);
      DJIMotorSetRef(friction_top_right, 24000);
      break;
    default: // 当前为了调试设定的默认值5450,因为还没有加入裁判系统无法读取弹速.
      DJIMotorSetRef(friction_bottom, 24000);
      DJIMotorSetRef(friction_top_left, 24000);
      DJIMotorSetRef(friction_top_right, 24000);
      break;
    }
  } else // 关闭摩擦轮
  {
    // 停止电机（这会设置stop_flag，清除PID状态，电机立即停转）
    DJIMotorSetRef(friction_bottom, 0);
    DJIMotorSetRef(friction_top_left, 0);
    DJIMotorSetRef(friction_top_right, 0);
  }

  // 开关弹舱盖
  if (shoot_cmd_recv.lid_mode == LID_CLOSE) {
    //...
  } else if (shoot_cmd_recv.lid_mode == LID_OPEN) {
    //...
  }

  // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
  PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
