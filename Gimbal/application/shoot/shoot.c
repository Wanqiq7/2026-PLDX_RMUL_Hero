#include "shoot.h"
#include "robot_def.h"

#include "bsp_dwt.h"
#include "dji_motor.h"
#include "general_def.h"
#include "message_center.h"

// 占位：电流前馈幅值（CAN 指令单位），需联调后修正
#define FRICTION_CURRENT_FF_CMD 800.0f // TODO: 实测标定

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

// 摩擦轮电流前馈（克服静摩擦/负载），按轮分别注入
static float fric_ff_cur_bottom = FRICTION_CURRENT_FF_CMD;
static float fric_ff_cur_top_left = FRICTION_CURRENT_FF_CMD;
static float fric_ff_cur_top_right = FRICTION_CURRENT_FF_CMD;

// 前馈方向常量：在初始化阶段根据电机反转标志决定（+1 或 -1）
static float fric_ff_dir_bottom = 1.0f;
static float fric_ff_dir_top_left = -1.0f;
static float fric_ff_dir_top_right = 1.0f;

void ShootInit() // 已适配三摩擦轮发射机构（倒三角布局）
{
  // 左摩擦轮
  Motor_Init_Config_s friction_config = {
      .can_init_config =
          {
              .can_handle = &hcan1, // 统一使用CAN1
          },
      .controller_param_init_config =
          {
              .speed_PID =
                  {
                      .Kp = 1.15f,
                      .Ki = 0.8f, // ✅ 从0.4f增大到0.6f（增强积分作用）
                      .Kd = 0.0f,
                      // ✅ 新增变速积分
                      .Improve = PID_Integral_Limit |
                                 PID_ChangingIntegrationRate | PID_OutputFilter,
                      .IntegralLimit =
                          2000, // ✅ 从1200增大到2000（允许更大积分输出）
                      .Output_LPF_RC =
                          0.02f, // 输出低通滤波时间常数（0.01-0.05，越大越平滑）
                      .CoefA = 1600.0f, // 变速积分阈值上限
                      .CoefB = 400.0f,  // 变速积分阈值下限
                      .MaxOut = 12000,
                  },
              .current_feedforward_ptr = &fric_ff_cur_bottom,
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,

              .outer_loop_type = SPEED_LOOP,
              .close_loop_type = SPEED_LOOP,
              .feedforward_flag = CURRENT_FEEDFORWARD,
              .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // ID1默认正转
          },
      .motor_type = M3508};

  // ID1: 底点(bottom)
  friction_config.can_init_config.tx_id = 1;
  friction_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE;
  friction_config.controller_param_init_config.current_feedforward_ptr =
      &fric_ff_cur_bottom;
  friction_bottom = DJIMotorInit(&friction_config);

  // ID2: 顶边右(top_right)
  friction_config.can_init_config.tx_id = 2;
  friction_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_REVERSE;
  friction_config.controller_param_init_config.current_feedforward_ptr =
      &fric_ff_cur_top_right;
  friction_top_right = DJIMotorInit(&friction_config);

  // ID3: 顶边左(top_left)
  friction_config.can_init_config.tx_id = 3;
  friction_config.controller_setting_init_config.motor_reverse_flag =
      MOTOR_DIRECTION_NORMAL;
  friction_config.controller_param_init_config.current_feedforward_ptr =
      &fric_ff_cur_top_left;
  friction_top_left = DJIMotorInit(&friction_config);

  // 基于电机反转标志，确定每个轮子的前馈方向常量
  fric_ff_dir_bottom = (friction_bottom->motor_settings.motor_reverse_flag ==
                        MOTOR_DIRECTION_REVERSE)
                           ? -1.0f
                           : 1.0f;
  fric_ff_dir_top_right =
      (friction_top_right->motor_settings.motor_reverse_flag ==
       MOTOR_DIRECTION_REVERSE)
          ? -1.0f
          : 1.0f;
  fric_ff_dir_top_left =
      (friction_top_left->motor_settings.motor_reverse_flag ==
       MOTOR_DIRECTION_REVERSE)
          ? -1.0f
          : 1.0f;

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
                      // ⚠️ 注意：误差=1140°（考虑了减速比），不是60°
                      .Kp = 1.65f, // P项（误差1140°→输出2280°/s）
                      .Ki = 1.2f,  // I项消除稳态误差
                      .Kd = 0.0f,
                      .Improve = PID_Integral_Limit,
                      .IntegralLimit = 2500, // 积分限幅
                      .MaxOut = 5000,        // 最大目标速度3000°/s
                  },
              .speed_PID =
                  {
                      // 无电流环模式：速度环直接输出电压指令到电机
                      // 输出单位：电压指令（范围约-16384到+16384）
                      .Kp = 3.25f, // 速度环比例（开环输出需要更大的Kp）
                      .Ki = 0.95f, // 速度环积分（消除稳态误差）
                      .Kd = 0.0f,
                      .Improve = PID_Integral_Limit,
                      .IntegralLimit = 7500, // 积分限幅
                      .MaxOut = 25000,       // 输出限幅（电压指令范围）
                  },
              .current_PID =
                  {
                      .Kp = 1.5f, // 电流环比例（速度环调参阶段可以保持这个值）
                      .Ki = 0.2f, // 电流环积分（消除稳态误差）
                      .Kd = 0.0f,
                      .Improve =
                          PID_Integral_Limit | PID_ChangingIntegrationRate,
                      .IntegralLimit = 5000, // 积分限幅
                      .CoefA = 1500.0f,      // 变速积分参数
                      .CoefB = 800.0f,
                      .MaxOut = 5000, // 电流环输出限幅
                  },
          },
      .controller_setting_init_config =
          {
              .angle_feedback_source = MOTOR_FEED,
              .speed_feedback_source = MOTOR_FEED,
              // ⚠️ 双环支持：单发/三连发用角度环，连发/反转用速度环
              // 运行时通过DJIMotorOuterLoop()动态切换outer_loop_type
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

    return; // 紧急停止后直接返回，不再执行后续逻辑
  }

  // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
  // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
  // 不应期保护：防止在持续触发状态下重复设置目标角度，实现间歇性拨弹
  if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    return;

  // 发射模块使能由各个load_mode自行控制（STOP会停止，BURSTFIRE会使能）

  // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
  switch (shoot_cmd_recv.load_mode) {
  // 停止拨盘
  case LOAD_STOP:
    // ✅ 先将当前角度设为目标，消除角度环静差，避免重启时反转
    DJIMotorSetRef(loader, loader->measure.total_angle);
    // 无电流环模式：直接停止电机，清除PID积分，避免抖动
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
    dead_time = 300;                       // 完成3发弹丸发射的时间
    break;
  // 连发模式 - ⚠️ 调参阶段：纯速度环控制（无电流环）
  case LOAD_BURSTFIRE:
    DJIMotorEnable(loader);                // 从STOP切换过来时需要重新使能
    DJIMotorOuterLoop(loader, SPEED_LOOP); // ⚠️ 必须切换到速度环！

    // 速度环控制：设置目标转速（单位：度/秒）
    // 调试阶段：先使用固定速度500°/s，调好后再改为动态计算
    DJIMotorSetRef(loader, 3060); // 设置目标速度4500°/s

    // 动态计算公式（调好后启用）：
    // float target_speed = shoot_cmd_recv.shoot_rate * 360 *
    // REDUCTION_RATIO_LOADER / 8; DJIMotorSetRef(loader, target_speed);
    // 说明：shoot_rate=1时，target_speed = 1×360×19/8 = 855°/s
    break;
  // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
  // 也有可能需要从switch-case中独立出来
  case LOAD_REVERSE:
    DJIMotorOuterLoop(loader, SPEED_LOOP);
    // ...
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
      DJIMotorSetRef(friction_bottom, 25000); // 15m/s弹速对应转速
      DJIMotorSetRef(friction_top_left, 25000);
      DJIMotorSetRef(friction_top_right, 25000);
      break;
    case SMALL_AMU_18:
      DJIMotorSetRef(friction_bottom, 25000); // 18m/s弹速对应转速
      DJIMotorSetRef(friction_top_left, 25000);
      DJIMotorSetRef(friction_top_right, 25000);
      break;
    case SMALL_AMU_30:
      DJIMotorSetRef(friction_bottom, 25000); // 30m/s弹速对应转速
      DJIMotorSetRef(friction_top_left, 25000);
      DJIMotorSetRef(friction_top_right, 25000);
      break;
    default: // 当前为了调试设定的默认值5450,因为还没有加入裁判系统无法读取弹速.
      DJIMotorSetRef(friction_bottom, 25000);
      DJIMotorSetRef(friction_top_left, 25000);
      DJIMotorSetRef(friction_top_right, 25000);
      break;
    }

    // 写入电流前馈（幅值固定，方向在初始化中由 motor_reverse_flag 预处理）
    fric_ff_cur_bottom = fric_ff_dir_bottom * FRICTION_CURRENT_FF_CMD;
    fric_ff_cur_top_left = fric_ff_dir_top_left * FRICTION_CURRENT_FF_CMD;
    fric_ff_cur_top_right = fric_ff_dir_top_right * FRICTION_CURRENT_FF_CMD;
  } else // 关闭摩擦轮
  {
    // 停止电机（这会设置stop_flag，清除PID状态，电机立即停转）
    DJIMotorSetRef(friction_bottom, 0);
    DJIMotorSetRef(friction_top_left, 0);
    DJIMotorSetRef(friction_top_right, 0);
    // 清零前馈
    fric_ff_cur_bottom = fric_ff_cur_top_left = fric_ff_cur_top_right = 0.0f;
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
