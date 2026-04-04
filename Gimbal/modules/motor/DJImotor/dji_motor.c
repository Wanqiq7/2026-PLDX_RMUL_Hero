#include "dji_motor.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "dji_motor_adapter.h"
#include "general_def.h"
#include "robot_def.h"
#include "user_lib.h"

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行
 */
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = {
    NULL}; // 会在control任务中遍历该指针数组进行pid计算

static float DJIMotorGetRawRefSign(const DJIMotorInstance *motor) {
  float sign = 1.0f;

  if (motor->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
    sign *= -1.0f;
  }
  if (motor->motor_settings.feedback_reverse_flag ==
      FEEDBACK_DIRECTION_REVERSE) {
    sign *= -1.0f;
  }
  return sign;
}

static void DJIMotorResetSMCState(DJIMotorInstance *motor, float target_ref) {
  if (motor == NULL) {
    return;
  }

  SMC_ControllerResetState(&motor->motor_controller.smc, target_ref);
  DWT_GetDeltaT(&motor->motor_controller.smc_dwt_cnt);
}

/**
 * @brief
 * 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can*3group)can_instance专门负责发送
 *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
 *
 * @note  因为只用于发送,所以不需要在bsp_can中注册
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
static CANInstance sender_assignment[6] = {
    [0] = {.can_handle = &hcan1,
           .txconf.StdId = 0x1ff,
           .txconf.IDE = CAN_ID_STD,
           .txconf.RTR = CAN_RTR_DATA,
           .txconf.DLC = 0x08,
           .tx_buff = {0}},
    [1] = {.can_handle = &hcan1,
           .txconf.StdId = 0x200,
           .txconf.IDE = CAN_ID_STD,
           .txconf.RTR = CAN_RTR_DATA,
           .txconf.DLC = 0x08,
           .tx_buff = {0}},
    [2] = {.can_handle = &hcan1,
           .txconf.StdId = 0x2ff,
           .txconf.IDE = CAN_ID_STD,
           .txconf.RTR = CAN_RTR_DATA,
           .txconf.DLC = 0x08,
           .tx_buff = {0}},
    [3] = {.can_handle = &hcan2,
           .txconf.StdId = 0x1ff,
           .txconf.IDE = CAN_ID_STD,
           .txconf.RTR = CAN_RTR_DATA,
           .txconf.DLC = 0x08,
           .tx_buff = {0}},
    [4] = {.can_handle = &hcan2,
           .txconf.StdId = 0x200,
           .txconf.IDE = CAN_ID_STD,
           .txconf.RTR = CAN_RTR_DATA,
           .txconf.DLC = 0x08,
           .tx_buff = {0}},
    [5] = {.can_handle = &hcan2,
           .txconf.StdId = 0x2ff,
           .txconf.IDE = CAN_ID_STD,
           .txconf.RTR = CAN_RTR_DATA,
           .txconf.DLC = 0x08,
           .tx_buff = {0}},
};

/**
 * @brief
 * 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 */
static uint8_t sender_enable_flag[6] = {0};

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static void MotorSenderGrouping(DJIMotorInstance *motor,
                                CAN_Init_Config_s *config) {
  uint8_t motor_id = config->tx_id - 1; // 下标从零开始,先减一方便赋值
  uint8_t motor_send_num;
  uint8_t motor_grouping;

  switch (motor->motor_type) {
  case M2006:
  case M3508:
    if (motor_id < 4) // 根据ID分组
    {
      motor_send_num = motor_id;
      motor_grouping = config->can_handle == &hcan1 ? 1 : 4;
    } else {
      motor_send_num = motor_id - 4;
      motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
    }

    // 计算接收id并设置分组发送id
    config->rx_id = 0x200 + motor_id + 1;   // 把ID+1,进行分组设置
    sender_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
    motor->message_num = motor_send_num;
    motor->sender_group = motor_grouping;

    // 检查是否发生id冲突
    for (size_t i = 0; i < idx; ++i) {
      if (dji_motor_instance[i]->motor_can_instance->can_handle ==
              config->can_handle &&
          dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id) {
        LOGERROR("[dji_motor] ID crash. Check in debug mode, add "
                 "dji_motor_instance to watch to get more information.");
        uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
        while (
            1) // 6020的id 1-4和2006/3508的id
               // 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
          LOGERROR("[dji_motor] id [%d], can_bus [%d]", config->rx_id, can_bus);
      }
    }
    break;

  case GM6020:
    if (motor_id < 4) {
      motor_send_num = motor_id;
      motor_grouping = config->can_handle == &hcan1 ? 0 : 3;
    } else {
      motor_send_num = motor_id - 4;
      motor_grouping = config->can_handle == &hcan1 ? 2 : 5;
    }

    config->rx_id = 0x204 + motor_id + 1; // 把ID+1,进行分组设置
    sender_enable_flag[motor_grouping] =
        1; // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
    motor->message_num = motor_send_num;
    motor->sender_group = motor_grouping;

    for (size_t i = 0; i < idx; ++i) {
      if (dji_motor_instance[i]->motor_can_instance->can_handle ==
              config->can_handle &&
          dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id) {
        LOGERROR("[dji_motor] ID crash. Check in debug mode, add "
                 "dji_motor_instance to watch to get more information.");
        uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
        while (
            1) // 6020的id 1-4和2006/3508的id
               // 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
          LOGERROR("[dji_motor] id [%d], can_bus [%d]", config->rx_id, can_bus);
      }
    }
    break;

  default: // other motors should not be registered here
    while (1)
      LOGERROR("[dji_motor]You must not register other motors using the API of "
               "DJI motor."); // 其他电机不应该在这里注册
  }
}

/**
 * @todo  是否可以简化多圈角度的计算？
 * @brief 根据返回的can_instance对反馈报文进行解析
 *
 * @param _instance
 * 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
static void DecodeDJIMotor(CANInstance *_instance) {
  // 这里对can instance的id进行了强制转换,从而获得电机的instance实例地址
  // _instance指针指向的id是对应电机instance的地址,通过强制转换为电机instance的指针,再通过->运算符访问电机的成员motor_measure,最后取地址获得指针
  uint8_t *rxbuff = _instance->rx_buff;
  DJIMotorInstance *motor = (DJIMotorInstance *)_instance->id;
  DJI_Motor_Measure_s *measure =
      &motor->measure; // measure要多次使用,保存指针减小访存开销

  DaemonReload(motor->daemon);
  motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

  // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
  measure->last_ecd = measure->ecd;
  measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
  measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
  measure->speed_aps = (1.0f - DJI_SPEED_SMOOTH_COEF) * measure->speed_aps +
                       RPM_2_ANGLE_PER_SEC * DJI_SPEED_SMOOTH_COEF *
                           (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
  measure->real_current =
      (1.0f - DJI_CURRENT_SMOOTH_COEF) * measure->real_current +
      DJI_CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
  measure->temperature = rxbuff[6];

  // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
  if (measure->ecd - measure->last_ecd > 4096)
    measure->total_round--;
  else if (measure->ecd - measure->last_ecd < -4096)
    measure->total_round++;
  measure->total_angle =
      measure->total_round * 360 + measure->angle_single_round;
}

static void DJIMotorLostCallback(void *motor_ptr) {
  DJIMotorInstance *motor = (DJIMotorInstance *)motor_ptr;
  uint16_t can_bus = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
  LOGWARNING("[dji_motor] Motor lost, can bus [%d] , id [%d]", can_bus,
             motor->motor_can_instance->tx_id);
}

static float DJIMotorCurrentToTauRef(const Motor_Physical_Param_s *param,
                                     float current_ref_a) {
  if (param == NULL || param->torque_constant_nm_per_a <= 0.0f) {
    return 0.0f;
  }
  return current_ref_a * param->torque_constant_nm_per_a;
}

static float DJIMotorRawCurrentToTauRef(const Motor_Physical_Param_s *param,
                                        float raw_current_cmd) {
  if (param == NULL || param->raw_to_current_coeff <= 0.0f ||
      param->torque_constant_nm_per_a <= 0.0f) {
    return 0.0f;
  }
  return raw_current_cmd * param->raw_to_current_coeff *
         param->torque_constant_nm_per_a;
}

// 电机初始化,返回一个电机实例
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config) {
  uint8_t physical_param_valid = 0U;
  DJIMotorInstance *instance =
      (DJIMotorInstance *)malloc(sizeof(DJIMotorInstance));
  memset(instance, 0, sizeof(DJIMotorInstance));

  // motor basic setting 电机基本设置
  instance->motor_type = config->motor_type; // 6020 or 2006 or 3508
  instance->motor_settings =
      config->controller_setting_init_config; // 正反转,闭环类型等
  physical_param_valid = DJIMotorResolvePhysicalParam(
      config->motor_type, &config->physical_param, &instance->physical_param);
  if (!physical_param_valid) {
    LOGERROR("[dji_motor] invalid physical parameters for motor_type=%d",
             config->motor_type);
  }

  // motor controller init 电机控制器初始化
  PIDInit(&instance->motor_controller.current_PID,
          &config->controller_param_init_config.current_PID);
  PIDInit(&instance->motor_controller.speed_PID,
          &config->controller_param_init_config.speed_PID);
  PIDInit(&instance->motor_controller.angle_PID,
          &config->controller_param_init_config.angle_PID);
  LQRInit(&instance->motor_controller.LQR,
          &config->controller_param_init_config.LQR);
  SMC_ControllerInitFromConfig(&instance->motor_controller.smc,
                               &config->controller_param_init_config.smc);
  DWT_GetDeltaT(&instance->motor_controller.smc_dwt_cnt);
  instance->motor_controller.other_angle_feedback_ptr =
      config->controller_param_init_config.other_angle_feedback_ptr;
  instance->motor_controller.other_speed_feedback_ptr =
      config->controller_param_init_config.other_speed_feedback_ptr;
  instance->motor_controller.current_feedforward_ptr =
      config->controller_param_init_config.current_feedforward_ptr;
  instance->motor_controller.speed_feedforward_ptr =
      config->controller_param_init_config.speed_feedforward_ptr;
  // 后续增加电机前馈控制器(速度和电流)

  // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
  MotorSenderGrouping(instance, &config->can_init_config);

  // 注册电机到CAN总线
  config->can_init_config.can_module_callback = DecodeDJIMotor; // set callback
  config->can_init_config.id = instance; // set id,eq to address(it is identity)
  instance->motor_can_instance = CANRegister(&config->can_init_config);

  // 注册守护线程
  Daemon_Init_Config_s daemon_config = {
      .callback = DJIMotorLostCallback,
      .owner_id = instance,
      .reload_count = 2, // 20ms未收到数据则丢失
  };
  instance->daemon = DaemonRegister(&daemon_config);

  if (physical_param_valid) {
    DJIMotorEnable(instance);
  }
  dji_motor_instance[idx++] = instance;
  return instance;
}

/* 电流只能通过电机自带传感器监测,后续考虑加入力矩传感器应变片等 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop,
                        Feedback_Source_e type) {
  if (loop == ANGLE_LOOP)
    motor->motor_settings.angle_feedback_source = type;
  else if (loop == SPEED_LOOP)
    motor->motor_settings.speed_feedback_source = type;
  else
    LOGERROR(
        "[dji_motor] loop type error, check memory access and func param"); // 检查是否传入了正确的LOOP类型,或发生了指针越界
}

void DJIMotorStop(DJIMotorInstance *motor) {
  motor->stop_flag = MOTOR_STOP;

  // 停止电机时重置PID状态，避免停机期间积分累积导致再次使能时冲击
  PIDReset(&motor->motor_controller.current_PID);
  PIDReset(&motor->motor_controller.speed_PID);
  PIDReset(&motor->motor_controller.angle_PID);
  DJIMotorResetSMCState(motor, 0.0f);
}

void DJIMotorEnable(DJIMotorInstance *motor) {
  motor->stop_flag = MOTOR_ENALBED;
}

/* 修改电机的实际闭环对象 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop) {
  if (motor->motor_settings.outer_loop_type == outer_loop)
    return;

  motor->motor_settings.outer_loop_type = outer_loop;

  // 切换外环时重置对应PID状态，避免长时间未调用导致dt异常或积分残留
  if (outer_loop == ANGLE_LOOP) {
    PIDReset(&motor->motor_controller.angle_PID);
  } else if (outer_loop == SPEED_LOOP) {
    PIDReset(&motor->motor_controller.speed_PID);
  }

  if (motor->motor_settings.controller_type == CONTROLLER_SMC) {
    DJIMotorResetSMCState(motor, motor->motor_controller.pid_ref);
  }
}

/* 切换电机控制器类型（PID/LQR/SMC） */
void DJIMotorChangeController(DJIMotorInstance *motor,
                              Controller_Type_e controller_type) {
  if (motor->motor_settings.controller_type == controller_type) {
    return;
  }

  if (motor->motor_settings.controller_type == CONTROLLER_SMC) {
    DJIMotorResetSMCState(motor, 0.0f);
  }

  motor->motor_settings.controller_type = controller_type;

  if (controller_type == CONTROLLER_LQR) {
    LQRReset(&motor->motor_controller.LQR);
  } else if (controller_type == CONTROLLER_SMC) {
    DJIMotorResetSMCState(motor, motor->motor_controller.pid_ref);
  } else {
    PIDReset(&motor->motor_controller.current_PID);
    PIDReset(&motor->motor_controller.speed_PID);
    PIDReset(&motor->motor_controller.angle_PID);
  }
}

// 设置参考值
void DJIMotorSetRef(DJIMotorInstance *motor, float ref) {
  motor->motor_controller.pid_ref = ref;
  memset(&motor->motor_controller.ref_effort, 0,
         sizeof(motor->motor_controller.ref_effort));
}

void DJIMotorSetEffort(DJIMotorInstance *motor,
                       const Controller_Effort_Output_s *effort) {
  if (motor == NULL) {
    return;
  }

  memset(&motor->motor_controller.ref_effort, 0,
         sizeof(motor->motor_controller.ref_effort));
  if (effort != NULL) {
    motor->motor_controller.ref_effort = *effort;
  }
}

void DJIMotorSetRawRef(DJIMotorInstance *motor, float raw_ref) {
  motor->motor_controller.pid_ref = raw_ref * DJIMotorGetRawRefSign(motor);
  memset(&motor->motor_controller.ref_effort, 0,
         sizeof(motor->motor_controller.ref_effort));
}

// 为所有电机实例计算三环PID,发送控制报文
void DJIMotorControl() {
  // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
  uint8_t group, num; // 电机组号和组内编号
  int16_t set;        // 电机控制CAN发送设定值
  DJIMotorInstance *motor;
  Motor_Control_Setting_s *motor_setting; // 电机控制参数
  Motor_Controller_s *motor_controller;   // 电机控制器
  DJI_Motor_Measure_s *measure;           // 电机测量值
  float pid_measure, pid_ref;             // 电机PID测量值和设定值

  // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
  for (size_t i = 0; i < idx; ++i) { // 减小访存开销,先保存指针引用
    motor = dji_motor_instance[i];
    motor_setting = &motor->motor_settings;
    motor_controller = &motor->motor_controller;
    measure = &motor->measure;
    pid_ref =
        motor_controller
            ->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改

    // 若电机处于停止状态：不计算控制器，直接清零输出，避免停机期间积分累积
    if (motor->stop_flag == MOTOR_STOP) {
      group = motor->sender_group;
      num = motor->message_num;
      memset(sender_assignment[group].tx_buff + 2 * num, 0, sizeof(uint16_t));
      continue;
    }

    if (motor_controller->ref_effort.semantic != CONTROLLER_OUTPUT_INVALID) {
      if (!DJIMotorBuildRawCommandFromEffort(&motor->physical_param,
                                             &motor_controller->ref_effort,
                                             &set,
                                             &motor_controller->controller_output)) {
        set = 0;
        memset(&motor_controller->controller_output, 0,
               sizeof(motor_controller->controller_output));
      }

      group = motor->sender_group;
      num = motor->message_num;
      sender_assignment[group].tx_buff[2 * num] = (uint8_t)(set >> 8);
      sender_assignment[group].tx_buff[2 * num + 1] =
          (uint8_t)(set & 0x00ff);
      continue;
    }

    // ==================== LQR控制器分支 ====================
    if (motor_setting->controller_type == CONTROLLER_LQR) {
      // LQR控制器：控制律本体输出转为输出轴扭矩，不经过串级PID
      float angle_feedback, velocity_feedback;
      float lqr_current_output; // LQR输出电流 [A]
      float lqr_tau_ref;        // LQR输出轴扭矩 [N·m]

      // 获取角度反馈
      if (motor_setting->angle_feedback_source == OTHER_FEED)
        angle_feedback = *motor_controller->other_angle_feedback_ptr;
      else
        angle_feedback = measure->total_angle; // 使用电机编码器

      // 获取速度反馈
      if (motor_setting->speed_feedback_source == OTHER_FEED)
        velocity_feedback = *motor_controller->other_speed_feedback_ptr;
      else
        velocity_feedback = measure->speed_aps;

      // 处理反馈方向反转
      if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) {
        angle_feedback *= -1;
        velocity_feedback *= -1;
      }

      // 处理电机反转（目标值取反）
      float target_angle = pid_ref;
      if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        target_angle *= -1;

      // LQR控制律计算
      lqr_current_output = LQRCalculate(&motor_controller->LQR, angle_feedback,
                                        velocity_feedback, target_angle);

      // 电流前馈（可选）
      if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
        lqr_current_output += *motor_controller->current_feedforward_ptr;

      lqr_tau_ref =
          DJIMotorCurrentToTauRef(&motor->physical_param, lqr_current_output);

      Controller_Effort_Output_s effort_output = {
          .semantic = CONTROLLER_OUTPUT_TAU_REF,
          .tau_ref_nm = lqr_tau_ref,
      };
      if (!DJIMotorBuildRawCommandFromEffort(&motor->physical_param,
                                             &effort_output, &set,
                                             &motor_controller->controller_output)) {
        set = 0;
        memset(&motor_controller->controller_output, 0,
               sizeof(motor_controller->controller_output));
      }
      motor_controller->output = lqr_tau_ref;
    }
    // ==================== SMC控制器分支 ====================
    else if (motor_setting->controller_type == CONTROLLER_SMC) {
      float angle_feedback = 0.0f;
      float velocity_feedback = 0.0f;
      float target_ref = pid_ref;
      float smc_output = 0.0f;
      float dt = DWT_GetDeltaT(&motor_controller->smc_dwt_cnt);

      if (dt <= 0.0f || dt > 0.02f) {
        dt = (motor_controller->smc.sample_period > 0.0f)
                 ? motor_controller->smc.sample_period
                 : ROBOT_CTRL_PERIOD_S;
      }

      if (motor_setting->angle_feedback_source == OTHER_FEED) {
        angle_feedback = *motor_controller->other_angle_feedback_ptr;
      } else {
        angle_feedback = measure->total_angle;
      }

      if (motor_setting->speed_feedback_source == OTHER_FEED) {
        velocity_feedback = *motor_controller->other_speed_feedback_ptr;
      } else {
        velocity_feedback = measure->speed_aps;
      }

      if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) {
        angle_feedback *= -1.0f;
        velocity_feedback *= -1.0f;
      }

      if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
        target_ref *= -1.0f;
      }

      SMC_ControllerSetSamplePeriod(&motor_controller->smc, dt);

      if ((motor_setting->close_loop_type & ANGLE_LOOP) &&
          motor_setting->outer_loop_type == ANGLE_LOOP) {
        SMC_ControllerUpdatePositionError(&motor_controller->smc, target_ref,
                                          angle_feedback, velocity_feedback);
        smc_output = SMC_ControllerCalculate(&motor_controller->smc);
      } else if ((motor_setting->close_loop_type & SPEED_LOOP) &&
                 motor_setting->outer_loop_type == SPEED_LOOP) {
        if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD) {
          target_ref += *motor_controller->speed_feedforward_ptr;
        }
        SMC_ControllerUpdateVelocityError(&motor_controller->smc, target_ref,
                                          velocity_feedback);
        smc_output = SMC_ControllerCalculate(&motor_controller->smc);
      }

      if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD) {
        smc_output += *motor_controller->current_feedforward_ptr;
      }

      smc_output = float_constrain(smc_output, -16384.0f, 16384.0f);
      float smc_tau_ref =
          DJIMotorRawCurrentToTauRef(&motor->physical_param, smc_output);
      Controller_Effort_Output_s effort_output = {
          .semantic = CONTROLLER_OUTPUT_TAU_REF,
          .tau_ref_nm = smc_tau_ref,
      };
      if (!DJIMotorBuildRawCommandFromEffort(&motor->physical_param,
                                             &effort_output, &set,
                                             &motor_controller->controller_output)) {
        set = 0;
        memset(&motor_controller->controller_output, 0,
               sizeof(motor_controller->controller_output));
      }
      motor_controller->output = smc_tau_ref;
    }
    // ==================== PID控制器分支（原有逻辑）====================
    else // CONTROLLER_PID
    {
      if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        pid_ref *= -1; // 设置反转

      // pid_ref会顺次通过被启用的闭环充当数据的载体
      // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
      if ((motor_setting->close_loop_type & ANGLE_LOOP) &&
          motor_setting->outer_loop_type == ANGLE_LOOP) {
        if (motor_setting->angle_feedback_source == OTHER_FEED)
          pid_measure = *motor_controller->other_angle_feedback_ptr;
        else
          pid_measure = measure->total_angle; // MOTOR_FEED,对total
                                              // angle闭环,防止在边界处出现突跃
        // 更新pid_ref进入下一个环
        pid_ref =
            PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
      }

      // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
      if ((motor_setting->close_loop_type & SPEED_LOOP) &&
          (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))) {
        if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
          pid_ref += *motor_controller->speed_feedforward_ptr;

        if (motor_setting->speed_feedback_source == OTHER_FEED)
          pid_measure = *motor_controller->other_speed_feedback_ptr;
        else // MOTOR_FEED
          pid_measure = measure->speed_aps;
        // 更新pid_ref进入下一个环
        pid_ref =
            PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
      }

      // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
      if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
        pid_ref += *motor_controller->current_feedforward_ptr;
      if (motor_setting->close_loop_type & CURRENT_LOOP) {
        pid_ref = PIDCalculate(&motor_controller->current_PID,
                               measure->real_current, pid_ref);
      }

      if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
        pid_ref *= -1;

      float pid_tau_ref =
          DJIMotorRawCurrentToTauRef(&motor->physical_param, pid_ref);
      Controller_Effort_Output_s effort_output = {
          .semantic = CONTROLLER_OUTPUT_TAU_REF,
          .tau_ref_nm = pid_tau_ref,
      };
      if (!DJIMotorBuildRawCommandFromEffort(&motor->physical_param,
                                             &effort_output, &set,
                                             &motor_controller->controller_output)) {
        set = 0;
        memset(&motor_controller->controller_output, 0,
               sizeof(motor_controller->controller_output));
      }
      motor_controller->output = pid_tau_ref;
    }

    // 分组填入发送数据
    group = motor->sender_group;
    num = motor->message_num;
    sender_assignment[group].tx_buff[2 * num] = (uint8_t)(set >> 8); // 低八位
    sender_assignment[group].tx_buff[2 * num + 1] =
        (uint8_t)(set & 0x00ff); // 高八位
  }

  // 遍历flag,检查是否要发送这一帧报文
  for (size_t i = 0; i < 6; ++i) {
    if (sender_enable_flag[i]) {
      CANTransmit(&sender_assignment[i], 1);
    }
  }
}
