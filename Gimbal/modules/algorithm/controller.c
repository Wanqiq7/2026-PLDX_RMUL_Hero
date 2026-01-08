/**
 * @file controller.c
 * @author wanghongxi
 * @author modified by neozng
 * @brief  PID控制器定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyrightCopyright (c) 2022 HNU YueLu EC all rights reserved
 */
#include "controller.h"
#include "memory.h"

/* ----------------------------下面是pid优化环节的实现----------------------------
 */

// 梯形积分
static void f_Trapezoid_Intergral(PIDInstance *pid) {
  // 计算梯形的面积,(上底+下底)*高/2
  pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

// 变速积分(误差小时积分作用更强)
static void f_Changing_Integration_Rate(PIDInstance *pid) {
  if (pid->Err * pid->Iout > 0) {
    // 积分呈累积趋势
    if (abs(pid->Err) <= pid->CoefB)
      return; // Full integral
    if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
      pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;
    else // 最大阈值,不使用积分
      pid->ITerm = 0;
  }
}

// 积分限幅
static void f_Integral_Limit(PIDInstance *pid) {
  static float temp_Output, temp_Iout;
  temp_Iout = pid->Iout + pid->ITerm;
  temp_Output = pid->Pout + pid->Iout + pid->Dout;
  if (abs(temp_Output) > pid->MaxOut) {
    if (pid->Err * pid->Iout > 0) // 积分却还在累积
    {
      pid->ITerm = 0; // 当前积分项置零
    }
  }

  if (temp_Iout > pid->IntegralLimit) {
    pid->ITerm = 0;
    pid->Iout = pid->IntegralLimit;
  }
  if (temp_Iout < -pid->IntegralLimit) {
    pid->ITerm = 0;
    pid->Iout = -pid->IntegralLimit;
  }
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
static void f_Derivative_On_Measurement(PIDInstance *pid) {
  pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 微分滤波(采集微分时,滤除高频噪声)
static void f_Derivative_Filter(PIDInstance *pid) {
  pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
              pid->Last_Dout * pid->Derivative_LPF_RC /
                  (pid->Derivative_LPF_RC + pid->dt);
}

// 输出滤波
static void f_Output_Filter(PIDInstance *pid) {
  pid->Output =
      pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
      pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

// 输出限幅
static void f_Output_Limit(PIDInstance *pid) {
  if (pid->Output > pid->MaxOut) {
    pid->Output = pid->MaxOut;
  }
  if (pid->Output < -(pid->MaxOut)) {
    pid->Output = -(pid->MaxOut);
  }
}

// 电机堵转检测
static void f_PID_ErrorHandle(PIDInstance *pid) {
  /*Motor Blocked Handle*/
  if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
    return;

  if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f) {
    // Motor blocked counting
    pid->ERRORHandler.ERRORCount++;
  } else {
    pid->ERRORHandler.ERRORCount = 0;
  }

  if (pid->ERRORHandler.ERRORCount > 500) {
    // Motor blocked over 1000times
    pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
  }
}

/* ---------------------------下面是PID的外部算法接口---------------------------
 */

/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 *
 * @param pid    PID实例
 * @param config PID初始化设置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config) {
  // config的数据和pid的部分数据是连续且相同的的,所以可以直接用memcpy
  // @todo:
  // 不建议这样做,可扩展性差,不知道的开发者可能会误以为pid和config是同一个结构体
  // 后续修改为逐个赋值
  memset(pid, 0, sizeof(PIDInstance));
  // utilize the quality of struct that its memeory is continuous
  memcpy(pid, config, sizeof(PID_Init_Config_s));
  // set rest of memory to 0
  DWT_GetDeltaT(&pid->DWT_CNT);
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref) {
  // 堵转检测
  if (pid->Improve & PID_ErrorHandle)
    f_PID_ErrorHandle(pid);

  pid->dt =
      DWT_GetDeltaT(&pid->DWT_CNT); // 获取两次pid计算的时间间隔,用于积分和微分

  // 保存上次的测量值和误差,计算当前error
  pid->Measure = measure;
  pid->Ref = ref;
  pid->Err = pid->Ref - pid->Measure;

  // 如果在死区外,则计算PID
  if (abs(pid->Err) > pid->DeadBand) {
    // 基本的pid计算,使用位置式
    pid->Pout = pid->Kp * pid->Err;
    pid->ITerm = pid->Ki * pid->Err * pid->dt;
    pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

    // 梯形积分
    if (pid->Improve & PID_Trapezoid_Intergral)
      f_Trapezoid_Intergral(pid);
    // 变速积分
    if (pid->Improve & PID_ChangingIntegrationRate)
      f_Changing_Integration_Rate(pid);
    // 微分先行
    if (pid->Improve & PID_Derivative_On_Measurement)
      f_Derivative_On_Measurement(pid);
    // 微分滤波器
    if (pid->Improve & PID_DerivativeFilter)
      f_Derivative_Filter(pid);
    // 积分限幅
    if (pid->Improve & PID_Integral_Limit)
      f_Integral_Limit(pid);

    pid->Iout += pid->ITerm;                         // 累加积分
    pid->Output = pid->Pout + pid->Iout + pid->Dout; // 计算输出

    // 输出滤波
    if (pid->Improve & PID_OutputFilter)
      f_Output_Filter(pid);

    // 输出限幅
    f_Output_Limit(pid);
  } else // 进入死区, 则清空积分和输出
  {
    pid->Output = 0;
    pid->ITerm = 0;
  }

  // 保存当前数据,用于下次计算
  pid->Last_Measure = pid->Measure;
  pid->Last_Output = pid->Output;
  pid->Last_Dout = pid->Dout;
  pid->Last_Err = pid->Err;
  pid->Last_ITerm = pid->ITerm;

  return pid->Output;
}

/* ----------------------------下面是SMC滑模控制器的实现----------------------------
 */

/**
 * @brief 初始化SMC滑模控制器,设置参数并将其他数据置零
 *
 * @param smc    SMC实例指针
 * @param config SMC初始化设置
 */
void SMCInit(SMCInstance *smc, SMC_Init_Config_s *config) {
  memset(smc, 0, sizeof(SMCInstance));
  // 复制配置参数
  smc->k1 = config->k1;
  smc->k2 = config->k2;
  smc->alpha = config->alpha;
  smc->beta = config->beta;
  smc->max_out = config->max_out;
  smc->feedforward_k1 = config->feedforward_k1;
  smc->feedforward_k2 = config->feedforward_k2;

  DWT_GetDeltaT(&smc->DWT_CNT);
}

/**
 * @brief          SMC滑模控制器计算
 * @param[in]      smc SMC结构体指针
 * @param[in]      measure 当前角度反馈值(单位:度)
 * @param[in]      measure_vel 当前角速度反馈值(单位:度/s)
 * @param[in]      ref 目标角度值(单位:度)
 * @retval         返回控制器输出
 */
float SMCCalculate(SMCInstance *smc, float measure, float measure_vel,
                   float ref) {
  smc->dt = DWT_GetDeltaT(&smc->DWT_CNT); // 获取两次计算的时间间隔

  // 更新目标值历史
  smc->last_last_ref = smc->last_ref;
  smc->last_ref = smc->ref;
  smc->ref = ref;

  // 保存测量值
  smc->measure = measure;
  smc->measure_vel = measure_vel;

  // 计算误差
  smc->error = smc->ref - smc->measure;

  // 计算目标角速度(目标值的一阶差分)
  // 注意:这里差分后不严格统一单位以减小噪声影响(参考仓库说明4.2节)
  float ref_vel = (smc->ref - smc->last_ref) / smc->dt;

  // 计算误差微分(误差的变化率)
  smc->error_dot = ref_vel - smc->measure_vel;

  // 计算滑模面: s = k1*e + k2*e_dot
  smc->sliding_surface = smc->k1 * smc->error + smc->k2 * smc->error_dot;

  // 计算目标加速度(目标值的二阶差分,用于前馈)
  // 同样不严格统一单位以减小噪声
  float ref_acc = (smc->ref - 2.0f * smc->last_ref + smc->last_last_ref) /
                  (smc->dt * smc->dt);

  // 前馈控制项: feedforward = k1*ref_vel + k2*ref_acc
  float feedforward =
      smc->feedforward_k1 * ref_vel + smc->feedforward_k2 * ref_acc;

  // 趋近律控制项: reaching_law = alpha*s + beta*sgn(s)
  // 使用连续的符号函数以减小抖振
  float sgn_s;
  float epsilon = 0.1f; // 边界层厚度
  if (fabsf(smc->sliding_surface) < epsilon) {
    sgn_s = smc->sliding_surface / epsilon; // 边界层内线性化
  } else {
    sgn_s = (smc->sliding_surface > 0) ? 1.0f : -1.0f;
  }

  float reaching_law = smc->alpha * smc->sliding_surface + smc->beta * sgn_s;

  // 总控制输出 = 前馈 + 趋近律
  smc->output = feedforward + reaching_law;

  // 输出限幅
  if (smc->output > smc->max_out) {
    smc->output = smc->max_out;
  } else if (smc->output < -(smc->max_out)) {
    smc->output = -(smc->max_out);
  }

  return smc->output;
}

/* ----------------------------下面是LQR控制器的实现----------------------------
 */

/**
 * @brief 初始化LQR控制器,设置参数并将其他数据置零
 *
 * @param lqr    LQR实例指针
 * @param config LQR初始化设置
 *
 * @note  LQR(线性二次调节器)是基于状态空间模型的最优控制方法
 *        控制律: u = -K*x，其中K通过求解Riccati方程得到
 *        对于云台系统: x = [θ; ω]，u = I (电流)
 *
 *        设计流程:
 *        1. MATLAB中系统辨识得到物理参数 J, b
 *        2. 构造状态空间: A = [0, 1; 0, -b/J], B = [0; Kt/J]
 *        3. 选择权重矩阵 Q, R
 *        4. 求解LQR: [K, S, e] = lqr(A, B, Q, R)
 *        5. 将K值配置到此处
 */
void LQRInit(LQRInstance *lqr, LQR_Init_Config_s *config) {
  // 清零整个结构体
  memset(lqr, 0, sizeof(LQRInstance));

  // 复制配置参数
  lqr->K_angle = config->K_angle;
  lqr->K_velocity = config->K_velocity;
  lqr->K_integral = config->K_integral;
  lqr->max_out = config->max_out;

  lqr->enable_integral = config->enable_integral;
  lqr->integral_limit = config->integral_limit;
  lqr->integral_deadband = config->integral_deadband;
  lqr->integral_decay_coef = config->integral_decay_coef;

  // 初始化时间计数器
  DWT_GetDeltaT(&lqr->DWT_CNT);
}

/**
 * @brief LQR控制器计算主函数
 *
 * @param lqr           LQR结构体指针
 * @param measure_angle 当前角度反馈值 [rad]
 * @param measure_vel   当前角速度反馈值 [rad/s]
 * @param ref           目标角度值 [rad]
 * @return float        控制器输出 (电流 [A])
 *
 * @note  控制律推导:
 *        对于二阶系统 J·θ'' + b·θ' = τ，其中 τ = Kt·I
 *        状态方程: ẋ = Ax + Bu，x = [θ; ω]
 *        LQR最优控制: u* = -K·x = -K1·θ - K2·ω
 *
 *        实际应用中，为消除稳态误差，可加入积分项:
 *        u = K1·(θ_ref - θ) - K2·ω + Ki·∫(θ_ref - θ)dt
 *
 *        参考Mas2025云台Yaw轴实现 (Gimbal.c line 209-214):
 *        tau = K3*(target - angle) - K4*velocity
 *        其中 K3=31.6228, K4=3.3113 (基于Q=diag([1000,1]), R=1设计)
 */
float LQRCalculate(LQRInstance *lqr, float measure_angle, float measure_vel,
                   float ref) {
  // 获取时间间隔
  lqr->dt = DWT_GetDeltaT(&lqr->DWT_CNT);

  // 保存状态变量
  lqr->ref = ref;
  lqr->measure_angle = measure_angle;
  lqr->measure_vel = measure_vel;

  // 计算角度误差 (归一化到 [-π, π])
  lqr->angle_error = lqr->ref - lqr->measure_angle;

  // 角度误差归一化 (处理角度跳变，如从-π到+π)
  while (lqr->angle_error > PI) {
    lqr->angle_error -= 2.0f * PI;
  }
  while (lqr->angle_error < -PI) {
    lqr->angle_error += 2.0f * PI;
  }

  // ===== LQR状态反馈控制律 =====
  // 基本LQR: u = K1·e_θ - K2·ω
  // 注意符号: 正误差需要正电流，角速度需要阻尼所以是负反馈
  float output_feedback =
      lqr->K_angle * lqr->angle_error - lqr->K_velocity * lqr->measure_vel;

  // ===== 积分项 (可选，用于消除稳态误差) =====
  float output_integral = 0.0f;

  if (lqr->enable_integral && lqr->dt > 0) {
    // 死区处理: 误差很小时不累积积分
    if (fabsf(lqr->angle_error) > lqr->integral_deadband) {
      // 积分项增量
      float integral_term = lqr->K_integral * lqr->angle_error * lqr->dt;

      // 变增益积分 (参考Mas2025 Pitch轴实现，Gimbal.c line 129-137)
      // 误差大时减小积分作用，防止积分饱和
      if (lqr->integral_decay_coef > 0 && lqr->integral_decay_coef < 1.0f) {
        // 当积分与误差同号时(累积趋势)，根据误差大小衰减积分增量
        if (lqr->angle_error * lqr->integral > 0) {
          float decay = lqr->integral_decay_coef;
          if (fabsf(lqr->angle_error) > 0.3f) { // 约17度
            integral_term *= (1.0f - decay);
          }
        }
      }

      // 累积积分
      lqr->integral += integral_term;

      // 积分限幅
      if (lqr->integral > lqr->integral_limit) {
        lqr->integral = lqr->integral_limit;
      } else if (lqr->integral < -lqr->integral_limit) {
        lqr->integral = -lqr->integral_limit;
      }
    }

    output_integral = lqr->integral;
  }

  // ===== 总输出 =====
  lqr->output = output_feedback + output_integral;

  // ===== 输出限幅 =====
  if (lqr->output > lqr->max_out) {
    lqr->output = lqr->max_out;
  } else if (lqr->output < -lqr->max_out) {
    lqr->output = -lqr->max_out;
  }

  return lqr->output;
}

/**
 * @brief 重置LQR控制器状态
 *
 * @param lqr LQR实例指针
 *
 * @note  在以下情况建议调用此函数:
 *        - 切换控制模式时
 *        - 电机失能后重新启用时
 *        - 检测到异常需要清除积分项时
 */
void LQRReset(LQRInstance *lqr) {
  lqr->integral = 0.0f;
  lqr->angle_error = 0.0f;
  lqr->output = 0.0f;

  // 重置时间计数器
  DWT_GetDeltaT(&lqr->DWT_CNT);
}