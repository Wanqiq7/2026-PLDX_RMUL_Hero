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

/* ======================== RLS递归最小二乘算法实现 ======================== */

/**
 * @brief 初始化RLS实例
 */
void RLSInit(RLSInstance *rls, RLS_Init_Config_s *config) {
  rls->lambda = config->lambda;
  rls->delta = config->delta;
  rls->update_cnt = 0;

  // 初始化转移矩阵为单位矩阵 * delta
  rls->trans_matrix[0] = config->delta; // P[0][0]
  rls->trans_matrix[1] = 0.0f;          // P[0][1]
  rls->trans_matrix[2] = 0.0f;          // P[1][0]
  rls->trans_matrix[3] = config->delta; // P[1][1]

  // 初始化增益向量为0
  rls->gain_vector[0] = 0.0f;
  rls->gain_vector[1] = 0.0f;

  // 初始化参数向量
  rls->params_vector[0] = config->init_k1;
  rls->params_vector[1] = config->init_k2;
}

/**
 * @brief RLS算法更新
 * @note  实现公式:
 *        K(k) = P(k-1)*φ(k) / [λ + φ^T(k)*P(k-1)*φ(k)]
 *        θ(k) = θ(k-1) + K(k) * [y(k) - φ^T(k)*θ(k-1)]
 *        P(k) = [P(k-1) - K(k)*φ^T(k)*P(k-1)] / λ
 */
void RLSUpdate(RLSInstance *rls, float sample_vector[2], float actual_output) {
  float phi[2];      // 采样向量 φ(k)
  float P_phi[2];    // P(k-1) * φ(k)
  float phi_T_P_phi; // φ^T(k) * P(k-1) * φ(k)
  float phi_T_theta; // φ^T(k) * θ(k-1)
  float denominator; // 分母
  float K[2];        // 增益向量
  float error;       // 预测误差
  float K_phi_T[4];  // K(k) * φ^T(k)
  float temp_P[4];   // 临时矩阵

  phi[0] = sample_vector[0];
  phi[1] = sample_vector[1];

  // 计算 P(k-1) * φ(k)
  P_phi[0] = rls->trans_matrix[0] * phi[0] + rls->trans_matrix[1] * phi[1];
  P_phi[1] = rls->trans_matrix[2] * phi[0] + rls->trans_matrix[3] * phi[1];

  // 计算 φ^T(k) * P(k-1) * φ(k)
  phi_T_P_phi = phi[0] * P_phi[0] + phi[1] * P_phi[1];

  // 计算分母 λ + φ^T(k)*P(k-1)*φ(k)
  denominator = rls->lambda + phi_T_P_phi;

  // 防止除零
  if (fabsf(denominator) < 1e-10f)
    return;

  // 计算增益向量 K(k) = P(k-1)*φ(k) / [λ + φ^T(k)*P(k-1)*φ(k)]
  K[0] = P_phi[0] / denominator;
  K[1] = P_phi[1] / denominator;

  rls->gain_vector[0] = K[0];
  rls->gain_vector[1] = K[1];

  // 计算 φ^T(k) * θ(k-1)
  phi_T_theta = phi[0] * rls->params_vector[0] + phi[1] * rls->params_vector[1];

  // 计算预测误差
  error = actual_output - phi_T_theta;

  // 更新参数向量 θ(k) = θ(k-1) + K(k) * error
  rls->params_vector[0] += K[0] * error;
  rls->params_vector[1] += K[1] * error;

  // 限制参数在合理范围内（基于M3508电机物理特性）
  // k1: 转速损耗系数，典型值0.1-0.5（摩擦损耗）
  // k2: 力矩平方损耗系数，典型值0.01-0.1（电阻损耗I²R）
  if (rls->params_vector[0] < 0.01f)
    rls->params_vector[0] = 0.01f;
  if (rls->params_vector[1] < 0.1f)
    rls->params_vector[1] = 0.1f;
  if (rls->params_vector[0] > 0.35f)
    rls->params_vector[0] = 0.35f;
  if (rls->params_vector[1] > 2.0f) // 放宽上限，允许RLS探索更大范围
    rls->params_vector[1] = 2.0f;

  // 计算 K(k) * φ^T(k)
  K_phi_T[0] = K[0] * phi[0]; // K[0] * phi[0]
  K_phi_T[1] = K[0] * phi[1]; // K[0] * phi[1]
  K_phi_T[2] = K[1] * phi[0]; // K[1] * phi[0]
  K_phi_T[3] = K[1] * phi[1]; // K[1] * phi[1]

  // 计算 P(k) = [P(k-1) - K(k)*φ^T(k)*P(k-1)] / λ
  temp_P[0] = rls->trans_matrix[0] - K_phi_T[0] * rls->trans_matrix[0] -
              K_phi_T[1] * rls->trans_matrix[2];
  temp_P[1] = rls->trans_matrix[1] - K_phi_T[0] * rls->trans_matrix[1] -
              K_phi_T[1] * rls->trans_matrix[3];
  temp_P[2] = rls->trans_matrix[2] - K_phi_T[2] * rls->trans_matrix[0] -
              K_phi_T[3] * rls->trans_matrix[2];
  temp_P[3] = rls->trans_matrix[3] - K_phi_T[2] * rls->trans_matrix[1] -
              K_phi_T[3] * rls->trans_matrix[3];

  rls->trans_matrix[0] = temp_P[0] / rls->lambda;
  rls->trans_matrix[1] = temp_P[1] / rls->lambda;
  rls->trans_matrix[2] = temp_P[2] / rls->lambda;
  rls->trans_matrix[3] = temp_P[3] / rls->lambda;

  rls->update_cnt++;
}

/**
 * @brief 重置RLS算法
 */
void RLSReset(RLSInstance *rls) {
  // 重置转移矩阵为单位矩阵 * delta
  rls->trans_matrix[0] = rls->delta;
  rls->trans_matrix[1] = 0.0f;
  rls->trans_matrix[2] = 0.0f;
  rls->trans_matrix[3] = rls->delta;

  // 重置增益向量
  rls->gain_vector[0] = 0.0f;
  rls->gain_vector[1] = 0.0f;

  // 参数向量保持不变，不重置
  rls->update_cnt = 0;
}

/**
 * @brief 获取当前识别的参数
 */
void RLSGetParams(RLSInstance *rls, float *k1, float *k2) {
  if (k1 != NULL)
    *k1 = rls->params_vector[0];
  if (k2 != NULL)
    *k2 = rls->params_vector[1];
}

/* ----------------------------下面是底盘LQR控制器的实现----------------------------
 */

/**
 * @brief 初始化底盘LQR速度控制器
 *
 * @param lqr    LQR实例指针
 * @param config LQR初始化设置
 *
 * @note  底盘LQR用于速度闭环力控
 *        系统模型: M * dv/dt = F - b*v
 *        状态空间: dx/dt = -b/M * x + 1/M * u
 *        其中 x = velocity, u = force
 *
 *        设计流程:
 *        1. 确定底盘质量M和阻尼系数b
 *        2. 构造状态空间: A = -b/M, B = 1/M
 *        3. 选择权重 Q, R
 *        4. MATLAB中求解: K = lqr(A, B, Q, R)
 *        5. 将K值配置到此处
 */
void LQR_Velocity_Init(LQR_Velocity_Instance *lqr,
                       LQR_Velocity_Init_Config_s *config) {
  // 清零整个结构体
  memset(lqr, 0, sizeof(LQR_Velocity_Instance));

  // 复制配置参数
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
 * @brief 底盘LQR速度控制器计算主函数
 *
 * @param lqr         LQR结构体指针
 * @param measure_vel 当前速度反馈值 [m/s] 或 [rad/s]
 * @param ref         目标速度值 [m/s] 或 [rad/s]
 * @return float      控制器输出 (力 [N] 或扭矩 [N·m])
 *
 * @note  控制律推导:
 *        对于一阶系统 M·v' + b·v = F
 *        状态方程: v' = -b/M*v + 1/M*F
 *        LQR最优控制: F* = -K·v
 *
 *        实际应用中，为消除稳态误差，加入积分项:
 *        F = K_v·(v_ref - v) + Ki·∫(v_ref - v)dt
 *
 *        参考robowalker底盘力控思想，PID输出力而非速度
 */
float LQR_Velocity_Calculate(LQR_Velocity_Instance *lqr, float measure_vel,
                             float ref) {
  // 获取时间间隔
  lqr->dt = DWT_GetDeltaT(&lqr->DWT_CNT);

  // 保存状态变量
  lqr->ref = ref;
  lqr->measure_vel = measure_vel;

  // 计算速度误差
  lqr->velocity_error = lqr->ref - lqr->measure_vel;

  // ===== LQR状态反馈控制律 =====
  // 基本LQR: u = K_v·e_v
  // 正误差需要正力/扭矩
  float output_feedback = lqr->K_velocity * lqr->velocity_error;

  // ===== 积分项 (可选，用于消除稳态误差) =====
  float output_integral = 0.0f;

  if (lqr->enable_integral && lqr->dt > 0) {
    // 死区处理: 误差很小时不累积积分
    if (fabsf(lqr->velocity_error) > lqr->integral_deadband) {
      // 积分项增量
      float integral_term = lqr->K_integral * lqr->velocity_error * lqr->dt;

      // 变增益积分（误差大时减小积分作用，防止积分饱和）
      if (lqr->integral_decay_coef > 0 && lqr->integral_decay_coef < 1.0f) {
        // 当积分与误差同号时(累积趋势)，根据误差大小衰减积分增量
        if (lqr->velocity_error * lqr->integral > 0) {
          float decay = lqr->integral_decay_coef;
          if (fabsf(lqr->velocity_error) > 0.5f) { // 约0.5 m/s 或 rad/s
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
 * @brief 重置底盘LQR控制器状态
 *
 * @param lqr LQR实例指针
 *
 * @note  在以下情况建议调用此函数:
 *        - 切换控制模式时
 *        - 底盘失能后重新启用时
 *        - 检测到异常需要清除积分项时
 */
void LQR_Velocity_Reset(LQR_Velocity_Instance *lqr) {
  lqr->integral = 0.0f;
  lqr->velocity_error = 0.0f;
  lqr->output = 0.0f;

  // 重置时间计数器
  DWT_GetDeltaT(&lqr->DWT_CNT);
}