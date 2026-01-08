/**
 ******************************************************************************
 * @file    user_lib.c
 * @author  Wang Hongxi
 * @author  Modified for LQR support
 * @version V1.1.0
 * @date    2021/2/18
 * @brief   通用工具函数库，包含数学运算、滤波、矩阵运算等
 ******************************************************************************
 * @attention 本文件提供的函数适用于各种控制算法
 *            包括但不限于：PID、LQR、SMC、MPC等
 ******************************************************************************
 */
#include "user_lib.h"
#include "stdarg.h"
#include "stdio.h"

/**
 * @brief 分配并清零一块内存
 * @param size 分配大小(字节)
 * @return void* 指向分配内存的指针
 * @note 返回的内存已清零，但仍需要强制转换为你需要的类型
 */
void *zmalloc(size_t size) {
  void *ptr = user_malloc(size);
  if (ptr)
    memset(ptr, 0, size);
  return ptr;
}

/**
 * @brief 快速平方根(使用ARM DSP库)
 * @param x 输入值
 * @return float 平方根结果
 */
float Sqrt(float x) {
  if (x <= 0.0f)
    return 0.0f;
  float out = 0.0f;
  arm_sqrt_f32(x, &out);
  return out;
}

/**
 * @brief 绝对值限幅
 * @param num 输入值
 * @param Limit 限幅值
 * @return float 限幅后的结果
 * @note 将num限制在[-Limit, +Limit]范围内
 */
float abs_limit(float num, float Limit) {
  if (num > Limit)
    return Limit;
  else if (num < -Limit)
    return -Limit;
  else
    return num;
}

/**
 * @brief 符号函数
 * @param value 输入值
 * @return float 1.0(正数), -1.0(负数), 0.0(零)
 */
float sign(float value) {
  if (value > 0.0f)
    return 1.0f;
  else if (value < 0.0f)
    return -1.0f;
  else
    return 0.0f;
}

/**
 * @brief 死区函数
 * @param Value 输入值
 * @param minValue 死区下限
 * @param maxValue 死区上限
 * @return float 处理后的值
 * @note 当输入在[minValue, maxValue]内时返回0
 */
float float_deadband(float Value, float minValue, float maxValue) {
  if (Value < maxValue && Value > minValue) {
    Value = 0;
  }
  return Value;
}

/**
 * @brief 浮点数限幅
 * @param Value 输入值
 * @param minValue 最小值
 * @param maxValue 最大值
 * @return float 限幅后的值
 */
float float_constrain(float Value, float minValue, float maxValue) {
  if (Value < minValue)
    return minValue;
  if (Value > maxValue)
    return maxValue;
  return Value;
}

/**
 * @brief 整数限幅
 * @param Value 输入值
 * @param minValue 最小值
 * @param maxValue 最大值
 * @return int16_t 限幅后的值
 */
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue) {
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

/**
 * @brief 循环限幅函数(用于角度等周期性变量)
 * @param Input 输入值
 * @param minValue 最小值
 * @param maxValue 最大值
 * @return float 处理后的值
 * @note 当输入超出范围时，将其循环映射回[minValue, maxValue]
 *       例如：loop_float_constrain(370, 0, 360) = 10
 */
float loop_float_constrain(float Input, float minValue, float maxValue) {
  if (maxValue < minValue) {
    return Input;
  }

  if (Input > maxValue) {
    float len = maxValue - minValue;
    while (Input > maxValue) {
      Input -= len;
    }
  } else if (Input < minValue) {
    float len = maxValue - minValue;
    while (Input < minValue) {
      Input += len;
    }
  }
  return Input;
}

/**
 * @brief 角度格式化为-180~180度
 * @param Ang 输入角度(度)
 * @return float 格式化后的角度(度)
 */
float theta_format(float Ang) {
  return loop_float_constrain(Ang, -180.0f, 180.0f);
}

/**
 * @brief 浮点数四舍五入
 * @param raw 输入浮点数
 * @return int 四舍五入后的整数
 */
int float_rounding(float raw) {
  static int integer;
  static float decimal;
  integer = (int)raw;
  decimal = raw - integer;
  if (decimal > 0.5f)
    integer++;
  return integer;
}

/**
 * @brief 三维向量归一化(单位化)
 * @param v 输入/输出向量 [x, y, z]
 * @return float* 归一化后的向量指针
 * @note 将向量长度归一化为1
 */
float *Norm3d(float *v) {
  float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (len > 0.0f) {
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
  }
  return v;
}

/**
 * @brief 计算三维向量的模长
 * @param v 输入向量 [x, y, z]
 * @return float 模长 = sqrt(x^2 + y^2 + z^2)
 */
float NormOf3d(float *v) {
  return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/**
 * @brief 三维向量叉乘 v1 × v2
 * @param v1 向量1 [x1, y1, z1]
 * @param v2 向量2 [x2, y2, z2]
 * @param res 结果向量 [rx, ry, rz]
 * @note 叉乘结果垂直于v1和v2构成的平面
 */
void Cross3d(float *v1, float *v2, float *res) {
  res[0] = v1[1] * v2[2] - v1[2] * v2[1];
  res[1] = v1[2] * v2[0] - v1[0] * v2[2];
  res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

/**
 * @brief 三维向量点乘 v1·v2
 * @param v1 向量1 [x1, y1, z1]
 * @param v2 向量2 [x2, y2, z2]
 * @return float 点积 = x1*x2 + y1*y2 + z1*z2
 */
float Dot3d(float *v1, float *v2) {
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

/**
 * @brief 均值滤波器
 * @param new_data 新的数据
 * @param buf 数据缓冲区
 * @param len 缓冲区长度
 * @return float 滤波后的平均值
 * @note 删除buffer中的最后一个元素，填入新的元素并求平均值
 *       实现滑动窗口平均滤波
 */
float AverageFilter(float new_data, float *buf, uint8_t len) {
  float sum = 0;
  for (uint8_t i = 0; i < len - 1; i++) {
    buf[i] = buf[i + 1];
    sum += buf[i];
  }
  buf[len - 1] = new_data;
  sum += new_data;
  return sum / len;
}

/**
 * @brief 初始化矩阵
 * @param m 矩阵指针
 * @param row 行数
 * @param col 列数
 * @note 为矩阵分配内存空间
 */
void MatInit(mat *m, uint8_t row, uint8_t col) {
  m->numCols = col;
  m->numRows = row;
  m->pData = (float *)zmalloc(row * col * sizeof(float));
}

/**
 * @brief 整数低通滤波器
 * @param Out 当前输出值
 * @param K 滤波系数(0.0~1.0)，越小滤波效果越强
 * @return uint16_t 滤波后的输出
 * @note 一阶低通滤波：y[k] = K*x[k] + (1-K)*y[k-1]
 */
uint16_t LowPassFilter(uint16_t Out, float K) {
  static uint16_t Last_Out = 0;
  Out = K * Out + (1 - K) * Last_Out;
  Last_Out = Out;
  return Out;
}

/**
 * @brief 符号函数(带死区)
 * @param value 输入值
 * @param deadband 死区范围
 * @return int16_t -1, 0, or 1
 * @note 当输入值在[-deadband, deadband]范围内时返回0
 *       否则返回输入值的符号，用于底盘阻力补偿等
 */
int16_t sign_with_deadband(float value, float deadband) {
  if (value > deadband)
    return 1;
  else if (value < -deadband)
    return -1;
  else
    return 0;
}

/**
 * @brief 浮点数低通滤波器
 * @param new_value 新的输入值
 * @param K 滤波系数(0.0~1.0)，越小滤波效果越强
 * @param last_value 上一次的输出值(指针，会被更新)
 * @return float 滤波后的输出值
 * @note 一阶IIR低通滤波：y[k] = K*x[k] + (1-K)*y[k-1]
 *       截止频率与K的关系：fc ≈ K*fs/(2π)，fs为采样频率
 */
float LowPassFilter_Float(float new_value, float K, float *last_value) {
  float out = K * new_value + (1.0f - K) * (*last_value);
  *last_value = out;
  return out;
}

/**
 * @brief 斜坡软规划器核心函数(单位: CAN指令值)
 * @param target 目标指令值
 * @param current 当前规划器的输出指令值(上一周期的结果)
 * @param accel 加速度(指令值/秒)
 * @param decel 减速度(指令值/秒)
 * @param brake_decel 反向制动减速度(指令值/秒)
 * @param dt 控制周期(秒)
 * @return float 当前周期规划好的输出指令值
 * @note 用于键盘控制等场景，实现平滑的加减速过程
 */
static float SoftRamp_CMD(float target, float current, float accel, float decel,
                          float brake_decel, float dt) {
  float ramp_out = current;
  float error = target - current;

  // 判断是加速、减速还是反向制动
  if (target * current >= 0) {            // 目标值和当前值同号
    if (fabsf(target) > fabsf(current)) { // 加速
      ramp_out += accel * dt * sign(error);
    } else { // 减速
      ramp_out += decel * dt * sign(error);
    }
  } else { // 目标值和当前值异号(反向制动)
    ramp_out += brake_decel * dt * sign(error);
  }

  // 限制规划值不能超过目标值
  if (sign(error) > 0) {
    ramp_out = float_constrain(ramp_out, current, target);
  } else {
    ramp_out = float_constrain(ramp_out, target, current);
  }

  // 最终再对输出进行一次总范围的限制
  return float_constrain(ramp_out, -16384.0f, 16384.0f);
}

/**
 * @brief 通用斜坡软规划接口
 * @param target 目标值
 * @param current 当前值
 * @param reserved_zero 预留零点范围
 * @param accel 加速度
 * @param decel 减速度
 * @param brake_decel 反向制动减速度
 * @param dt 控制周期
 * @return float 规划后的输出值
 */
float SoftRamp(float target, float current, float reserved_zero, float accel,
               float decel, float brake_decel, float dt) {
  if (fabsf(target) < reserved_zero)
    target = 0;
  return SoftRamp_CMD(target, current, accel, decel, brake_decel, dt);
}

/* ==================== 矩阵运算辅助函数实现 (用于LQR等高级控制)
 * ==================== */

/**
 * @brief 2x2矩阵乘以2维向量
 *
 * @param mat_2x2 2x2矩阵，按行优先存储 [a11, a12, a21, a22]
 * @param vec_2   2维向量 [x1, x2]
 * @param result  结果向量 [y1, y2]
 *
 * @note  计算: result = mat_2x2 * vec_2
 *
 *        矩阵形式:
 *        [y1]   [a11  a12]   [x1]
 *        [y2] = [a21  a22] * [x2]
 *
 *        展开计算:
 *        y1 = a11*x1 + a12*x2
 *        y2 = a21*x1 + a22*x2
 *
 *        用于LQR控制律: u = -K*x，其中K是1x2矩阵(增益向量)
 *
 * @example
 *        float K[2] = {31.6228f, 3.3113f};  // LQR增益
 *        float state[2] = {0.1f, 0.5f};     // 状态 [角度误差, 角速度]
 *        float u = Vec2_DotProduct(K, state); // u = K1*e + K2*w
 */
void Mat2x2_Mult_Vec2(const float mat_2x2[4], const float vec_2[2],
                      float result[2]) {
  // 矩阵按行优先存储: [a11, a12, a21, a22]
  // result[0] = mat[0]*vec[0] + mat[1]*vec[1]  (第一行乘以向量)
  // result[1] = mat[2]*vec[0] + mat[3]*vec[1]  (第二行乘以向量)

  result[0] = mat_2x2[0] * vec_2[0] + mat_2x2[1] * vec_2[1];
  result[1] = mat_2x2[2] * vec_2[0] + mat_2x2[3] * vec_2[1];
}

/**
 * @brief 计算2维向量的内积(点积)
 *
 * @param vec1 向量1 [x1, x2]
 * @param vec2 向量2 [y1, y2]
 * @return float 内积结果 = x1*y1 + x2*y2
 *
 * @note  数学定义: <v1, v2> = v1·v2 = Σ(v1_i * v2_i)
 *
 *        用于LQR状态反馈:
 *        u = K1*e_θ + K2*e_ω = [K1, K2]·[e_θ, e_ω]
 *
 *        几何意义: 表示两个向量的相似程度和投影关系
 *
 * @example
 *        // LQR控制律计算
 *        float K[2] = {31.6f, 3.3f};           // 增益向量
 *        float error_state[2] = {0.1f, 0.5f};  // [角度误差, 角速度]
 *        float control = Vec2_DotProduct(K, error_state);
 *        // control = 31.6*0.1 + 3.3*0.5 = 4.81 A
 */
float Vec2_DotProduct(const float vec1[2], const float vec2[2]) {
  return vec1[0] * vec2[0] + vec1[1] * vec2[1];
}

/**
 * @brief 向量元素限幅(每个元素限制到[-limit, limit])
 *
 * @param vec   输入/输出向量(in-place操作)
 * @param n     向量维度
 * @param limit 限制范围(绝对值)
 *
 * @note  对向量的每个元素进行饱和限幅
 *        v_i ∈ [-limit, +limit]
 *
 *        用于LQR中限制状态量或控制量的大小
 *
 * @example
 *        float state[2] = {1.5f, -2.5f};
 *        Vec_Constrain(state, 2, 1.0f);
 *        // state = [1.0f, -1.0f]
 */
void Vec_Constrain(float *vec, uint8_t n, float limit) {
  for (uint8_t i = 0; i < n; i++) {
    if (vec[i] > limit) {
      vec[i] = limit;
    } else if (vec[i] < -limit) {
      vec[i] = -limit;
    }
  }
}

/**
 * @brief 角度归一化到 [-π, π]
 *
 * @param angle 输入角度 [rad]
 * @return float 归一化后的角度 [rad]
 *
 * @note  处理角度跳变问题，确保误差计算的连续性
 *
 *        数学原理:
 *        角度在圆周上是周期性的，θ 与 θ±2π 等价
 *        将角度映射到主值区间 [-π, π] 可以:
 *        1. 避免大角度累积导致的数值溢出
 *        2. 确保角度误差走最短路径
 *        3. 正确处理从-180°到+180°的跳变
 *
 *        应用场景:
 *        - LQR/PID角度控制的误差计算
 *        - 云台/底盘的角度反馈处理
 *        - 姿态解算的欧拉角归一化
 *
 * @example
 *        float angle1 = 3.5f;           // 约200度
 *        float norm1 = AngleNormalize(angle1);
 *        // norm1 = -2.78 (约-159度)，走最短路径
 *
 *        float angle2 = -4.0f;          // 约-229度
 *        float norm2 = AngleNormalize(angle2);
 *        // norm2 = 2.28 (约131度)
 */
float AngleNormalize(float angle) {
  // 方法: 利用fmodf函数取模，然后调整到[-π, π]
  // fmodf(angle, 2*PI) 将角度映射到 [-2π, 2π]
  float normalized = fmodf(angle, 2.0f * PI);

  // 调整到 [-π, π]
  if (normalized > PI) {
    normalized -= 2.0f * PI;
  } else if (normalized < -PI) {
    normalized += 2.0f * PI;
  }

  return normalized;
}
