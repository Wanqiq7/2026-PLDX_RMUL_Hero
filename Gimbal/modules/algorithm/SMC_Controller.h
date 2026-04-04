#ifndef SMC_CONTROLLER_H
#define SMC_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SMC_CONTROLLER_DEFAULT_SAMPLE_PERIOD 0.002f

typedef enum {
  SMC_CONTROLLER_MODE_EXPONENT = 0,
  SMC_CONTROLLER_MODE_POWER,
  SMC_CONTROLLER_MODE_TFSMC,
  SMC_CONTROLLER_MODE_VELSMC,
  SMC_CONTROLLER_MODE_EISMC,
} SMC_ControllerMode_e;

typedef struct {
  SMC_ControllerMode_e mode;

  float sample_period;

  float J;
  float K;
  float c;

  float c1;
  float c2;

  float p;
  float q;
  float beta;
  float epsilon;

  float sat_limit;
  float output_limit;
  float position_epsilon;
} SMC_ControllerInitConfig_s;

typedef struct {
  float target_now;                 // 当前目标值
  float target_last;                // 上一次目标值
  float target_differential;        // 目标值一阶导
  float target_differential_last;   // 上一次目标值一阶导
  float target_second_differential; // 目标值二阶导

  float position_feedback; // 当前位置反馈
  float velocity_feedback; // 当前速度反馈

  float position_error; // 位置误差
  float velocity_error; // 速度误差

  float position_error_integral; // 位置误差积分
  float velocity_error_integral; // 速度误差积分

  float position_error_epsilon; // 位置误差死区
  float velocity_error_epsilon; // 速度误差死区
  float error_last;             // 上一次位置误差
} SMC_ControllerError_s;

typedef struct {
  float J;
  float K;
  float c;

  float c1;
  float c2;

  float p;
  float q;
  float beta;
  float epsilon;
} SMC_ControllerParam_s;

typedef struct {
  float output;          // 控制器输出
  float sliding_surface; // 滑模面

  SMC_ControllerParam_s param;
  SMC_ControllerParam_s param_last;

  SMC_ControllerError_s error;

  float output_limit;  // 输出限幅
  float sat_limit;     // 饱和函数线性区阈值
  float sample_period; // 采样周期
  SMC_ControllerMode_e mode;
} SMC_ControllerInstance;

/**
 * @brief 初始化滑模控制器实例
 *
 * @param controller 控制器实例
 */
void SMC_ControllerInit(SMC_ControllerInstance *controller);

/**
 * @brief 使用配置结构初始化滑模控制器
 *
 * @param controller 控制器实例
 * @param config 初始化配置
 */
void SMC_ControllerInitFromConfig(SMC_ControllerInstance *controller,
                                  const SMC_ControllerInitConfig_s *config);

/**
 * @brief 修改采样周期
 *
 * @param controller 控制器实例
 * @param sample_period 采样周期，单位秒
 */
void SMC_ControllerSetSamplePeriod(SMC_ControllerInstance *controller,
                                   float sample_period);

/**
 * @brief 设置线性滑模面/速度滑模参数
 *
 * @param controller 控制器实例
 * @param J 惯量等效系数
 * @param K 趋近律系数
 * @param c 滑模面系数
 * @param epsilon 边界层厚度
 * @param sat_limit 饱和函数线性区阈值
 * @param output_limit 输出限幅
 * @param mode 模式，支持 EXPONENT/POWER/VELSMC
 * @param position_epsilon 位置误差死区
 */
void SMC_ControllerSetParam(SMC_ControllerInstance *controller, float J,
                            float K, float c, float epsilon, float sat_limit,
                            float output_limit, SMC_ControllerMode_e mode,
                            float position_epsilon);

/**
 * @brief 设置快速终端滑模参数
 */
void SMC_ControllerSetTFParam(SMC_ControllerInstance *controller, float J,
                              float K, float p, float q, float beta,
                              float epsilon, float sat_limit,
                              float output_limit, SMC_ControllerMode_e mode,
                              float position_epsilon);

/**
 * @brief 设置比例积分滑模面参数
 */
void SMC_ControllerSetEIParam(SMC_ControllerInstance *controller, float J,
                              float K, float c1, float c2, float epsilon,
                              float sat_limit, float output_limit,
                              SMC_ControllerMode_e mode,
                              float position_epsilon);

/**
 * @brief 更新位置控制用误差
 *
 * @param controller 控制器实例
 * @param target 目标位置
 * @param position_now 当前位置信号
 * @param velocity_now 当前速度信号
 */
void SMC_ControllerUpdatePositionError(SMC_ControllerInstance *controller,
                                       float target, float position_now,
                                       float velocity_now);

/**
 * @brief 更新速度控制用误差
 *
 * @param controller 控制器实例
 * @param target 目标速度
 * @param velocity_now 当前速度
 */
void SMC_ControllerUpdateVelocityError(SMC_ControllerInstance *controller,
                                       float target, float velocity_now);

/**
 * @brief 清空控制器内部误差状态，但保留参数配置
 *
 * @param controller 控制器实例
 */
void SMC_ControllerClear(SMC_ControllerInstance *controller);

/**
 * @brief 清空运行时状态并将目标历史对齐到当前参考值
 *
 * @param controller 控制器实例
 * @param target 当前参考值
 */
void SMC_ControllerResetState(SMC_ControllerInstance *controller, float target);

/**
 * @brief 仅清空积分状态
 *
 * @param controller 控制器实例
 */
void SMC_ControllerClearIntegral(SMC_ControllerInstance *controller);

/**
 * @brief 计算当前控制器输出
 *
 * @param controller 控制器实例
 * @return float 输出值
 */
float SMC_ControllerCalculate(SMC_ControllerInstance *controller);

/**
 * @brief 读取当前输出
 *
 * @param controller 控制器实例
 * @return float 输出值
 */
float SMC_ControllerGetOutput(const SMC_ControllerInstance *controller);

/**
 * @brief 手动设置输出值，便于调试或上层接管
 *
 * @param controller 控制器实例
 * @param output 输出值
 */
void SMC_ControllerSetOutput(SMC_ControllerInstance *controller, float output);

#ifdef __cplusplus
}
#endif

#endif
