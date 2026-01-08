# LQR线性二次调节器控制器使用说明

## 概述

LQR(Linear Quadratic Regulator)线性二次调节器是一种基于状态空间模型的最优控制方法。本实现参考了Mas2025云台控制代码，适用于云台Yaw/Pitch轴的角度控制。

## 理论基础

### 系统模型

对于云台系统，动力学方程为：
```
J·θ'' + b·θ' = τ = Kt·I
```

状态空间表示：
```
ẋ = Ax + Bu
x = [θ; ω]  (角度, 角速度)
u = I       (电流)

A = [0,    1  ]
    [0, -b/J  ]

B = [  0  ]
    [Kt/J ]
```

### LQR控制律

基本LQR控制律：
```
u = -K·x = K1·(θ_ref - θ) - K2·ω
```

带积分增强（消除稳态误差）：
```
u = K1·(θ_ref - θ) - K2·ω + Ki·∫(θ_ref - θ)dt
```

### 增益设计

增益K通过MATLAB离线计算：
```matlab
% 系统参数（从系统辨识获得）
J = 0.005;      % 转动惯量 [kg·m²]
b = 0.0005;     % 阻尼系数 [N·m·s/rad]
Kt = 0.741;     % 电机力矩常数 [N·m/A]

% 状态空间
A = [0, 1; 0, -b/J];
B = [0; Kt/J];

% 权重矩阵
Q = diag([1000, 5]);  % 状态权重
R = 1;                % 控制输入权重

% 求解LQR
[K, S, e] = lqr(A, B, Q, R);
% K = [K1, K2]
```

## 代码结构

### 数据类型

```c
typedef struct {
  float K_angle;           // 角度反馈增益 [A/rad]
  float K_velocity;        // 角速度反馈增益 [A·s/rad]
  float K_integral;        // 积分增益 [A/(rad·s)]
  float max_out;           // 最大输出限幅 [A]
  
  uint8_t enable_integral; // 是否启用积分项
  float integral_limit;    // 积分限幅
  float integral_deadband; // 积分死区 [rad]
  float integral_decay_coef; // 积分衰减系数
  
  // ... 内部计算变量
} LQRInstance;
```

### API接口

#### 1. 初始化

```c
void LQRInit(LQRInstance *lqr, LQR_Init_Config_s *config);
```

#### 2. 计算控制输出

```c
float LQRCalculate(LQRInstance *lqr, float measure_angle, 
                   float measure_vel, float ref);
```

#### 3. 重置状态

```c
void LQRReset(LQRInstance *lqr);
```

## 使用示例

### 示例1：云台Yaw轴LQR控制（基本）

```c
#include "controller.h"

// 定义LQR实例
LQRInstance yaw_lqr;

void GimbalInit(void) {
  // LQR初始化配置
  LQR_Init_Config_s yaw_lqr_config = {
    // 基本参数（从MATLAB计算得到）
    .K_angle = 31.6228f,      // 角度增益 [A/rad]
    .K_velocity = 3.3113f,    // 角速度增益 [A·s/rad]
    .K_integral = 0.0f,       // 不启用积分
    .max_out = 15.0f,         // 最大电流限制 [A]
    
    // 优化参数
    .enable_integral = 0,     // 禁用积分项
    .integral_limit = 0.0f,
    .integral_deadband = 0.0f,
    .integral_decay_coef = 0.0f,
  };
  
  // 初始化LQR控制器
  LQRInit(&yaw_lqr, &yaw_lqr_config);
}

void GimbalYawControl(void) {
  // 获取反馈
  float current_angle = imu_data->YawTotalAngle;  // [rad]
  float current_vel = -imu_data->Gyro[2];         // [rad/s], 注意取反
  
  // 设置目标
  float target_angle = 0.5f;  // [rad]
  
  // LQR计算
  float current_cmd = LQRCalculate(&yaw_lqr, current_angle, 
                                   current_vel, target_angle);
  
  // 转换为CAN指令并发送
  int16_t can_value = (int16_t)(current_cmd * 819.2f); // 819.2 = 16384/20
  GM6020_SetCurrent(can_value);
}
```

### 示例2：带积分项的LQR（消除稳态误差）

```c
void GimbalPitchInit(void) {
  LQRInstance pitch_lqr;
  
  LQR_Init_Config_s pitch_lqr_config = {
    // 基本参数
    .K_angle = 40.0f,         // 角度增益
    .K_velocity = 2.3802f,    // 角速度增益
    .K_integral = 0.15f,      // 积分增益
    .max_out = 18.0f,         // 最大电流
    
    // 积分优化参数（参考Mas2025 Pitch轴）
    .enable_integral = 1,              // 启用积分
    .integral_limit = 2.0f,            // 积分限幅
    .integral_deadband = 0.01f,        // 死区约0.57度
    .integral_decay_coef = 0.5f,       // 大误差时衰减50%
  };
  
  LQRInit(&pitch_lqr, &pitch_lqr_config);
}
```

### 示例3：模式切换

```c
// 切换控制模式时重置LQR
void SwitchToLQRMode(void) {
  LQRReset(&yaw_lqr);  // 清除积分项，防止积分饱和
  // ... 其他初始化
}
```

## 参数调整指南

### LQR增益来源

增益K由MATLAB通过求解Riccati方程得到，调整方法：

| 现象 | 原因 | 调整方法 |
|------|------|----------|
| 响应慢 | K1太小 | MATLAB中增大Q(1,1) |
| 超调大 | K2太小 | MATLAB中增大Q(2,2) |
| 振荡 | K2太小 | MATLAB中增大Q(2,2) |
| 电流过大 | K1太大 | MATLAB中增大R |

### 积分参数

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| K_integral | 0.1~0.5 | 过大会导致振荡 |
| integral_limit | 1.0~5.0 | 防止积分饱和 |
| integral_deadband | 0.01~0.05 rad | 约0.5°~3° |
| integral_decay_coef | 0.3~0.7 | 大误差时的衰减比例 |

## 注意事项

### 1. 单位制

**所有物理量必须使用标准单位制**：
- 角度: **弧度 [rad]**，不是度
- 角速度: **rad/s**，不是 deg/s
- 电流: **安培 [A]**，不是CAN值
- 时间: **秒 [s]**

### 2. 角速度方向

由于电机安装方向问题，陀螺仪角速度可能需要取反：
```c
float velocity = -imu_data->Gyro[2];  // 根据实际情况决定是否取反
```

### 3. 电流转换

LQR输出的是电流(A)，需要转换为CAN指令：
```c
// 电流 [A] → CAN值
int16_t can_cmd = (int16_t)(current_A * 819.2f);  // 819.2 = 16384/20

// 或者使用宏
#define CURRENT_TO_CAN(I) ((int16_t)((I) * 819.2f))
```

### 4. 输出限幅

确保电流不超过电机额定值：
```c
.max_out = 15.0f,  // GM6020额定电流约16A，留余量设为15A
```

## 性能对比

| 控制器 | 响应时间 | 超调量 | 稳态误差 | 抗扰动 | 参数调节 |
|--------|---------|--------|---------|--------|---------|
| PID | 中等 | 可能较大 | 需要I项 | 中等 | 经验调参 |
| LQR | 快 | 可控 | 需要积分补偿 | 好 | 理论设计 |
| SMC | 快 | 小 | 无 | 很好 | 需要模型 |

## 调试建议

1. **小角度测试**：先用±5°的目标角度测试
2. **确认方向**：检查正电流是否对应期望的转向
3. **监控电流**：使用Ozone监控LQR输出是否合理
4. **观察响应**：检查上升时间、超调、稳态误差
5. **逐步增大**：确认无误后逐步增大目标角度范围

## 参考资料

- Mas2025云台控制代码：`Mas2025_Hero_Control-main/代码/云台主/CarBody/Gimbal.c`
  - Yaw轴LQR: line 166-214 (纯LQR)
  - Pitch轴LQR: line 72-157 (LQR+积分)
  
- MATLAB系统辨识与LQR设计脚本：参考本目录提供的完整脚本

- 理论参考：
  - 《现代控制理论》- LQR章节
  - 《最优控制理论》- Riccati方程求解

## 版本历史

- v1.0 (2025-11-03): 初始版本，实现基本LQR和积分增强

