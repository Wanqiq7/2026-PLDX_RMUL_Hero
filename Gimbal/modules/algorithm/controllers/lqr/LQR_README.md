# LQR控制器模块

## 📚 模块概述

本模块实现了LQR(Linear Quadratic Regulator)线性二次调节器控制算法，用于云台等需要高性能角度控制的场景。

**参考实现**: Mas2025云台控制代码  
**适用对象**: 云台Yaw/Pitch轴、底盘舵轮等二阶旋转系统  
**框架版本**: basic-framework v2.0+

## 🎯 功能特点

- ✅ 基于状态空间模型的最优控制
- ✅ 支持积分增强，消除稳态误差
- ✅ 变增益积分，防止积分饱和
- ✅ 完整的单位制和量纲验证
- ✅ 详细的注释和使用示例
- ✅ 配套MATLAB设计工具

## 📁 文件结构

```
modules/algorithm/
├── controller.h              # 控制器头文件（添加LQR定义）
├── controller.c              # 控制器实现（添加LQR实现）
├── user_lib.h                # 工具函数（添加矩阵运算）
├── user_lib.c                # 工具函数实现
├── LQR控制器使用说明.md      # 详细使用文档
├── LQR_Example.c             # 使用示例代码
├── LQR_MATLAB_Design.m       # MATLAB设计脚本
└── LQR_README.md             # 本文件
```

## 🚀 快速开始

### 步骤1：系统辨识（获取LQR增益）

使用项目内的系统辨识功能采集数据：

```c
// 1. 在robot_def.h中启用系统辨识
#define GIMBAL_SYSID_MODE

// 2. 编译烧录，运行系统辨识任务
// 3. 在Ozone中导出sysid_data为CSV
```

### 步骤2：MATLAB设计LQR

运行`LQR_MATLAB_Design.m`：
1. 选择导出的CSV文件
2. 手动选择有效数据区间
3. 自动完成系统辨识和LQR设计
4. 获得K_angle和K_velocity参数

### 步骤3：嵌入式代码集成

在云台应用中添加LQR控制：

```c
#include "controller.h"

// 定义LQR实例
static LQRInstance yaw_lqr;

void GimbalInit(void) {
  // LQR配置（从MATLAB获得）
  LQR_Init_Config_s config = {
    .K_angle = 31.6228f,      // [A/rad]
    .K_velocity = 3.3113f,    // [A·s/rad]
    .max_out = 15.0f,         // [A]
    .enable_integral = 0,     // 不启用积分
  };
  
  LQRInit(&yaw_lqr, &config);
}

void GimbalYawTask(void) {
  float angle = imu_data->YawTotalAngle;      // [rad]
  float velocity = -imu_data->Gyro[2];        // [rad/s]
  float target = yaw_target_angle;            // [rad]
  
  // LQR计算
  float current_A = LQRCalculate(&yaw_lqr, angle, velocity, target);
  
  // 转CAN指令
  int16_t can_cmd = (int16_t)(current_A * 819.2f);
  GM6020_SetCurrent(-can_cmd);  // 注意电机方向
}
```

## 📖 详细文档

- **理论与设计**: 参见`LQR控制器使用说明.md`
- **代码示例**: 参见`LQR_Example.c`
- **MATLAB工具**: 参见`LQR_MATLAB_Design.m`

## 🔧 参数配置

### 基本参数（必需）

| 参数 | 类型 | 单位 | 说明 | 推荐范围 |
|------|------|------|------|---------|
| K_angle | float | A/rad | 角度反馈增益 | 10~50 |
| K_velocity | float | A·s/rad | 角速度反馈增益 | 1~10 |
| max_out | float | A | 最大电流限制 | 10~20 |

### 积分参数（可选）

| 参数 | 类型 | 单位 | 说明 | 推荐值 |
|------|------|------|------|--------|
| enable_integral | uint8_t | - | 是否启用积分 | 0或1 |
| K_integral | float | A/(rad·s) | 积分增益 | 0.1~0.5 |
| integral_limit | float | - | 积分限幅 | 1.0~5.0 |
| integral_deadband | float | rad | 积分死区 | 0.01~0.05 |
| integral_decay_coef | float | - | 衰减系数 | 0.3~0.7 |

## ⚠️ 注意事项

### 1. 单位制

**必须使用标准单位制**（框架要求）：
- ✅ 角度: **弧度 [rad]**
- ✅ 角速度: **rad/s**
- ✅ 电流: **安培 [A]**
- ❌ 不要使用度、CAN值等非标准单位

### 2. 方向问题

```c
// 陀螺仪方向可能与电机相反，需要根据实际情况：
float velocity = -imu_data->Gyro[2];  // Yaw轴，可能需要取反

// CAN指令也可能需要取反
GM6020_SetCurrent(-can_cmd);
```

### 3. 电流转换

```c
// LQR输出 → CAN指令
int16_t can = (int16_t)(current_A * 819.2f);  // 819.2 = 16384/20
```

## 📊 性能对比

与Mas2025云台控制相比：

| 项目 | Mas2025 | 本框架实现 |
|------|---------|-----------|
| LQR增益 | 硬编码 | 结构体配置 |
| 积分实现 | 手动计算 | 统一接口 |
| 代码风格 | 过程式 | 面向对象 |
| 可维护性 | 中 | 高 |
| 可扩展性 | 低 | 高 |

## 🐛 调试建议

### Ozone监控变量

在Ozone中添加以下变量到Watch窗口：

```
yaw_lqr.angle_error      // 角度误差
yaw_lqr.measure_vel      // 角速度反馈
yaw_lqr.integral         // 积分项
yaw_lqr.output           // 输出电流
yaw_lqr.dt               // 计算周期
```

### 常见问题

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| 电机不转 | 方向错误/电流为0 | 检查陀螺仪和CAN方向 |
| 反向转 | 电流符号错 | 调整取反位置 |
| 振荡 | K2太小 | 重新MATLAB设计，增大Q(2,2) |
| 响应慢 | K1太小 | 重新MATLAB设计，增大Q(1,1) |
| 稳态误差 | 无积分 | 启用积分项 |

## 📚 扩展阅读

- 《现代控制理论》- LQR章节
- 《最优控制》- Riccati方程
- Mas2025代码：`代码/云台主/CarBody/Gimbal.c`
- 框架文档：`.Doc/架构介绍与开发指南.md`

## 版本记录

- **v1.0** (2025-11-03): 初始版本
  - 实现基本LQR控制律
  - 支持积分增强
  - 配套MATLAB工具
  - 完整文档和示例

