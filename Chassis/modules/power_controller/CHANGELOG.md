# 功率控制模块重构说明

> 版本: v2.1.0 | 日期: 2026-02-03

## 目录

- [重构概述](#重构概述)
- [主要变更](#主要变更)
- [API 变更对照](#api-变更对照)
- [使用指南](#使用指南)
- [迁移指南](#迁移指南)

---

## 重构概述

本次重构主要解决两个问题：

1. **语义一致性问题**：`PowerUpdateCapData` 参数名与实际含义不符
2. **架构合规性问题**：模块未遵循 basic-framework 的"App层实例化"原则

### 重构目标

| 目标 | 状态 |
|------|------|
| 参数语义与实际用途一致 | ✅ |
| 符合框架实例化规范 | ✅ |
| 保持向后兼容（单实例模式） | ✅ |
| 编译通过 | ✅ |

---

## 主要变更

### 1. 语义修复

**问题**：`PowerUpdateCapData` 的参数名为 `cap_voltage`，但实际传入的是电容能量百分比。

**修复**：

```c
// 修复前
void PowerUpdateCapData(uint8_t cap_voltage, uint8_t cap_online);

// 修复后
void PowerUpdateCapData(PowerControllerInstance *instance,
                        uint8_t cap_energy, uint8_t cap_online);
```

### 2. 实例化重构

**问题**：原设计使用静态全局变量，不符合框架"App层实例化"原则。

**修复**：采用实例指针模式，App层持有实例句柄。

```c
// 修复前：静态单例，无需实例指针
PowerControllerInit(&config);
PowerControllerTask();
PowerGetLimitedOutput(motor_objs, output);

// 修复后：实例化模式，需传入实例指针
PowerControllerInstance *inst = PowerControllerRegister(&config);
PowerControllerTask(inst);
PowerGetLimitedOutput(inst, motor_objs, output);
```

---

## API 变更对照

### 初始化函数

| 项目 | 修复前 | 修复后 |
|------|--------|--------|
| 函数名 | `PowerControllerInit` | `PowerControllerRegister` |
| 返回值 | `void` | `PowerControllerInstance*` |
| 参数 | `config` | `config` |

### 接口函数签名变更

所有接口函数增加 `instance` 参数作为第一个参数：

```c
// 任务函数
void PowerControllerTask(PowerControllerInstance *instance);

// 获取限制输出
void PowerGetLimitedOutput(PowerControllerInstance *instance,
                           PowerMotorObj_t motor_objs[4],
                           float output[4]);

// 更新裁判系统数据
void PowerUpdateRefereeData(PowerControllerInstance *instance,
                            float chassis_power_limit,
                            float chassis_power_buffer,
                            float chassis_power);

// 更新电容数据（参数名也修复）
void PowerUpdateCapData(PowerControllerInstance *instance,
                        uint8_t cap_energy,    // 原 cap_voltage
                        uint8_t cap_online);

// 更新电机反馈
void PowerUpdateMotorFeedback(PowerControllerInstance *instance,
                              float motor_speeds[4],
                              float motor_torques[4]);

// 获取状态
const PowerControllerStatus_t *PowerGetStatus(PowerControllerInstance *instance);

// 设置 RLS 使能
void PowerSetRLSEnable(PowerControllerInstance *instance, uint8_t enable);

// 设置用户功率限制
void PowerSetUserLimit(PowerControllerInstance *instance, float power_limit);

// 更新裁判系统在线状态
void PowerUpdateRefereeOnline(PowerControllerInstance *instance,
                              uint8_t online,
                              uint8_t robot_level);

// 更新电机在线状态
void PowerUpdateMotorOnline(PowerControllerInstance *instance,
                            uint8_t motor_index,
                            uint8_t online);

// 获取错误标志
uint8_t PowerGetErrorFlags(PowerControllerInstance *instance);
```

---

## 使用指南

### 步骤 1：声明实例指针

在应用层文件中声明实例指针变量：

```c
#include "power_controller.h"

#if POWER_CONTROLLER_ENABLE
static PowerControllerInstance *power_ctrl = NULL;
#endif
```

### 步骤 2：初始化（注册）

在初始化函数中调用 `PowerControllerRegister`：

```c
void ChassisInit() {
    // ... 其他初始化代码 ...

#if POWER_CONTROLLER_ENABLE
    PowerControllerConfig_t power_config = {
        .k1_init = 0.22f,
        .k2_init = 1.2f,
        .k3 = 2.78f,
        .rls_lambda = 0.9999f,
        .torque_constant = M3508_TORQUE_CONSTANT,
        .current_scale = 20.0f / 16384.0f,
        .robot_division = ROBOT_HERO,
    };
    power_ctrl = PowerControllerRegister(&power_config);
#endif
}
```

### 步骤 3：周期任务中使用

在控制任务中传入实例指针：

```c
void ChassisTask() {
#if POWER_CONTROLLER_ENABLE
    // 更新数据
    PowerUpdateRefereeData(power_ctrl, power_limit, buffer, power);
    PowerUpdateCapData(power_ctrl, cap_energy, cap_online);
    PowerUpdateMotorFeedback(power_ctrl, speeds, torques);

    // 执行控制
    PowerControllerTask(power_ctrl);

    // 获取限制输出
    PowerGetLimitedOutput(power_ctrl, motor_objs, limited_output);
#endif
}
```

### 步骤 4：Ozone 调试查看状态

在 Ozone 中可直接查看以下变量：

**Watch 窗口添加变量**：

| 变量路径 | 说明 |
|----------|------|
| `power_ctrl->status.k1` | RLS 辨识的 k1 参数 |
| `power_ctrl->status.k2` | RLS 辨识的 k2 参数 |
| `power_ctrl->status.max_power_limit` | 当前功率上限 |
| `power_ctrl->status.estimated_power` | 估算功率 |
| `power_ctrl->status.energy_feedback` | 能量反馈值 |
| `power_ctrl->status.error_flags` | 错误标志位 |
| `power_ctrl->status.cap_online` | 电容在线状态 |
| `power_ctrl->status.referee_online` | 裁判系统在线状态 |

**Timeline 记录变量**（用于绘图分析）：

```
power_ctrl->status.k1
power_ctrl->status.k2
power_ctrl->status.max_power_limit
power_ctrl->status.estimated_power
```

> 提示：由于实例结构体定义在 `.c` 文件中，Ozone 需要加载带调试信息的 `.elf` 文件才能正确解析结构体成员。

---

## 迁移指南

如果你有其他代码使用了旧版 API，按以下步骤迁移：

### 快速迁移清单

- [ ] 添加实例指针变量声明
- [ ] 将 `PowerControllerInit` 改为 `PowerControllerRegister`
- [ ] 保存返回的实例指针
- [ ] 所有函数调用增加实例指针参数
- [ ] 将 `cap_voltage` 参数名改为 `cap_energy`（如有）

### 查找替换参考

| 查找 | 替换为 |
|------|--------|
| `PowerControllerInit(` | `power_ctrl = PowerControllerRegister(` |
| `PowerControllerTask()` | `PowerControllerTask(power_ctrl)` |
| `PowerGetLimitedOutput(` | `PowerGetLimitedOutput(power_ctrl, ` |
| `PowerUpdateRefereeData(` | `PowerUpdateRefereeData(power_ctrl, ` |
| `PowerUpdateCapData(` | `PowerUpdateCapData(power_ctrl, ` |
| `PowerUpdateMotorFeedback(` | `PowerUpdateMotorFeedback(power_ctrl, ` |
| `PowerUpdateRefereeOnline(` | `PowerUpdateRefereeOnline(power_ctrl, ` |
| `PowerUpdateMotorOnline(` | `PowerUpdateMotorOnline(power_ctrl, ` |

---

## 内部实现说明

### 实例结构体（对外隐藏）

实例结构体定义在 `.c` 文件中，对 App 层隐藏实现细节：

```c
struct PowerControllerInstance {
    PowerControllerConfig_t config;  // 配置参数
    RLSInstance rls;                 // RLS 辨识器
    float k1, k2, k3;                // 功率模型参数

    struct { ... } referee;          // 裁判系统数据
    struct { ... } cap;              // 电容数据
    struct { ... } motor;            // 电机反馈
    struct { ... } limit;            // 功率限制状态

    float pd_last_error_full;        // PD 控制器状态
    float pd_last_error_base;
    uint8_t error_flags;             // 错误标志
    uint8_t last_robot_level;        // 记录的机器人等级

    PowerControllerStatus_t status;  // 对外状态
};
```

### 单实例模式保留

当前实现保留单实例模式（静态分配），重复调用 `PowerControllerRegister` 返回同一实例：

```c
static PowerControllerInstance power_ctrl_instance;
static uint8_t instance_initialized = 0;

PowerControllerInstance *PowerControllerRegister(...) {
    if (instance_initialized) {
        return &power_ctrl_instance;  // 返回已有实例
    }
    // ... 初始化 ...
    instance_initialized = 1;
    return &power_ctrl_instance;
}
```

---

## 常见问题

### Q: 为什么要传入实例指针？

A: 符合 basic-framework 的设计规范，使模块可以支持多实例（如多底盘机器人），同时让 App 层明确持有资源。

### Q: 会影响性能吗？

A: 几乎不影响。指针传递的开销可忽略不计，编译器会优化。

### Q: 旧代码不改会怎样？

A: 编译错误。所有函数签名已变更，必须更新调用代码。

---

## 修改文件清单

| 文件 | 修改类型 |
|------|----------|
| `power_controller.h` | 接口声明重构 |
| `power_controller.c` | 实现重构 |
| `chassis.c` | 调用方式更新 |

---

*文档生成时间: 2026-02-03*
