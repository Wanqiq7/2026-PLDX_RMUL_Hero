# 功率控制器快速启动指南

## 适用范围

本文档描述 `Chassis/modules/power_controller/` 在当前 `tau-ref-unified-local` 分支中的接入方式。

当前实现已经完成 Phase 4 主链路迁移，底盘功率限制路径为：

```text
wheel_tau_ref
-> power_controller (native tau domain)
-> limited wheel_tau_ref
-> DJIMotorSetEffort()
```

应用层不再通过 `LegacyPowerBridge` 或 `DJIMotorSetRef()` 走旧 current/raw 语义。

---

## 已完成的集成

### 1. 模块文件

- `modules/power_controller/power_controller.h`
- `modules/power_controller/power_controller.c`
- `modules/power_controller/power_controller.md`
- `modules/power_controller/QUICK_START.md`

### 2. 底盘集成

- `application/chassis/chassis.c` 已完成：
  - 调用 `PowerControllerRegister()` 注册实例
  - 通过 `PowerUpdateRefereeData()` / `PowerUpdateCapData()` / `PowerUpdateMotorFeedback()` 更新观测数据
  - 调用 `PowerControllerTask()` 刷新能量环与 RLS
  - 使用 `PowerGetLimitedWheelTauRef()` 获取受限后的轮端 `tau_ref`
  - 通过 `DJIMotorSetEffort()` 下发限幅后的轮端扭矩参考

### 3. 回归门槛

建议至少运行：

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\power_controller_native_tau_domain_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\power_controller_golden_regression_bundle.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\debug_build_hard_gate.ps1
```

---

## 系统架构

```text
RobotInit()
  -> ChassisInit()
     -> PowerControllerRegister()

ChassisTask()
  -> ChassisForceControlGetWheelTauRef()
  -> PowerUpdateRefereeData()
  -> PowerUpdateCapData()
  -> PowerUpdateMotorFeedback()
  -> PowerControllerTask()
  -> PowerGetLimitedWheelTauRef()
  -> DJIMotorSetEffort()
```

`power_controller` 仍保持原有核心控制逻辑：

- `EnergyLoopControl()`
- `PowerRLSUpdate()`
- `PredictPower()`
- `PowerGetLimitedOutput()`（兼容 shim）

但主线外部接口已经收敛到 native tau domain。

---

## 最小接入示例

### 1. 初始化

```c
PowerControllerConfig_t power_config = {
    .k1_init = 0.22f,
    .k2_init = 1.2f,
    .k3 = 5.1f,
    .rls_lambda = 0.98f,
    .torque_constant = 0.3f,
    .current_scale = 20.0f / 16384.0f,
    .robot_division = ROBOT_INFANTRY,
};

PowerControllerInstance *power_ctrl = PowerControllerRegister(&power_config);
```

### 2. 周期更新状态

```c
PowerUpdateRefereeOnline(power_ctrl, referee_online, robot_level);
PowerUpdateRefereeData(power_ctrl, ref_limit_w, buffer_energy, chassis_power_w);
PowerUpdateCapData(power_ctrl, cap_energy_percent, cap_online);
PowerUpdateMotorFeedback(power_ctrl, motor_speeds, motor_torques);
PowerControllerTask(power_ctrl);
```

### 3. 获取限幅后的轮端扭矩参考

```c
PowerWheelObj_t wheel_objs[4] = {0};
float limited_wheel_tau_ref[4] = {0};

PowerGetLimitedWheelTauRef(power_ctrl, wheel_objs, limited_wheel_tau_ref);
```

### 4. 交给电机主线

```c
Controller_Effort_Output_s effort = {
    .semantic = CONTROLLER_OUTPUT_TAU_REF,
    .tau_ref_nm = limited_wheel_tau_ref[0],
};

DJIMotorSetEffort(motor_rf, &effort);
```

---

## 推荐观测变量

```c
const PowerControllerStatus_t *status = PowerGetStatus(power_ctrl);

float est_power_w = status->est_power_w;
float allowed_power_w = status->allowed_power_w;
float upper_limit_w = status->upper_limit_w;
float lower_limit_w = status->lower_limit_w;
float cmd_power_sum_w = status->cmd_power_sum_w;
float buffer_feedback = status->buffer_feedback;
float cap_energy_est = status->cap_energy_est;
float k1 = status->k1;
float k2 = status->k2;
uint8_t error_flags = status->error_flags;
```

说明：

- `allowed_power_w`：当前实际生效的功率上限
- `upper_limit_w / lower_limit_w`：能量环上下边界
- `cmd_power_sum_w`：本轮指令功率总和
- `buffer_feedback`：当前进入能量环的缓冲反馈量
- `cap_energy_est`：统一量纲后的储能估计值

---

## 常见误区

1. `PowerGetLimitedOutput()` 不是当前推荐主线接口。
它仍可作为兼容 shim 存在，但应用层新代码应优先使用 `PowerGetLimitedWheelTauRef()`。

2. `power_controller` 不会替代 `DJIMotorSetEffort()`。
它负责限功率，最终执行出口仍然是电机模块的 effort 主线。

3. 当前状态结构体字段名是：

```text
est_power_w
allowed_power_w
upper_limit_w
lower_limit_w
ref_limit_w
hard_limit_w
cmd_power_sum_w
buffer_feedback
cap_energy_est
```

不要再使用旧文档里的 `estimated_power`、`max_power_limit`、`energy_feedback` 等字段名。

---

## 验证建议

### 1. 结构与语义回归

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\power_controller_native_tau_domain_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\chassis_tau_mainline_regression.ps1
```

### 2. 行为等价回归

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\power_controller_golden_regression_bundle.ps1
```

### 3. 编译硬门槛

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\debug_build_hard_gate.ps1
```

---

如需进一步调参与参数解释，请继续参考 `功率控制器参数使用指南.md` 和 `power_controller.md`。
