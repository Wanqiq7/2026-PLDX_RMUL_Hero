# Power Controller Phase 4 Native Tau Contract

## 1. 目标

Phase 4 的目标不是重写 `power_controller` 算法，而是把底盘功率限制链路的输入输出语义统一到轮端 `tau_ref` 域。

迁移后的主链路为：

```text
wheel_tau_ref
-> power_controller (native tau domain)
-> limited wheel_tau_ref
-> DJIMotorSetEffort()
```

## 2. Public API

`power_controller.h` 需要以 `PowerWheelObj_t` 作为主线输入对象：

```c
typedef struct {
  float requested_tau_nm;
  float feedback_speed_rad_s;
  float target_speed_rad_s;
  float feedback_tau_nm;
  float max_tau_nm;
  uint8_t online;
} PowerWheelObj_t;
```

主线限幅接口：

```c
void PowerGetLimitedWheelTauRef(
    PowerControllerInstance *instance,
    const PowerWheelObj_t wheel_objs[4],
    float limited_wheel_tau_ref[4]);
```

## 3. 语义约束

1. `requested_tau_nm` 表示轮端请求扭矩，不再接受应用层 raw current/can cmd。
2. `feedback_speed_rad_s` 与 `target_speed_rad_s` 保持同一轮端角速度量纲，用于延续原有限功率分配中的误差权重逻辑。
3. `feedback_tau_nm` 仅作为观测信息保留，不要求改变现有 Phase 3 统计与 RLS 路径。
4. `limited_wheel_tau_ref` 是限幅后的轮端扭矩参考，直接交给 `DJIMotorSetEffort()`。

## 4. Transitional Shim

Phase 4 允许短期保留：

```c
void PowerGetLimitedOutput(...);
```

但它只能作为兼容 shim：

1. 外部新代码不得再以它作为主线入口。
2. shim 内部必须桥接到 `PowerGetLimitedWheelTauRef()`。
3. 后续清理阶段应删除该 shim 以及所有 `PowerMotorObj_t` 主线路径依赖。

## 5. Regression Policy

Phase 4 结束时，以下门槛必须同时成立：

1. `tests/power_controller_native_tau_domain_regression.ps1` 通过。
2. `tests/chassis_legacy_power_bridge_boundary_regression.ps1` 通过。
3. `tests/chassis_tau_mainline_regression.ps1` 通过。
4. `tests/power_controller_golden_regression_bundle.ps1` 通过。
5. `tests/debug_build_hard_gate.ps1` 通过。
