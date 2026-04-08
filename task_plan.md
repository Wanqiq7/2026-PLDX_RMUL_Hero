# Task Plan

## Active Plan

- 当前阶段计划文档：`docs/superpowers/plans/2026-04-08-power-controller-phase4-native-tau-upgrade-plan.md`

## Current Goal

- 推进 `power_controller` Phase 4 原生 tau 域升级：
  - public contract 切到 `PowerWheelObj_t / PowerGetLimitedWheelTauRef`
  - 删除 `Chassis` 应用层 `LegacyPowerBridge*` helper
  - 保持 Phase 3 黄金样本与等价回归继续全绿

## Current Status

- `power_controller.h/.c` 已补齐 `PowerWheelObj_t` 与 `PowerGetLimitedWheelTauRef()`
- `Chassis/application/chassis/chassis.c` 已切到 `wheel_tau_ref -> power_controller(native tau) -> limited wheel_tau_ref -> DJIMotorSetEffort()`
- `LegacyPowerBridge*` 已从 `Chassis` 应用层代码路径移除
- `tests/power_controller_native_tau_domain_regression.ps1` 已建立并通过
- `tests/chassis_legacy_power_bridge_boundary_regression.ps1`、`tests/chassis_tau_mainline_regression.ps1`、`tests/legacy_power_bridge_whitelist_regression.ps1` 已切换到 Phase 4 口径并通过
- `tests/power_controller_golden_regression_bundle.ps1` 继续通过
- `tests/debug_build_hard_gate.ps1` 继续通过

## Next Focus

1. 继续收敛样本 harness / 文档到原生 tau 域术语
2. 评估并清理 `PowerGetLimitedOutput` 兼容 shim 的残留引用
3. 统一剩余技术文档，避免继续描述 `LegacyPowerBridge` 为现行主链路

## Legacy Whitelist

当前阶段应用层不再允许出现 `LegacyPowerBridge*` 标识符。

当前阶段常规动力执行器主线继续要求：

1. `SetEffort` 作为主线入口
2. `SetRef / SetRawRef` 只保留在明确兼容或调试白名单

## Execution Order

1. `power_controller_native_tau_domain_regression.ps1`
2. `chassis_legacy_power_bridge_boundary_regression.ps1`
3. `chassis_tau_mainline_regression.ps1`
4. `legacy_power_bridge_whitelist_regression.ps1`
5. `power_controller_golden_regression_bundle.ps1`
6. `debug_build_hard_gate.ps1`

## Hard Constraints

- 不重写 `PredictPower / EnergyLoopControl / RLS / PowerGetLimitedOutput` 的核心控制策略
- 不通过修改黄金样本期望值来掩盖行为漂移
- 编译通过是硬门槛，regex 回归只是辅助门槛
- 新增主线代码不得重新引入 `LegacyPowerBridge`

## Verification Commands

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\power_controller_native_tau_domain_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\chassis_legacy_power_bridge_boundary_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\chassis_tau_mainline_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\legacy_power_bridge_whitelist_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\power_controller_golden_regression_bundle.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\debug_build_hard_gate.ps1
```
