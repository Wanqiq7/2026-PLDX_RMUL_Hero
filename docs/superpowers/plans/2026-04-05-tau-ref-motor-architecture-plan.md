# Tau Ref Motor Architecture Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 完成动力执行器统一扭矩主线的第一阶段实现，并继续清理 `Gimbal`、`Chassis` 现有兼容接口。

**Architecture:** 模块层统一到 `Controller_Effort_Output -> Actuator_Command`。当前阶段保留 `Chassis` 旧功率控制器，通过 `LegacyPowerBridge` 在桥内做 current/raw 兼容换算；`Gimbal Yaw` 已统一到 `TAU_REF` 语义。

**Tech Stack:** STM32F4 bare-metal/FreeRTOS firmware, C11, CMake, existing BSP/Module/APP layered architecture

---

## Chunk 1: 已完成的第一批实现

### Task 1: 底盘扭矩主线抬升

**Files:**
- Modify: `Chassis/modules/algorithm/chassis_force_control.h`
- Modify: `Chassis/modules/algorithm/chassis_force_control.c`
- Modify: `Chassis/application/chassis/chassis.c`
- Test: `tests/chassis_tau_mainline_regression.ps1`

- [x] 将 `wheel_current` 改为 `wheel_tau_ref`
- [x] 将摩擦补偿和速度反馈参数切换到扭矩域
- [x] 在 `Chassis` 中增加 `LegacyPowerBridge`
- [x] 用回归脚本验证扭矩主线关键接口存在
- [x] 编译 `Chassis` Debug 固件通过

### Task 2: 共享 DJI effort 入口

**Files:**
- Modify: `Chassis/modules/motor/motor_def.h`
- Modify: `Chassis/modules/motor/DJImotor/dji_motor.h`
- Modify: `Chassis/modules/motor/DJImotor/dji_motor.c`
- Modify: `Gimbal/modules/motor/motor_def.h`
- Modify: `Gimbal/modules/motor/DJImotor/dji_motor.h`
- Modify: `Gimbal/modules/motor/DJImotor/dji_motor.c`

- [x] 增加 `ref_effort`
- [x] 增加 `DJIMotorSetEffort(...)`
- [x] 保持 `SetRef` 作为兼容接口

### Task 3: Gimbal Yaw 扭矩语义统一

**Files:**
- Modify: `Gimbal/modules/motor/DJImotor/dji_motor.c`
- Test: `tests/gimbal_yaw_tau_semantic_regression.ps1`

- [x] `LQR` 最终输出改为 `TAU_REF`
- [x] `SMC` 最终输出改为 `TAU_REF`
- [x] `PID` 最终输出改为 `TAU_REF`
- [x] 回归脚本通过
- [x] 编译 `Gimbal` Debug 固件通过

## Chunk 2: 下一批待实施

### Task 4: 清理 Gimbal 上层接线

**Files:**
- Modify: `Gimbal/application/gimbal/gimbal.c`
- Modify: `Gimbal/modules/motor/DJImotor/dji_motor.h`
- Modify: `Gimbal/modules/motor/DJImotor/dji_motor.c`

- [ ] 明确哪些路径属于主线 `SetEffort`
- [ ] 明确哪些路径属于 raw-current bypass
- [ ] 把 bypass 接口显式命名并加注释
- [ ] 重新验证 `AUTOAIM` 与模式切换

### Task 5: 文档同步

**Files:**
- Modify: `Gimbal/modules/motor/tau_ref_adapter.md`
- Modify: `Gimbal/modules/motor/DJImotor/dji_motor.md`
- Modify: `Chassis/modules/algorithm/chassis_force_control.h`
- Modify: `Chassis/application/chassis/chassis.c`

- [ ] 删除“主线仍输出 current/raw”的旧表述
- [ ] 增加 `LegacyPowerBridge` 说明
- [ ] 增加 `SetEffort` / `SetRef` 语义边界说明

### Task 6: 第二阶段统一入口清理

**Files:**
- Modify: `Chassis/modules/motor/DJImotor/dji_motor.h`
- Modify: `Gimbal/modules/motor/DJImotor/dji_motor.h`
- Modify: `Chassis/application/chassis/chassis.c`
- Modify: `Gimbal/application/gimbal/gimbal.c`

- [ ] 评估是否将 `SetRef` 重命名为 `SetCurrentRef`
- [ ] 若不重命名，至少补齐“兼容接口”注释
- [ ] 确认后续所有新增主线调用只走 `SetEffort`

## 当前验证命令

- [x] `powershell.exe -ExecutionPolicy Bypass -File .\tests\chassis_tau_mainline_regression.ps1`
- [x] `powershell.exe -ExecutionPolicy Bypass -File .\tests\gimbal_yaw_tau_semantic_regression.ps1`
- [x] `powershell -ExecutionPolicy Bypass -File .\Gimbal\compile.ps1 Debug`
- [x] `powershell -ExecutionPolicy Bypass -File .\Chassis\compile.ps1 Debug`
