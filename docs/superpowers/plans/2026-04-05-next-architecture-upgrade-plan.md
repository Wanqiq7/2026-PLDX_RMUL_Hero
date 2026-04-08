# Next Architecture Upgrade Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `tau-ref-unified-local` 已完成第一阶段扭矩主线收敛的基础上，推进第二阶段架构升级：建立 `Gimbal` 参考仲裁层，收紧 `SetEffort / SetRef / SetRawRef` 边界，明确 `DM` / `DJI` 常规主线约束，并把“编译通过”提升为硬门槛。

**Architecture:** 常规动力执行器统一固定为 `参考源 -> Ref Manager / Mode Arbiter -> Controller -> Controller_Effort_Output(TAU_REF) -> SetEffort -> Adapter -> Protocol -> Actuator`。`Chassis power_controller` 本轮不迁移，只通过 `LegacyPowerBridge` 保持兼容。

**Tech Stack:** STM32F4 bare-metal/FreeRTOS firmware, C11, CMake, existing BSP/Module/APP layered architecture

---

## Scope

### In Scope

- `Gimbal` 参考仲裁层独立化
- `SetEffort` 成为常规主线唯一执行入口
- `SetRef / SetRawRef / DM MIT full / PVT` 明确降级为兼容或扩展能力
- `Gimbal` / `Chassis` 文档口径统一
- 编译与回归门槛重构

### Out of Scope

- 不把 `power_controller` 迁移到扭矩域
- 不删除 `LegacyPowerBridge`
- 不删除 `DM MIT full / PVT`
- 不重写系统辨识链路

---

## Phase 2A: Gimbal Reference Manager

### Task 1: 建立 `gimbal_ref_manager` 模块

**Files:**
- Create: `Gimbal/modules/algorithm/gimbal_ref_manager.h`
- Create: `Gimbal/modules/algorithm/gimbal_ref_manager.c`
- Modify: `Gimbal/modules/algorithm/algorithm.md`

- [x] 定义 `Gimbal_Ref_Output_s`
- [x] 定义 `Gimbal_Ref_Manager_s` 或等价状态结构
- [x] 提供 `Init / Reset / Step` 三类接口
- [x] 为接口增加空指针与输入边界保护
- [x] 在 `algorithm.md` 中补充职责说明和使用示例
- [x] 为新模块补充独立说明文档 `gimbal_ref_manager.md`

**Acceptance:**
- `gimbal_ref_manager` 只处理参考与接管，不直接接触电机协议与执行量
- 输出结构至少包含 `yaw_ref_rad / pitch_ref_rad / yaw_rate_ff_rad_s / pitch_rate_ff_rad_s / vision_takeover`

### Task 2: 从 `gimbal.c` 中抽离参考仲裁逻辑

**Files:**
- Modify: `Gimbal/application/gimbal/gimbal.c`
- Modify: `Gimbal/application/gimbal/gimbal.md`

- [x] 把手操参考、自瞄参考、前馈和接管状态机迁移到 `gimbal_ref_manager`
- [ ] 把 Pitch/Yaw 限位迁移到 `gimbal_ref_manager` 或其明确依赖
- [x] 保留 `gimbal.c` 只做模式编排、执行调用和反馈发布
- [ ] 确认 `SYSID` 参考接管入口不被破坏

**Acceptance:**
- `gimbal.c` 不再承担“参考组装器”角色
- `GimbalTask()` 主体可压缩为“取命令 -> RefManagerStep -> CalculateEffort -> SetEffort -> 发布反馈”

### Task 3: 统一自瞄参考收口

**Files:**
- Modify: `Gimbal/application/vision/vision.c`
- Modify: `Gimbal/modules/algorithm/vision_control.h`
- Modify: `Gimbal/modules/algorithm/vision_control.c`

- [ ] 确认视觉模块只输出参考值与前馈
- [ ] 去除一切执行量语义残留
- [ ] 明确视觉接管进入/退出时的重置策略由谁负责
- [ ] 校准 `dt` 回退策略与文档描述一致

**Acceptance:**
- `vision` 不再拥有任何电流/扭矩/协议层语义
- 控制器类型切换不需要修改 `vision` 对外接口

---

## Phase 2B: Interface Tightening

### Task 4: 收紧 `SetEffort / SetRef / SetRawRef` 角色边界

**Files:**
- Modify: `Gimbal/modules/motor/DJImotor/dji_motor.h`
- Modify: `Gimbal/modules/motor/DJImotor/dji_motor.c`
- Modify: `Chassis/modules/motor/DJImotor/dji_motor.h`
- Modify: `Chassis/modules/motor/DJImotor/dji_motor.c`

- [ ] 在头文件注释中把 `SetEffort` 定义为常规主线唯一入口
- [ ] 把 `SetRef` 标注为 compatibility only
- [ ] 把 `SetRawRef` 标注为 bypass only（辨识/调试/注入）
- [ ] 检查 `ref_effort` 优先级与清空时机，避免与兼容接口互相覆盖

**Acceptance:**
- 新增业务代码不再允许把 `SetRef` 当作常规主线接口使用
- `SetRawRef` 只保留给明确的特殊用途

### Task 5: 对称收紧 DM 侧接口边界

**Files:**
- Modify: `Gimbal/modules/motor/DMmotor/dmmotor.h`
- Modify: `Gimbal/modules/motor/DMmotor/dmmotor.c`
- Modify: `Gimbal/modules/motor/DMmotor/dmmotor.md`

- [x] 明确 `DMMotorSetEffort()` 是常规主线入口
- [x] 明确 `MIT full / PVT / velocity-only` 是兼容或扩展能力
- [x] 明确 `DMMotorCalculateTorqueEffort()` 的输入输出口径
- [ ] 检查 `ref_effort`、`use_mit_full_command`、`use_pvt_command_frame` 等状态位是否互斥清晰

**Acceptance:**
- `DM` 文档和实现对“常规主线 vs 扩展能力”的边界描述与 `DJI` 对称

### Task 6: 扫描并收紧主线调用点

**Files:**
- Modify: `Gimbal/application/gimbal/gimbal.c`
- Modify: `Chassis/application/chassis/chassis.c`
- Modify: 其他发现的调用点

- [ ] 检查常规动力链路是否全部走 `CalculateEffort -> SetEffort`
- [ ] 确认没有新的模块继续引入 `SetRef` 作为主线
- [ ] 确认 `SetRawRef` 只在辨识/调试中出现

**Acceptance:**
- 所有常规主线调用点都能按 grep 和构建验证定位

---

## Phase 2C: Verification and Documentation Gates

### Task 7: 统一文档口径

**Files:**
- Modify: `Gimbal/application/gimbal/gimbal.md`
- Modify: `Gimbal/modules/motor/DJImotor/dji_motor.md`
- Modify: `Gimbal/modules/motor/DMmotor/dmmotor.md`
- Modify: `Gimbal/modules/motor/tau_ref_adapter.md`
- Modify: `Chassis/modules/motor/tau_ref_adapter.md`

- [x] 删除“当前主线仍是 OPEN_LOOP/raw current bypass”的旧表述
- [x] 补充 `Ref Manager / Controller / SetEffort / Adapter` 的完整链路说明
- [x] 标注兼容路径和扩展路径不属于常规主线
- [x] 给 `LegacyPowerBridge` 增加显式兼容层说明

**Acceptance:**
- 文档描述与代码实现一致
- 新成员只看文档即可区分“主线”“兼容层”“扩展能力”

### Task 8: 编译通过升级为硬门槛

**Files:**
- Modify: `tests/*.ps1`（如需要）
- Modify: `docs/superpowers/plans/2026-04-05-tau-ref-motor-architecture-plan.md`
- Modify: `task_plan.md`

- [x] 明确 regex 回归只作为辅助门槛
- [x] 增加 `Gimbal` Debug 构建通过要求
- [x] 增加 `Chassis` Debug 构建通过要求
- [x] 在计划文档中补充“先编译后宣称完成”的流程

**Acceptance:**
- 任何阶段性完成都必须附带构建证据
- 仅脚本全绿不能宣称任务完成

### Task 9: 扩充回归脚本到结构级别

**Files:**
- Modify: `tests/autoaim_ff_and_vision_params_regression.ps1`
- Modify: `tests/autoaim_reference_mainline_regression.ps1`
- Modify: `tests/gimbal_seteffort_mainline_regression.ps1`
- Modify: `tests/gimbal_yaw_tau_semantic_regression.ps1`
- Modify: `tests/pitch_dm_torque_mainline_regression.ps1`

- [ ] 避免整文件字符串误报
- [ ] 尽量把断言约束到正确模块或配置块
- [ ] 为“兼容接口仍存在但非主线”建立更精确的断言

**Acceptance:**
- 回归脚本不再因为其他模块包含相同字符串而误判通过

---

## Phase 2D: Power Controller Refactor Preparation

### Task 10: 固化 `LegacyPowerBridge` 的兼容层定位

**Files:**
- Modify: `Chassis/application/chassis/chassis.c`
- Modify: `Chassis/modules/algorithm/chassis_force_control.h`
- Modify: `Chassis/modules/algorithm/chassis_force_control.c`
- Modify: `Chassis/modules/motor/tau_ref_adapter.md`

- [ ] 明确 `LegacyPowerBridge` 是 current/raw 旧域孤岛
- [ ] 禁止桥外新代码接触旧域接口
- [ ] 明确桥前后分别是什么物理语义

**Acceptance:**
- `Chassis` 新增代码不会继续把旧域语义向主线外扩散

### Task 11: 设计 `power_controller` 等价重构回归基线

**Files:**
- Create or Modify: `tests/power_controller_equivalence_*`
- Modify: `docs/superpowers/specs/2026-04-05-next-architecture-upgrade-design.md`（如需补充）

- [ ] 固化 `PredictPower` 等价回归输入输出
- [ ] 固化 `EnergyLoopControl` 等价回归输入输出
- [ ] 固化 `PowerRLSUpdate` 等价回归输入输出
- [ ] 固化 `PowerGetLimitedOutput` 等价回归输入输出

**Acceptance:**
- 在不迁移控制逻辑的前提下，为未来等价重构建立黄金样本

---

## Dependencies

- Phase 2A 是 Phase 2B 的前置依赖
- Phase 2B 与 Phase 2C 可部分并行，但编译门槛定义必须在本阶段内完成
- Phase 2D 不阻塞 `Gimbal` / `Chassis` 主线收口，但其文档定位应在本轮先固化

---

## Verification Commands

- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\autoaim_reference_mainline_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\gimbal_sysid_interface_boundary_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\gimbal_legacy_chassis_compat_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\chassis_legacy_power_bridge_boundary_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\legacy_interface_whitelist_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\shoot_seteffort_mainline_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\shoot_loader_state_boundary_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\gimbal_seteffort_mainline_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\gimbal_yaw_tau_semantic_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\pitch_dm_torque_mainline_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\autoaim_ff_and_vision_params_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\yaw_native_tau_domain_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\debug_build_hard_gate.ps1`
- [ ] `cmake --build Gimbal\build\Debug -j4`
- [ ] `cmake --build Chassis\build\Debug -j4`

---

## Done Criteria

- [ ] `gimbal_ref_manager` 建立并接入 `GimbalTask()`
- [x] `gimbal_ref_manager` 建立并接入 `GimbalTask()`
- [ ] `Gimbal` 常规链路严格固定为 `CalculateEffort -> SetEffort`
- [ ] `SetRef / SetRawRef / MIT full / PVT` 均有明确兼容或扩展标签
- [ ] `LegacyPowerBridge` 的兼容孤岛定位清晰
- [ ] 文档口径与代码实现一致
- [x] `Gimbal` / `Chassis` 构建通过
- [x] regex 回归与编译门槛都通过

## Verification Snapshot

- `powershell.exe -ExecutionPolicy Bypass -File .\tests\gimbal_ref_manager_regression.ps1`
- `powershell.exe -ExecutionPolicy Bypass -File .\tests\autoaim_reference_mainline_regression.ps1`
- `powershell.exe -ExecutionPolicy Bypass -File .\tests\gimbal_sysid_interface_boundary_regression.ps1`
- `powershell.exe -ExecutionPolicy Bypass -File .\tests\gimbal_legacy_chassis_compat_regression.ps1`
- `powershell.exe -ExecutionPolicy Bypass -File .\tests\chassis_legacy_power_bridge_boundary_regression.ps1`
- `powershell.exe -ExecutionPolicy Bypass -File .\tests\legacy_interface_whitelist_regression.ps1`
- `powershell.exe -ExecutionPolicy Bypass -File .\tests\shoot_seteffort_mainline_regression.ps1`
- `powershell.exe -ExecutionPolicy Bypass -File .\tests\shoot_loader_state_boundary_regression.ps1`
- `powershell.exe -ExecutionPolicy Bypass -File .\tests\gimbal_seteffort_mainline_regression.ps1`
- `powershell.exe -ExecutionPolicy Bypass -File .\tests\gimbal_yaw_tau_semantic_regression.ps1`
- `powershell.exe -ExecutionPolicy Bypass -File .\tests\pitch_dm_torque_mainline_regression.ps1`
- `powershell.exe -ExecutionPolicy Bypass -File .\tests\autoaim_ff_and_vision_params_regression.ps1`
- `powershell.exe -ExecutionPolicy Bypass -File .\tests\yaw_native_tau_domain_regression.ps1`
- `powershell.exe -ExecutionPolicy Bypass -File .\tests\debug_build_hard_gate.ps1`
- `cmake -S Chassis -B Chassis\build\Debug`
- `cmake --build Chassis\build\Debug -j4`
- `cmake -S Gimbal -B Gimbal\build\Debug`
- `cmake --build Gimbal\build\Debug -j4`
