# Power Controller Phase 4 Native Tau Domain Upgrade Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在保持 Phase 3 黄金回归等价的前提下，把 `Chassis` 的 `power_controller` 从 current/raw 兼容孤岛升级为原生 `tau_ref` 域，并移除 `LegacyPowerBridge`。

**Architecture:** Phase 4 不重写功率控制核心算法，而是把 `power_controller` 的输入输出语义从旧 raw/current 域迁到原生扭矩域。`ChassisTask()` 改为直接提交轮侧 `wheel_tau_ref` 给功率控制器，再直接拿到受限后的 `wheel_tau_ref` 送给 `DJIMotorSetEffort()`；桥接换算逻辑从应用层删除，Phase 3 黄金样本与硬构建门槛继续作为迁移等价证明。

**Tech Stack:** STM32 C firmware, PowerShell regression scripts, existing `power_controller` module, `DJIMotorSetEffort()` tau mainline

---

## Scope

### In Scope

- 将 `power_controller` public contract 升级为原生 `tau_ref` 域
- 移除 `Chassis/application/chassis/chassis.c` 内 `LegacyPowerBridge*` helper
- 让 `Chassis` 主链路变为 `wheel_tau_ref -> power_controller(native tau) -> limited wheel_tau_ref -> DJIMotorSetEffort()`
- 保持 `PredictPower / EnergyLoopControl / RLS / PowerGetLimitedOutput` 的行为等价
- 更新回归脚本，使 `LegacyPowerBridge` 从“允许的兼容孤岛”变为“禁止残留”

### Out of Scope

- 不重写 `power_controller` 内部功率分配算法
- 不修改比赛参数、RLS 调参策略或能量环策略
- 不同时推进其他执行器主线重构
- 不在本轮引入新的控制器类型或额外业务功能

---

## File Map

### Existing Files To Modify

- `Chassis/modules/power_controller/power_controller.h`
- `Chassis/modules/power_controller/power_controller.c`
- `Chassis/application/chassis/chassis.c`
- `Chassis/modules/motor/tau_ref_adapter.md`
- `tests/chassis_legacy_power_bridge_boundary_regression.ps1`
- `tests/legacy_power_bridge_whitelist_regression.ps1`
- `tests/chassis_tau_mainline_regression.ps1`
- `tests/chassis_dji_native_tau_semantic_regression.ps1`
- `tests/power_controller_golden_regression_common.ps1`
- `tests/power_predict_equivalence_regression.ps1`
- `tests/energy_loop_equivalence_regression.ps1`
- `tests/rls_equivalence_regression.ps1`
- `tests/limited_output_equivalence_regression.ps1`
- `tests/power_controller_golden_regression_bundle.ps1`
- `task_plan.md`

### New Files To Create

- `tests/power_controller_native_tau_domain_regression.ps1`
- `docs/superpowers/specs/2026-04-08-power-controller-phase4-native-tau-contract.md`

### Optional Helper Files

- `tests/harnesses/power_controller_tau_runner.c`
- `tests/harnesses/power_controller_tau_runner.h`

---

## Chunk 1: Freeze Phase 4 Contract

### Task 1: 定义原生 tau 域输入输出契约

**Files:**
- Create: `docs/superpowers/specs/2026-04-08-power-controller-phase4-native-tau-contract.md`
- Modify: `Chassis/modules/power_controller/power_controller.h`
- Modify: `Chassis/modules/motor/tau_ref_adapter.md`

- [ ] **Step 1: 写失败中的契约回归**

在 `tests/power_controller_native_tau_domain_regression.ps1` 中先断言：

```text
- power_controller.h 暴露 tau-domain motor object / API
- chassis.c 不再声明 LegacyPowerBridgeTauToRawCurrentCmd
- chassis.c 不再声明 LegacyPowerBridgeRawCurrentCmdToTau
- tau_ref_adapter.md 不再把 LegacyPowerBridge 记为主流程组成部分
```

- [ ] **Step 2: 运行回归确认失败**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\power_controller_native_tau_domain_regression.ps1
```

Expected:

```text
FAIL: power_controller public API is still legacy raw/current oriented
```

- [ ] **Step 3: 设计新的 public contract**

契约文档至少明确：

```text
- 输入物理量：wheel_tau_ref_nm / wheel_speed_rad_s / wheel_feedback_tau_nm
- 输出物理量：limited_wheel_tau_ref_nm[4]
- online/referee/cap 状态仍由现有 Update API 提供
- PowerControllerStatus_t 继续保留，用于观测而非驱动业务
```

- [ ] **Step 4: 在头文件中定义 tau-domain object**

示例目标形态：

```c
typedef struct {
    float requested_tau_nm;
    float feedback_speed_rad_s;
    float feedback_tau_nm;
    float max_tau_nm;
    uint8_t online;
} PowerWheelObj_t;

void PowerGetLimitedWheelTauRef(
    PowerControllerInstance *instance,
    const PowerWheelObj_t wheel_objs[4],
    float limited_wheel_tau_ref[4]);
```

- [ ] **Step 5: 更新说明文档**

文档必须把 `Chassis` 主链路改写为：

```text
wheel_tau_ref
-> power_controller (native tau domain)
-> limited wheel_tau_ref
-> DJIMotorSetEffort()
```

---

## Chunk 2: Refactor power_controller Interface Without Rewriting Control Logic

### Task 2: 在模块内部吸收旧域换算，保留算法等价

**Files:**
- Modify: `Chassis/modules/power_controller/power_controller.c`
- Modify: `Chassis/modules/power_controller/power_controller.h`
- Test: `tests/power_predict_equivalence_regression.ps1`
- Test: `tests/energy_loop_equivalence_regression.ps1`
- Test: `tests/rls_equivalence_regression.ps1`
- Test: `tests/limited_output_equivalence_regression.ps1`

- [ ] **Step 1: 写一个最小 failing regression**

在 `tests/power_controller_native_tau_domain_regression.ps1` 中补充断言：

```text
- power_controller.c 提供 native tau-domain limit API
- 旧 PowerGetLimitedOutput 若暂时保留，只能成为内部或兼容 shim
```

- [ ] **Step 2: 实现最小 tau-domain 入口**

要求：

```text
- 新入口接受轮侧 tau 对象
- 内部如确有必要，可在模块内部做 tau <-> current/raw 的局部换算
- 旧核心控制逻辑、样本语义、状态更新顺序不变
```

- [ ] **Step 3: 保持旧行为等价**

迁移时必须保留以下顺序：

```text
update referee/cap/motor status
-> PowerControllerTask()
-> native tau limiting
-> status snapshot
```

- [ ] **Step 4: 跑四条结构等价回归**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\power_predict_equivalence_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\energy_loop_equivalence_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\rls_equivalence_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\limited_output_equivalence_regression.ps1
```

Expected:

```text
全部 PASS
```

---

## Chunk 3: Remove LegacyPowerBridge From Chassis Application

### Task 3: 让 `ChassisTask()` 只组织 tau 语义，不再承载旧域桥接

**Files:**
- Modify: `Chassis/application/chassis/chassis.c`
- Test: `tests/chassis_legacy_power_bridge_boundary_regression.ps1`
- Test: `tests/legacy_power_bridge_whitelist_regression.ps1`
- Test: `tests/chassis_tau_mainline_regression.ps1`

- [ ] **Step 1: 先写失败中的边界回归**

把现有桥边界测试从“必须存在桥 helper”改成：

```text
- chassis.c 不再包含 LegacyPowerBridge*
- 应用层不再出现 tau->raw / raw->tau 的旧域换算 helper
- chassis tau mainline 直接消费 limited wheel_tau_ref
```

- [ ] **Step 2: 运行回归确认失败**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\chassis_legacy_power_bridge_boundary_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\legacy_power_bridge_whitelist_regression.ps1
```

Expected:

```text
FAIL: LegacyPowerBridge is still present in chassis mainline
```

- [ ] **Step 3: 删除应用层桥 helper**

删除或内联替换以下 helper：

```text
LegacyPowerBridgeTauToRawCurrentCmd
LegacyPowerBridgeRawCurrentCmdToTau
LegacyPowerBridgeSetMotorTauRef
LegacyPowerBridgeApplyWheelTauRef
LegacyPowerBridgeBuildPowerMotorObjs
LegacyPowerBridgeApplyLimitedOutput
```

- [ ] **Step 4: 重写底盘功率控制主链路**

目标形态：

```text
wheel_tau_ref
-> build PowerWheelObj_t[4]
-> PowerControllerTask()
-> PowerGetLimitedWheelTauRef()
-> DJIMotorSetEffort()
```

- [ ] **Step 5: 跑应用层边界回归**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\chassis_legacy_power_bridge_boundary_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\legacy_power_bridge_whitelist_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\chassis_tau_mainline_regression.ps1
```

Expected:

```text
全部 PASS，且 whitelist 中不再需要 LegacyPowerBridge 特例
```

---

## Chunk 4: Rebind Golden Harness To The Native Tau Contract

### Task 4: 让黄金样本继续证明“行为不变”

**Files:**
- Modify: `tests/power_controller_golden_regression_common.ps1`
- Modify: `tests/power_predict_golden_regression.ps1`
- Modify: `tests/energy_loop_golden_regression.ps1`
- Modify: `tests/rls_golden_regression.ps1`
- Modify: `tests/limited_output_golden_regression.ps1`
- Optional Create/Modify: `tests/harnesses/power_controller_tau_runner.c`

- [ ] **Step 1: 写 failing harness check**

断言 golden harness 已切到 native tau contract：

```text
- 不再依赖 LegacyPowerBridge helper
- fixture 仍可复用 Phase 3 JSON
- limited_output 样本在外部语义上变成 limited_wheel_tau_ref，对内部等价做镜像比较
```

- [ ] **Step 2: 适配 harness / common helper**

要求：

```text
- 尽量复用现有 Phase 3 fixture
- 若 limited_output fixture 字段名需要兼容保留，必须在 common helper 中集中做映射
- 不允许散落多套样本格式
```

- [ ] **Step 3: 跑黄金回归 bundle**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\power_controller_golden_regression_bundle.ps1
```

Expected:

```text
PASS: power controller golden regression bundle
```

---

## Chunk 5: Tighten Phase 4 Guards

### Task 5: 把回归门槛从“允许兼容孤岛”升级为“禁止回流旧域”

**Files:**
- Modify: `tests/chassis_legacy_power_bridge_boundary_regression.ps1`
- Modify: `tests/legacy_power_bridge_whitelist_regression.ps1`
- Modify: `tests/chassis_dji_native_tau_semantic_regression.ps1`
- Create: `tests/power_controller_native_tau_domain_regression.ps1`
- Modify: `task_plan.md`

- [ ] **Step 1: 新增 native tau domain regression**

至少检查：

```text
- power_controller public API 使用 tau 语义
- chassis 主链路不再包含 LegacyPowerBridge
- DJIMotorSetEffort 仍是底盘执行出口
- 没有新的 SetRef / raw current 回流到主线
```

- [ ] **Step 2: 收紧现有 whitelist**

目标：

```text
- LegacyPowerBridge whitelist 为空，或仅允许历史文档说明文件保留文字描述
- 应用层和模块层代码不再允许出现 LegacyPowerBridge 标识符
```

- [ ] **Step 3: 统一 task_plan gate**

在 `task_plan.md` 中把下一阶段 gate 更新为：

```text
native tau domain regression
-> golden regression bundle
-> debug_build_hard_gate
-> 才允许声称 Phase 4 完成
```

---

## Chunk 6: Final Verification

### Task 6: 执行 Phase 4 完成判定

**Files:**
- Verify only

- [ ] **Step 1: 跑结构与语义回归**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\power_controller_native_tau_domain_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\chassis_tau_mainline_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\chassis_dji_native_tau_semantic_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\legacy_power_bridge_whitelist_regression.ps1
```

- [ ] **Step 2: 跑等价与黄金回归**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\power_controller_golden_regression_bundle.ps1
```

- [ ] **Step 3: 跑硬构建 gate**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\debug_build_hard_gate.ps1
```

- [ ] **Step 4: 记录结果并提交**

```bash
git add Chassis/modules/power_controller/power_controller.h ^
        Chassis/modules/power_controller/power_controller.c ^
        Chassis/application/chassis/chassis.c ^
        Chassis/modules/motor/tau_ref_adapter.md ^
        tests/*.ps1 ^
        docs/superpowers/specs/2026-04-08-power-controller-phase4-native-tau-contract.md ^
        docs/superpowers/plans/2026-04-08-power-controller-phase4-native-tau-upgrade-plan.md
git commit -m "refactor: migrate chassis power controller to native tau domain"
```

---

## Done Criteria

- [ ] `power_controller` public input/output contract 已升级为 native tau domain
- [ ] `Chassis/application/chassis/chassis.c` 不再保留 `LegacyPowerBridge*` helper
- [ ] 底盘主链路已变为 `wheel_tau_ref -> power_controller(native tau) -> limited wheel_tau_ref -> DJIMotorSetEffort()`
- [ ] Phase 3 四类黄金样本在 Phase 4 实现后仍全部通过
- [ ] `LegacyPowerBridge` 不再作为代码路径存在，只允许在文档中保留历史说明
- [ ] `debug_build_hard_gate.ps1` fresh 通过

---

## Verification Commands

- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\power_controller_native_tau_domain_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\chassis_legacy_power_bridge_boundary_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\legacy_power_bridge_whitelist_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\chassis_tau_mainline_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\chassis_dji_native_tau_semantic_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\power_predict_equivalence_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\energy_loop_equivalence_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\rls_equivalence_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\limited_output_equivalence_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\power_controller_golden_regression_bundle.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\debug_build_hard_gate.ps1`

---

## Notes

- Phase 4 的关键词是“语义迁移”，不是“算法重写”。
- 若 `PowerGetLimitedOutput` 需要短期兼容层，必须明确标注为 transitional shim，并在 Phase 4 收尾时删除。
- 若迁移过程中发现某一类黄金样本无法直接复用，优先修正 harness 映射层，不要先改 fixture 期望值。
- 任一等价回归或黄金样本回归失败，都应先回滚到最近的语义边界点排查，不要继续叠加改动。
