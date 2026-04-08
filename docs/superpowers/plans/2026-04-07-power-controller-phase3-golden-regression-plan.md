# Power Controller Phase 3 Golden Regression Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不重构 `power_controller` 逻辑的前提下，建立“行为样本级”黄金回归基线，用于后续 Phase 4 扭矩域升级时证明 `PredictPower / EnergyLoopControl / RLS / PowerGetLimitedOutput` 的行为等价。

**Architecture:** 先冻结当前 `power_controller` 的输入/输出样本和关键中间状态，再用独立回归脚本对比“给定样本输入 -> 期望输出/状态快照”。样本层放在 `tests/fixtures/`，验证层放在 `tests/*_golden_regression*`，模块代码只做最小可观测性补充，不改变控制逻辑。

**Tech Stack:** STM32 C firmware, PowerShell regression scripts, lightweight C sample harness, existing `power_controller` module

---

## Scope

### In Scope

- 为 `power_controller` 建立行为样本级黄金回归
- 固化四条关键路径的输入/输出样本
- 补最小可观测性接口或 sample harness
- 形成 Phase 4 前的“等价行为证据”

### Out of Scope

- 不迁移 `power_controller` 到扭矩域
- 不删除 `LegacyPowerBridge`
- 不改变 `power_controller` 控制逻辑
- 不在本轮调整比赛参数或重新整定控制器

---

## File Map

### Existing Files To Read / Potentially Modify

- `Chassis/modules/power_controller/power_controller.h`
- `Chassis/modules/power_controller/power_controller.c`
- `Chassis/application/chassis/chassis.c`
- `tests/power_predict_equivalence_regression.ps1`
- `tests/energy_loop_equivalence_regression.ps1`
- `tests/rls_equivalence_regression.ps1`
- `tests/limited_output_equivalence_regression.ps1`
- `task_plan.md`

### New Files To Create

- `tests/fixtures/power_controller/predict_power_samples.json`
- `tests/fixtures/power_controller/energy_loop_samples.json`
- `tests/fixtures/power_controller/rls_samples.json`
- `tests/fixtures/power_controller/limited_output_samples.json`
- `tests/power_predict_golden_regression.ps1`
- `tests/energy_loop_golden_regression.ps1`
- `tests/rls_golden_regression.ps1`
- `tests/limited_output_golden_regression.ps1`
- `tests/harnesses/power_controller_sample_harness.c`
- `tests/harnesses/power_controller_sample_harness.h`
- `docs/superpowers/specs/2026-04-07-mainline-roadmap-and-power-controller-timing.md`（如需补一小节说明样本回归已建立）

### Optional Helper Files

- `tests/harnesses/power_controller_sample_vectors.h`
- `tests/fixtures/power_controller/README.md`

---

## Chunk 1: Sample Contract Definition

### Task 1: 定义黄金样本文件格式与字段语义

**Files:**
- Create: `tests/fixtures/power_controller/README.md`
- Create: `tests/fixtures/power_controller/predict_power_samples.json`
- Create: `tests/fixtures/power_controller/energy_loop_samples.json`
- Create: `tests/fixtures/power_controller/rls_samples.json`
- Create: `tests/fixtures/power_controller/limited_output_samples.json`

- [ ] **Step 1: 写出样本契约文档**

要求至少写清：

```text
sample_id
purpose
input
expected_output
expected_state
tolerance
notes
```

- [ ] **Step 2: 为 PredictPower 定义样本字段**

```json
{
  "sample_id": "predict-hero-nominal-01",
  "input": {
    "k1": 0.22,
    "k2": 1.2,
    "k3": 5.10,
    "torque_nm": 1.8,
    "speed_rad_s": 28.0
  },
  "expected_output": {
    "predicted_power_w": 56.25
  },
  "tolerance": {
    "predicted_power_w": 0.001
  }
}
```

- [ ] **Step 3: 为 EnergyLoopControl 定义样本字段**

```json
{
  "sample_id": "energy-ref-online-cap-online-01",
  "input": {
    "referee_online": 1,
    "cap_online": 1,
    "robot_level": 3,
    "ref_limit_w": 100.0,
    "buffer_energy": 42.0,
    "cap_energy_percent": 180
  },
  "expected_state": {
    "allowed_power_w": 123.4,
    "upper_limit_w": 130.0,
    "lower_limit_w": 95.0,
    "hard_limit_w": 400.0
  },
  "tolerance": {
    "allowed_power_w": 0.01,
    "upper_limit_w": 0.01,
    "lower_limit_w": 0.01,
    "hard_limit_w": 0.01
  }
}
```

- [ ] **Step 4: 为 RLSUpdate 定义样本字段**

```json
{
  "sample_id": "rls-steady-positive-loss-01",
  "input": {
    "k1_init": 0.22,
    "k2_init": 1.2,
    "k3": 5.10,
    "feedback_power_w": 88.0,
    "motor_speeds": [25.0, 24.0, 26.0, 25.5],
    "motor_torques": [1.5, 1.6, 1.4, 1.5]
  },
  "expected_state": {
    "k1_after": 0.2198,
    "k2_after": 1.2031
  },
  "tolerance": {
    "k1_after": 0.0001,
    "k2_after": 0.0001
  }
}
```

- [ ] **Step 5: 为 PowerGetLimitedOutput 定义样本字段**

```json
{
  "sample_id": "limit-overpower-proportional-01",
  "input": {
    "max_power_w": 90.0,
    "motor_objs": [
      { "pid_output": 8000.0, "current_av": 20.0, "target_av": 35.0, "pid_max_output": 16384.0 },
      { "pid_output": 7800.0, "current_av": 19.0, "target_av": 34.0, "pid_max_output": 16384.0 },
      { "pid_output": 7600.0, "current_av": 18.0, "target_av": 33.0, "pid_max_output": 16384.0 },
      { "pid_output": 7400.0, "current_av": 17.0, "target_av": 32.0, "pid_max_output": 16384.0 }
    ]
  },
  "expected_output": {
    "limited_output": [6400.0, 6220.0, 6010.0, 5850.0]
  },
  "tolerance": {
    "limited_output": 0.1
  }
}
```

- [ ] **Step 6: 明确样本来源**

说明每类样本的来源必须是以下之一：

```text
1. 当前 power_controller 实现跑出的基线样本
2. 既有台架/实机日志回放样本
3. 明确标注的人工构造边界样本
```

禁止“拍脑袋填写期望值”。

---

## Chunk 2: Harness Design

### Task 2: 建立可重复运行的 power_controller 样本执行器

**Files:**
- Create: `tests/harnesses/power_controller_sample_harness.h`
- Create: `tests/harnesses/power_controller_sample_harness.c`
- Modify: `Chassis/modules/power_controller/power_controller.h`（仅在必要时增加只读可观测接口）

- [ ] **Step 1: 设计 harness API**

```c
typedef struct {
  PowerControllerConfig_t config;
  float referee_limit_w;
  float referee_buffer_energy;
  float referee_power_w;
  uint8_t referee_online;
  uint8_t robot_level;
  uint8_t cap_energy_percent;
  uint8_t cap_online;
  float motor_speeds[4];
  float motor_torques[4];
  PowerMotorObj_t motor_objs[4];
} PowerControllerSampleInput;

typedef struct {
  float predicted_power_w;
  float allowed_power_w;
  float upper_limit_w;
  float lower_limit_w;
  float hard_limit_w;
  float k1_after;
  float k2_after;
  float limited_output[4];
} PowerControllerSampleOutput;

uint8_t PowerControllerRunSample(
    const PowerControllerSampleInput *input,
    PowerControllerSampleOutput *output);
```

- [ ] **Step 2: 明确 harness 的限制**

```text
- 不在 harness 中改控制逻辑
- 只复用现有 public API 与必要的只读状态接口
- 若必须暴露额外内部状态，新增“只读查询”而不是 public mutator
```

- [ ] **Step 3: 规划最小只读接口**

若现有 `PowerGetStatus()` 不足以观测某些结果，可考虑只新增：

```c
const PowerControllerStatus_t *PowerGetStatus(PowerControllerInstance *instance);
```

优先复用它，不要新开大量 getter。

- [ ] **Step 4: 约束 harness 的初始化顺序**

```text
Register
-> update referee/cap/motor online state
-> update referee data
-> update cap data
-> update motor feedback
-> run task if sample requires state evolution
-> get limited output if sample requires actuator allocation
-> read status snapshot
```

---

## Chunk 3: PredictPower Golden Samples

### Task 3: 建立 PredictPower 行为样本回归

**Files:**
- Create: `tests/power_predict_golden_regression.ps1`
- Create/Fill: `tests/fixtures/power_controller/predict_power_samples.json`
- Create/Use: `tests/harnesses/power_controller_sample_harness.c`

- [ ] **Step 1: 设计样本分层**

至少包含以下样本族：

```text
- 零速零矩样本
- 正转正矩样本
- 负转负矩样本
- 正转负矩样本（回馈/制动场景）
- 高速低矩样本
- 低速高矩样本
```

- [ ] **Step 2: 固化输出判定**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\power_predict_golden_regression.ps1
```

Expected:

```text
PASS: all predict_power golden samples matched within tolerance
```

- [ ] **Step 3: 记录容差策略**

```text
- 纯代数公式样本：建议 1e-4 ~ 1e-3
- 若样本经过 float 序列化：单值 1e-3
```

---

## Chunk 4: Energy Loop Golden Samples

### Task 4: 建立 EnergyLoopControl 行为样本回归

**Files:**
- Create: `tests/energy_loop_golden_regression.ps1`
- Create/Fill: `tests/fixtures/power_controller/energy_loop_samples.json`

- [ ] **Step 1: 覆盖关键状态分支**

至少覆盖：

```text
- referee online + cap online
- referee online + cap offline
- referee offline + cap online
- referee offline + cap offline
- dual-disconnect conservative branch
- power_upper / power_lower clamp crossing branch
```

- [ ] **Step 2: 样本中同时冻结这些状态**

```text
- allowed_power_w
- upper_limit_w
- lower_limit_w
- ref_limit_w
- hard_limit_w
- buffer_feedback
- error_flags
```

- [ ] **Step 3: 单独加入等级回退样本**

验证：

```text
referee disconnect -> GetPowerLimitByLevel() fallback
```

---

## Chunk 5: RLS Golden Samples

### Task 5: 建立 PowerRLSUpdate 行为样本回归

**Files:**
- Create: `tests/rls_golden_regression.ps1`
- Create/Fill: `tests/fixtures/power_controller/rls_samples.json`

- [ ] **Step 1: 定义样本类别**

至少覆盖：

```text
- 低功率噪声样本（应不更新）
- 正常正损耗样本（应更新）
- 高速低矩样本
- 低速高矩样本
- 多步序列样本（连续 3~5 步）
```

- [ ] **Step 2: 对多步序列样本冻结轨迹**

```json
{
  "sample_id": "rls-sequence-01",
  "steps": [
    { "feedback_power_w": 82.0, "motor_speeds": [...], "motor_torques": [...] },
    { "feedback_power_w": 84.0, "motor_speeds": [...], "motor_torques": [...] },
    { "feedback_power_w": 86.0, "motor_speeds": [...], "motor_torques": [...] }
  ],
  "expected_state": {
    "k1_after": 0.2211,
    "k2_after": 1.1987
  }
}
```

- [ ] **Step 3: 明确冻结目标**

```text
- k1_after
- k2_after
- 是否发生更新
- 低功率输入时是否保持原值
```

---

## Chunk 6: Limited Output Golden Samples

### Task 6: 建立 PowerGetLimitedOutput 行为样本回归

**Files:**
- Create: `tests/limited_output_golden_regression.ps1`
- Create/Fill: `tests/fixtures/power_controller/limited_output_samples.json`

- [ ] **Step 1: 覆盖分配策略分支**

至少覆盖：

```text
- sum_positive_power <= max_power 直通样本
- 纯比例分配样本
- 高误差权重分配样本
- 中间混合权重样本
- 带负功率支路的样本
- 某轮断连样本
```

- [ ] **Step 2: 冻结输出与中间统计**

```text
- limited_output[4]
- cmd_power_sum_w
- max_power after negative-power compensation
```

- [ ] **Step 3: 特别加入边界样本**

```text
- torque very close to zero
- discriminant <= 0 branch in SolveMaxTorque()
- torque_scale saturates to [0,1]
```

---

## Chunk 7: Regression Orchestration

### Task 7: 汇总行为样本回归入口

**Files:**
- Modify: `task_plan.md`
- Optionally Create: `tests/power_controller_golden_regression_bundle.ps1`

- [ ] **Step 1: 统一推荐执行顺序**

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\power_predict_equivalence_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\energy_loop_equivalence_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\rls_equivalence_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\limited_output_equivalence_regression.ps1

powershell.exe -ExecutionPolicy Bypass -File .\tests\power_predict_golden_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\energy_loop_golden_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\rls_golden_regression.ps1
powershell.exe -ExecutionPolicy Bypass -File .\tests\limited_output_golden_regression.ps1
```

- [ ] **Step 2: 规定执行 gate**

```text
结构等价回归必须先绿
-> 行为样本回归必须全绿
-> 才允许开始 Phase 4 设计或实现
```

---

## Done Criteria

- [ ] `PredictPower` 有多组行为样本且带数值容差
- [ ] `EnergyLoopControl` 关键状态分支都有样本冻结
- [ ] `PowerRLSUpdate` 至少有单步样本和多步序列样本
- [ ] `PowerGetLimitedOutput` 关键分配分支和边界分支都有样本冻结
- [ ] 样本来源、字段语义、容差策略都有文档说明
- [ ] 这些样本可以作为 Phase 4 迁移前后的等价判定依据

---

## Verification Commands

- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\power_predict_equivalence_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\energy_loop_equivalence_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\rls_equivalence_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\limited_output_equivalence_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\power_predict_golden_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\energy_loop_golden_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\rls_golden_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\limited_output_golden_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\debug_build_hard_gate.ps1`

---

## Notes

- 本计划的目标是“冻结行为”，不是“优化实现”。
- 若某个样本需要暴露额外状态，优先新增只读观测手段，不要修改控制逻辑。
- 若当前 worktree 仍处于大范围重排期，建议先冻结一版基线再采样，避免样本跟着结构抖动。
- 本文档只定义 Phase 3 方案，不执行其中任何步骤。
