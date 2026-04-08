# Public Header And Include Graph Tightening Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 以 `CMake + debug_build_hard_gate` 为唯一真源，先收口公共头内部依赖，再建立 include graph 回归，最后把双板 CMake 构建入口从“递归暴露全仓头文件”收紧为“显式 public include 入口”。

**Architecture:** 本轮不直接碰 `Makefile`，也不一次性重塑所有公共头。先以 `power_controller.h` 和 `ins_task.h` 这类低风险头文件为切入口，建立最小可执行的依赖边界；随后用脚本把 include graph 规则制度化；最后仅收紧 `Chassis/Gimbal` 的 `CMakeLists.txt` include 暴露面，并用 `debug_build_hard_gate.ps1` 做硬验证。

**Tech Stack:** STM32 C firmware, CMake, PowerShell regression scripts, existing `debug_build_hard_gate.ps1`, dual-board Chassis/Gimbal firmware targets

---

## Scope

### In Scope

- 以 `CMakeLists.txt` 为唯一可信构建入口收紧 include 暴露
- 去掉 `power_controller.h`、`ins_task.h` 中不必要的内部算法层依赖
- 新增 include graph 扫描脚本并纳入回归
- 用 `debug_build_hard_gate.ps1` 验证双板构建未被破坏

### Out of Scope

- 不把 `Makefile` 作为本轮强约束对象
- 不在本轮重塑 `bmi088.h` 的 public struct 设计
- 不迁移业务逻辑或重构 `power_controller.c / ins_task.c / bmi088.c` 主行为
- 不调整 `Phase 3` 黄金回归样本本身

### Deferred Follow-Ups

- `bmi088.h` 去内部化需要额外拆分 `public config / private runtime state`，单列后续计划
- `Makefile` 是否继续维护为一等入口，待 `CMake` 方案稳定后再决定

---

## File Map

### Existing Files To Read / Potentially Modify

- `Chassis/CMakeLists.txt`
- `Gimbal/CMakeLists.txt`
- `Chassis/modules/power_controller/power_controller.h`
- `Chassis/modules/power_controller/power_controller.c`
- `Chassis/modules/imu/ins_task.h`
- `Chassis/modules/imu/ins_task.c`
- `Gimbal/modules/imu/ins_task.h`（如双板共用模式需要同步）
- `tests/debug_build_hard_gate.ps1`
- `tests/algorithm_api_contract_regression.ps1`
- `docs/superpowers/specs/2026-04-07-algorithm-layer-api-contract.md`

### New Files To Create

- `tests/public_header_internal_dependency_regression.ps1`
- `tests/include_graph_regression.ps1`
- `tests/cmake_explicit_include_regression.ps1`

### Optional Files To Modify

- `Chassis/modules/BMI088/bmi088.h`（仅补 TODO 注释或审计标记，不做接口重塑）
- `task_plan.md`（如需同步下一阶段焦点）

---

## Chunk 1: Public Header De-Internalization

### Task 1: 清理 `power_controller.h` 的内部算法依赖

**Files:**
- Modify: `Chassis/modules/power_controller/power_controller.h`
- Modify: `Chassis/modules/power_controller/power_controller.c`
- Test: `tests/public_header_internal_dependency_regression.ps1`

- [ ] **Step 1: 写 public header 约束测试**

新增脚本，至少断言：

```text
- power_controller.h 不再 include estimation/identification/rls_estimator.h
- power_controller.h 仍只暴露 public config / status / API
- power_controller.c 仍包含实际需要的 rls_estimator.h
```

- [ ] **Step 2: 先运行脚本验证当前失败**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\public_header_internal_dependency_regression.ps1
```

Expected:

```text
FAIL: power_controller.h still exposes internal estimation dependency
```

- [ ] **Step 3: 最小化修改 `power_controller.h`**

要求：

```text
- 删除对 rls_estimator.h 的直接 include
- 保持 PowerControllerConfig_t / PowerControllerStatus_t / PowerMotorObj_t 不变
- 不改变对外 API 签名
```

- [ ] **Step 4: 把内部依赖下沉到 `power_controller.c`**

要求：

```text
- 仅在 .c 中 include 需要的 estimation/identification/rls_estimator.h
- 保持编译通过
```

- [ ] **Step 5: 重新运行测试验证通过**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\public_header_internal_dependency_regression.ps1
```

Expected:

```text
PASS: public header internal dependency regression checks
```

### Task 2: 清理 `ins_task.h` 的内部估计依赖

**Files:**
- Modify: `Chassis/modules/imu/ins_task.h`
- Modify: `Chassis/modules/imu/ins_task.c`
- Optionally Modify: `Gimbal/modules/imu/ins_task.h`
- Optionally Modify: `Gimbal/modules/imu/ins_task.c`
- Test: `tests/public_header_internal_dependency_regression.ps1`

- [ ] **Step 1: 扩展回归脚本覆盖 `ins_task.h`**

新增断言：

```text
- ins_task.h 不再直接 include estimation/attitude/quaternion_ekf.h（若 public API 不需要）
- 实际实现文件仍在 .c 内包含所需估计头
```

- [ ] **Step 2: 运行脚本验证当前失败**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\public_header_internal_dependency_regression.ps1
```

Expected:

```text
FAIL: ins_task.h still exposes internal estimation dependency
```

- [ ] **Step 3: 最小化修改 `ins_task.h`**

原则：

```text
- 如果 header 中未暴露 QuaternionEKF 的类型或宏，则删除 include
- 如确实只需 float/基础类型，保持 public API 不变
- 不在本步引入更大范围接口改名
```

- [ ] **Step 4: 把实际依赖补到 `ins_task.c`**

要求：

```text
- 将 quaternion_ekf.h 下沉到实现文件
- 若 Chassis/Gimbal 两边共用模式一致，则同步处理
```

- [ ] **Step 5: 重新运行回归验证通过**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\public_header_internal_dependency_regression.ps1
```

Expected:

```text
PASS: public header internal dependency regression checks
```

### Task 3: 记录 `bmi088.h` 的后续重塑边界，但不在本轮实施

**Files:**
- Optionally Modify: `Chassis/modules/BMI088/bmi088.h`
- Optionally Modify: `docs/superpowers/specs/2026-04-07-algorithm-layer-api-contract.md`

- [ ] **Step 1: 明确 `bmi088.h` 当前不纳入首批去内部化**

说明点：

```text
- public struct 直接暴露 SPIInstance / PIDInstance / PWMInstance
- 这是 public runtime layout 问题，不是单纯删 include 能解决
- 本轮只记录风险，不动接口
```

- [ ] **Step 2: 如有必要补 TODO 注释或文档占位**

建议内容：

```text
Follow-up: split bmi088 public config from private runtime state before removing bsp/controller includes.
```

---

## Chunk 2: Include Graph Regression

### Task 4: 新增 include graph 扫描脚本

**Files:**
- Create: `tests/include_graph_regression.ps1`
- Reference: `Chassis/modules/algorithm/algorithm.md`
- Reference: `Gimbal/modules/algorithm/algorithm.md`
- Reference: `docs/superpowers/specs/2026-04-07-algorithm-layer-api-contract.md`

- [ ] **Step 1: 定义第一版规则**

第一版只拦截高价值回流：

```text
1. utils 不得 include controllers/ 或 estimation/
2. estimation 不得 include controllers/
3. application 不得 include历史顶层 algorithm 头（如 user_lib.h / kalman_filter.h）
4. 指定 public header 不得 include内部 estimation 头
```

- [ ] **Step 2: 实现脚本扫描 `#include`**

建议策略：

```text
- 用 rg 扫源码中的 #include
- 按路径分类为 application / controllers / estimation / utils / public header
- 对命中的非法模式直接 throw
```

- [ ] **Step 3: 先跑一轮，修正脚本误报**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\include_graph_regression.ps1
```

Expected:

```text
PASS: include graph regression checks
```

- [ ] **Step 4: 将脚本纳入后续架构回归集合**

要求：

```text
- 在计划和总结中把 include_graph_regression.ps1 列为新 gate
- 本轮不要求接入 bundle，只要求能独立稳定运行
```

### Task 5: 补 public header 内部依赖回归脚本

**Files:**
- Create: `tests/public_header_internal_dependency_regression.ps1`

- [ ] **Step 1: 覆盖首批已知目标**

至少覆盖：

```text
- Chassis/modules/power_controller/power_controller.h
- Chassis/modules/imu/ins_task.h
```

- [ ] **Step 2: 为后续扩展预留 allowlist / denylist 结构**

脚本结构建议：

```powershell
$denyRules = @(
  @{ Path = '...power_controller.h'; Pattern = 'estimation/' },
  @{ Path = '...ins_task.h'; Pattern = 'estimation/' }
)
```

- [ ] **Step 3: 运行验证通过**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\public_header_internal_dependency_regression.ps1
```

Expected:

```text
PASS: public header internal dependency regression checks
```

---

## Chunk 3: CMake Include Tightening

### Task 6: 仅以 `CMakeLists.txt` 为真源收紧 include 暴露

**Files:**
- Modify: `Chassis/CMakeLists.txt`
- Modify: `Gimbal/CMakeLists.txt`
- Create: `tests/cmake_explicit_include_regression.ps1`
- Verify: `tests/debug_build_hard_gate.ps1`

- [ ] **Step 1: 写 CMake 回归脚本**

断言至少包括：

```text
- 不再存在 include_sub_directories_recursively 函数
- 不再递归 include ${CMAKE_SOURCE_DIR}/modules / application / bsp
- 顶层 include 入口改为显式 include_directories(...) 或 target_include_directories(...)
```

- [ ] **Step 2: 先运行回归验证当前失败**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\cmake_explicit_include_regression.ps1
```

Expected:

```text
FAIL: CMake still exposes recursive include directories
```

- [ ] **Step 3: 先收 `Chassis/CMakeLists.txt`**

要求：

```text
- 删除递归 include helper
- 保留 algorithm 根入口（如 modules/algorithm）以支持 controllers/... include 风格
- 为 power_controller / motor / can_comm / referee / application 等模块显式列出需要的 include 根
- 不追求“最少目录”，只追求“非递归、可解释、可维护”
```

- [ ] **Step 4: 同步收 `Gimbal/CMakeLists.txt`**

要求：

```text
- 采用与 Chassis 一致的显式 include 策略
- 保留 gimbal/vision/robot_cmd 所需的 public include 根
- 不顺手修改 Makefile
```

- [ ] **Step 5: 运行 CMake 回归脚本**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\cmake_explicit_include_regression.ps1
```

Expected:

```text
PASS: cmake explicit include regression checks
```

- [ ] **Step 6: 运行双板硬构建门槛**

Run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\tests\debug_build_hard_gate.ps1
```

Expected:

```text
PASS: debug build hard gate checks
```

### Task 7: 明确 `Makefile` 不作为本轮真源

**Files:**
- Optionally Modify: `task_plan.md`
- Optionally Modify: `docs/superpowers/specs/2026-04-07-algorithm-layer-api-contract.md`

- [ ] **Step 1: 记录当前决策**

说明：

```text
- 本轮只保证 CMake + debug_build_hard_gate
- Makefile 暂不做同步收口
- 若后续仍需保留 Makefile，单独立项处理
```

---

## Done Criteria

- [ ] `power_controller.h` 不再直接依赖 `estimation/identification/rls_estimator.h`
- [ ] `ins_task.h` 不再直接暴露不必要的 estimation 依赖
- [ ] `bmi088.h` 的高风险 public 泄漏已被明确标记为后续任务，不在本轮硬做
- [ ] 已新增 `public_header_internal_dependency_regression.ps1`
- [ ] 已新增 `include_graph_regression.ps1`
- [ ] 已新增 `cmake_explicit_include_regression.ps1`
- [ ] `Chassis/Gimbal` 的 `CMakeLists.txt` 已从递归 include 改为显式 include 根
- [ ] `debug_build_hard_gate.ps1` fresh 通过
- [ ] 明确记录 `CMake + debug_build_hard_gate` 是唯一真源，`Makefile` 非本轮 gate

---

## Verification Commands

- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\public_header_internal_dependency_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\include_graph_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\cmake_explicit_include_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\algorithm_api_contract_regression.ps1`
- [ ] `powershell.exe -ExecutionPolicy Bypass -File .\tests\debug_build_hard_gate.ps1`

---

## Notes

- 本计划刻意把“公共头去内部化”放在“构建层 include 收口”之前，避免先被全局 include 缩紧引爆历史耦合。
- `power_controller.h` 与 `ins_task.h` 属于首批低风险切口；`bmi088.h` 属于接口重塑问题，单列后续处理。
- include graph 回归第一版只拦高价值违规，不做全仓最严格依赖审计，避免被历史噪声淹没。
- 只要 `debug_build_hard_gate.ps1` 仍以 CMake 为执行入口，CMake 就是本轮唯一真源。Makefile 不参与完成判定。 
