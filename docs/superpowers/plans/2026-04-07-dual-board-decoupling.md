# Dual-Board Decoupling Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove mutually exclusive execution logic from `Gimbal` and `Chassis`, keep only strict dual-board responsibilities, and preserve the CAN protocol boundary between the two boards.

**Architecture:** First shrink the build surface so only board-owned application files are compiled, then delete the opposite-board execution layers, and finally simplify runtime branches so each board has a single fixed role. Preserve board-to-board protocol structs and CAN link helpers throughout the change.

**Tech Stack:** STM32 C, CMake, board-local pub/sub (`message_center`), CAN communication (`can_comm`), referee/power modules, vision/VTM input modules.

**Execution Status:** Completed in-place on 2026-04-07.

**Execution Notes:** Intermediate git commits from the original plan were intentionally skipped because this worktree already contained unrelated local modifications, including in files touched by the decoupling work. Creating intermediate commits would have mixed unrelated changes into the same history slice.

---

### Task 1: 冻结双板协议边界与板型宏

**Files:**
- Modify: `Gimbal/application/robot_def.h`
- Modify: `Chassis/application/robot_def.h`
- Modify: `Gimbal/application/robot.c`
- Modify: `Chassis/application/robot.c`
- Test: `Gimbal/application/robot_def.h`
- Test: `Chassis/application/robot_def.h`

- [ ] **Step 1: 列出双板协议白名单并写进注释或设计文档**
确认本轮必须保留的协议结构体：`Chassis_Ctrl_Cmd_s`、`Chassis_Upload_Data_s`、`Chassis_Ctrl_*_Pkt_s`、`Chassis_Feed_*_Pkt_s`。

- [ ] **Step 2: 固定板型宏**
在 `Gimbal/application/robot_def.h` 中仅保留 `GIMBAL_BOARD`，在 `Chassis/application/robot_def.h` 中仅保留 `CHASSIS_BOARD`，删除 `ONE_BOARD` 相关可执行路径说明。

- [ ] **Step 3: 清理入口函数中的混合职责分支**
将两侧 `RobotInit/RobotTask` 收敛为固定板型入口，不再保留 `ONE_BOARD` 或对侧板执行分支。

- [ ] **Step 4: 增加协议一致性保护**
为关键双板协议结构体补齐尺寸断言，确保结构体不会因为后续修改突破 CAN 负载或双侧不一致。

- [ ] **Step 5: 运行静态检索确认角色宏收敛**
Run: `rg -n "ONE_BOARD|defined\\(ONE_BOARD\\)" Gimbal Chassis`
Expected: 不再命中可执行代码；若仍命中，仅允许出现在历史文档中。

- [ ] **Step 6: 提交这一批角色收敛改动**
Run: `git add Gimbal/application/robot_def.h Chassis/application/robot_def.h Gimbal/application/robot.c Chassis/application/robot.c docs/superpowers/specs/2026-04-07-dual-board-decoupling-design.md`
Run: `git commit -m "refactor: lock fixed board roles for dual-board split"`

### Task 2: 收缩 Gimbal 构建面并删除底盘执行层

**Files:**
- Modify: `Gimbal/CMakeLists.txt`
- Delete: `Gimbal/application/chassis/chassis.c`
- Delete: `Gimbal/application/chassis/chassis.h`
- Delete: `Gimbal/application/chassis/chassis.md`
- Modify: `Gimbal/application/robot.c`
- Test: `Gimbal/CMakeLists.txt`

- [ ] **Step 1: 将 Gimbal 的 application 源文件改为显式白名单**
只保留 `cmd`、`gimbal`、`shoot`、`vision`、`sysid`、`robot.c` 对应源文件进入构建。

- [ ] **Step 2: 删除 Gimbal 侧底盘执行目录**
删除 `Gimbal/application/chassis/*`，确保云台工程不再携带底盘运动解算和底盘执行任务。

- [ ] **Step 3: 去掉 Gimbal 入口中的底盘任务调用**
从 `Gimbal/application/robot.c` 中删除 `ChassisInit()` 与 `ChassisTask()` 调用。

- [ ] **Step 4: 检查删除后的引用残留**
Run: `rg -n "ChassisInit|ChassisTask|application/chassis|#include \\\"chassis.h\\\"" Gimbal`
Expected: 仅允许命中文档或明确保留的协议说明，不能再命中运行时代码。

- [ ] **Step 5: 验证 Gimbal 仍能收集正确源文件**
Run: `rg -n "application/.*\\.c" Gimbal/CMakeLists.txt`
Expected: `application/chassis/chassis.c` 不再位于构建白名单。

- [ ] **Step 6: 提交 Gimbal 构建面裁剪**
Run: `git add Gimbal/CMakeLists.txt Gimbal/application/robot.c Gimbal/application`
Run: `git commit -m "refactor: remove chassis execution layer from gimbal board"`

### Task 3: 简化 Gimbal 运行时控制链

**Files:**
- Modify: `Gimbal/application/cmd/robot_cmd.c`
- Modify: `Gimbal/application/cmd/robot_cmd.h`
- Modify: `Gimbal/application/robot_def.h`
- Test: `Gimbal/application/cmd/robot_cmd.c`

- [ ] **Step 1: 删除 Gimbal 中基于本板 Pub/Sub 的底盘路径**
移除 `ONE_BOARD` 分支下的 `chassis_cmd_pub`、`chassis_feed_sub` 注册与使用，仅保留 CAN 分包发送和底盘回传接收。

- [ ] **Step 2: 保留并收敛底盘 CAN 客户端逻辑**
保留 `UpdateChassisFetchDataFromCan()`、`BuildChassisCanPackets()`、`SendChassisCommandCanIfDue()`，但让它们成为唯一底盘链路。

- [ ] **Step 3: 保留本板 message_center**
确认 `cmd/gimbal/shoot/vision/sysid` 之间的消息主题仍使用 `message_center`，不要为了“去冗余”误删本板解耦机制。

- [ ] **Step 4: 清理无效分支与注释**
把仍在描述单板兼容或本板底盘执行的注释改为“云台板仅下发底盘命令并接收状态摘要”。

- [ ] **Step 5: 运行链路残留检查**
Run: `rg -n "ONE_BOARD|chassis_cmd_pub|chassis_feed_sub" Gimbal/application/cmd`
Expected: 不再出现单板路径变量和分支。

- [ ] **Step 6: 提交 Gimbal 控制链收敛**
Run: `git add Gimbal/application/cmd/robot_cmd.c Gimbal/application/cmd/robot_cmd.h Gimbal/application/robot_def.h`
Run: `git commit -m "refactor: make CAN the only chassis link on gimbal board"`

### Task 4: 收缩 Chassis 构建面并删除 cmd/gimbal/shoot 执行层

**Files:**
- Modify: `Chassis/CMakeLists.txt`
- Delete: `Chassis/application/cmd/robot_cmd.c`
- Delete: `Chassis/application/cmd/robot_cmd.h`
- Delete: `Chassis/application/cmd/robot_cmd.md`
- Delete: `Chassis/application/gimbal/gimbal.c`
- Delete: `Chassis/application/gimbal/gimbal.h`
- Delete: `Chassis/application/gimbal/gimbal.md`
- Delete: `Chassis/application/shoot/shoot.c`
- Delete: `Chassis/application/shoot/shoot.h`
- Modify: `Chassis/application/robot.c`
- Test: `Chassis/CMakeLists.txt`

- [ ] **Step 1: 将 Chassis 的 application 源文件改为显式白名单**
只保留 `chassis`、`sysid`、`robot.c` 进入构建。

- [ ] **Step 2: 删除 Chassis 侧对侧执行目录**
删除 `application/cmd/*`、`application/gimbal/*`、`application/shoot/*`，让底盘板只保留底盘相关执行逻辑。

- [ ] **Step 3: 去掉 Chassis 入口中的 RobotCMD/Gimbal/Shoot 调用**
从 `Chassis/application/robot.c` 中移除 `RobotCMDInit/Task`、`GimbalInit/Task`、`ShootInit/Task`。

- [ ] **Step 4: 检查删除后的引用残留**
Run: `rg -n "RobotCMD|GimbalInit|GimbalTask|ShootInit|ShootTask|application/(cmd|gimbal|shoot)" Chassis`
Expected: 不再命中运行时代码。

- [ ] **Step 5: 验证 Chassis 构建白名单**
Run: `rg -n "application/.*\\.c" Chassis/CMakeLists.txt`
Expected: `cmd/gimbal/shoot` 源文件不再被构建。

- [ ] **Step 6: 提交 Chassis 执行层裁剪**
Run: `git add Chassis/CMakeLists.txt Chassis/application/robot.c Chassis/application`
Run: `git commit -m "refactor: remove cmd gimbal shoot execution from chassis board"`

### Task 5: 简化 Chassis 运行时主线并保留必要公共模块

**Files:**
- Modify: `Chassis/application/chassis/chassis.c`
- Modify: `Chassis/application/chassis/chassis.h`
- Modify: `Chassis/application/robot_def.h`
- Review: `Chassis/application/sysid/sysid_task.c`
- Review: `Chassis/modules/message_center/message_center.c`
- Review: `Chassis/modules/can_comm/chassis_can_link.c`
- Test: `Chassis/application/chassis/chassis.c`

- [ ] **Step 1: 删除 Chassis 执行层中的单板 Pub/Sub 路径**
移除 `ONE_BOARD` 分支下的 `RegisterSubscriber("chassis_cmd")`、`RegisterPublisher("chassis_feed")` 以及对应 `SubGetMessage/PubPushMessage`。

- [ ] **Step 2: 保留底盘 CAN 服务端链路**
保留 `ChassisCanLinkInit()`、`ChassisCanLinkUpdateCommand()`、`ChassisCanLinkSendFeedbackIfDue()`，让其成为底盘板唯一控制输入/状态输出路径。

- [ ] **Step 3: 审核 message_center 的真实依赖**
若 `application/sysid/sysid_task.c` 仍依赖 `message_center`，则保留模块；如果后续确认完全无依赖，再独立开任务裁掉，不并入本轮。

- [ ] **Step 4: 清理注释与状态机描述**
把仍描述“单板兼容”或“底盘通过本板 pubsub 接收 cmd”的注释，统一改为“底盘板通过 CAN 接收云台板指令”。

- [ ] **Step 5: 运行残留检查**
Run: `rg -n "ONE_BOARD|RegisterSubscriber\\(\"chassis_cmd\"\\)|RegisterPublisher\\(\"chassis_feed\"\\)|SubGetMessage\\(chassis_sub|PubPushMessage\\(chassis_pub" Chassis/application/chassis`
Expected: 不再命中执行路径中的单板逻辑。

- [ ] **Step 6: 提交 Chassis 主线收敛**
Run: `git add Chassis/application/chassis/chassis.c Chassis/application/chassis/chassis.h Chassis/application/robot_def.h`
Run: `git commit -m "refactor: make CAN the only control path on chassis board"`

### Task 6: 双工程验证与文档收尾

**Files:**
- Modify: `docs/superpowers/specs/2026-04-07-dual-board-decoupling-design.md`
- Modify: `docs/superpowers/plans/2026-04-07-dual-board-decoupling.md`
- Test: `Gimbal/compile.ps1`
- Test: `Chassis/compile.ps1`

- [ ] **Step 1: 全文检查裁剪目标是否达成**
Run: `rg -n "ONE_BOARD|application/chassis|application/cmd|application/gimbal|application/shoot" Gimbal Chassis`
Expected: 只命中文档或已知保留说明，不应再命中运行时代码。

- [ ] **Step 2: 编译 Gimbal**
Run: `powershell -ExecutionPolicy Bypass -File .\\Gimbal\\compile.ps1 Debug`
Expected: Gimbal 工程独立编译通过。

- [ ] **Step 3: 编译 Chassis**
Run: `powershell -ExecutionPolicy Bypass -File .\\Chassis\\compile.ps1 Debug`
Expected: Chassis 工程独立编译通过。

- [ ] **Step 4: 检查双板协议相关静态断言与链路文件**
确认协议结构体尺寸断言全部通过，`modules/can_comm/chassis_can_link.*` 无缺失引用。

- [ ] **Step 5: 更新设计稿中的“实际完成情况”**
将与实施结果有关的差异、放弃项、保留项写回设计文档，尤其是 `message_center` 是否保留的最终结论。

- [ ] **Step 6: 提交验证与文档收尾**
Run: `git add docs/superpowers/specs/2026-04-07-dual-board-decoupling-design.md docs/superpowers/plans/2026-04-07-dual-board-decoupling.md`
Run: `git commit -m "docs: finalize dual-board decoupling plan and verification notes"`

### Execution Summary

- Completed: Task 1 through Task 6 in the existing worktree.
- Verified: both dual-board boundary regression scripts pass.
- Verified: both `Gimbal` and `Chassis` build successfully with their `Debug` CMake presets.
- Deviation from plan: no intermediate commits were created because the worktree was already dirty in overlapping files.
