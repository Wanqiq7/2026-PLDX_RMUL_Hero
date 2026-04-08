# Algorithm Layer

<p align='right'>neozng1@hnu.edu.cn</p>

## 1. 目标

`modules/algorithm` 是云台板的算法层，不直接承担协议组帧、电机驱动或 application 调度职责。

当前目录已经收口为三层结构：

```text
modules/algorithm/
├── controllers/
├── estimation/
├── utils/
└── algorithm.md
```

其中 `reference/`、`models/`、`domain/` 都归属于 `controllers/` 这一层。

本文件是云台板 algorithm 层的**顶层 API 契约说明**。

## 2. 三层职责

### controllers

负责参考仲裁、控制律、努力量计算和业务模型。

当前云台板常用入口包括：

- `controllers/pid/pid_controller.h`
- `controllers/lqr/lqr_controller.h`
- `controllers/smc/smc_controller.h`
- `controllers/reference/gimbal_ref_manager.h`
- `controllers/domain/vision_control.h`
- `controllers/domain/shoot_effort_controller.h`
- `controllers/models/heat_gate_model.h`

当前云台板主线应继续理解为：

```text
Ref Manager / Arbiter
-> Effort Controller / CalculateEffort
-> Adapter / Protocol
```

algorithm 层承担前两层，不承担 `Adapter / Protocol`。

### estimation

负责姿态与状态估计。

当前常用入口包括：

- `estimation/kalman/kalman_filter.h`
- `estimation/attitude/quaternion_ekf.h`

### utils

负责数学工具与校验工具，不表达业务语义。

当前常用入口包括：

- `utils/math/user_lib.h`
- `utils/math/arm_math_compat.h`
- `utils/checksum/crc8.h`
- `utils/checksum/crc16.h`

## 3. 依赖方向

允许的依赖方向只有：

```text
controllers
-> estimation
-> utils
```

更精确地说：

- `utils` 只能依赖标准库、CMSIS/ARM 数学兼容层和同层 `utils/*`
- `estimation` 可以依赖 `utils/*`，也可以依赖同层 `estimation/*`
- `controllers` 可以依赖 `estimation/*`、`utils/*` 和同层 `controllers/*`

明确禁止：

- `utils -> estimation`
- `utils -> controllers`
- `estimation -> controllers`

## 4. 对外入口规则

application 或其他 module 只能包含**叶子目录中的 public header**。

推荐形式：

```c
#include "controllers/reference/gimbal_ref_manager.h"
#include "controllers/domain/vision_control.h"
#include "estimation/attitude/quaternion_ekf.h"
#include "utils/math/user_lib.h"
```

禁止形式：

```c
#include "gimbal_ref_manager.h"
#include "vision_control.h"
#include "kalman_filter.h"
#include "user_lib.h"
```

也禁止：

- 直接包含 `.c` 文件
- 在顶层目录重新堆放算法源码
- 让 application 直接跨过 controller/helper 层拼装协议输出

## 5. 与 application 的边界

云台板 application 侧应把 algorithm 看作“参考仲裁 + 努力量计算 + 状态估计”的能力层。

板内职责分工应保持：

- `robot_cmd`：输入汇总、模式仲裁、跨板摘要下发
- `gimbal_ref_manager`：参考仲裁（Ref Manager / Arbiter）
- `CalculateEffort` / effort helper：努力量计算（Effort Controller / CalculateEffort）
- `tau_ref_adapter` 与 motor protocol：协议与执行层（Adapter / Protocol）

也就是说，algorithm 层只承担前两层，不承担后两层中的协议组帧。

## 6. 当前实仓依赖事实

当前代码中的典型依赖已经符合该方向：

- `controllers/reference/gimbal_ref_manager.c` 依赖 `utils/math/user_lib.h`
- `controllers/domain/vision_control.c` 依赖 `utils/math/user_lib.h`
- `estimation/attitude/quaternion_ekf.h` 依赖 `estimation/kalman/kalman_filter.h`
- `estimation/kalman/kalman_filter.h` 依赖 `utils/math/arm_math_compat.h`

后续新增云台算法若不满足这个方向，应先调整分层，再合入代码。

## 7. 相关说明

共享层级制度和双板共识见：

- [2026-04-07-algorithm-layer-api-contract.md](D:/RoboMaster/HeroCode/Code/.worktrees/tau-ref-unified-local/docs/superpowers/specs/2026-04-07-algorithm-layer-api-contract.md)
