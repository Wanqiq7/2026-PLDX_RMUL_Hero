# Algorithm Layer

<p align='right'>neozng1@hnu.edu.cn</p>

## 1. 目标

`modules/algorithm` 是底盘板的算法层，不直接承担外设驱动、协议组帧或 application 编排职责。

当前目录已经收口为三层结构：

```text
modules/algorithm/
├── controllers/
├── estimation/
├── utils/
└── algorithm.md
```

本文件是底盘板 algorithm 层的**顶层 API 契约说明**。

## 2. 三层职责

### controllers

负责控制律与业务侧努力量计算。

当前底盘板常用入口包括：

- `controllers/pid/pid_controller.h`
- `controllers/lqr/lqr_controller.h`
- `controllers/domain/chassis_force_control.h`
- `controllers/domain/shoot_effort_controller.h`

### estimation

负责状态估计、姿态解算和辨识。

当前底盘板常用入口包括：

- `estimation/kalman/kalman_filter.h`
- `estimation/attitude/quaternion_ekf.h`
- `estimation/identification/rls_estimator.h`

### utils

负责纯工具能力，不表达业务语义。

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

application 或其他 module 只能包含**叶子目录中的 public header**，不要再依赖历史顶层头文件。

推荐形式：

```c
#include "controllers/domain/chassis_force_control.h"
#include "estimation/identification/rls_estimator.h"
#include "utils/math/user_lib.h"
```

禁止形式：

```c
#include "controller.h"
#include "kalman_filter.h"
#include "user_lib.h"
```

也禁止：

- 直接包含 `.c` 文件
- 通过相对路径越层引用私有实现
- 在顶层目录重新堆放新算法源码

## 5. 与 application 的边界

底盘板 application 只应把 algorithm 看作“可复用算法库”，而不是跨层运行时总线。

application 侧应遵守：

- 业务编排放在 `application/*`
- 算法控制律放在 `controllers/*`
- 状态估计与辨识放在 `estimation/*`
- 通用工具放在 `utils/*`

`controllers/domain/chassis_force_control.h` 是底盘主线相关的高层算法入口，`rls_estimator.h` 则是 `power_controller` 这类模块的辨识基础能力。

## 6. 当前实仓依赖事实

当前代码中的典型依赖已经符合该方向：

- `controllers/domain/chassis_force_control.h` 依赖 `controllers/pid/pid_controller.h`
- `controllers/domain/chassis_force_control.c` 依赖 `utils/math/user_lib.h`
- `estimation/attitude/quaternion_ekf.h` 依赖 `estimation/kalman/kalman_filter.h`
- `estimation/kalman/kalman_filter.h` 依赖 `utils/math/arm_math_compat.h`

后续新增算法若不满足这个方向，应先调整分层，再合入代码。

## 7. 相关说明

共享层级制度和双板共识见：

- [2026-04-07-algorithm-layer-api-contract.md](D:/RoboMaster/HeroCode/Code/.worktrees/tau-ref-unified-local/docs/superpowers/specs/2026-04-07-algorithm-layer-api-contract.md)
