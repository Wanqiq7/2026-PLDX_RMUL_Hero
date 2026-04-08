# Algorithm Layer API Contract

## 1. 文档目的

本文档用于冻结 `tau-ref-unified-local` 中 `modules/algorithm` 新目录结构的公共契约，明确：

- `controllers / estimation / utils` 三层分别负责什么
- 谁可以依赖谁
- 哪些头文件是对外入口
- 哪些旧做法不允许再回流

该契约同时适用于 `Chassis` 和 `Gimbal`，板间差异只体现在 `controllers/*` 的具体子目录内容上。

## 2. 三层结构

统一结构如下：

```text
modules/algorithm/
├── controllers/
├── estimation/
├── utils/
└── algorithm.md
```

职责划分如下：

- `controllers`：控制律、参考仲裁、业务模型、努力量计算
- `estimation`：状态估计、姿态融合、辨识
- `utils`：数学工具、校验工具、纯辅助能力

## 3. 依赖方向

唯一允许的方向是：

```text
controllers -> estimation -> utils
```

补充规则：

- `utils` 不能依赖 `estimation` 或 `controllers`
- `estimation` 不能依赖 `controllers`
- 同层内部可以按子目录继续拆分，但不得形成反向耦合

## 4. Public Header 规则

application 或其他 module 只能包含叶子目录中的 public header。

正确示例：

```c
#include "controllers/domain/chassis_force_control.h"
#include "controllers/reference/gimbal_ref_manager.h"
#include "estimation/identification/rls_estimator.h"
#include "utils/math/user_lib.h"
```

错误示例：

```c
#include "controller.h"
#include "gimbal_ref_manager.h"
#include "kalman_filter.h"
#include "user_lib.h"
```

这条规则的目标是：

- 彻底停止旧顶层平铺头文件回流
- 让 include 路径直接表达层级和职责
- 为后续依赖审计和自动检查保留明确抓手

## 5. 与双板主线的关系

### Chassis

底盘板主要消费：

- `controllers/domain/chassis_force_control.h`
- `controllers/domain/shoot_effort_controller.h`
- `estimation/identification/rls_estimator.h`

### Gimbal

云台板主要消费：

- `controllers/reference/gimbal_ref_manager.h`
- `controllers/domain/vision_control.h`
- `controllers/domain/shoot_effort_controller.h`
- `estimation/attitude/quaternion_ekf.h`

两边共同遵守的边界是：

- algorithm 只负责算法与仲裁，不负责协议组帧
- `Adapter / Protocol` 留在 motor / communication 相关模块

## 6. 禁止事项

从本契约起，以下做法视为违规：

- 在 `modules/algorithm` 顶层新增平铺 `.c/.h`
- application 直接 include 历史顶层头文件名
- `utils` 反向依赖 `controllers` 或 `estimation`
- 把协议层逻辑塞回 `controllers/*`

## 7. 实施要求

后续若新增算法能力：

1. 先判断它属于 `controllers`、`estimation` 还是 `utils`
2. 再决定具体叶子目录
3. 对外只暴露叶子目录 public header
4. 必须满足依赖方向审计

如果无法清晰归类，先补文档说明，再决定目录落点。
