# 算法层目录重组设计

## 目标

将 `Gimbal/modules/algorithm` 与 `Chassis/modules/algorithm` 从“顶层平铺多个算法文件”的结构，重组为按算法家族和职责分层的目录结构，降低 `controller.*` 这种混合文件的认知负担，并为后续继续引入控制器/观测器留出稳定位置。

## 目标结构

### Gimbal

```text
modules/algorithm/
├── controllers/
│   ├── pid/
│   ├── lqr/
│   ├── smc/
│   ├── domain/
│   ├── reference/
│   └── models/
├── estimation/
│   ├── kalman/
│   └── attitude/
├── utils/
│   ├── checksum/
│   └── math/
└── algorithm.md
```

### Chassis

```text
modules/algorithm/
├── controllers/
│   ├── pid/
│   ├── lqr/
│   └── domain/
├── estimation/
│   ├── kalman/
│   ├── attitude/
│   └── identification/
├── utils/
│   ├── checksum/
│   └── math/
└── algorithm.md
```

## 分类原则

- `controllers/pid`：纯 PID 控制器实现与数据结构
- `controllers/lqr`：LQR 控制器实现；底盘侧为速度型 LQR
- `controllers/smc`：滑模控制器
- `controllers/domain`：业务控制算法，例如 `vision_control`、`shoot_effort_controller`、`chassis_force_control`
- `controllers/reference`：参考值仲裁与切换
- `controllers/models`：控制相关状态/门控模型
- `estimation/kalman`：通用卡尔曼滤波器
- `estimation/attitude`：姿态估计与四元数 EKF
- `estimation/identification`：辨识类算法，例如 RLS
- `utils/checksum`：CRC 工具
- `utils/math`：数学工具和兼容头

## 执行结果

- `Gimbal` 侧已拆分 `controller.*` 为 `controllers/pid/pid_controller.*` 与 `controllers/lqr/lqr_controller.*`
- `Chassis` 侧已拆分 `controller.*` 为 `controllers/pid/pid_controller.*`、`controllers/lqr/lqr_controller.*` 与 `estimation/identification/rls_estimator.*`
- 两边的 `QuaternionEKF`、`kalman_filter`、`crc8/16`、`user_lib`、`arm_math_compat` 已迁入对应子目录
- `Gimbal` 侧的 `vision_control`、`gimbal_ref_manager`、`shoot_effort_controller`、`heat_gate_model` 已迁入业务域子目录
- `Chassis` 侧的 `chassis_force_control`、`shoot_effort_controller` 已迁入业务域子目录
- 旧顶层算法源/头文件已删除
- 运行时代码 include、Makefile、CMake 排除规则已同步更新
