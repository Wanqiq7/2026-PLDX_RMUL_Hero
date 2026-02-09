# Hero_Vision 参数调整指南

> 本文档总结了 Vision/Hero_Vision 中所有可调参数及其作用，特别针对英雄机器人的调参建议。

## 变更记录


| 日期       | 变更内容                           |
| ---------- | ---------------------------------- |
| 2026-02-02 | 初始化文档，分析英雄射击不稳定问题 |

---

## 问题背景

**现象**：英雄机器人在自瞄 0.4rad/s 旋转的小陀螺步兵时，云台能跟踪但弹丸难以射出（拨弹盘 1Hz 转动，未到指定角度就停止）。

**根因**：视觉端参数是为步兵设计的，不适合大惯量的英雄云台。

---

## 一、MPC Planner 参数（关键）

> 文件位置：`configs/*.yaml` 中的 `#####-----planner-----#####` 部分
> 代码位置：`tasks/auto_aim/planner/planner.cpp`

### 1.1 射击阈值


| 参数          | 步兵默认值 | 英雄建议值    | 说明               |
| ------------- | ---------- | ------------- | ------------------ |
| `fire_thresh` | 0.003      | **0.01~0.02** | 射击误差阈值 (rad) |

**原理**：

```cpp
// planner.cpp:87-92
plan.fire = std::hypot(
    traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, ...),
    traj(2, HALF_HORIZON + shoot_offset_) - pitch_solver_->work->x(0, ...))
    < fire_thresh_;
```

- 比较 520ms 后的**目标预测位置**与**云台预测位置**的合成误差
- `0.003 rad ≈ 0.17°`，对英雄大惯量云台太严格
- 建议放宽到 `0.015 rad ≈ 0.86°`

### 1.2 加速度约束


| 参数            | 步兵默认值 | 英雄建议值 | 说明                         |
| --------------- | ---------- | ---------- | ---------------------------- |
| `max_yaw_acc`   | 50         | **20~30**  | Yaw 轴最大加速度 (rad/s²)   |
| `max_pitch_acc` | 100        | **40~60**  | Pitch 轴最大加速度 (rad/s²) |

**原理**：

- MPC 基于此约束规划可行轨迹
- 英雄云台惯量大，实际加速度约为步兵的 50%
- 设置过高会导致 MPC 规划出云台无法执行的轨迹

### 1.3 MPC 权重矩阵


| 参数      | 步兵默认值 | 英雄建议值     | 说明                  |
| --------- | ---------- | -------------- | --------------------- |
| `Q_yaw`   | [9e6, 0]   | **[9e6, 1e4]** | 状态权重 [位置, 速度] |
| `R_yaw`   | [1]        | **[10]**       | 控制输入权重          |
| `Q_pitch` | [9e6, 0]   | **[9e6, 1e4]** | Pitch 状态权重        |
| `R_pitch` | [1]        | **[10]**       | Pitch 控制权重        |

**原理**：

- `Q` 矩阵：状态误差惩罚，[位置权重, 速度权重]
- `R` 矩阵：控制输入惩罚
- 增加速度权重 → 更平滑的跟踪
- 增加 R 权重 → 减少激进控制，避免云台抖动

---

## 二、Aimer 参数（小陀螺处理）

> 文件位置：`configs/*.yaml` 中的 `#####-----aimer参数-----#####` 部分
> 代码位置：`tasks/auto_aim/aimer.cpp`


| 参数                    | 默认值 | 调整建议     | 说明                                |
| ----------------------- | ------ | ------------ | ----------------------------------- |
| `yaw_offset`            | -2     | 根据实测调整 | Yaw 静态偏移补偿 (degree)           |
| `pitch_offset`          | 0      | 根据实测调整 | Pitch 静态偏移补偿 (degree)         |
| `comming_angle`         | 60     | 50~70        | 装甲板"正在出现"的角度阈值 (degree) |
| `leaving_angle`         | 20     | 15~25        | 装甲板"正在离开"的角度阈值 (degree) |
| `decision_speed`        | 8      | **5~6**      | 高/低速小陀螺判断阈值 (rad/s)       |
| `high_speed_delay_time` | 0.015  | 0.02~0.03    | 高速小陀螺延迟补偿 (s)              |
| `low_speed_delay_time`  | 0.015  | 0.01~0.02    | 低速小陀螺延迟补偿 (s)              |

**小陀螺处理逻辑** (`aimer.cpp:choose_aim_point()`):

```cpp
if (std::abs(target.v_yaw) > decision_speed_) {
    // 高速小陀螺：选择"正在出现"的装甲板
    // 使用 high_speed_delay_time 补偿
} else {
    // 低速小陀螺：选择"正在出现"的装甲板
    // 使用 low_speed_delay_time 补偿
}
```

**英雄调参建议**：

- `decision_speed: 5~6`：0.4rad/s 的目标属于低速，确保正确分类
- 延迟时间需要根据英雄云台响应特性实测调整

---

## 三、Shooter 参数（传统射击决策）

> 注意：使用 `standard_mpc.cpp` 时，Shooter 不生效，射击由 Planner 决定
> 文件位置：`configs/*.yaml` 中的 `#####-----shooter参数-----#####` 部分


| 参数               | 默认值     | 说明                    |
| ------------------ | ---------- | ----------------------- |
| `first_tolerance`  | 3~5        | 近距离射击容差 (degree) |
| `second_tolerance` | 2          | 远距离射击容差 (degree) |
| `judge_distance`   | 2~3        | 距离判断阈值 (m)        |
| `auto_fire`        | true/false | 是否由自瞄控制射击      |

**注意**：这些参数仅在使用 `standard.cpp`（非 MPC 版本）时生效。

---

## 四、Tracker 参数（目标跟踪）

> 文件位置：`configs/*.yaml` 中的 `#####-----tracker参数-----#####` 部分
> 代码位置：`tasks/auto_aim/tracker.cpp`


| 参数                          | 默认值 | 调整建议  | 说明                       |
| ----------------------------- | ------ | --------- | -------------------------- |
| `min_detect_count`            | 5      | 3~7       | 确认跟踪所需的连续检测次数 |
| `max_temp_lost_count`         | 15~25  | **30~40** | 临时丢失后保持跟踪的帧数   |
| `outpost_max_temp_lost_count` | 75     | 50~100    | 前哨站模式的丢失容忍帧数   |

**英雄调参建议**：

- `max_temp_lost_count: 30~40`：英雄云台响应慢，需要更长的丢失容忍时间
- 避免因短暂丢失导致跟踪状态重置

---

## 五、检测器参数

### 5.1 神经网络参数


| 参数              | 默认值  | 说明                 |
| ----------------- | ------- | -------------------- |
| `yolo_name`       | yolov5  | YOLO 版本选择        |
| `device`          | GPU/CPU | 推理设备             |
| `min_confidence`  | 0.8     | 最小置信度阈值       |
| `use_traditional` | true    | 是否启用传统视觉辅助 |

### 5.2 传统视觉参数


| 参数                  | 默认值 | 说明                      |
| --------------------- | ------ | ------------------------- |
| `threshold`           | 150    | 二值化阈值                |
| `max_angle_error`     | 45     | 灯条角度误差容忍 (degree) |
| `min_lightbar_ratio`  | 1.5    | 灯条最小长宽比            |
| `max_lightbar_ratio`  | 20     | 灯条最大长宽比            |
| `min_lightbar_length` | 8      | 灯条最小长度 (pixel)      |
| `min_armor_ratio`     | 1      | 装甲板最小长宽比          |
| `max_armor_ratio`     | 5      | 装甲板最大长宽比          |

---

## 六、相机与标定参数


| 参数               | 说明                                     |
| ------------------ | ---------------------------------------- |
| `camera_name`      | 相机类型 (hikrobot/mindvision/usbcamera) |
| `exposure_ms`      | 曝光时间 (ms)                            |
| `gain`             | 增益                                     |
| `camera_matrix`    | 相机内参矩阵 (3x3)                       |
| `distort_coeffs`   | 畸变系数                                 |
| `R_camera2gimbal`  | 相机到云台旋转矩阵                       |
| `t_camera2gimbal`  | 相机到云台平移向量                       |
| `R_gimbal2imubody` | 云台到 IMU 旋转矩阵                      |

**注意**：标定参数需要通过标定程序获取，不要手动修改。

---

## 七、通信参数


| 参数                 | 默认值      | 说明                  |
| -------------------- | ----------- | --------------------- |
| `quaternion_canid`   | 0x100       | 接收四元数的 CAN ID   |
| `bullet_speed_canid` | 0x101       | 接收弹速的 CAN ID     |
| `send_canid`         | 0xff        | 发送控制指令的 CAN ID |
| `can_interface`      | can0        | CAN 接口名称          |
| `com_port`           | /dev/gimbal | 串口设备路径          |

---

## 八、英雄机器人推荐配置

基于问题分析，建议创建 `hero_mpc.yaml`：

```yaml
#####-----aimer参数 (英雄专用)-----#####
yaw_offset: -0.8
pitch_offset: -1
comming_angle: 60
leaving_angle: 20
decision_speed: 5          # 降低，确保 0.4rad/s 被识别为低速
high_speed_delay_time: 0.030
low_speed_delay_time: 0.015

#####-----tracker参数 (英雄专用)-----#####
min_detect_count: 5
max_temp_lost_count: 35    # 增加丢失容忍

#####-----planner (英雄专用，关键！)-----#####
fire_thresh: 0.015         # 放宽 5 倍

max_yaw_acc: 25            # 降低到英雄实际能力
Q_yaw: [9e6, 1e4]          # 增加速度权重
R_yaw: [10]                # 增加控制惩罚

max_pitch_acc: 50
Q_pitch: [9e6, 1e4]
R_pitch: [10]
```

---

## 九、调参流程建议

1. **确认使用的入口程序**

   - `standard.cpp` → 使用 Shooter 参数
   - `standard_mpc.cpp` → 使用 Planner 参数
2. **先调 MPC 参数**

   - 从 `fire_thresh: 0.02` 开始，逐步收紧
   - 观察云台跟踪是否平滑
3. **再调加速度约束**

   - 用 Ozone/日志观察实际云台加速度
   - 设置 `max_yaw_acc` 为实际值的 80%
4. **最后调权重矩阵**

   - 如果云台抖动，增加 `R` 权重
   - 如果跟踪滞后，增加 `Q` 的速度分量

---

## 十、相关代码文件索引


| 文件                                 | 职责                   |
| ------------------------------------ | ---------------------- |
| `tasks/auto_aim/planner/planner.cpp` | MPC 规划与射击决策     |
| `tasks/auto_aim/aimer.cpp`           | 瞄准点选择与小陀螺处理 |
| `tasks/auto_aim/shooter.cpp`         | 传统射击决策（非 MPC） |
| `tasks/auto_aim/tracker.cpp`         | 目标跟踪状态机         |
| `tasks/auto_aim/detector.cpp`        | 装甲板检测             |
| `tools/yaml.hpp`                     | 配置文件读取           |
