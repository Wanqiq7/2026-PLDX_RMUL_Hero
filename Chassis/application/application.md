# Chassis Application

<p align='right'>neozng1@hnu.edu.cn</p>

## 1. 定位

当前工作树固定为**严格双板专用**的底盘板工程，`robot_def.h` 中只保留 `#define CHASSIS_BOARD`。

底盘板的职责不是“运行整车所有应用”，而是承担以下本板执行链：

- `chassis`：底盘运动主线与执行输出
- `sysid`：底盘侧系统辨识/诊断链路
- `power / super_cap / referee / UI`：底盘功率管理、裁判信息处理与回传
- `can_comm`：与云台板交换控制摘要和状态反馈

以下应用**不再属于底盘板运行时模型**：

- `robot_cmd`
- `gimbal`
- `shoot`
- `vision`

这些职责已经完全拆分到对侧云台板工程。

## 2. 实际运行入口

当前运行入口以 [robot.c](D:/RoboMaster/HeroCode/Code/.worktrees/tau-ref-unified-local/Chassis/application/robot.c) 为准：

```c
void RobotInit(void) {
  BSPInit();
  ChassisInit();
  OSTaskInit();
}

void RobotTask(void) {
  ChassisTask();
}
```

这意味着底盘板应用层对外只有一条主运行入口：

```text
RobotInit
-> ChassisInit
-> OSTaskInit

RobotTask
-> ChassisTask
```

`sysid / power / referee / UI / can link` 属于 `ChassisInit()` 与 `ChassisTask()` 内部管理的底盘侧职责，而不是独立的跨板应用编排入口。

## 3. 双板运行模型

底盘板运行时只接受来自云台板的 CAN 控制摘要，并回传底盘状态。

### 输入

- 云台板下发的底盘控制摘要
- 裁判系统状态
- 超级电容状态
- 本板底盘电机和 IMU 反馈

### 输出

- 四轮执行输出
- 功率限制后的底盘状态
- 裁判/UI 相关回传
- 底盘状态反馈给云台板

运行时抽象应理解为：

```text
Gimbal board
-> CAN control summary
-> Chassis board
-> chassis mainline / power controller / referee-ui feedback
-> CAN state feedback
-> Gimbal board
```

## 4. 板内边界

底盘板内部仍然允许使用 `message_center` 组织本板模块，但**不能再把它理解为跨板总线**。

明确约束如下：

- 板间通信只能通过 `CANComm` 链路完成。
- 底盘板不能再假设本地存在 `robot_cmd` 发布底盘控制话题。
- 底盘板不能保留“单板整车 pub/sub”叙事。
- 文档和代码都应把“来自云台板的 CAN 摘要”视为唯一常规上游控制源。

## 5. 初始化与任务建议

在 `main()` 中：

1. 包含 `robot.h`
2. 在启动 RTOS 前调用 `RobotInit()`
3. 在任务循环中调用 `RobotTask()`

基础任务频率仍按原工程约束维护：

- `INStask`：1 kHz
- `motortask`：200 Hz 到 1000 Hz
- `monitortask`：100 Hz
- `robottask`：建议不低于 150 Hz

这些频率描述只说明底盘板本地调度要求，不意味着底盘板重新承担云台/发射/视觉应用。

## 6. 维护规则

后续若继续优化底盘板 application 层，请遵守以下规则：

- 不要重新引入历史单板运行宏或“单板整车”叙事。
- 不要把云台板应用职责写回底盘板 application 文档。
- 不要把跨板 CAN 摘要误写成 `message_center` 的本地应用消息。
- 若新增底盘侧辅助能力，应明确它是 `ChassisTask()` 的内部子职责，还是新的底盘板专用应用入口。

## 7. 相关文档

- [chassis.md](D:/RoboMaster/HeroCode/Code/.worktrees/tau-ref-unified-local/Chassis/application/chassis/chassis.md)
- [task_plan.md](D:/RoboMaster/HeroCode/Code/.worktrees/tau-ref-unified-local/task_plan.md)
- [2026-04-07-mainline-roadmap-and-power-controller-timing.md](D:/RoboMaster/HeroCode/Code/.worktrees/tau-ref-unified-local/docs/superpowers/specs/2026-04-07-mainline-roadmap-and-power-controller-timing.md)
