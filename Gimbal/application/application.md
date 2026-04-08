# Gimbal Application

<p align='right'>neozng1@hnu.edu.cn</p>

## 1. 定位

当前工作树固定为**严格双板专用**的云台板工程，`robot_def.h` 中只保留 `#define GIMBAL_BOARD`。

云台板负责整车上游控制与本板执行链，当前常规运行职责为：

- `vision`：视觉输入与目标信息接入
- `robot_cmd`：控制汇总、模式仲裁、底盘摘要下发
- `gimbal`：云台本体主线
- `shoot`：发射机构主线

底盘执行层**不再属于云台板本地 application**。云台板与底盘板之间的常规耦合面只剩 CAN 数据交换。

## 2. 实际运行入口

当前运行入口以 [robot.c](D:/RoboMaster/HeroCode/Code/.worktrees/tau-ref-unified-local/Gimbal/application/robot.c) 为准：

```c
void RobotInit(void) {
  BSPInit();
  VisionAppInit();
  RobotCMDInit();
  GimbalInit();
  ShootInit();
  OSTaskInit();
}

void RobotTask(void) {
  VisionAppTask();
  RobotCMDTask();
  GimbalTask();
  ShootTask();
}
```

当前云台板的主线顺序应理解为：

```text
VisionAppTask
-> RobotCMDTask
-> GimbalTask
-> ShootTask
```

其中：

- `vision` 先更新上游视觉输入
- `robot_cmd` 汇总遥控/键鼠/视觉信息，并向底盘板下发控制摘要
- `gimbal` 与 `shoot` 分别完成本板执行输出

## 3. 双板运行模型

云台板是双板系统的上游控制中心，但不是“整车所有执行都在本板完成”。

常规链路应理解为：

```text
vision / remote / keyboard / operator intent
-> robot_cmd
-> gimbal mainline + shoot mainline
-> chassis control summary over CAN
-> chassis board executes wheel / power / referee-ui path
-> chassis state feedback over CAN
-> robot_cmd / gimbal side consumers
```

这意味着云台板 application 文档必须同时表达两件事：

- 云台板是控制源头
- 底盘板是底盘执行落点

不能再把 `chassis` 视作云台板本地应用，也不能保留旧的单板整车模型。

## 4. 板内边界

云台板内部仍然使用 `message_center` 连接 `vision / robot_cmd / gimbal / shoot`，但这个机制只适用于**本板内部**。

明确约束如下：

- `robot_cmd` 负责本板控制汇总与底盘摘要下发。
- `gimbal`、`shoot` 只消费本板控制输入，不负责直接驱动底盘执行。
- 跨板信息只能通过 `CANComm` 链路交换。
- `sysid`、兼容/诊断路径若存在，也不是云台板常规主线的一部分。

## 5. 当前主线抽象

结合现有架构收口，云台板常规主线应按以下心智模型理解：

```text
Ref Manager / Arbiter
-> Effort Controller / CalculateEffort
-> Adapter / Protocol
```

这里的第三层也就是 `adapter / protocol` 层，不属于 application 仲裁或 algorithm 控制器本体。

应用层侧重点如下：

- `robot_cmd`：决定参考输入和模式
- `gimbal`：消费参考输入并走 `CalculateEffort -> SetEffort`
- `shoot`：消费发射参考并走 `shoot_effort_controller -> SetEffort`
- `vision`：只提供输入，不直接越过主线下发执行量

## 6. 初始化与任务建议

在 `main()` 中：

1. 包含 `robot.h`
2. 在启动 RTOS 前调用 `RobotInit()`
3. 在任务循环中调用 `RobotTask()`

基础任务频率仍按原工程约束维护：

- `INStask`：1 kHz
- `motortask`：200 Hz 到 1000 Hz
- `monitortask`：100 Hz
- `robottask`：建议不低于 150 Hz

任务频率说明只描述本板调度，不意味着底盘应用重新回到云台板本地。

## 7. 维护规则

后续若继续优化云台板 application 层，请遵守以下规则：

- 不要重新引入历史单板运行宏或“本板同时运行底盘执行”的叙事。
- 不要在 application 文档里把 `chassis` 写成本板本地运行应用。
- 不要把跨板 CAN 摘要误写成 `message_center` 的本地应用话题。
- 若新增云台板仲裁逻辑，应明确它属于 `robot_cmd`、`gimbal_ref_manager`，还是新的本板专用控制层。

## 8. 相关文档

- [robot_cmd.md](D:/RoboMaster/HeroCode/Code/.worktrees/tau-ref-unified-local/Gimbal/application/cmd/robot_cmd.md)
- [gimbal.md](D:/RoboMaster/HeroCode/Code/.worktrees/tau-ref-unified-local/Gimbal/application/gimbal/gimbal.md)
- [2026-04-07-mainline-roadmap-and-power-controller-timing.md](D:/RoboMaster/HeroCode/Code/.worktrees/tau-ref-unified-local/docs/superpowers/specs/2026-04-07-mainline-roadmap-and-power-controller-timing.md)
