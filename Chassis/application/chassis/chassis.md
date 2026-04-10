# chassis


@Todo 使用条件编译，选择麦轮(全向轮),舵轮,平衡底盘
## 工作流程

首先进行初始化，`ChasissInit()`会被`RobotInit()`调用，进行裁判系统、底盘电机、功率控制和力控模块的初始化。如果为双板模式，则还会初始化 IMU，并通过 `modules/can_comm/chassis_can_link` 完成 CANComm 桥接初始化。

操作系统启动后，工作顺序为：

1. 从 cmd 模块获取数据；双板模式下通过 `ChassisCanLinkUpdateCommand()` 读取 CAN 桥接命令。
2. 判断当前控制模式；若为 `CHASSIS_ZERO_FORCE`，立即停机并重置跟随 PID 与力控模块状态。
3. 根据控制模式决定 `wz` 目标。
4. 将控制量根据 `offset_angle` 投影到底盘坐标系下。
5. 调用 `ChassisForceControlSetCommand()` 和 `ChassisForceControlStep()`，由独立算法模块完成：
   - 麦轮正运动学
   - 速度估算
   - 速度闭环到力/扭矩
   - 力分配
   - 力到电流转换与摩擦补偿
6. 获取裁判系统和超级电容数据，并通过 `power_controller(native tau domain)` 对轮端输出做功率限制。
7. 设置底盘反馈数据，包括热量、功率上限和弹速限制等。
8. 更新 UI 状态和遥测。
9. 收尾反馈数据并更新 UI/遥测；双板模式下通过 `ChassisCanLinkSendFeedbackIfDue()` 分周期发送。

在当前版本中，`ChassisTask()` 内部进一步按“文件内分层”的方式组织：

1. `BuildChassisSafetyInput()` 负责采样本周期安全输入（链路、裁判、轮在线位图）。
2. `StepChassisSafetyGate()` 负责推进底盘本地安全 gate 状态机。
3. `ShouldHoldChassisControl()` 统一决定当前周期是否必须禁止底盘进入控制主线。
4. `ApplyChassisSafetyFeedback()` 仅把安全 gate 结果映射到双板摘要反馈。
5. `LogChassisSafetyTransition()` 只在底盘本地输出详细状态切换日志。

这样可以保持 `ChassisTask()` 仍然是任务编排入口，而不是把全部安全状态机细节直接堆在主循环里。

## 当前分层

- `application/chassis/chassis.c`
  只保留底盘应用编排、模式选择、功率控制调用、UI/反馈收尾。
- `modules/can_comm/chassis_can_link.c`
  负责双板 CANComm 命令接收与反馈发包。
- `modules/algorithm/chassis_force_control.c`
  负责底盘力控控制核，包括速度估算、力分配、轮端电流计算。
- `modules/power_controller/*`
  负责功率限制、RLS 参数辨识和能量环，并直接消费/输出轮端 `tau_ref`。

## 功率控制主链路

当前 `Chassis` 常规主线为：

```text
wheel_tau_ref
-> power_controller (native tau domain)
-> limited wheel_tau_ref
-> DJIMotorSetEffort()
```

这里的要求是：

1. 主线语义始终保持在 `TAU_REF`
2. 应用层不再承载旧 current/raw 域换算
3. `power_controller` 的受限结果直接回到 `wheel_tau_ref`
4. `DJIMotorSetEffort()` 仍然是底盘执行出口

### 后续支持平衡底盘

新增一个app balance_chassis

## 安全降级约束（双板）

1. 底盘板每周期通过 `ChassisCanLinkUpdateCommand()` 读取控制命令后，必须检查链路在线状态。
2. 一旦链路离线，立即执行“失联即零力”：
- `chassis_mode = CHASSIS_ZERO_FORCE`
- `vx/vy/wz` 清零
- 禁止沿用上一帧有效控制指令
3. 链路恢复后再按正常控制路径运行，不允许跳过安全检查。

## 底盘安全门控（阵亡/缺轮/恢复）

当前底盘应用增加了一个仅存在于 APP 层的本地安全 gate，用于解决“阵亡或轮电机未完全恢复时底盘缺轮运行”的问题。

### 本地判定原则

1. 阵亡判定只使用底盘板本地裁判数据：
- `current_HP == 0`
- 或 `power_management_chassis_output == 0`
2. 轮在线状态只使用本地 `DJIMotor daemon` 聚合，不修改 `DJIMotor` 模块职责。
3. 任意一个轮电机离线时，底盘保持在本地 hold 状态，不允许进入常规控制主线。
4. 所有轮重新在线后，还必须等待 `CHASSIS_ALL_WHEELS_STABLE_MS` 稳定窗口，才能重新进入 ready。

### 双板摘要策略

双板快包不再上传轮级细节，而只上传最小必要状态：

```c
uint8_t chassis_safety_status;
```

其位定义为：

- `CHASSIS_SAFETY_STATUS_DEAD`：机器人处于本地裁判判定的阵亡状态
- `CHASSIS_SAFETY_STATUS_READY`：底盘本地 gate 允许恢复控制

以下信息只保留在底盘本地日志中，不通过双板 CAN 摘要发送：

- `wheel_online_mask`
- 本地抑制原因（链路离线 / 阵亡 / 缺轮 / 等待稳定）

### 设计意图

1. 云台板只消费“阵亡 / 是否 ready”两个业务事实，不感知底盘内部轮级细节。
2. 底盘板保留完整的本地调试信息，满足联调定位需要。
3. 双板协议保持最小摘要，避免快包持续膨胀。

## 联调最小回归（建议）

1. 断开双板 CAN 链路，确认一个控制周期内四轮进入零输出。
2. 恢复链路，确认可重新受控且无持续抖动。
3. 在失联/恢复期间观察关键遥测：`comm_online`、`chassis_mode`、`vx/vy/wz`。
4. 阵亡后确认 `chassis_safety_status` 置位 `DEAD`，并且底盘维持零力。
5. 仅有部分 3508 在线时，确认 `READY` 不置位，底盘不会缺轮恢复。
6. 四轮全部在线并稳定后，确认 `READY` 才重新置位。
