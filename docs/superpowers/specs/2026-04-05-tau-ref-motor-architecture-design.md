# 动力执行器扭矩主线模块层总方案

## 目标

在不破坏现有 `APP / Module / BSP` 三层结构的前提下，统一 `Gimbal` 与 `Chassis` 的动力执行器主线：

```text
控制目标
-> 控制器
-> Controller_Effort_Output
-> 执行器适配层
-> Actuator_Command
-> 电机协议发送
```

其中：

- 最高统一接口是 `Controller_Effort_Output`
- 对 `DJI` 主线，控制语义收敛到 `CONTROLLER_OUTPUT_TAU_REF`
- 对 `DM MIT`，保留 `Actuator_Command` 作为执行器出口

## 统一接口

### 1. 控制层输出

`modules/motor/motor_def.h`

- `Controller_Output_Semantic_e`
- `Controller_Effort_Output_s`
- `Motor_Physical_Param_s`
- `Actuator_Command_s`

用途：

- 控制器只表达“我想输出什么物理努力量”
- 执行器适配层负责把努力量转成协议所需命令

### 2. 驱动层私有接口

`DJIMotorSetEffort(...)`

说明：

- 这是 `DJI` 私有最终入口，不是模块层公共语义接口
- 它接收 `Controller_Effort_Output_s`
- 内部通过 `dji_motor_adapter` 转成 raw current CAN 命令

## 当前实施状态

### 已完成

1. `Chassis` 力控主线抬升为扭矩语义
   - `wheel_current` -> `wheel_tau_ref`
   - 摩擦补偿与速度反馈增益迁到扭矩域

2. `Chassis` 保留旧功率控制器
   - 新增 `LegacyPowerBridge`
   - 仅在功率控制器前后做 `tau_ref <-> raw current cmd` 兼容换算

3. `DJI motor` 共享模块补齐 `DJIMotorSetEffort(...)`
   - `Chassis` 已接入
   - `Gimbal` 已补齐入口

4. `Gimbal Yaw` 三控制分支统一为扭矩语义
   - `PID` 最终输出改为 `TAU_REF`
   - `LQR` 最终输出改为 `TAU_REF`
   - `SMC` 最终输出改为 `TAU_REF`

5. `Pitch DM MIT` 主线收敛为 torque-only
   - `GimbalTask()` 常规路径改为 `DMMotorSetRef(pitch_ref_rad, torque_ff)`
   - `DMmotor` 模块内执行 `angle PID -> speed PID -> tau_ref`
   - MIT 常规下发时默认屏蔽 angle/velocity/kp/kd，只保留 torque 通道

### 暂未完成

1. `Gimbal` 上层模式链路还未完全显式使用 `SetEffort`
2. 旧 `SetRef` 语义仍存在，暂作为兼容接口保留
3. 文档和说明文件尚未同步到新主线表述

## Chassis 冲突处理结论

`Chassis` 与新扭矩主线确实存在冲突，但已采用兼容方案化解：

```text
wheel_tau_ref
-> LegacyPowerBridgeTauToRawCurrentCmd
-> 旧功率控制器
-> LegacyPowerBridgeRawCurrentCmdToTau
-> DJIMotorSetEffort
```

因此：

- 不需要重写功率控制器
- 仍可让底盘主线前后保持扭矩语义
- current/raw 仅在兼容桥内部存在

## 后续原则

1. 所有新增动力执行器控制链路，优先输出 `TAU_REF`
2. raw current / current_A 不再作为主线控制语义
3. bypass 路径必须显式命名，不能伪装成主线接口
4. `Pitch DM MIT` 不强行压成单一扭矩接口，但继续服从统一模块边界
