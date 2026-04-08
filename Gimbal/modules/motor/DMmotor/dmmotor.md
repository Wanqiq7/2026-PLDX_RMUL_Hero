# dmmotor（达妙电机模块）

本模块封装达妙电机的 CAN 协议发送、反馈解析，以及若干控制入口（PVT/MIT/模块内 PID 路径）。

## 方向（正反转）约定

为避免在应用层到处写正负号，模块遵循与 `DJImotor` 类似的“方向映射”思路：

- `motor_reverse_flag`（电机正反转标志）
  - 含义：将**上层的“逻辑目标”**映射到**电机物理正方向**。
  - 生效位置：
    - PVT：在下发 PVT 位置 `p_des` 前对位置取反。
    - MIT（含 `DMMotorSetMITTargets` / `DMMotorSetMITVelocity` / 模块内 torque-only PID 路径）：在打包发送前对 **位置/速度/力矩**统一做方向映射。

- `feedback_reverse_flag`（反馈量正反标志）
  - 含义：当反馈数据（如 IMU 的 Pitch/Gyro 或外部传感器）方向与控制坐标系相反时，对反馈做方向纠正。
  - 生效位置：
    - 仅在“模块内 torque-only 串级路径”（`DMMotorSetRef` 对应的 `use_cascade_pid_path`）中，对角度/速度反馈取反后再进入 PID 计算。

> 建议：优先用 `motor_reverse_flag` 把“遥控器/上层期望方向”与“电机转动方向”对齐；若仅反馈方向相反，再使用 `feedback_reverse_flag`。

## 当前主线

`DMmotor` 当前推荐的常规动力执行器主线是：

```text
DMMotorCalculateTorqueEffort()
-> Controller_Effort_Output(TAU_REF)
-> DMMotorSetEffort()
-> torque-only MIT
```

`DMMotorSetRef()` 仍保留为兼容接口，但 `Gimbal` 常规主线已经优先改为 `CalculateEffort -> SetEffort`。

## 兼容与扩展接口边界

以下接口仍然保留，但都不属于当前常规动力主线：

- `DMMotorSetRef()`
  - 历史模块内串级路径兼容入口
- `DMMotorSetMITTargets()`
  - MIT full-command 兼容入口
- `DMMotorSetMITVelocity()`
  - velocity-only 特殊测试入口
- `DMMotorSetMITTargetByProfile()`
  - 历史 profile 兼容入口
- `DMMotorSetPVT()`
  - PVT 扩展能力入口
- `DMMotorSetMode()`
  - 仅切换底层驱动模式，不单独激活任何命令路径

当前推荐原则是：

```text
常规控制 -> CalculateTorqueEffort -> SetEffort
兼容/扩展 -> 显式调用 MIT full / velocity-only / PVT
```

也就是说，若要进入 PVT 扩展路径，应显式调用 `DMMotorSetPVT()`；
`DMMotorSetMode(DM_MODE_PVT)` 只负责切换电机驱动模式，不应被视为“已经进入 PVT 控制”。

## 常见配置建议

- 云台 Pitch 轴（PVT 控制）方向相反：将 `Motor_Init_Config_s.controller_setting_init_config.motor_reverse_flag` 设为 `MOTOR_DIRECTION_REVERSE`。
- 若同时使用 IMU 做闭环（模块内 PID 路径）：在确认 IMU 坐标系后，必要时再配置 `feedback_reverse_flag`。

