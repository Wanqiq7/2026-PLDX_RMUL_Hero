# shoot_effort_controller

## 作用

`shoot_effort_controller` 用于把 `shoot` 模块中仍以“速度参考/角度参考”表达的业务目标，
统一收敛为：

```text
参考值
-> DJIMotorCalculateEffort()
-> Controller_Effort_Output(TAU_REF)
-> DJIMotorSetEffort()
```

它只负责：

1. 选择 `ANGLE_LOOP` 或 `SPEED_LOOP`
2. 调用 `DJIMotorCalculateEffort()`
3. 输出统一的 `Controller_Effort_Output`

它不负责：

1. 状态机
2. 协议组帧
3. 电机使能/停机策略

因此它的边界应理解为：

```text
业务参考
-> shoot_effort_controller
-> Controller_Effort_Output(TAU_REF)
-> SetEffort
-> adapter / protocol
```

它属于 “Ref Manager / Effort Controller / Adapter” 三层中的 **Effort Controller**，
不应回退去承担业务仲裁，也不应越层去拼接协议命令。

## 接口

- `ShootFrictionCalculateEffort()`
  - 摩擦轮速度参考 -> `TAU_REF`
- `ShootLoaderCalculateAngleEffort()`
  - 拨盘角度参考 -> `TAU_REF`
- `ShootLoaderCalculateSpeedEffort()`
  - 拨盘速度参考 -> `TAU_REF`
