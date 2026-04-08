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

## 接口

- `ShootFrictionCalculateEffort()`
  - 摩擦轮速度参考 -> `TAU_REF`
- `ShootLoaderCalculateAngleEffort()`
  - 拨盘角度参考 -> `TAU_REF`
- `ShootLoaderCalculateSpeedEffort()`
  - 拨盘速度参考 -> `TAU_REF`
