# shoot
## 工作流程
初始化摩擦轮电机和拨盘电机：当前版本为**三摩擦轮（正三角分布）** + 拨盘（M3508）方案。订阅 `shoot_cmd` 话题（`robot_cmd` 发布）并发布 `shoot_feed` 话题（`robot_cmd` 订阅）。

1. 从shoot_cmd获取消息
2. 根据工作模式确定是否急停
3. 如果之前是单发模式或3发模式并且冷却时间没到，直接结束本次任务，等待下一次进入
4. 如果已经冷却完成，根据发来的拨盘模式，设定拨盘电机的闭环类型和参考值
5. 根据发来的弹速数据，设定摩擦轮的参考值(未做)
6. 根据发来的弹舱数据进行开合
7. 设定反馈数据，推送到shoot_feed话题

## 当前动力主线

`shoot` 当前常规动力执行器链路已经统一为：

```text
摩擦轮目标速度 / 拨盘目标角度 / 拨盘目标速度
-> shoot_effort_controller
-> DJIMotorCalculateEffort()
-> Controller_Effort_Output(TAU_REF)
-> DJIMotorSetEffort()
```

其中：

1. 摩擦轮常规路径走 `ShootFrictionCalculateEffort()`
2. 拨盘单发/三发/卡弹恢复走 `ShootLoaderCalculateAngleEffort()`
3. 拨盘连发/反转走 `ShootLoaderCalculateSpeedEffort()`

## 接口边界

当前 `shoot` 常规主线中不再使用：

- `DJIMotorSetRef()`

保留给特殊用途的接口只有：

- `DJIMotorStop()`：安全停机

因此这次迁移的重点是“统一执行语义为 TAU_REF”，而不是改变拨盘状态机本身。
