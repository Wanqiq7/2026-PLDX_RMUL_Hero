# Gimbal Yaw准力矩直控使用说明

## 1. 这是什么

本说明文档对应当前 `Gimbal` 工程中 Yaw 轴的新主链路。

这次改动后，`Yaw` 在以下模式下不再把“目标角度”直接交给电机内部串级 PID，而是统一走：

```text
目标角度
  -> 目标角速度前馈
  -> 角度误差生成角速度参考
  -> 角速度误差生成电流指令
  -> GM6020 OPEN_LOOP 组帧发送
```

也就是说，它本质上是一个 **准力矩直控链路**。

## 2. 什么时候会生效

当前生效模式：

- `GIMBAL_GYRO_MODE`
- `GIMBAL_AUTOAIM_MODE`

当前不生效模式：

- `GIMBAL_ZERO_FORCE`
- `GIMBAL_LQR_MODE`
- `GIMBAL_SYS_ID_CHIRP`

说明：

- `GIMBAL_GYRO_MODE` 下，Yaw 会根据 `gimbal_cmd_send.yaw` 进入电机层 `CONTROLLER_FC` 力控链路。
- `GIMBAL_AUTOAIM_MODE` 下：
  - 如果 `vision_data.vision_takeover = 1`，则直接使用视觉给出的 `vision_data.yaw_current_cmd`
  - 如果 `vision_data.vision_takeover = 0`，则回退到与手动模式相同的 Yaw FC（Force Control）链路

## 3. 上层怎么用

### 3.1 手动云台控制

上层继续像以前一样给角度目标：

```c
gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
gimbal_cmd_send.yaw = target_yaw_rad;
gimbal_cmd_send.pitch = target_pitch_rad;
```

不需要额外给 Yaw 电流。

### 3.2 自瞄模式

如果视觉侧已经算好了 Yaw 电流指令：

```c
gimbal_cmd_send.gimbal_mode = GIMBAL_AUTOAIM_MODE;
// vision 模块发布：
// vision_data.vision_takeover = 1;
// vision_data.yaw_current_cmd = yaw_current_cmd;
// vision_data.pitch_ref_limited = pitch_ref_rad;
```

如果视觉侧没有直接给 Yaw 电流，只给目标角：

```c
gimbal_cmd_send.gimbal_mode = GIMBAL_AUTOAIM_MODE;
gimbal_cmd_send.yaw = target_yaw_rad;
```

此时会自动回退到 Yaw FC（Force Control）。

### 3.3 Pitch 轴怎么配合

Pitch 轴本次仍走达妙 MIT，但 manual/vision 两套 profile 已经下沉到 `DMmotor`：

```c
DMMotorSelectMITProfile(pitch_motor, DM_MIT_PROFILE_MANUAL);
DMMotorSetMITTargetByProfile(pitch_motor, pitch_ref_rad);
```

视觉接管时改为选择 `DM_MIT_PROFILE_VISION`，然后仍然只下发目标角度。

## 4. 关键代码位置

主文件：

- `Gimbal/application/gimbal/gimbal.c`

- `Gimbal/application/gimbal/gimbal.c`

关键函数：

- `FCReset()`
- `FCCalculate()`
- `DJIMotorSetRawRef()`

关键切换点：

- `GIMBAL_GYRO_MODE` 下切换到 `CONTROLLER_FC`
- `GIMBAL_AUTOAIM_MODE` 下视觉直流电流接管
- `GIMBAL_AUTOAIM_MODE` 下回退到 `CONTROLLER_FC`

## 5. 可直接联调的参数

以下变量都定义在 `gimbal.c` 顶部，适合在 Ozone 中直接调整：

```c
volatile float yaw_angle_loop_kp;
volatile float yaw_angle_loop_ki;
volatile float yaw_rate_loop_kp;
volatile float yaw_rate_loop_ki;
volatile float yaw_rate_ref_max;
volatile float yaw_current_cmd_max;
volatile float yaw_target_rate_ff_gain;
volatile float yaw_target_rate_lpf_alpha;
volatile float yaw_angle_err_integral_limit;
volatile float yaw_rate_err_integral_limit;
volatile float yaw_current_cmd;
volatile float yaw_target_rate_est;
```

参数含义：

- `yaw_angle_loop_kp`
  - 角度误差转角速度参考的比例项
- `yaw_angle_loop_ki`
  - 角度误差积分项
- `yaw_rate_loop_kp`
  - 角速度误差转电流指令的比例项
- `yaw_rate_loop_ki`
  - 角速度误差积分项
- `yaw_rate_ref_max`
  - 角速度参考限幅
- `yaw_current_cmd_max`
  - Yaw 输出电流限幅，单位是 GM6020 CAN 原始量
- `yaw_target_rate_ff_gain`
  - 目标角速度前馈系数
- `yaw_target_rate_lpf_alpha`
  - 目标角速度估计低通系数

## 6. 第一轮联调建议

建议按下面顺序来，不要一上来就把所有参数一起拉大。

### 第一步：确认方向

使用：

```c
gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
```

然后给一个很小的 Yaw 阶跃，比如 `2°~5°`。

观察：

- 云台转动方向是否正确
- 到位后是否会持续发散

如果方向不对，先检查：

- `MOTOR_DIRECTION_REVERSE`
- IMU 的 `YawTotalAngle_rad`
- 视觉/遥控坐标系定义

### 第二步：先调外环，再调内环

 建议顺序：

1. 先把 `yaw_angle_loop_ki = 0`
2. 先把 `yaw_rate_loop_ki = 0`
3. 调 `yaw_angle_loop_kp`
4. 调 `yaw_rate_loop_kp`
5. 最后少量加入 `pos_ki` / `rate_ki`

### 第三步：最后再开前馈

一开始建议：

```c
yaw_target_rate_ff_gain = 0.0f;
```

等基础稳定以后，再慢慢增加。

如果前馈开太大，常见现象是：

- 跟手变快，但超调增大
- 停下时有“甩一下”

## 7. Ozone 建议观察量

建议至少盯下面这些量：

- `gimba_IMU_data->YawTotalAngle_rad`
- `gimba_IMU_data->Gyro[2]`
- `yaw_target_rate_est`
- `yaw_current_feedforward`
- `yaw_current_cmd`
- `yaw_angle_err_integral`
- `yaw_rate_err_integral`

重点判断：

- `yaw_current_cmd` 是否频繁顶到限幅
- 积分项是否明显累积后不回落
- 小角度目标下是否有明显高频抖动

## 8. 现在的限制

这条链路当前已经能用，但还不是最终形态。

当前已完成：

- 手动模式与自瞄回退模式统一到同一套 Yaw 控制语义
- 视觉直给电流路径继续保留
- `Yaw` 进入直控模式时会切换到 `OPEN_LOOP`

当前还没做：

- Yaw 摩擦补偿
- Yaw 扰动观测或外力估计
- Pitch 重力补偿闭环化
- 手动 / 自瞄 / LQR 三套参数自动分组

## 9. 你应该怎么理解这条链路

不要把它理解成“普通位置环换了几个参数”。

更准确的理解是：

> 现在的 Yaw 控制目标，已经从“直接追目标角度”变成了“根据角度和角速度状态，生成一个更物理化的电流/力矩指令”。

这就是为什么它更接近“准力控”，也更适合后面继续接：

- 摩擦补偿
- 扰动抑制
- 更平滑的自瞄回退
- Pitch/Yaw 双轴统一状态反馈

## 10. 最简使用结论

如果你现在只想把它跑起来，按下面做就够了：

1. 保持 `Pitch` 不动
2. 把云台切到 `GIMBAL_GYRO_MODE`
3. 在 Ozone 里重点调：
   - `yaw_angle_loop_kp`
   - `yaw_rate_loop_kp`
   - `yaw_current_cmd_max`
4. 稳定后再加：
   - `yaw_target_rate_ff_gain`
   - `yaw_angle_loop_ki`
   - `yaw_rate_loop_ki`

如果你是调自瞄：

1. 优先先让 `vision_data.vision_takeover = 0`，走回退链路
2. 回退链路稳定后，再切 `vision_data.vision_takeover = 1`
3. 最后再比较“视觉直接给电流”和“视觉只给目标角”哪种手感更好
