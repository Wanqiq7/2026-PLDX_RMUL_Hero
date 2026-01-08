# LQR单位统一修复报告

## 🚨 问题描述

LQR控制器出现以下症状：
1. ❌ 摇杆拨大了，起步卡顿，甚至往反方向转
2. ❌ 摇杆拨小了，转一下停一下
3. ❌ 云台不硬，用手能轻易推动
4. ❌ 放手后回到目标值但不准确

**根本原因：单位不统一！**

---

## 🔍 问题根源分析

### 原始数据流

```
四元数 → atan2f → [弧度]
                    ↓
                  ×57.3
                    ↓
                  [角度] → YawTotalAngle
```

### LQR期望的单位

```
系统辨识：
  输入：电流 [A]
  输出：角速度 [rad/s]  ← 弧度制

LQR控制律：
  状态1：角度 [rad]     ← 弧度制
  状态2：角速度 [rad/s] ← 弧度制
```

### 实际给LQR的单位（修复前）

| 变量 | 期望单位 | 实际单位 | 差异 |
|------|----------|----------|------|
| **目标角度** | rad | **度** | ❌ 57.3倍！ |
| **角度反馈** | rad | **度** | ❌ 57.3倍！ |
| **速度反馈** | rad/s | rad/s | ✅ 正确 |

**误差计算：**
```c
误差 = 目标[度] - 反馈[度]
     = 10 - 5 = 5度（看起来对，实际错了！）

LQR输出 = K_angle × 5度 + K_velocity × 速度
        ≈ 31.6 × 5 = 158A （远超20A限幅！）
```

---

## ✅ 修复方案

### 核心思想

**让IMU直接输出弧度制数据，避免反复转换！**

```
四元数 → atan2f → [弧度] → YawTotalAngle_rad (新增)
                    ↓
                  ×57.3
                    ↓
                  [角度] → YawTotalAngle (保留，兼容)
```

---

## 📝 修改文件清单

### 1. **QuaternionEKF 底层** (最关键)

**文件：`modules/algorithm/QuaternionEKF.h`**
- 新增弧度制成员

**文件：`modules/algorithm/QuaternionEKF.c`**
- 在计算欧拉角后，同时输出弧度制和角度制
- 添加弧度制多圈累加逻辑

```c
// 先计算弧度值（原始输出）
QEKF_INS.Yaw_rad = atan2f(...);
QEKF_INS.Pitch_rad = atan2f(...);

// 转换为角度（兼容现有代码）
QEKF_INS.Yaw = QEKF_INS.Yaw_rad * 57.295779513f;

// 多圈累加（弧度制）
if (QEKF_INS.Yaw_rad - YawAngleLast_rad > PI)
    QEKF_INS.YawRoundCount_rad--;
else if (QEKF_INS.Yaw_rad - YawAngleLast_rad < -PI)
    QEKF_INS.YawRoundCount_rad++;
QEKF_INS.YawTotalAngle_rad = 2.0f * PI * QEKF_INS.YawRoundCount_rad + QEKF_INS.Yaw_rad;
```

### 2. **INS任务层**

**文件：`modules/imu/ins_task.h`**
- attitude_t 结构体新增弧度制成员

**文件：`modules/imu/ins_task.c`**
- 同步弧度制数据到 attitude_t

```c
INS.Pitch_rad = QEKF_INS.Pitch_rad;
INS.YawTotalAngle_rad = QEKF_INS.YawTotalAngle_rad;
```

### 3. **控制应用层**

**文件：`application/gimbal/gimbal.c`**
- Yaw电机反馈改为弧度制指针
- Pitch电机保持角度制（兼容现有代码）

```c
// Yaw: 使用弧度制
.other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle_rad,

// Pitch: 保持角度制
.other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
```

### 4. **命令层（关键！）**

**文件：`application/cmd/robot_cmd.c`**
- **Yaw目标值转换为弧度**

```c
// 遥控器控制
float yaw_increment = 0.001f * rc_data * PI / 180.0f;  // 度→弧度

// 鼠标控制
gimbal_cmd_send.yaw += mouse.x / 660 * 10 * PI / 180.0f;  // 度→弧度

// 初始化
gimbal_cmd_send.yaw = 0.0f;  // 0弧度
```

---

## 📊 修复后的单位统一

| 变量 | 单位 | 来源 |
|------|------|------|
| **Yaw目标角度** | rad | robot_cmd.c（已转换） |
| **Yaw角度反馈** | rad | YawTotalAngle_rad |
| **Yaw速度反馈** | rad/s | Gyro[2] |
| **Pitch目标角度** | 度 | robot_cmd.c（保持） |
| **Pitch角度反馈** | 度 | Pitch（兼容） |
| **Pitch速度反馈** | rad/s | Gyro[0] |

**注意：Pitch轴保持角度制，因为PID控制器不关心单位，只要一致即可！**

---

## 🎯 预期改善

修复后，LQR控制应该表现出：

| 症状 | 修复前 | 修复后 |
|------|--------|--------|
| **目标跟踪** | 几乎不动 | ✅ 快速响应 |
| **反向转动** | 有时发生 | ✅ 方向正确 |
| **硬度** | 很软 | ✅ 抗干扰能力强 |
| **精度** | 不准 | ✅ 误差<0.1rad(6度) |
| **起步** | 卡顿 | ✅ 平滑启动 |

---

## 🔬 验证方法

### 在Ozone中观察（关键变量）

```c
// 1. 目标与反馈单位应该一致
yaw_motor->motor_controller.pid_ref           // 目标 [rad]，范围：±3.14
yaw_motor->motor_controller.LQR.measure_angle // 反馈 [rad]，范围：±3.14

// 2. 误差应该合理
yaw_motor->motor_controller.LQR.angle_error   // 误差 [rad]，应该<0.5

// 3. 输出应该在限幅内
yaw_motor->motor_controller.LQR.output        // 输出 [A]，应该<18A
```

**正常波形特征：**
- ✅ 目标和反馈的数值范围相同（都是 ±3.14）
- ✅ 误差快速收敛到0
- ✅ 输出电流平滑，无突变

---

## 💡 性能优化建议

如果还是有轻微问题，可以调整LQR参数：

### 如果云台还是偏软（抗干扰能力弱）
```c
.LQR = {
    .K_angle = 40.0f,      // 增大角度增益（原31.6）
    .K_velocity = 6.5f,    // 增大速度增益（原5.58）
    .max_out = 18.0f,
},
```

### 如果有轻微抖动
```c
.LQR = {
    .K_angle = 25.0f,      // 降低角度增益
    .K_velocity = 7.0f,    // 增大速度阻尼
    .max_out = 18.0f,
},
```

### 如果响应偏慢
```c
.LQR = {
    .K_angle = 45.0f,      // 增大角度增益
    .K_velocity = 5.0f,    // 降低速度阻尼
    .max_out = 18.0f,
},
```

---

## 🎓 技术总结

### 关键教训

1. **单位一致性是控制系统的基础**
   - 系统辨识用什么单位
   - 控制器就必须用什么单位
   
2. **从源头输出正确单位**
   - 弧度是数学运算的自然单位
   - 在最底层（EKF）直接输出弧度
   - 避免反复转换

3. **数据流追溯的重要性**
   ```
   传感器 → 融合算法 → 控制器 → 执行器
     ↓         ↓          ↓         ↓
   单位?    单位?      单位?     单位?
   ```

---

## 📌 后续工作

如果需要进一步优化性能：

1. **重新进行系统辨识**（可选）
   - 现在单位已经统一
   - 可以获得更准确的系统模型
   
2. **使用MATLAB脚本重新设计LQR**
   - 使用优化后的 `LQR_MATLAB_Design.m`
   - 手动调整Q、R参数
   
3. **对比PID性能**
   ```c
   // 切换到PID
   DJIMotorChangeController(yaw_motor, CONTROLLER_PID);
   
   // 切换到LQR
   DJIMotorChangeController(yaw_motor, CONTROLLER_LQR);
   ```

---

## ✨ 总结

**这次修复解决了根本性的单位不一致问题！**

- ✅ IMU直接输出弧度制（零转换损耗）
- ✅ 目标值统一为弧度制
- ✅ LQR控制器单位完全匹配
- ✅ 兼容现有代码（角度制数据保留）

**现在LQR应该能正常工作了！** 🚀

---

**日期**: 2025-11-03  
**版本**: v2.0  
**状态**: ✅ 编译成功，待测试

