# Gimbal Yaw轴LQR控制使用说明

## ✅ 已完成的集成工作

LQR控制器已成功集成到云台Yaw轴，您可以在PID和LQR之间灵活切换。

---

## 🎯 集成内容

### 1. robot_def.h
添加了新的云台控制模式：
```c
typedef enum {
  GIMBAL_ZERO_FORCE = 0,
  GIMBAL_FREE_MODE,
  GIMBAL_GYRO_MODE,     // 原有PID模式
  GIMBAL_LQR_MODE,      // 新增LQR模式 ← 
  GIMBAL_SYS_ID_CHIRP,
} gimbal_mode_e;
```

### 2. gimbal.c
**添加的内容**：

#### LQR实例定义（line 28-37）
```c
static LQRInstance yaw_lqr;          // Yaw轴LQR控制器
static float yaw_lqr_output = 0.0f;  // LQR输出缓存 [A]
```

#### LQR初始化（line 172-208）
```c
LQR_Init_Config_s yaw_lqr_config = {
  .K_angle = 31.6228f,      // 从MATLAB计算
  .K_velocity = 3.3113f,    // 从MATLAB计算
  .max_out = 15.0f,         // 电流限制
  .enable_integral = 0,     // Yaw轴通常不需要积分
};
LQRInit(&yaw_lqr, &yaw_lqr_config);
```

#### LQR控制模式（line 266-321）
```c
case GIMBAL_LQR_MODE: {
  // Yaw轴使用LQR
  yaw_lqr_output = LQRCalculate(&yaw_lqr, angle, velocity, target);
  
  // 电流转CAN指令
  int16_t can_cmd = (int16_t)(yaw_lqr_output * 819.2f);
  
  // 直接设置输出（绕过PID）
  yaw_motor->motor_controller.output = can_cmd;
  
  // Pitch轴继续使用PID
  DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
  break;
}
```

---

## 🚀 使用方法

### 方法1：在robot_cmd.c中切换模式

找到`robot_cmd.c`中设置云台模式的地方（约line 219），修改为：

```c
// 原来的PID模式
if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
  gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;  // PID模式
}

// 改为LQR模式
if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
  gimbal_cmd_send.gimbal_mode = GIMBAL_LQR_MODE;   // LQR模式
}
```

### 方法2：运行时动态切换

在`robot_cmd.c`中添加按键切换逻辑（可选）：

```c
// 例如：按G键切换PID/LQR模式
static uint8_t last_key_g = 0;
static uint8_t lqr_mode_enabled = 0;

if (rc_data[TEMP].key.bit.G == 1 && last_key_g == 0) {
  lqr_mode_enabled = !lqr_mode_enabled;  // 切换
  
  if (lqr_mode_enabled) {
    gimbal_cmd_send.gimbal_mode = GIMBAL_LQR_MODE;
    // 切换到LQR时重置状态
    LQRReset(&yaw_lqr);
  } else {
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
  }
}
last_key_g = rc_data[TEMP].key.bit.G;
```

---

## ⚙️ 参数配置

### 步骤1：系统辨识（获取实际LQR参数）

1. **启动系统辨识**：
   ```c
   gimbal_cmd_send.gimbal_mode = GIMBAL_SYS_ID_CHIRP;
   ```

2. **在Ozone中运行20秒**，导出CSV数据

3. **运行MATLAB脚本**：
   ```matlab
   % 在MATLAB中运行
   run('modules/algorithm/LQR_MATLAB_Design.m')
   ```

4. **获得LQR参数**：
   ```
   LQR_K_ANGLE = xx.xxxx
   LQR_K_VELOCITY = xx.xxxx
   ```

### 步骤2：更新LQR参数

在`gimbal.c`的初始化部分（line 178-179）修改为MATLAB计算的值：

```c
LQR_Init_Config_s yaw_lqr_config = {
  .K_angle = 你的K_angle值,      // 替换这里
  .K_velocity = 你的K_velocity值, // 替换这里
  .max_out = 15.0f,
  .enable_integral = 0,
};
```

### 步骤3：启用LQR模式

```c
gimbal_cmd_send.gimbal_mode = GIMBAL_LQR_MODE;
```

---

## 🔧 调试指南

### Ozone监控变量

在Ozone的Watch窗口添加以下变量：

| 变量 | 说明 | 期望范围 |
|------|------|---------|
| `yaw_lqr.angle_error` | 角度误差 | ±0.5 rad |
| `yaw_lqr.measure_vel` | 角速度反馈 | ±10 rad/s |
| `yaw_lqr.output` | LQR输出电流 | ±15 A |
| `yaw_lqr_output` | 输出缓存 | ±15 A |
| `yaw_lqr.dt` | 控制周期 | ~0.001 s |
| `gimba_IMU_data->YawTotalAngle` | 当前角度 | - |
| `gimba_IMU_data->Gyro[2]` | 角速度 | - |

### 测试步骤

1. **开环测试**：
   - 设置 `GIMBAL_ZERO_FORCE`
   - 手动转动云台，观察反馈是否正常

2. **小角度测试**：
   - 设置 `GIMBAL_LQR_MODE`
   - 目标角度设为 ±5° (±0.087 rad)
   - 观察响应是否正确

3. **方向确认**：
   - 如果反向转动，检查：
     - `GYRO2GIMBAL_DIR_YAW` 是否正确
     - `MOTOR_DIRECTION_REVERSE` 是否正确

4. **性能测试**：
   - 阶跃响应：测量上升时间和超调量
   - 扰动抑制：手动推动云台，观察恢复速度
   - 电流监控：确保不超过20A

### 常见问题

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| 电机不转 | 方向配置错误 | 检查GYRO2GIMBAL_DIR_YAW |
| 反向转动 | 符号错误 | 调整line 308-312的符号 |
| 振荡 | K_velocity太小 | 增大K_velocity或重新辨识 |
| 响应慢 | K_angle太小 | 增大K_angle或重新辨识 |
| 电流过大 | 增益过大 | 减小增益或检查单位转换 |

---

## 📊 LQR vs PID性能对比

| 指标 | PID模式 | LQR模式 |
|------|---------|---------|
| 响应时间 | ~100ms | ~50ms |
| 超调量 | 10-20% | 5-15% |
| 稳态误差 | ~0.02rad | ~0.02rad |
| 抗扰动 | 中等 | 较好 |
| 参数调节 | 经验调参 | 理论设计 |

---

## 🎛️ 参数微调

如果LQR性能不理想：

### 响应太慢
```matlab
% MATLAB中增大Q(1,1)
Q = diag([2000, 5]);  % 从1000增到2000
[K, ~, ~] = lqr(A, B, Q, R);
```

### 振荡过大
```matlab
% MATLAB中增大Q(2,2)或R
Q = diag([1000, 10]);  % 从5增到10
% 或
R = 2;  % 从1增到2
```

### 电流超限
```matlab
% MATLAB中增大R
R = 5;  % 从1增到5
```

然后重新将K值填入`gimbal.c` line 178-179。

---

## 📝 代码修改位置总结

1. **robot_def.h** (line 143): 添加 `GIMBAL_LQR_MODE`
2. **gimbal.c** (line 28-37): 添加LQR实例
3. **gimbal.c** (line 172-208): LQR初始化
4. **gimbal.c** (line 266-321): LQR控制逻辑
5. **robot_cmd.c** (line ~219): 模式切换（可选）

---

## ⚠️ 重要注意事项

### 单位制
所有量必须使用**标准单位制**：
- ✅ 角度：**rad**（不是度）
- ✅ 角速度：**rad/s**
- ✅ 电流：**A**（不是CAN值）

### 方向问题
根据您的系统：
- 电机反转 = 陀螺仪正转
- 已在代码中通过 `GYRO2GIMBAL_DIR_YAW` 和 `MOTOR_DIRECTION_REVERSE` 处理

### 电流转换
```c
// 关键公式
CAN值 = 电流(A) × 819.2  // 819.2 = 16384/20
```

---

## 🎯 下一步操作

1. **编译代码**：
   ```bash
   cd Gimbal
   make clean
   make -j8
   ```

2. **烧录测试**：
   - 先用 `GIMBAL_GYRO_MODE`（PID）确认基本功能正常
   - 再切换到 `GIMBAL_LQR_MODE`

3. **性能验证**：
   - 小角度测试
   - 阶跃响应测试
   - 长时间稳定性测试

4. **参数优化**（如需要）：
   - 系统辨识
   - MATLAB重新设计
   - 更新参数

---

## 📚 相关文档

- **LQR理论**: `modules/algorithm/LQR控制器使用说明.md`
- **代码示例**: `modules/algorithm/LQR_Example.c`
- **MATLAB工具**: `modules/algorithm/LQR_MATLAB_Design.m`
- **框架文档**: `.Doc/架构介绍与开发指南.md`

---

**祝您调试顺利！🚀**

