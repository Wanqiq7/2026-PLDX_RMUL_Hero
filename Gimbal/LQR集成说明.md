# LQR控制器集成完成说明

## ✅ 完成的工作

### 1. 代码集成

#### 📝 修改的文件

| 文件 | 修改内容 | 说明 |
|------|---------|------|
| `controller.h` | 添加LQR结构体定义和API | 遵循框架面向对象设计 |
| `controller.c` | 实现LQR控制器核心算法 | 参考Mas2025 Yaw/Pitch轴 |
| `user_lib.h` | 添加矩阵运算函数声明 | 支持LQR计算 |
| `user_lib.c` | 实现矩阵运算辅助函数 | 详细注释和优化 |

#### 📄 新增的文件

| 文件 | 用途 |
|------|------|
| `LQR控制器使用说明.md` | 详细的理论和使用文档 |
| `LQR_Example.c` | 多个使用示例 |
| `LQR_MATLAB_Design.m` | MATLAB设计工具脚本 |
| `LQR_README.md` | 快速入门指南 |
| `LQR集成说明.md` | 本文件 |

### 2. 核心功能

#### LQR控制器 (controller.h/c)

```c
// 数据结构
typedef struct {
  float K_angle;           // 角度增益 [A/rad]
  float K_velocity;        // 角速度增益 [A·s/rad]
  float K_integral;        // 积分增益 [A/(rad·s)]
  uint8_t enable_integral; // 积分开关
  // ... 其他参数
} LQRInstance;

// API接口
void LQRInit(LQRInstance *lqr, LQR_Init_Config_s *config);
float LQRCalculate(LQRInstance *lqr, float angle, float vel, float ref);
void LQRReset(LQRInstance *lqr);
```

**特点**：
- ✅ 遵循框架面向对象设计规范
- ✅ 使用标准单位制（rad, rad/s, A）
- ✅ 支持积分增强和变增益积分
- ✅ 参考Mas2025实现，经过实战验证

#### 矩阵运算库 (user_lib.h/c)

```c
// 2x2矩阵乘向量
void Mat2x2_Mult_Vec2(const float mat[4], const float vec[2], float res[2]);

// 向量内积
float Vec2_DotProduct(const float vec1[2], const float vec2[2]);

// 向量限幅
void Vec_Constrain(float *vec, uint8_t n, float limit);

// 角度归一化
float AngleNormalize(float angle);
```

**特点**：
- ✅ 详细的数学注释和示例
- ✅ 优化的计算效率
- ✅ 支持LQR等高级控制算法

## 🔧 使用流程

### 完整开发流程

```
┌─────────────────┐
│ 1. 系统辨识     │ → 使用sysid_task采集数据
└────────┬────────┘
         ↓
┌─────────────────┐
│ 2. MATLAB设计   │ → 运行LQR_MATLAB_Design.m
└────────┬────────┘
         ↓
┌─────────────────┐
│ 3. 获取增益K    │ → K_angle, K_velocity
└────────┬────────┘
         ↓
┌─────────────────┐
│ 4. 配置参数     │ → LQR_Init_Config_s
└────────┬────────┘
         ↓
┌─────────────────┐
│ 5. 初始化LQR    │ → LQRInit()
└────────┬────────┘
         ↓
┌─────────────────┐
│ 6. 控制循环     │ → LQRCalculate()
└────────┬────────┘
         ↓
┌─────────────────┐
│ 7. 调试优化     │ → Ozone监控+参数微调
└─────────────────┘
```

### 最小示例代码

```c
// 1. 包含头文件
#include "controller.h"

// 2. 定义实例
static LQRInstance yaw_lqr;

// 3. 初始化
void Init(void) {
  LQR_Init_Config_s config = {
    .K_angle = 31.6228f,    // 从MATLAB获得
    .K_velocity = 3.3113f,  // 从MATLAB获得
    .max_out = 15.0f,
    .enable_integral = 0,
  };
  LQRInit(&yaw_lqr, &config);
}

// 4. 控制循环（1kHz）
void ControlTask(void) {
  float angle = imu->YawTotalAngle;       // [rad]
  float velocity = -imu->Gyro[2];         // [rad/s]
  float target = 0.0f;                    // [rad]
  
  float current_A = LQRCalculate(&yaw_lqr, angle, velocity, target);
  
  int16_t can_cmd = (int16_t)(current_A * 819.2f);
  GM6020_SetCurrent(-can_cmd);
}
```

## 📐 量纲说明（重要！）

### 输入输出单位

| 变量 | 单位 | 说明 |
|------|------|------|
| measure_angle | rad | 弧度，不是度 |
| measure_vel | rad/s | 弧度/秒，不是度/秒 |
| ref | rad | 目标角度，弧度 |
| **返回值** | **A** | **电流，安培** |

### 电流转换

```c
// LQR输出(A) → CAN指令
CAN值 = 电流(A) × (16384 / 20) = 电流(A) × 819.2

// 示例
float current = 10.0f;  // 10A
int16_t can = (int16_t)(10.0f * 819.2f);  // 8192
```

## 🎚️ 参数调整

### 方法1：MATLAB重新设计（推荐）

修改`LQR_MATLAB_Design.m`中的Q和R矩阵：

```matlab
% 响应太慢 → 增大Q(1,1)
Q = diag([2000, 5]);  % 从1000增大到2000

% 振荡过大 → 增大Q(2,2)或R
Q = diag([1000, 10]); % 从5增大到10
R = 2;                % 从1增大到2
```

### 方法2：现场微调（不推荐）

```c
// 紧急情况下可以直接修改增益
config.K_angle *= 1.2f;     // 加快响应
config.K_velocity *= 1.5f;  // 增加阻尼
```

**⚠️ 注意**: 手动调整可能破坏LQR的最优性，建议用MATLAB重新设计。

## 🔍 调试技巧

### Ozone监控

添加这些变量到Watch窗口：

```
yaw_lqr.angle_error      # 应该在±1 rad范围内
yaw_lqr.measure_vel      # 应该合理的角速度值
yaw_lqr.output           # 应该在±20A范围内
yaw_lqr.dt               # 应该约0.001s (1kHz)
```

### 日志输出

```c
// 在控制循环中添加
LOGINFO("LQR: err=%.3f, vel=%.3f, I=%.2f", 
        yaw_lqr.angle_error, 
        yaw_lqr.measure_vel, 
        yaw_lqr.output);
```

## 🆚 LQR vs PID

| 特性 | PID | LQR |
|------|-----|-----|
| 理论基础 | 经验 | 最优控制理论 |
| 调参方式 | 试错 | 数学求解 |
| 多变量 | 困难 | 自然支持 |
| 稳定性 | 无保证 | 数学保证 |
| 响应速度 | 中等 | 快速 |
| 实现复杂度 | 简单 | 中等 |

## 📚 参考代码

### Mas2025云台实现位置

**Yaw轴LQR（纯LQR）**:
```
文件: Mas2025_Hero_Control-main/代码/云台主/CarBody/Gimbal.c
位置: line 166-214
增益: K3=31.6228, K4=3.3113
```

**Pitch轴LQR+积分**:
```
文件: 同上
位置: line 72-157
增益: K1=40.0, K2=2.3802, Ki=0.15
特点: 带变增益积分，消除重力影响
```

## 💡 最佳实践

1. **先系统辨识**：获得准确的物理参数
2. **MATLAB设计**：使用理论方法得到最优增益
3. **小角度测试**：先测试±5°，确认方向
4. **监控电流**：确保不超过电机额定值
5. **性能评估**：测量实际的上升时间、超调量
6. **必要时微调**：根据实测调整Q、R

## 🎓 学习资源

- **框架文档**: `.Doc/架构介绍与开发指南.md`
- **PID调参**: `.Doc/合理地进行PID参数整定.md`
- **系统辨识**: `application/sysid/sysid_task.md`
- **MATLAB官方**: LQR设计工具箱文档

## 📧 技术支持

如有问题，请：
1. 查看详细文档：`LQR控制器使用说明.md`
2. 参考示例代码：`LQR_Example.c`
3. 联系团队技术支持

---

**祝您调试顺利！🚀**

