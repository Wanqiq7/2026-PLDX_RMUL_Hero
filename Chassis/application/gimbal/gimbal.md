# gimbal

## 工作流程

初始化pitch和yaw电机以及一个imu。订阅gimbal_cmd消息（来自robot_cmd）并发布gimbal_feed话题。

1. 从消息中心获取gimbal_cmd话题的消息
2. **检查系统辨识模式**：如果辨识模式未关闭且处于激活状态，执行辨识任务并跳过正常控制逻辑
3. 根据消息中的控制模式进行模式切换，如果急停则关闭所有电机
4. 由设定的模式，进行电机反馈数据来源的切换，修改反馈数据指针，设置前馈控制数据指针等
5. 设置反馈数据，包括yaw电机的绝对角度和imu数据
6. 推送反馈数据到gimbal_feed话题下

## 系统辨识功能使用说明

本模块提供了云台电机系统辨识功能，用于采集电机的频率响应数据，以便进行系统建模和PID控制器参数整定。

### 功能说明

系统辨识功能通过生成线性扫频信号（Chirp Signal）作为电机输入激励，并采集电机的输出反馈，从而获取系统的频率响应特性。支持两种辨识模式，分别用于整定速度环和角度环PID参数。

**主要特点：**
- 扫频信号：频率从0.5Hz线性增加到80Hz（可配置）
- 扫频时间：20秒（可配置）
- 信号幅值：8000（可配置，根据电机类型调整）
- 采样周期：1ms
- 数据采集：通过Ozone等调试工具实时查看`sys_id_data`变量
- 双模式支持：速度环辨识和角度环辨识

### 辨识模式说明

#### 模式1：速度环辨识 (SYS_ID_MODE_SPEED_LOOP)

**目的：** 辨识电机开环传递函数，用于整定速度环PID参数

**工作原理：**
- **输入信号：** 扫频信号作为直接电流指令（绕过所有PID控制器）
- **输出信号：** 电机编码器反馈的转速 [rpm]
- **辨识对象：** 电机本体的开环传递函数（电流→转速）

**适用场景：**
- 首次调试电机，速度环PID参数未知
- 更换电机或负载后，需要重新整定速度环
- 速度环响应不理想，需要重新优化参数

**注意事项：**
- 需要实现 `DJIMotorSetDirectCurrent()` 函数以支持直接电流控制
- 建议幅值从小开始尝试，避免电机过载
- 如果电机震动剧烈，立即停止并降低幅值

#### 模式2：角度环辨识 (SYS_ID_MODE_ANGLE_LOOP)

**目的：** 辨识速度闭环系统传递函数，用于整定角度环PID参数

**工作原理：**
- **输入信号：** 扫频信号作为速度环目标速度 [deg/s]
- **输出信号：** IMU陀螺仪反馈的角速度 [deg/s]
- **辨识对象：** 速度闭环后的系统传递函数（速度指令→实际角速度）

**适用场景：**
- 速度环已整定完成，需要整定角度环
- 角度环响应过慢或有振荡
- 系统负载变化，需要重新优化角度环

**注意事项：**
- 必须先整定好速度环，否则辨识结果不可靠
- 使用IMU角速度作为反馈，确保IMU数据准确

### 使用步骤

#### 步骤1：配置辨识参数

在 `gimbal.c` 文件开头找到系统辨识参数定义区域，根据需要修改以下参数：

```c
// 扫频信号参数配置
#define SYS_ID_CHIRP_START_FREQ 0.5f     // 扫频起始频率 [Hz]
#define SYS_ID_CHIRP_END_FREQ 80.0f      // 扫频结束频率 [Hz]
#define SYS_ID_CHIRP_DURATION 20.0f      // 扫频持续时间 [s]
#define SYS_ID_CHIRP_AMPLITUDE 8000.0f   // 扫频信号幅值
#define SYS_ID_TARGET_MOTOR_YAW 0        // 0-辨识YAW轴，1-辨识PITCH轴
```

**参数调整建议：**
- `SYS_ID_CHIRP_START_FREQ`：起始频率建议 0.1~1Hz
- `SYS_ID_CHIRP_END_FREQ`：结束频率根据系统带宽调整，通常 50~100Hz
- `SYS_ID_CHIRP_DURATION`：时长越长频率分辨率越高，但需考虑电机发热
- `SYS_ID_CHIRP_AMPLITUDE`：
  - 速度环辨识：电流指令幅值，建议从 3000~5000 开始
  - 角度环辨识：速度指令幅值，建议从 100~500 deg/s 开始

#### 步骤2：选择辨识模式

在 `GimbalInit()` 函数中，找到系统辨识初始化部分：

```c
// 系统辨识初始化（默认关闭）
// 若需要开启辨识，请修改此处的模式参数：
// SYS_ID_MODE_SPEED_LOOP - 辨识速度环
// SYS_ID_MODE_ANGLE_LOOP - 辨识角度环
SystemID_Init(SYS_ID_MODE_OFF);
```

**修改为对应的辨识模式：**

**辨识速度环：**
```c
SystemID_Init(SYS_ID_MODE_SPEED_LOOP);
```

**辨识角度环：**
```c
SystemID_Init(SYS_ID_MODE_ANGLE_LOOP);
```

**关闭辨识（正常控制）：**
```c
SystemID_Init(SYS_ID_MODE_OFF);
```

#### 步骤3：编译并烧录

修改辨识模式后，重新编译并烧录程序到开发板：

```bash
make -j4
# 或使用 VSCode 的编译任务
```

#### 步骤4：使用Ozone采集数据

1. **打开Ozone调试器并连接开发板**

2. **添加监控变量**
   - 在 Watch 窗口中添加变量：`sys_id_data`
   - 展开结构体，可以看到以下字段：
     - `mode`：当前辨识模式（0=关闭，1=速度环，2=角度环）
     - `chirp_input`：扫频输入信号（发送给电机的控制指令）
     - `motor_output`：电机输出反馈（根据模式不同而不同）
     - `time_elapsed`：已运行时间 [s]
     - `is_active`：辨识激活状态（1=运行中，0=已结束）

3. **配置数据采样**
   - 右键 `sys_id_data` -> "Add to Data Sampling"
   - 或分别添加 `sys_id_data.chirp_input` 和 `sys_id_data.motor_output`
   - 配置采样周期：**1ms**（与控制周期一致）
   - 配置采样持续时间：**至少20秒**（根据 `SYS_ID_CHIRP_DURATION` 调整）

4. **开始采集**
   - 点击 Ozone 的 "Start Sampling" 按钮
   - 观察 `sys_id_data.is_active` 变为 1，表示辨识开始
   - 等待扫频完成（`is_active` 变为 0）
   - 停止采样并导出数据为 CSV 格式

5. **安全监控**
   - 实时观察电机运动是否正常
   - 如出现剧烈振动或异常声音，立即断电
   - 检查电机温度，避免过热

#### 步骤5：数据后处理与分析

将导出的CSV数据导入MATLAB或Python进行系统辨识分析：

##### MATLAB 示例

**方法1：使用系统辨识工具箱（推荐）**

```matlab
%% 读取数据
data = readtable('sys_id_data.csv');
input_signal = data.chirp_input;
output_signal = data.motor_output;
fs = 1000;  % 采样频率 1kHz

%% 创建辨识数据对象
sys_data = iddata(output_signal, input_signal, 1/fs);

%% 辨识传递函数模型
% 对于速度环：一阶或二阶系统
sys_model = tfest(sys_data, 2, 1);  % 二阶分子，一阶分母

% 对于角度环：二阶或三阶系统
% sys_model = tfest(sys_data, 2, 2);  % 二阶分子，二阶分母

%% 显示模型参数
disp('辨识得到的传递函数：');
tf(sys_model)

%% 绘制Bode图
figure;
bode(sys_model);
grid on;
title('系统频率响应（Bode图）');

%% 验证拟合效果
figure;
compare(sys_data, sys_model);
title('模型拟合验证');
```

**方法2：频域分析**

```matlab
%% 计算频率响应
[Pxy, f] = cpsd(input_signal, output_signal, [], [], [], fs);
[Pxx, ~] = cpsd(input_signal, input_signal, [], [], [], fs);
H = Pxy ./ Pxx;  % 传递函数

%% 绘制Bode图
figure;
subplot(2,1,1);
semilogx(f, 20*log10(abs(H)));
grid on;
ylabel('幅值 [dB]');
title('频率响应');

subplot(2,1,2);
semilogx(f, angle(H)*180/pi);
grid on;
ylabel('相位 [度]');
xlabel('频率 [Hz]');
```

##### Python 示例

```python
import pandas as pd
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

# 读取数据
data = pd.read_csv('sys_id_data.csv')
input_signal = data['chirp_input'].values
output_signal = data['motor_output'].values
fs = 1000  # 采样频率 1kHz

# 计算频率响应
f, Pxy = signal.csd(input_signal, output_signal, fs=fs, nperseg=2048)
f, Pxx = signal.csd(input_signal, input_signal, fs=fs, nperseg=2048)
H = Pxy / Pxx  # 传递函数

# 绘制Bode图
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# 幅频特性
ax1.semilogx(f, 20*np.log10(np.abs(H)))
ax1.grid(True, which='both', alpha=0.3)
ax1.set_ylabel('幅值 [dB]')
ax1.set_title('系统频率响应（Bode图）')

# 相频特性
ax2.semilogx(f, np.angle(H, deg=True))
ax2.grid(True, which='both', alpha=0.3)
ax2.set_ylabel('相位 [度]')
ax2.set_xlabel('频率 [Hz]')

plt.tight_layout()
plt.savefig('bode_plot.png', dpi=300)
plt.show()

# 系统辨识（使用最小二乘法拟合传递函数）
# 提取截止频率和共振峰等特征用于PID整定
```

#### 步骤6：PID参数整定

根据辨识结果整定PID参数：

**速度环PID整定（基于速度环辨识结果）：**

1. **增益裕度法**：
   - 从Bode图读取增益交界频率 ω_c 和相位裕度 PM
   - 建议相位裕度 PM = 45°~60°
   - Kp ≈ 1 / |H(jω_c)|

2. **经验公式法**（二阶系统）：
   - 自然频率 ω_n 和阻尼比 ζ 从传递函数获取
   - Kp ≈ 2ζω_n
   - Ki ≈ ω_n²

**角度环PID整定（基于角度环辨识结果）：**

1. **带宽法**：
   - 角度环带宽应为速度环带宽的 1/3~1/5
   - Kp = 2ζω_n （ω_n 为期望角度环带宽）

2. **迭代优化**：
   - 先设置较小的 Kp，观察响应
   - 逐步增大 Kp 直到出现轻微振荡
   - 回退 20%~30% 作为最终参数

#### 步骤7：关闭辨识模式

完成数据采集和参数整定后，恢复正常控制模式：

```c
// 在 GimbalInit() 中修改为：
SystemID_Init(SYS_ID_MODE_OFF);
```

重新编译并烧录程序。

### 安全注意事项

1. **安全第一**：
   - 系统辨识会驱动电机持续高速运动
   - **必须**将机器人固定在支架上，防止倾倒
   - 确保云台周围无障碍物和人员

2. **电气安全**：
   - 使用稳压电源，确保供电稳定
   - 准备急停开关，随时可以断电
   - 监控电机温度，避免过热损坏

3. **参数安全**：
   - 幅值从小开始尝试，逐步增大
   - 速度环辨识电流幅值不超过电机额定电流的 50%
   - 角度环辨识速度幅值根据机械限位合理设置

4. **数据有效性**：
   - 确保IMU已正确初始化且数据稳定
   - 检查电机CAN通信正常
   - 辨识前先进行短时间测试，确认系统响应正常

5. **环境要求**：
   - 在低负载、无外部干扰的环境下进行
   - 避免机械卡滞或摩擦力过大
   - 温度适中，避免极端环境

### 常见问题排查

**问题1：电机不动或响应很小**
- 检查 `SYS_ID_CHIRP_AMPLITUDE` 是否过小
- 确认电机已使能（`DJIMotorEnable`）
- 检查CAN通信是否正常

**问题2：电机振动剧烈**
- 降低 `SYS_ID_CHIRP_AMPLITUDE`
- 检查机械装配是否牢固
- 确认负载是否过重

**问题3：数据中有明显噪声**
- 检查电源纹波，使用稳压电源
- 增加机械阻尼，减少共振
- 延长扫频时间以提高频率分辨率

**问题4：辨识结果不理想**
- 确保速度环辨识时实现了 `DJIMotorSetDirectCurrent()` 函数
- 角度环辨识前先确保速度环已整定好
- 检查传感器反馈是否准确（IMU、编码器）

**问题5：`is_active` 一直为1，不自动停止**
- 检查 `SYS_ID_CHIRP_DURATION` 设置
- 确认时间累加正常（`sys_id_data.time_elapsed`）

### 数据结构说明

```c
/**
 * @brief 系统辨识工作模式枚举
 */
typedef enum
{
    SYS_ID_MODE_OFF = 0,         // 关闭辨识
    SYS_ID_MODE_SPEED_LOOP = 1,  // 辨识速度环（电流→转速）
    SYS_ID_MODE_ANGLE_LOOP = 2,  // 辨识角度环（速度→角速度）
} SystemID_Mode_e;

/**
 * @brief 系统辨识数据结构
 * @note 通过Ozone监控此结构体进行数据采集
 */
typedef struct
{
    SystemID_Mode_e mode;     // 当前辨识模式
    float chirp_input;        // 扫频输入信号
    float motor_output;       // 电机输出反馈
    float time_elapsed;       // 已运行时间 [s]
    uint8_t is_active;        // 辨识激活标志
} SystemID_Data_s;
```

### 高级功能（待实现）

1. **迭代相位更新法**：
   - 对于超长时间扫频（>60秒），使用迭代更新相位避免浮点精度损失
   - 实现方法：`phase += 2π * f(t) * dt`

2. **多频段扫频**：
   - 低频段（0.1~10Hz）：慢速扫频，提高低频分辨率
   - 高频段（10~100Hz）：快速扫频，覆盖高频特性

3. **自动PID整定**：
   - 基于辨识结果自动计算推荐的PID参数
   - 提供多组参数方案（保守/适中/激进）

4. **闭环稳定性分析**：
   - 基于开环传递函数和PID参数，预测闭环稳定性
   - 计算相位裕度、幅值裕度、稳定裕度等指标
