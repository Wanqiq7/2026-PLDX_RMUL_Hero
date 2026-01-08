# 系统辨识任务 (System Identification Task)

## 概述

系统辨识任务是一个独立的1kHz FreeRTOS任务，用于云台电机的开环特性测试和PID参数整定。该任务从`gimbal.c`中分离出来，确保精确的采样周期，避免与其他任务竞争导致的时序不准确问题。

## 主要特点

- **独立1kHz任务**：使用`osDelay(1)`实现1ms周期，确保采样频率稳定
- **DWT精确计时**：使用DWT（Data Watchpoint and Trace）硬件计时器测量实际执行周期
- **消息中心交互**：通过发布/订阅模式与云台任务通信
- **实时监控**：提供`actual_dt`和`task_freq`等诊断变量，可通过Ozone实时查看
- **自动完成**：测试完成后自动停止并重置状态

## 工作原理

### 1. 任务初始化

在`GimbalInit()`中调用`SysIDTaskInit()`，传递以下参数：
- YAW电机实例指针
- PITCH电机实例指针
- IMU数据指针

### 2. 任务调度

系统辨识任务以`osPriorityAboveNormal`优先级运行（与INS任务相同），确保：
- 高于普通控制任务，获得更稳定的执行时序
- 不会被其他任务抢占导致采样周期不准

### 3. 方波阶跃信号生成

任务生成方波阶跃信号，用于电机开环特性测试：

```c
时间轴:  0~2s: +8000 → 2~4s: -8000 → 4~6s: +8000 → ...
```

参数配置：
- `SYS_ID_STEP_AMPLITUDE`：阶跃信号幅值（默认8000）
- `SYS_ID_STEP_INTERVAL`：每个阶跃的持续时间（默认2秒）
- `SYS_ID_TOTAL_DURATION`：总测试时间（默认20秒）
- `SYS_ID_TARGET_MOTOR_YAW`：辨识目标（0-YAW轴，1-PITCH轴）

### 4. 电机控制

- **辨识轴**：配置为开环模式，直接输出电流指令
- **非辨识轴**：保持闭环模式，维持指定角度不动

### 5. 数据采集

每个1ms周期采集：
- `step_input`：当前电流指令值
- `motor_output`：IMU陀螺仪反馈的角速度（rad/s）
- `actual_dt`：实际测量的任务周期（秒）
- `task_freq`：实际任务频率（Hz）
- `time_elapsed`：已运行时间（秒）
- `call_counter`：任务调用次数

## 消息中心交互

### 发布话题：`sysid_feedback`

类型：`SysID_Feedback_s`

包含系统辨识的实时数据，供云台任务和调试工具订阅。

### 订阅话题：`sysid_cmd`

类型：`SysID_Ctrl_Cmd_s`

接收云台任务发布的控制指令，用于控制非辨识轴的位置。

## 使用方法

### 1. 配置参数

在`sysid_task.c`中修改宏定义：

```c
#define SYS_ID_STEP_AMPLITUDE 8000.0f  // 根据电机类型调整
#define SYS_ID_STEP_INTERVAL 2.0f      // 根据测试需求调整
#define SYS_ID_TOTAL_DURATION 20.0f    // 根据测试需求调整
#define SYS_ID_TARGET_MOTOR_YAW 0      // 0-YAW轴，1-PITCH轴
```

### 2. 启动辨识

在`robot_cmd.c`中设置云台模式为`GIMBAL_SYS_ID_CHIRP`：

```c
gimbal_cmd_send.gimbal_mode = GIMBAL_SYS_ID_CHIRP;
```

### 3. 监控数据

使用Ozone或其他调试工具，添加变量到Watch窗口：
- 在系统辨识任务中，找到`sysid_data`变量
- 展开结构体，查看实时数据

推荐监控的变量：
- `actual_dt`：应稳定在0.001秒左右（1kHz）
- `task_freq`：应稳定在1000Hz左右
- `step_input`：当前电流指令
- `motor_output`：电机速度反馈
- `time_elapsed`：测试进度

### 4. 导出数据

在Ozone中配置Data Sampling：
- 采样周期：1ms
- 采样持续时间：20秒以上
- 导出为CSV格式

### 5. 系统辨识

使用MATLAB或Python对采集的数据进行系统辨识，参考`gimbal.md`中的详细说明。

## 技术细节

### DWT时间戳预热

为避免首次调用时`dt`异常大，在`SystemID_Reset()`中进行预热：

```c
// 预热DWT时间戳，避免首次调用时dt异常大
DWT_GetDeltaT(&step_tick_last);
```

### 异常保护

如果检测到`dt`异常（>50ms），会：
1. 使用默认值1ms替代
2. 重新同步DWT时间戳
3. 记录到诊断变量中

### 任务优先级

- `osPriorityAboveNormal`：高于普通控制任务
- 与INS任务相同优先级
- 确保1kHz采样频率的稳定性

## 注意事项

1. **任务启动顺序**：系统辨识任务会等待100ms以确保云台初始化完成
2. **电机安全**：测试前确认电机和云台安装牢固，避免剧烈运动
3. **幅值选择**：根据电机类型选择合适的`SYS_ID_STEP_AMPLITUDE`，避免过大导致机械损坏
4. **测试时长**：默认20秒，可根据需要调整，但不建议超过30秒
5. **模式切换**：测试完成后，切换回正常控制模式（`GIMBAL_GYRO_MODE`等）

## 故障排查

### 问题1：`actual_dt`不稳定

- **原因**：任务优先级过低或CPU负载过高
- **解决**：检查其他任务的优先级配置，确保系统辨识任务不被抢占

### 问题2：首次`actual_dt`异常大（0.7~0.79秒）

- **原因**：DWT时间戳未预热（已修复）
- **解决**：确保使用最新版本的`sysid_task.c`

### 问题3：电机不动

- **原因**：模式未正确切换或消息中心通信异常
- **解决**：检查`robot_cmd.c`中的模式设置，确认消息中心注册正确

### 问题4：`task_freq`远低于1000Hz

- **原因**：任务执行时间过长或`osDelay(1)`不准确
- **解决**：检查任务执行时间（`sysid_dt`），优化代码或调整FreeRTOS配置

## 相关文件

- `application/sysid/sysid_task.c`：系统辨识任务实现
- `application/sysid/sysid_task.h`：系统辨识任务头文件
- `application/robot_task.h`：任务调度和启动函数
- `application/robot_def.h`：消息结构体定义
- `application/gimbal/gimbal.c`：云台任务（消息交互）
- `application/gimbal/gimbal.md`：云台功能和系统辨识详细说明

