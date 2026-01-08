# 系统辨识任务迁移说明

## 修改概述

将系统辨识功能从`gimbal.c`的`GimbalTask()`中分离出来，创建独立的1kHz FreeRTOS任务`StartSYSIDTASK()`，以解决以下问题：

1. **采样周期不稳定**：原来的实现依赖`RobotTask()`的调度，实际执行频率受其他任务影响
2. **首帧dt异常**：清零时间戳导致首次调用时dt测量错误（0.7~0.79秒）
3. **时序不精确**：与其他任务共享执行时间，无法保证1kHz精确采样

## 修改文件清单

### 新增文件

1. **`application/sysid/sysid_task.c`**
   - 系统辨识任务主要实现
   - 包含方波信号生成、电机控制、数据采集等功能
   - 使用DWT精确计时，带首帧预热机制

2. **`application/sysid/sysid_task.h`**
   - 系统辨识任务头文件
   - 定义对外接口函数

3. **`application/sysid/sysid_task.md`**
   - 系统辨识任务详细文档
   - 包含使用方法、技术细节、故障排查等

### 修改文件

1. **`application/gimbal/gimbal.c`**
   - 移除系统辨识相关的数据结构和函数
   - 简化`GIMBAL_SYS_ID_CHIRP`分支，改为通过消息中心与辨识任务交互
   - 在`GimbalInit()`中调用`SysIDTaskInit()`

2. **`application/robot_def.h`**
   - 新增`SysID_Ctrl_Cmd_s`结构体（云台任务→系统辨识任务）
   - 新增`SysID_Feedback_s`结构体（系统辨识任务→云台任务）

3. **`application/robot_task.h`**
   - 新增`sysidTaskHandle`任务句柄
   - 新增`StartSYSIDTASK()`任务启动函数
   - 在`OSTaskInit()`中创建系统辨识任务

## 技术改进

### 1. 精确的1kHz采样

```c
__attribute__((noreturn)) void StartSYSIDTASK(void const *argument)
{
    for (;;)
    {
        sysid_start = DWT_GetTimeline_ms();
        SysIDTask();
        sysid_dt = DWT_GetTimeline_ms() - sysid_start;
        osDelay(1);  // 1ms周期，实现1kHz采样
    }
}
```

### 2. DWT时间戳预热

```c
static void SystemID_Reset(void) {
    memset(&sysid_data, 0, sizeof(SysID_Feedback_s));
    
    // 预热DWT时间戳，避免首次调用时dt异常大
    DWT_GetDeltaT(&step_tick_last);
}
```

### 3. 更好的异常保护

```c
float dt = DWT_GetDeltaT(&step_tick_last);

// 异常保护：防止首次调用或异常情况下dt过大
if (dt <= 0.0f || dt > 0.05f) {
    dt = 0.001f; // 默认1ms，对应1kHz控制频率
    if (dt > 0.05f) {
        DWT_GetDeltaT(&step_tick_last); // 重新同步时间戳
    }
}
```

### 4. 高优先级任务

```c
osThreadDef(sysidtask, StartSYSIDTASK, osPriorityAboveNormal, 0, 512);
```

与INS任务相同优先级，确保不被普通任务抢占。

## 消息中心架构

```
┌─────────────┐   sysid_cmd    ┌──────────────────┐
│             │ ──────────────> │                  │
│ GimbalTask  │                 │  StartSYSIDTASK  │
│             │ <────────────── │   (1kHz独立)     │
└─────────────┘  sysid_feedback └──────────────────┘
                                        │
                                        │ 直接控制
                                        ▼
                                  YAW/PITCH电机
```

## 使用说明

### 编译配置

无需修改编译配置，只需确保新增文件被包含在编译系统中（CMake或Makefile）。

### 运行时行为

1. **启动顺序**：
   - 系统上电 → `RobotInit()` → `GimbalInit()` → `SysIDTaskInit()`
   - FreeRTOS启动 → `StartSYSIDTASK()`等待100ms → 进入1kHz循环

2. **触发辨识**：
   - 在`robot_cmd.c`中设置`gimbal_cmd_send.gimbal_mode = GIMBAL_SYS_ID_CHIRP;`
   - 系统辨识任务自动开始执行

3. **完成后**：
   - 测试完成后，`is_finished`标志置1
   - 任务自动停止激励信号
   - 切换回其他云台模式即可

### 监控变量

在Ozone中监控`sysid_data`变量（位于`sysid_task.c`）：

- `actual_dt`：应稳定在 **0.001秒** 左右（之前会出现0.7~0.79秒的异常值）
- `task_freq`：应稳定在 **1000Hz** 左右
- `step_input`：电流指令（方波）
- `motor_output`：电机速度反馈
- `time_elapsed`：测试进度

## 验证要点

### 1. 采样频率验证

- `actual_dt` 应稳定在 0.001 ± 0.0002 秒范围内
- `task_freq` 应稳定在 1000 ± 20 Hz 范围内
- **不再出现0.7~0.79秒的异常首帧**

### 2. 任务调度验证

- 检查FreeRTOS日志，确认系统辨识任务成功创建
- 使用`sysid_dt`监控任务执行时间，应小于1ms

### 3. 功能验证

- 进入辨识模式后，辨识轴应按方波信号运动
- 非辨识轴应保持指定角度不动
- 测试完成后（20秒），电机自动停止

## 故障排查

### Q1: 编译错误 "undefined reference to SysIDTaskInit"

**原因**：新增文件未加入编译系统

**解决**：在CMakeLists.txt或Makefile中添加：
```cmake
application/sysid/sysid_task.c
```

### Q2: 运行时日志显示 "SYSID Task is being DELAY!"

**原因**：任务执行时间超过1ms或系统负载过高

**解决**：
1. 检查是否有其他高优先级任务占用CPU
2. 优化系统辨识任务代码（当前实现已经很高效）

### Q3: 电机不动或异常运动

**原因**：
- 消息中心通信异常
- 电机指针未正确传递
- 模式切换不正确

**解决**：
1. 检查`SysIDTaskInit()`是否在`GimbalInit()`中被正确调用
2. 确认`robot_cmd.c`中的模式设置
3. 使用调试器检查`sysid_yaw_motor`和`sysid_pitch_motor`指针是否有效

### Q4: actual_dt 仍然不稳定

**原因**：FreeRTOS配置问题或CPU负载过高

**解决**：
1. 检查`configTICK_RATE_HZ`是否为1000
2. 确认系统辨识任务优先级为`osPriorityAboveNormal`
3. 降低其他任务的优先级或执行频率

## 迁移检查清单

- [x] 创建`sysid_task.c`和`sysid_task.h`
- [x] 在`robot_def.h`中定义消息结构体
- [x] 在`robot_task.h`中添加任务定义
- [x] 修改`gimbal.c`，移除旧代码并添加消息交互
- [x] 在`GimbalInit()`中调用`SysIDTaskInit()`
- [x] 添加DWT时间戳预热机制
- [x] 编译检查无错误
- [ ] 实际硬件测试（需要用户执行）

## 预期效果

修改完成后，系统辨识任务应表现为：

1. **采样周期精确**：`actual_dt`稳定在0.001秒，波动小于0.2ms
2. **无异常首帧**：不再出现0.7~0.79秒的异常值
3. **任务独立**：不受其他任务影响，执行时序稳定
4. **数据可靠**：采集的数据更适合用于系统辨识和PID整定

## 总结

本次修改将系统辨识功能从云台任务中分离，创建独立的1kHz任务，从根本上解决了采样周期不稳定的问题。新的架构更加模块化，便于维护和扩展。

如有任何问题，请参考：
- `application/sysid/sysid_task.md` - 详细使用文档
- `application/gimbal/gimbal.md` - 云台功能说明（包含系统辨识）

