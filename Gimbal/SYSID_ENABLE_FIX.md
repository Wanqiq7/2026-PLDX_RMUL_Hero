# 系统辨识任务自动启动问题修复

## 问题描述

用户反馈：一打开遥控器，还没拨到调试档位，机器人的YAW轴就自动开始执行系统辨识，导致云台异常运动。

## 问题根源

系统辨识任务（`StartSYSIDTASK`）在FreeRTOS启动后就立即开始以1kHz频率执行，无论云台是否处于辨识模式。任务没有检查云台的当前模式，导致即使在普通控制模式下也会执行辨识逻辑。

### 原始逻辑缺陷

```c
void SysIDTask(void) {
    // 获取控制指令
    SubGetMessage(sysid_sub, &sysid_cmd);

    // 首次进入时：重置数据 + 配置电机
    if (!sysid_initialized) {
        SystemID_Reset();
        SystemID_ConfigMotors();  // ❌ 直接配置电机为开环，无论模式
        sysid_initialized = 1;
        sysid_active = 1;
    }

    // ❌ 直接生成信号并控制电机，无条件执行
    sysid_data.step_input = SystemID_GenerateStepSignal();
    yaw_motor->motor_controller.pid_ref = sysid_data.step_input;
}
```

## 解决方案

在消息结构体中添加使能标志（`enable`），让云台任务通过消息中心控制系统辨识任务的启动/停止。

### 1. 修改消息结构体

在`robot_def.h`中添加`enable`字段：

```c
typedef struct {
  uint8_t enable;   // ✅ 使能标志：1-启动辨识，0-停止辨识
  float yaw_ref;    // 非辨识轴的参考位置（YAW）
  float pitch_ref;  // 非辨识轴的参考位置（PITCH）
} SysID_Ctrl_Cmd_s;
```

### 2. 云台任务发送控制指令

在`gimbal.c`的每个模式分支中发送相应的使能指令：

#### 辨识模式（启动）

```c
case GIMBAL_SYS_ID_CHIRP: {
    static SysID_Ctrl_Cmd_s sysid_cmd;
    sysid_cmd.enable = 1;  // ✅ 启动辨识
    sysid_cmd.yaw_ref = gimbal_cmd_recv.yaw;
    sysid_cmd.pitch_ref = gimbal_cmd_recv.pitch;
    PubPushMessage(sysid_pub, &sysid_cmd);
    break;
}
```

#### 其他模式（停止）

```c
case GIMBAL_GYRO_MODE: {
    // ... 正常控制逻辑 ...
    
    // ✅ 确保系统辨识任务停止
    static SysID_Ctrl_Cmd_s sysid_stop_cmd;
    sysid_stop_cmd.enable = 0;
    PubPushMessage(sysid_pub, &sysid_stop_cmd);
    break;
}
```

### 3. 系统辨识任务检查使能标志

在`sysid_task.c`中添加使能检查：

```c
void SysIDTask(void) {
    // 获取控制指令
    static SysID_Ctrl_Cmd_s sysid_cmd;
    SubGetMessage(sysid_sub, &sysid_cmd);

    // ✅ 检查是否使能辨识
    if (!sysid_cmd.enable) {
        // 如果之前在运行，现在停止了，需要重置状态
        if (sysid_initialized) {
            sysid_initialized = 0;
            sysid_active = 0;
            // 恢复电机闭环控制
            DJIMotorEnable(sysid_yaw_motor);
            DJIMotorChangeFeed(sysid_yaw_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(sysid_yaw_motor, SPEED_LOOP, OTHER_FEED);
        }
        return; // ✅ 未使能，直接返回，不执行辨识
    }

    // ✅ 只有使能后才配置电机和生成信号
    if (!sysid_initialized) {
        SystemID_Reset();
        SystemID_ConfigMotors();
        sysid_initialized = 1;
        sysid_active = 1;
    }

    // ... 后续辨识逻辑 ...
}
```

## 修改文件清单

1. **`application/robot_def.h`**
   - 在`SysID_Ctrl_Cmd_s`中添加`enable`字段

2. **`application/gimbal/gimbal.c`**
   - 在`GIMBAL_SYS_ID_CHIRP`模式：设置`enable = 1`
   - 在其他所有模式：设置`enable = 0`

3. **`application/sysid/sysid_task.c`**
   - 在`SysIDTask()`开头检查`enable`标志
   - 未使能时直接返回，不执行辨识逻辑
   - 退出辨识时恢复电机闭环控制

## 控制流程

### 正常模式（非辨识）

```
遥控器 → RobotCMD → GimbalTask
                        ↓
                   gimbal_mode = GIMBAL_GYRO_MODE
                        ↓
                   发送 enable=0 → SysIDTask
                                      ↓
                                   检测到 enable=0
                                      ↓
                                   直接返回（不执行）
```

### 辨识模式

```
遥控器 → RobotCMD → GimbalTask
                        ↓
                   gimbal_mode = GIMBAL_SYS_ID_CHIRP
                        ↓
                   发送 enable=1 → SysIDTask
                                      ↓
                                   检测到 enable=1
                                      ↓
                                   配置电机 + 生成信号
```

## 验证要点

### 1. 启动时不应自动辨识

- ✅ 上电后，遥控器未连接或处于中位，YAW轴应保持静止
- ✅ 打开遥控器，未拨到辨识档位，云台应正常响应控制

### 2. 切换到辨识模式

- ✅ 拨到辨识档位，YAW轴开始方波运动
- ✅ `actual_dt`稳定在0.001秒左右

### 3. 退出辨识模式

- ✅ 切换回普通模式，YAW轴立即停止辨识，恢复正常控制
- ✅ 电机恢复闭环控制，响应遥控器指令

### 4. 重复进入/退出

- ✅ 可以多次切换模式，系统辨识正确启动/停止
- ✅ 每次进入辨识模式，时间从0开始重新计时

## 编译验证

```bash
make -j4
# 编译成功，无错误
```

编译输出确认：
- ✅ 所有源文件编译成功
- ✅ 链接成功，生成 `basic_framework.bin`
- ✅ 代码大小：86 KB（FLASH使用率 8.24%）

## 总结

通过添加`enable`使能标志，实现了对系统辨识任务的精确控制：

1. **按需启动**：只有在辨识模式下才执行辨识逻辑
2. **自动停止**：退出辨识模式时自动停止并恢复电机控制
3. **状态管理**：正确管理初始化状态，支持重复进入/退出

修复后，系统辨识任务不会在启动时自动运行，只有在用户明确切换到辨识模式时才会激活。

## 相关文档

- `SYSID_TASK_MIGRATION.md` - 系统辨识任务迁移说明
- `application/sysid/sysid_task.md` - 系统辨识任务使用文档


