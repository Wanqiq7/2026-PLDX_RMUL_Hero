# 功率控制器快速启动指南

## ✅ 已完成的集成

### 1. 模块文件 ✓
- ✅ `modules/power_controller/power_controller.h` - 接口定义
- ✅ `modules/power_controller/power_controller.c` - 核心实现
- ✅ `modules/power_controller/power_controller.md` - 完整文档
- ✅ `modules/power_controller/QUICK_START.md` - 本文件

### 2. 底盘集成 ✓
- ✅ `application/chassis/chassis.c` 已重构
  - 包含 `power_controller.h`
  - 调用 `PowerControllerInit()` 初始化
  - 使用 `PowerGetLimitedOutput()` 获取限制输出
  - 更新数据到功率控制器

### 3. 任务系统集成 ✓
- ✅ `application/robot_task.h` 已添加
  - `osThreadId powerTaskHandle` 任务句柄
  - `void StartPOWERTASK()` 任务函数
  - 在 `OSTaskInit()` 中创建任务（仅底盘板）
  - 任务周期：5ms (200Hz)
  - 任务优先级：osPriorityNormal

---

## 🎯 系统架构

```
启动流程：
  RobotInit() 
    └─> ChassisInit()
          └─> PowerControllerInit()  // 初始化功率控制器

任务运行：
  StartROBOTTASK (500Hz, 高优先级)     StartPOWERTASK (200Hz, 普通优先级)
       │                                    │
       ├─> ChassisTask()                   └─> PowerControllerTask()
       │     │                                    ├─> EnergyLoopControl()
       │     ├─> PowerUpdateXXXData()             └─> RLSUpdate()
       │     ├─> PowerGetLimitedOutput()
       │     └─> DJIMotorSetRef()
       │
       └─> GimbalTask() / ShootTask()
```

---

## 🚀 验证步骤

### 第1步：编译项目

```bash
# 清理并重新编译
make clean
make -j8
```

**预期结果**：编译成功，无错误

---

### 第2步：查看日志输出

上电后，通过串口/日志查看任务启动信息：

```
[freeRTOS] INS Task Start
[freeRTOS] MOTOR Task Start
[freeRTOS] Daemon Task Start
[freeRTOS] ROBOT core Task Start
[freeRTOS] UI Task Start
[freeRTOS] System Identification Task Start
[freeRTOS] Power Controller Task Start  ← 应该看到这行！
```

**如果没有看到**：检查是否定义了 `ONE_BOARD` 或 `CHASSIS_BOARD` 宏

---

### 第3步：测试功率控制

#### 测试A：基本功能测试

1. **连接裁判系统**
   - 确保裁判系统连接正常
   - 功率限制会根据机器人等级自动设置

2. **观察功率限制**
   ```c
   // 在ChassisTask()中添加调试代码
   const PowerControllerStatus_t* status = PowerGetStatus();
   printf("Power: %.1fW, Limit: %.1fW\n", 
          status->estimated_power, 
          status->max_power_limit);
   ```

3. **预期现象**：
   - 功率限制根据裁判系统数据动态调整
   - 加速时功率接近上限
   - 匀速时功率维持稳定

#### 测试B：RLS参数辨识测试

```c
// 在ChassisTask()中添加
const PowerControllerStatus_t* status = PowerGetStatus();
printf("k1=%.3f, k2=%.3f\n", status->k1, status->k2);
```

**预期收敛过程**：
- **初始值**：k1=0.220, k2=1.200
- **3-5秒后**：参数开始变化
- **10-20秒后**：参数趋于稳定
- **稳定值**：k1约在0.15-0.30，k2约在0.8-1.5（取决于实际电机）

#### 测试C：能量环控制测试

1. **不接超级电容**
   ```c
   printf("Cap Online: %d, Energy: %.1fJ\n", 
          status->cap_online, 
          status->energy_feedback);
   ```
   - 应显示 `Cap Online: 0`
   - 能量反馈来自裁判系统 (0-60J)

2. **接入超级电容**
   - 应显示 `Cap Online: 1`
   - 能量反馈来自超级电容 (0-2100J)
   - 功率上限增加约300W

---

## 🔧 参数调优

### 情况1：功率控制过于激进（经常超功率）

**原因**：PD控制器响应太慢

**解决方案**：增大Kp
```c
// 在power_controller.h中
#define POWER_PD_KP 70.0f  // 从50增到70
```

### 情况2：功率控制过于保守（性能损失大）

**原因**：PD控制器响应太快

**解决方案**：减小Kp
```c
#define POWER_PD_KP 35.0f  // 从50降到35
```

### 情况3：RLS参数发散

**原因**：遗忘因子太小

**解决方案**：增大lambda
```c
// 在chassis.c的PowerControllerInit()中
.rls_lambda = 0.99995f,  // 从0.9999增到0.99995
```

### 情况4：底盘响应有震荡

**原因**：功率分配阈值不合适

**解决方案**：调整阈值
```c
// 在power_controller.h中
#define ERROR_POWER_DISTRIBUTION_THRESHOLD 25.0f  // 从20增到25
#define PROP_POWER_DISTRIBUTION_THRESHOLD 18.0f   // 从15增到18
```

---

## 📊 性能监控

### 推荐监控变量

```c
const PowerControllerStatus_t* status = PowerGetStatus();

// 1. 功率相关
float estimated_power = status->estimated_power;     // 估算功率
float max_power = status->max_power_limit;           // 当前功率限制
float sum_cmd_power = status->sum_cmd_power;         // 指令功率

// 2. RLS参数
float k1 = status->k1;  // 转速损耗系数
float k2 = status->k2;  // 力矩损耗系数

// 3. 能量环
float energy = status->energy_feedback;    // 当前能量
uint8_t cap_online = status->cap_online;   // 电容在线状态

// 4. 功率限制范围
float power_upper = status->power_upper;   // 功率上限
float power_lower = status->power_lower;   // 功率下限
```

### 典型值参考

| 变量 | 静止 | 匀速 | 加速 | 单位 |
|-----|------|------|------|------|
| estimated_power | 10-20 | 30-50 | 60-100 | W |
| k1 | 0.20-0.25 | 0.20-0.25 | 0.20-0.25 | - |
| k2 | 1.0-1.3 | 1.0-1.3 | 1.0-1.3 | - |
| energy_feedback | 50-60 | 45-55 | 30-50 | J |

---

## ⚠️ 常见问题排查

### 问题1：编译错误 "undefined reference to PowerControllerInit"

**原因**：未添加power_controller.c到编译系统

**解决**：
```cmake
# 在CMakeLists.txt中添加
file(GLOB_RECURSE POWER_CONTROLLER_SOURCES 
     "modules/power_controller/*.c")
```

### 问题2：任务未启动（日志中没有Power Controller Task Start）

**原因**：宏定义问题

**检查**：
1. 确认在 `robot_def.h` 中定义了 `ONE_BOARD` 或 `CHASSIS_BOARD`
2. 重新编译

### 问题3：功率始终不限制

**原因**：POWER_CONTROLLER_ENABLE未定义

**解决**：
```c
// 在power_controller.h中确认
#define POWER_CONTROLLER_ENABLE 1
```

### 问题4：RLS参数一直是初始值

**原因**：裁判系统数据未更新

**检查**：
1. 裁判系统是否连接
2. `PowerUpdateRefereeData()` 是否被调用
3. `referee_data` 指针是否有效

### 问题5：程序卡死/Hard Fault

**原因**：栈空间不足

**解决**：
```c
// 在robot_task.h中增加栈空间
osThreadDef(powertask, StartPOWERTASK, osPriorityNormal, 0, 512);  // 从256增到512
```

---

## 🎓 进阶调试

### 使用RTT/ITM实时监控

```c
// 在StartPOWERTASK()中添加
for (;;) {
    power_start = DWT_GetTimeline_ms();
    PowerControllerTask();
    power_dt = DWT_GetTimeline_ms() - power_start;
    
    // 实时输出
    const PowerControllerStatus_t* status = PowerGetStatus();
    SEGGER_RTT_printf(0, "Power: %.1f/%.1f, k1=%.3f, k2=%.3f\n",
                      status->estimated_power,
                      status->max_power_limit,
                      status->k1,
                      status->k2);
    
    osDelay(5);
}
```

### 使用vofa+可视化

```c
// 发送格式：float1,float2,float3,...,floatN,\n
void SendToVofa(void) {
    const PowerControllerStatus_t* status = PowerGetStatus();
    printf("%.2f,%.2f,%.3f,%.3f,%.1f\n",
           status->estimated_power,
           status->max_power_limit,
           status->k1,
           status->k2,
           status->energy_feedback);
}
```

---

## 📞 技术支持

如遇到问题：
1. 查看 `modules/power_controller/power_controller.md` 完整文档
2. 检查本文档的常见问题排查部分
3. 使用日志和监控工具定位问题

---

**祝调试顺利！** 🎉

