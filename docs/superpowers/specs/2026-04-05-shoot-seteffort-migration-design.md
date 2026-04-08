# Shoot 模块迁移到 SetEffort 主线方案

## 1. 文档目的

本文档用于定义 `shoot` 模块后续迁移到 `SetEffort` 主线的专项方案。

目标不是简单把 `DJIMotorSetRef()` 全部改名成 `DJIMotorSetEffort()`，而是把 `shoot` 内部所有常规动力执行器链路的**最终控制语义**统一到：

```text
参考值
-> 控制器
-> Controller_Effort_Output(TAU_REF)
-> SetEffort()
-> adapter
-> protocol
```

这份方案明确回答：

1. 为什么 `shoot` 现在还停在 `SetRef()`
2. 摩擦轮与拨盘分别该怎么迁
3. 哪些链路属于常规主线
4. 哪些链路继续保留为兼容或扩展路径

---

## 2. 当前状态

## 2.1 当前 `shoot` 的执行器组成

`shoot` 当前包含两类动力执行器：

1. **摩擦轮**
   - `friction_bottom`
   - `friction_top_left`
   - `friction_top_right`

2. **拨盘**
   - `loader`

两者都使用 `DJI` 电机模块，但它们在应用层的控制语义并不一样。

---

## 2.2 当前摩擦轮链路

当前摩擦轮是标准的**速度闭环应用**：

```text
目标弹速
-> 目标电机转速
-> DJIMotorSetRef()
-> speed PID
-> current/raw
-> 电机
```

特点：

1. 应用层直接给的是目标转速
2. 常规控制目标是稳定转速，而不是直接调力矩
3. 当前参数仍然是旧 `SetRef/current` 语义

---

## 2.3 当前拨盘链路

拨盘当前是**角度/速度混合状态机**：

### 单发 / 三发

```text
目标角度
-> DJIMotorSetRef()
-> angle PID -> speed PID
-> 电机
```

### 连发

```text
目标速度
-> DJIMotorSetRef()
-> speed PID
-> 电机
```

### 反转 / 卡弹恢复

有的是：

- 速度反转
- 角度回退
- 自动恢复角度目标

也就是说，拨盘不是一个单纯的“恒速轮子”，而是一个状态机驱动的组合执行器。

---

## 3. 为什么 `shoot` 现在还停在 `SetRef()`

原因不是“它不用编码器反馈”，而是：

> 它当前应用层输出语义仍然是“速度参考/角度参考”，而不是统一扭矩语义。

这和 `Yaw / Pitch / Chassis` 已经完成的迁移有本质差别。

### 已完成主线的共同点

- `Yaw`：最终定义成 `TAU_REF`
- `Pitch`：最终定义成 `TAU_REF`
- `Chassis`：最终定义成 `wheel_tau_ref`

### `shoot` 当前的不同点

- 摩擦轮：上层仍给**速度参考**
- 拨盘：上层仍给**角度参考或速度参考**

所以如果直接把 `SetRef()` 替换成 `SetEffort()`，而不改变控制语义，
只会得到“接口名变了，控制含义没变”的假迁移。

---

## 4. 迁移目标

## 4.1 总目标

`shoot` 中所有**常规动力执行器路径**最终都要统一成：

```text
参考值
-> 模块内控制器
-> TAU_REF
-> SetEffort()
-> adapter
```

### 重要补充

这不意味着：

- 应用层不能再表达目标速度/目标角度

而是意味着：

- 应用层仍然可以给“速度目标/角度目标”
- 但模块层必须把它们转换成 `TAU_REF`
- 应用层不再直接依赖 `SetRef()`

---

## 4.2 常规主线与兼容路径的区分

### 迁移完成后的常规主线

- 摩擦轮常规主线：`CalculateFrictionWheelEffort -> SetEffort`
- 拨盘常规主线：`CalculateLoaderEffort -> SetEffort`

### 仍保留的兼容/特殊路径

- 系统辨识用原始注入
- 调试注入
- 必要时的绕过测试接口

但这些都不能再作为默认控制路径。

---

## 5. 推荐架构

## 5.1 新增模块定位

建议在 `Gimbal/modules/algorithm/` 下新增：

- `shoot_ref_manager.h/.c`
- 或至少 `shoot_effort_controller.h/.c`

但如果你想少加模块，也可以先只新增一个：

- `shoot_effort_controller.h`
- `shoot_effort_controller.c`

用途：

1. 统一摩擦轮与拨盘的参考解释
2. 输出统一的 `Controller_Effort_Output`
3. 保证 `ShootTask()` 最终只走 `SetEffort`

---

## 5.2 建议的层次

### 摩擦轮

```text
弹速档位 / 目标轮速
-> ShootEffortController
-> speed PID
-> TAU_REF
-> DJIMotorSetEffort()
```

### 拨盘

```text
单发 / 三发 / 连发 / 反转 / 卡弹恢复
-> ShootEffortController
-> angle/speed state controller
-> TAU_REF
-> DJIMotorSetEffort()
```

---

## 6. 摩擦轮迁移方案

## 6.1 当前问题

摩擦轮虽然本质是速度控制，但当前主线仍是：

```text
target_speed -> SetRef()
```

这不符合统一主线要求。

## 6.2 目标链路

目标应改为：

```text
target_speed_aps
-> FrictionWheelCalculateEffort()
-> TAU_REF
-> SetEffort()
```

## 6.3 实现建议

建议增加一个模块内计算函数：

```c
uint8_t ShootFrictionCalculateEffort(DJIMotorInstance *motor,
                                     float target_speed_aps,
                                     Controller_Effort_Output_s *effort);
```

其行为应为：

1. 读取实际速度反馈 `measure.speed_aps`
2. 运行速度 PID
3. 速度 PID 最终输出直接定义为 `tau_ref_nm`
4. 返回：

```c
effort->semantic = CONTROLLER_OUTPUT_TAU_REF;
effort->tau_ref_nm = ...
```

## 6.4 参数迁移

当前摩擦轮 `speed_PID` 参数明显仍是旧 current/raw 量级：

- `MaxOut = 16384`

这一步不能直接沿用。

建议：

1. 用当前电机物理参数先做一次电流域 -> 扭矩域换算
2. 再按实机重新整定

换算起点可写成：

```text
K_tau = K_current * Kt_out
MaxOut_tau = MaxOut_current * Kt_out
```

但最终必须实机重调。

## 6.5 摩擦轮迁移收益

迁移后摩擦轮就会具备：

1. 和姿态轴一致的主线接口
2. 可统一的输出语义
3. 更容易接入统一限幅与后续功率管理

---

## 7. 拨盘迁移方案

## 7.1 当前问题

拨盘不是单一控制模式，而是混合状态机：

- 单发/三发：角度目标
- 连发：速度目标
- 反转：速度反向
- 卡弹恢复：角度回退 / 状态机恢复

所以它比摩擦轮复杂。

## 7.2 目标链路

目标应改为：

```text
loader state machine
-> 角度/速度参考
-> LoaderCalculateEffort()
-> TAU_REF
-> SetEffort()
```

## 7.3 拨盘不建议一步到位强统一

拨盘建议分两级迁移：

### 阶段 1

保留现有状态机逻辑不变，只把最终输出端统一：

- 单发/三发仍先产生角度参考
- 连发仍先产生速度参考
- 最终不再 `SetRef()`
- 改成 `CalculateEffort() -> SetEffort()`

### 阶段 2

再继续思考更高层统一：

- 是否把拨盘状态机也抽出成参考仲裁层
- 是否把卡弹恢复统一成一套扭矩域逻辑

## 7.4 推荐函数

建议类似：

```c
uint8_t ShootLoaderCalculateEffort(DJIMotorInstance *motor,
                                   Closeloop_Type_e mode,
                                   float ref,
                                   Controller_Effort_Output_s *effort);
```

或者更明确：

```c
uint8_t ShootLoaderCalculateAngleEffort(...);
uint8_t ShootLoaderCalculateSpeedEffort(...);
```

我更推荐拆成两个函数，避免把语义混掉。

## 7.5 拨盘的特殊点

拨盘迁移时必须注意：

1. 当前卡弹恢复逻辑依赖角度/速度阈值判断
2. 单发目标角度有明确机械意义
3. 连发速度目标有明确节奏意义
4. 这些状态机逻辑不应因为迁移到 `SetEffort()` 被破坏

也就是说：

> 迁移的目标是“统一执行语义”，不是“抹平业务状态机”。

---

## 8. Shoot 主线最终形态

迁移完成后，`ShootTask()` 应变成：

```text
读 shoot_cmd
-> 解析模式
-> 选择摩擦轮参考 / 拨盘参考
-> 计算 effort
-> SetEffort
-> 发布反馈
```

也就是说，应用层不再直接出现：

- `DJIMotorSetRef(friction_*, ...)`
- `DJIMotorSetRef(loader, ...)`

只允许出现：

- `...CalculateEffort(...)`
- `...SetEffort(...)`

---

## 9. 建议接口边界

## 9.1 保留接口

- `DJIMotorSetEffort()`  
  作为唯一常规执行接口

## 9.2 兼容接口

- `DJIMotorSetRef()`  
  继续保留，但 `shoot` 常规主线不应再用

## 9.3 特殊接口

- `DJIMotorSetRawRef()`  
  仅用于辨识、调试或显式注入

---

## 10. 推荐实施顺序

### Step 1

先迁移摩擦轮：

1. 为三摩擦轮加 `CalculateEffort`
2. 保持 `ShootTask()` 业务逻辑不变
3. 把最终下发从 `SetRef()` 改成 `SetEffort()`

### Step 2

再迁移拨盘连发路径：

1. 先处理 `SPEED_LOOP` 场景
2. 保证连发行为不变

### Step 3

最后迁移拨盘单发/三发/反转/卡弹恢复：

1. 角度目标改成 `CalculateAngleEffort() -> SetEffort()`
2. 保持现有状态机逻辑不变

### Step 4

补文档与回归：

1. `shoot.md` 更新
2. 增加 `shoot_seteffort_mainline_regression`
3. 编译门槛纳入 `debug_build_hard_gate`

---

## 11. 风险清单

## 11.1 摩擦轮参数量纲迁移风险

速度 PID 当前参数仍在旧域，直接切主线可能导致：

- 输出过大
- 启动过冲
- 稳态抖动

必须重整定。

## 11.2 拨盘状态机行为漂移

单发/三发/连发/反转逻辑很容易在迁移后出现：

- 触发时序改变
- 角度回退不一致
- 卡弹恢复动作变慢或过冲

必须逐模式回归。

## 11.3 “接口统一但语义不统一”风险

不能只把：

```c
SetRef() -> SetEffort()
```

表面替换。

如果控制器内部仍是旧域，迁移就是假的。

---

## 12. 验证建议

## 12.1 摩擦轮

验证：

1. 启停响应
2. 稳态转速误差
3. 三摩擦轮一致性
4. 切换弹速档时是否有过冲

## 12.2 拨盘

验证：

1. 单发角度准确性
2. 三发总角度准确性
3. 连发节奏稳定性
4. 反转动作稳定性
5. 卡弹恢复能否正确结束

## 12.3 回归门槛

建议至少增加：

- `tests/shoot_seteffort_mainline_regression.ps1`
- `tests/shoot_loader_state_boundary_regression.ps1`
- 纳入 `debug_build_hard_gate.ps1`

---

## 13. 最终结论

如果最终目标是：

> 所有常规动力执行器都统一走 `SetEffort()` 主线

那么 `shoot` 也必须迁。

但迁移方式不能是“直接全局替换接口”，而应该分成：

1. **摩擦轮先迁**
2. **拨盘后迁**
3. **业务状态机保持，执行语义统一**

一句话总结：

```text
shoot 不是不该迁，
而是必须拆成“摩擦轮速度主线”和“拨盘状态机主线”两条方案来迁。
```
