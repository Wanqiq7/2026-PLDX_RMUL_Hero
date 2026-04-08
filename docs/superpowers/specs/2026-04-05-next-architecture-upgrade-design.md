# 云台与底盘后续架构升级完整方案

## 1. 文档目的

本文档用于定义当前 `tau-ref-unified-local` 分支之后的**下一阶段架构升级方案**。

目标不是继续做零散修补，而是把已经完成的：

- `Chassis` 扭矩主线
- `Gimbal Yaw` 原生扭矩域
- `Pitch DM torque-only`
- `AutoAim` 参考主线

进一步收敛成一套长期稳定、边界清晰、可持续维护的统一架构。

本文档同时明确：

1. 哪些部分已经完成
2. 哪些部分仍是兼容层
3. 下一阶段哪些内容必须升级
4. `power_controller` 若重构，如何保证核心控制逻辑不变

---

## 2. 当前状态总结

### 2.1 已完成的架构收敛

当前分支已实现以下主线：

#### `Gimbal Yaw`

```text
手操 / 自瞄参考
-> DJIMotorCalculateEffort()
-> Controller_Effort_Output(TAU_REF)
-> DJIMotorSetEffort()
-> DJI adapter
-> raw current cmd
-> CAN
```

特点：

- `PID / LQR / SMC` 最终语义已经统一为 `TAU_REF`
- 自瞄不再直接输出电流
- 自瞄只提供 `yaw_ref_rad / yaw_rate_ff_rad_s`

#### `Gimbal Pitch`

```text
手操 / 自瞄参考
-> DMMotorCalculateTorqueEffort()
-> Controller_Effort_Output(TAU_REF)
-> DMMotorSetEffort()
-> torque-only MIT
-> CAN
```

特点：

- 常规主线已经不再走 `MIT profile/full command`
- `MIT full / PVT` 仍保留为兼容或扩展能力

#### `Chassis`

```text
速度目标
-> 底盘力控
-> wheel_tau_ref[4]
-> LegacyPowerBridge
-> power_controller(旧 current/raw 域)
-> LegacyPowerBridge
-> DJIMotorSetEffort()
-> DJI adapter
-> raw current cmd
-> CAN
```

特点：

- 主线语义已经抬到 `wheel_tau_ref`
- 旧功率控制器仍保留
- `LegacyPowerBridge` 作为兼容孤岛存在

### 2.2 当前仍然存在的兼容层

以下内容还没有彻底退出系统：

1. `SetRef()`  
   仍作为兼容接口存在，虽然常规主线已不再优先使用它。

2. `SetRawRef()`  
   当前仅应保留给辨识、调试、历史 bypass 使用。

3. `LegacyPowerBridge`  
   当前底盘功率控制器的 current/raw 域兼容桥。

4. `DM MIT full / PVT`  
   仍是 `DM` 执行器的扩展能力，但不应再充当常规控制主线。

---

## 3. 下一阶段总目标

下一阶段不再是“证明架构可行”，而是要完成**严格主线化**。

总目标定义为：

```text
参考源
-> 参考仲裁层
-> 控制器
-> Controller_Effort_Output(TAU_REF)
-> SetEffort
-> actuator adapter
-> protocol
```

对应约束如下：

1. **视觉永远只输出参考和前馈**
2. **控制器永远只输出 `TAU_REF`**
3. **常规动力执行器永远只走 `SetEffort`**
4. **`SetRef / SetRawRef / MIT full / PVT` 只能是兼容或扩展路径**
5. **`Chassis power_controller` 保持现状，不进入本轮迁移目标**

---

## 4. 下一阶段建议范围

### 4.1 必做范围

1. 把 `Gimbal` 的参考处理从 `gimbal.c` 中抽离成独立“参考仲裁层”
2. 收紧 `SetRef / SetRawRef / SetEffort` 的角色边界
3. 对 `DM` 侧建立与 `DJI` 对称的主线文档和接口约束
4. 强化回归门槛：编译通过必须是硬门槛

### 4.2 明确不做

1. **不迁移 `power_controller` 到扭矩域**
2. **不删除 `LegacyPowerBridge`**
3. **不删除 `MIT full / PVT`**
4. **不在本轮重写系统辨识链路**

---

## 5. 目标架构

## 5.1 顶层结构

建议长期结构固定为：

```text
APP
-> Ref Manager / Mode Arbiter
-> Controller
-> Controller_Effort_Output
-> SetEffort
-> Adapter
-> Protocol
-> Actuator
```

### 解释

- `APP`  
  只负责模式、状态机、消息调度。

- `Ref Manager / Mode Arbiter`  
  只负责参考的选择、融合、限幅、释放和平滑接管。

- `Controller`  
  只负责根据反馈与参考计算 `TAU_REF`。

- `SetEffort`  
  作为执行器主线统一入口。

- `Adapter`  
  只负责 `tau_ref -> protocol command`。

---

## 5.2 Gimbal 侧目标结构

建议在 `Gimbal/modules/algorithm/` 增加：

- `gimbal_ref_manager.h`
- `gimbal_ref_manager.c`

### 作用

把下面这些从 `gimbal.c` 中抽出去：

1. 手操参考
2. 自瞄参考
3. 自瞄前馈
4. 自瞄接管/释放
5. Pitch/Yaw 限位
6. 系统辨识参考接管

输出统一结构，例如：

```c
typedef struct {
  float yaw_ref_rad;
  float pitch_ref_rad;
  float yaw_rate_ff_rad_s;
  float pitch_rate_ff_rad_s;
  uint8_t vision_takeover;
} Gimbal_Ref_Output_s;
```

### 收益

这样 `gimbal.c` 只剩下：

```text
取命令
-> RefManagerStep()
-> Yaw CalculateEffort -> SetEffort
-> Pitch CalculateEffort -> SetEffort
-> 发布反馈
```

而不会继续承担“参考组装器”的责任。

---

## 5.3 Chassis 侧目标结构

`Chassis` 下一阶段不改动 `power_controller` 本体。

因此 `Chassis` 的目标不是“更纯”，而是“更稳定和更可解释”：

```text
wheel_tau_ref
-> LegacyPowerBridge
-> power_controller
-> LegacyPowerBridge
-> SetEffort
```

### 要求

1. `LegacyPowerBridge` 明确标记为兼容层
2. 不能再把 current/raw 语义泄漏到主线外侧
3. 所有新代码一律不直接接触 `power_controller` 的旧域接口

---

## 6. 接口收紧方案

## 6.1 `SetEffort`

### 定位

主线接口，长期保留。

### 要求

1. 常规动力执行器路径必须走它
2. 文档中应明确它是“唯一常规执行入口”

---

## 6.2 `SetRef`

### 定位

兼容接口，仅供历史路径使用。

### 要求

1. 不再允许新模块直接依赖它
2. 文档中明确标注为 `compatibility only`
3. 长期可考虑重命名为更直白的旧接口名，例如：
   - `SetLegacyReference()`
   - 或 `SetControllerRefCompat()`

### 本轮建议

先不重命名，先把注释和使用范围收紧。

---

## 6.3 `SetRawRef`

### 定位

bypass 接口，仅用于：

1. 系统辨识
2. 调试
3. 特殊测试注入

### 要求

1. 不得再作为常规自瞄路径使用
2. 文档中必须显式写明“不属于常规主线”

---

## 7. 自瞄链路长期目标

当前自瞄已经完成第一阶段收敛：

```text
vision -> ref / feedforward
-> unified controller
-> tau_ref
-> adapter
```

下一阶段不再改大方向，只做结构收口：

### 要保留的原则

1. 自瞄只是参考源
2. 自瞄不是执行旁路
3. 控制器类型切换不应影响视觉模块接口

### 要继续优化的点

1. 自瞄参考生成与限幅逻辑从 `vision.c` / `gimbal.c` 分离
2. 自瞄接管状态机独立化
3. 自瞄前馈参数纳入统一参考管理器

---

## 8. `power_controller` 后续建议

你已经明确：

> 功率控制器无需迁移，保持现状

因此本轮架构升级方案中：

- `power_controller` 不是迁移目标
- 但它未来仍可进行**等价重构**

### 什么叫“等价重构”

可以重构以下内容而不改变核心控制逻辑：

1. 文件拆分
2. 命名整理
3. 结构体整理
4. 单位注释补齐
5. 输入输出接口封装
6. `LegacyPowerBridge` 位置调整

### 什么不允许变

1. 功率模型公式  
   `P = τω + k1|ω| + k2τ² + k3`

2. 能量环输入输出关系

3. RLS 更新机制

4. 功率分配和限幅策略

5. 双断连和裁判/电容离线保守策略

### 如果未来要重构 `power_controller`

必须先补以下“黄金回归”：

1. `PredictPower` 等价回归
2. `EnergyLoopControl` 等价回归
3. `PowerRLSUpdate` 等价回归
4. `PowerGetLimitedOutput` 等价回归

这样才能严肃地说“重构后核心控制逻辑不变”。

---

## 9. 推荐实施顺序

### Phase 2A：参考仲裁层独立

新增：

- `Gimbal/modules/algorithm/gimbal_ref_manager.*`

完成：

1. 手操、自瞄、辨识参考统一仲裁
2. `gimbal.c` 只保留模式编排与执行调用

### Phase 2B：接口收紧

完成：

1. `SetEffort` 成为唯一常规入口
2. `SetRef` 显式降级为 compatibility
3. `SetRawRef` 显式降级为 bypass

### Phase 2C：文档和测试门槛统一

完成：

1. 所有主文档统一口径
2. 编译通过成为硬门槛
3. regex 回归只作为辅助门槛

### Phase 2D：可选的 `power_controller` 等价重构准备

注意：

这一步只做设计和回归，不做控制逻辑迁移。

---

## 10. 交付完成标准

认为下一阶段完成，至少要满足：

1. `gimbal_ref_manager` 建立
2. `Gimbal` 常规链路只走 `SetEffort`
3. `SetRef / SetRawRef` 有明确兼容标签
4. `Chassis` 仍通过 `LegacyPowerBridge` 稳定运行
5. `Gimbal` / `Chassis` 编译通过
6. 回归脚本与编译门槛一致

---

## 11. 结论

下一阶段最合理的架构升级方向不是继续堆控制器，而是：

1. 把**参考仲裁层**独立出来
2. 把**主线接口**收紧到 `SetEffort`
3. 把**兼容层**明确隔离
4. 把 `power_controller` 明确留在“后续可等价重构，但当前不迁移”的位置

也就是说，下一阶段的关键词不是：

```text
更多功能
```

而是：

```text
边界更清晰
主线更严格
兼容层更可见
测试门槛更可靠
```
