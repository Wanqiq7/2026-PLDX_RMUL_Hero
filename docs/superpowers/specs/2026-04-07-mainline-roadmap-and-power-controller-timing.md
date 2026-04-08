# 主线架构升级路线图与底盘功率控制升级时机

## 1. 文档目的

本文档用于回答两个问题：

1. 当前分支之后，整仓“主线架构”应该如何继续推进
2. `power_controller` 应该在什么时候升级，为什么不是现在

本文档不是实现计划，而是一个**阶段路线图**。  
目的是把当前已经完成的收敛工作，放进一个长期一致的架构节奏里。

---

## 2. 当前主线状态

当前 `tau-ref-unified-local` 分支已经完成的核心收敛如下：

### 2.1 Gimbal

#### Yaw

```text
手操 / 自瞄参考
-> DJIMotorCalculateEffort()
-> Controller_Effort_Output(TAU_REF)
-> DJIMotorSetEffort()
-> adapter
-> protocol
```

#### Pitch

```text
手操 / 自瞄参考
-> DMMotorCalculateTorqueEffort()
-> Controller_Effort_Output(TAU_REF)
-> DMMotorSetEffort()
-> torque-only MIT
```

#### AutoAim

```text
vision -> ref / feedforward
-> gimbal
-> CalculateEffort
-> SetEffort
```

说明：

- 自瞄已经不是电流旁路
- 视觉只提供参考和前馈
- 常规主线已经收敛到 `SetEffort`

### 2.2 Chassis

```text
wheel_tau_ref
-> LegacyPowerBridge
-> power_controller (旧域)
-> LegacyPowerBridge
-> DJIMotorSetEffort()
```

说明：

- 底盘外层主线已经是扭矩语义
- 但中间仍然保留 `LegacyPowerBridge + power_controller` 兼容孤岛

### 2.3 Shoot

#### Gimbal shoot

已基本迁移到：

```text
摩擦轮/拨盘参考
-> shoot_effort_controller
-> DJIMotorCalculateEffort()
-> TAU_REF
-> DJIMotorSetEffort()
```

#### Chassis shoot

当前方向已经正确：

- 应用层接口已迁到 `SetEffort`
- 需要确认 `Chassis DJI CalculateEffort()` 与主线语义完全一致

---

## 3. 现阶段真正的主问题

当前不再是“是否要统一主线”，而是：

> 如何把剩余兼容层、旧接口和孤岛控制在可接受范围内，防止架构反复污染。

具体来说，剩下的问题可以分成三类：

### 3.1 主线层问题

需要继续统一：

- `SetEffort` 成为常规执行器唯一入口
- `SetRef` 继续收缩为 compatibility only
- `SetRawRef` 继续收缩为 bypass only

### 3.2 模块边界问题

需要继续收紧：

- `gimbal_ref_manager` 的真实定位
- `shoot_effort_controller` 的边界
- `LegacyPowerBridge` 的兼容层边界

### 3.3 功率控制问题

当前不是算法错误，而是阶段问题：

- `power_controller` 仍保留旧域接口
- 它已经与主线产生一个清晰可见的兼容边界
- 这意味着它现在**可以暂时不动**

---

## 4. 推荐的主线升级路线

## Phase 1：当前分支收口

### 目标

把当前已经推进的内容收成“可稳定落地的第一阶段架构”。

### 要求

1. `Gimbal` 常规链路全部固定到 `SetEffort`
2. `Shoot` 常规链路全部固定到 `SetEffort`
3. `Chassis` 常规链路保持 `wheel_tau_ref -> bridge -> SetEffort`
4. 文档口径全部一致
5. `debug_build_hard_gate` 成为可信门槛

### 判断标准

若满足以下条件，则认为 Phase 1 完成：

- 常规主线不再直接调用 `SetRef`
- 自瞄不再直接下电流
- `shoot` 常规路径不再用 `SetRef`
- `power_controller` 仍是唯一旧域孤岛

---

## Phase 2：严格主线化

### 目标

进一步完成“主线/兼容/扩展”的制度化隔离。

### 核心动作

1. 明确规定：
   - `SetEffort` = 主线
   - `SetRef` = compatibility
   - `SetRawRef` = bypass

2. 给各模块补上明确白名单：
   - 哪些文件还允许 `SetRef`
   - 哪些文件还允许 `SetRawRef`

3. 强化测试：
   - 脚本不能只查“有没有 SetEffort”
   - 要查“主线有没有误回到 SetRef”

### 阶段价值

这一步不会立即改变控制性能，
但会显著降低后续维护时重新把旧接口用回来的概率。

---

## Phase 3：功率控制升级前准备

### 目标

在不重构 `power_controller` 的前提下，为未来升级建立完整前提。

### 必须完成的准备

1. 固化 `LegacyPowerBridge` 边界
2. 固化 `power_controller` 当前输入输出语义
3. 建立黄金回归基线：
   - `PredictPower`
   - `EnergyLoopControl`
   - `RLSUpdate`
   - `PowerGetLimitedOutput`

### 为什么要先做准备

因为 `power_controller` 一旦升级，不只是接口改动，而是会影响：

- 底盘功率分配
- 电容能量环
- RLS 参数辨识
- 限幅曲线

如果没有黄金回归，你没法严格证明“逻辑没变”。

---

## Phase 4：底盘功率控制升级

这一步才是正式进入 `power_controller` 升级的时机。

---

## 5. 底盘功率控制应该什么时候升级

我的明确建议是：

> **不要在当前阶段升级。**

应该在 **Phase 1 和 Phase 2 完成之后，再进入 Phase 3 准备，最后才做 Phase 4 升级。**

也就是说：

```text
先把常规执行主线完全稳定
-> 再冻结兼容边界
-> 再补功率控制黄金回归
-> 最后升级 power_controller
```

---

## 6. 为什么不是现在

原因有四个。

### 6.1 当前系统仍在主线收敛期

现在还在处理：

- `shoot` 主线收口
- `gimbal_ref_manager` 定位
- `SetRef / SetEffort / SetRawRef` 的接口制度化

如果这时再动 `power_controller`，你会把两类变化叠在一起：

1. 主线层变化
2. 功率层变化

一旦实机行为异常，很难定位问题来自哪一层。

### 6.2 当前 `LegacyPowerBridge` 已经足够稳定

现在底盘外层主线已经是扭矩域，  
`power_controller` 虽然还是旧域，但它已经被桥包住了。

也就是说：

- 架构上你已经“把脏东西隔离起来了”
- 只要桥边界不继续外泄，它暂时不会阻碍主线继续收口

### 6.3 当前没有功率控制等价回归

如果现在直接重构 `power_controller`，
你几乎无法证明下面这些没有变化：

- 限功率策略
- 能量环策略
- RLS 收敛路径
- 双断连保护

这会让你在调试时陷入“代码更优雅了，但行为不确定”的危险状态。

### 6.4 当前优先级更高的是接口治理

从架构收益看，当前最值钱的不是重写功率控制器，
而是把整仓“常规主线”和“兼容主线”的边界彻底写死。

因为一旦这一步没完成，
后面重构再多模块，旧语义还是会不断回流。

---

## 7. 什么时候可以开始升级 `power_controller`

建议满足以下 **4 个前置条件** 后再开始：

### 条件 1：常规动力执行器全部稳定在 `SetEffort`

包括：

- `Gimbal Yaw`
- `Gimbal Pitch`
- `Chassis`
- `Shoot`

且这些主线的接口边界已经稳定一段时间，不再频繁改。

### 条件 2：兼容接口已经制度化

即：

- `SetRef` 只在明确白名单中存在
- `SetRawRef` 只在辨识/调试白名单中存在
- `LegacyPowerBridge` 已被显式标注为兼容孤岛

### 条件 3：黄金回归基线已建立

至少要有：

- `power_predict_equivalence`
- `energy_loop_equivalence`
- `rls_equivalence`
- `limited_output_equivalence`

### 条件 4：当前主线已有一轮实机稳定期

建议至少经过：

- 一轮台架验证
- 一轮实机联调

确认主线问题已经主要收敛，不再大幅变动。

---

## 8. 升级 `power_controller` 时的正确目标

升级它的目标不应是“换个更好看的接口”，而应是：

```text
保持核心控制逻辑等价
把输入输出语义迁到扭矩域
去掉 LegacyPowerBridge
```

所以最终目标应是：

```text
wheel_tau_ref
-> power_controller (native tau domain)
-> limited wheel_tau_ref
-> DJIMotorSetEffort()
```

而不是：

```text
把整个功率控制逻辑重写一遍
```

---

## 9. 推荐时间点

如果用“工程里程碑”来表达，我建议：

### 现在

继续做：

- 主线收口
- 兼容接口治理
- 文档/测试门槛统一

### 下一阶段

做：

- `power_controller` 黄金回归准备
- `LegacyPowerBridge` 边界冻结

### 再下一阶段

才正式开始：

- `power_controller` 扭矩域升级

换句话说：

> **它应该是“下一阶段之后”的工作，而不是当前阶段的工作。**

---

## 10. 最终建议

### 当前主线方案

你接下来应该继续推进的是：

1. 常规动力主线全部统一为 `SetEffort`
2. 兼容接口明确降级
3. 模块职责进一步收紧
4. 编译与回归门槛固定下来

### `power_controller` 升级时机

我的建议非常明确：

> **等主线稳定后再做，不是现在。**

更具体一点：

> **在“主线完全收口 + 兼容层白名单固定 + 功率控制黄金回归建立”之后，再进入底盘功率控制升级。**

这时升级，收益最大，风险最小，也最容易证明“控制逻辑没有被改坏”。

---

## 11. Phase 3 基线现状补记

截至 `2026-04-07`，`tau-ref-unified-local` 已补齐 Phase 3 所需的样本级黄金回归基线：

- `PredictPower`：已冻结代数输出样本。
- `EnergyLoopControl`：已冻结关键在线/离线状态分支及等级回退样本。
- `PowerRLSUpdate`：已冻结低功率守卫、单步更新与多步序列轨迹终点。
- `PowerGetLimitedOutput`：已冻结直通、mixed、error、负功率补偿与历史断连语义样本。

当前阶段的结论保持不变：

> Phase 3 的职责是“冻结行为”，不是“修正实现”或“提前进入扭矩域重构”。
