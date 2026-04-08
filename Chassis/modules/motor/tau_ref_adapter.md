# tau_ref 主干与动力执行器 Adapter

本说明文档描述 `Chassis/modules/motor/` 下新增的 `tau_ref` 主干抽象。

## 目标

将真正的动力执行器控制路径统一为：

```text
PID / LQR / SMC / legacy output
    -> 归一为 tau_ref
    -> 电机 adapter
    -> raw current / MIT torque
```

当前 `Chassis` 已接入：

- DJI 电机主线：`TAU_REF -> raw current`
- 底盘功率控制：`wheel_tau_ref -> power_controller(native tau) -> limited wheel_tau_ref`
- DM 电机：`tau_ref -> MIT torque`
- `SetRef()`：compatibility only

## 相关文件

- `motor_def.h`
  - 统一控制器输出语义与物理参数
  - `tau_ref` 明确表示 **output_shaft_torque（输出轴扭矩）**
- `DJImotor/dji_motor_adapter.h/.c`
  - `raw/current -> output_shaft_torque -> DJI raw current`
- `DMmotor/dmmotor_adapter.h/.c`
  - `tau_ref -> DM MIT torque`

## 使用原则

1. `app` 层不应该直接拼协议帧。
2. 新主线中，控制器应尽量直接输出 `TAU_REF`。
3. `Chassis` 底盘功率控制已经直接停留在 `TAU_REF` 语义，current/raw 不再是应用层主线语义。
4. `SetRef()` 仅用于 compatibility path，不得再作为新主线入口。
5. `DJI` 默认转矩常数按输出轴扭矩解释：
   `GM6020 = 0.741 N·m/A`，`M3508 = 0.3 N·m/A`，`M2006 = 0.18 N·m/A`。
6. `power_controller` 输入输出语义已经收敛到 native tau domain，桥接旧域语义不再允许回流到应用层。
7. 若 `physical_param` 不完整，`DJIMotorInit()` 会记录错误并保持电机失能，而不是运行期静默输出 0。
8. 不再维护独立的 `control_effort.h` 或 `motor_effort_normalize.*` 文件，相关概念已压缩进电机层。
9. 最终发送给电机的命令必须经过 adapter 构建。

## Chassis 底盘功率控制主链路

`Chassis` 当前推荐保持：

```text
wheel_tau_ref
-> power_controller (native tau domain)
-> limited wheel_tau_ref
-> DJIMotorSetEffort()
```

也就是说，`ChassisTask()` 只组织轮端 `TAU_REF` 与限幅后的 `TAU_REF`，
不再承载旧 current/raw 域换算逻辑。

若需要回顾历史兼容方案，应仅在设计文档中说明，不应在代码路径中重新恢复桥接 helper。

## 示例

```c
Controller_Effort_Output_s effort = {
    .semantic = CONTROLLER_OUTPUT_TAU_REF,
    .tau_ref_nm = wheel_tau_ref[i],
};

Controller_Effort_Output_s normalized = {0};
int16_t raw_cmd = 0;

DJIMotorBuildRawCommandFromEffort(&motor->physical_param,
                                  &effort,
                                  &raw_cmd,
                                  &normalized);
```

## 注意事项

- `tau_ref` 单位为 N·m，且表示 **输出轴扭矩**。
- `current_ref_a` 单位为 A。
- `raw_current_cmd` 单位为电调协议原始量。
- 若物理参数未知，不允许在 adapter 中偷偷猜值，必须通过 `Motor_Physical_Param_s` 显式配置或使用已验证默认值。
