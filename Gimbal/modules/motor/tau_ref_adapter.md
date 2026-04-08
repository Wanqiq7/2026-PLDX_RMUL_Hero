# tau_ref 主干与动力执行器 Adapter

本说明文档描述 `Gimbal/modules/motor/` 下新增的 `tau_ref` 主干抽象。

## 目标

将真正的动力执行器控制路径统一为：

```text
PID / LQR / SMC / legacy output
    -> 归一为 tau_ref
    -> 电机 adapter
    -> raw current / MIT torque
```

当前 `Gimbal` 已接入：

- DJI 电机主线：`TAU_REF -> raw current`
- DJI bypass：`SetRawRef() -> raw current`
- DM 电机：`tau_ref -> MIT torque`
- `SetRef()`：compatibility only
- `MIT full / PVT`：extension only

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
3. `SetRawRef()` 仅用于显式 bypass，不属于统一扭矩主线。
4. `SetRef()` 仅用于 compatibility path，不得再作为新主线入口。
4. `DJI` 默认转矩常数按输出轴扭矩解释：
   `GM6020 = 0.741 N·m/A`，`M3508 = 0.3 N·m/A`，`M2006 = 0.18 N·m/A`。
5. `DM MIT full / velocity-only / PVT` 仅用于兼容或扩展能力，不属于常规主线。
6. `SMC` 的周期回退使用 `sample_period`，并应与 `MotorTask` 真实周期一致。
7. 不再维护独立的 `control_effort.h` 或 `motor_effort_normalize.*` 文件，相关概念已压缩进电机层。
8. 最终发送给电机的命令必须经过 adapter 构建。

## 三层边界

在当前主线中，应始终保持：

```text
Ref Manager / Arbiter
-> Effort Controller / CalculateEffort
-> Adapter / Protocol
```

其中 adapter 只负责“统一努力量 -> 执行器协议命令”的翻译，
不负责参考仲裁、业务状态机或控制器求解。

## 示例

```c
Controller_Effort_Output_s effort = {
    .semantic = CONTROLLER_OUTPUT_TAU_REF,
    .tau_ref_nm = yaw_tau_ref,
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
- `DM` 的主线优先走 `MIT torque`，`MIT full / PVT` 属于扩展能力，不是统一主线默认路径。
