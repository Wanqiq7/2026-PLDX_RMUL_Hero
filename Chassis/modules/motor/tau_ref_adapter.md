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

- DJI 电机：`raw/current -> tau_ref -> raw current`
- DM 电机：`tau_ref -> MIT torque`

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
2. 控制器若仍输出 `current_A` 或 `raw_current_cmd`，必须通过 `DJI adapter` 内部归一为 `tau_ref`。
3. `DJI` 默认转矩常数按输出轴扭矩解释：
   `GM6020 = 0.741 N·m/A`，`M3508 = 0.3 N·m/A`，`M2006 = 0.18 N·m/A`。
4. 若 `physical_param` 不完整，`DJIMotorInit()` 会记录错误并保持电机失能，而不是运行期静默输出 0。
5. 不再维护独立的 `control_effort.h` 或 `motor_effort_normalize.*` 文件，相关概念已压缩进电机层。
6. 最终发送给电机的命令必须经过 adapter 构建。

## 示例

```c
Controller_Effort_Output_s effort = {
    .semantic = CONTROLLER_OUTPUT_RAW_CURRENT_CMD,
    .raw_current_cmd = pid_ref,
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
