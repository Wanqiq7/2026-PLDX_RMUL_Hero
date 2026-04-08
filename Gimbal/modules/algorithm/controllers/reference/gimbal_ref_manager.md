# gimbal_ref_manager

## 作用

`gimbal_ref_manager` 用于统一云台常规控制链路中的**最小参考仲裁逻辑**。

它只负责：

1. 手操参考透传
2. 自瞄参考接管
3. 自瞄速度前馈透传
4. Pitch 参考限位
5. 输出当前是否处于视觉接管

它不负责：

1. 电机协议组帧
2. 电流/扭矩执行量计算
3. 模式切换业务状态机

因此它当前的定位是：

```text
APP 命令 / Vision 参考
-> gimbal_ref_manager
-> yaw_ref / pitch_ref / ff
-> motor CalculateEffort -> SetEffort
```

这里的关键词是“最小仲裁器”，不是“完整接管状态机”。

## 相关类型

### `Gimbal_Ref_Input_s`

输入结构，包含：

- 手操参考
- 自瞄模式标志
- 视觉接管标志
- 视觉参考
- 视觉前馈

### `Gimbal_Ref_Output_s`

输出结构，包含：

- `yaw_ref_rad`
- `pitch_ref_rad`
- `yaw_rate_ff_rad_s`
- `pitch_rate_ff_rad_s`
- `vision_takeover`

### `Gimbal_Ref_Manager_s`

当前只保存最小仲裁器所需的边沿状态：

- 上一拍是否处于视觉接管
- 上一拍输出的参考值

## 对外接口

### `GimbalRefManagerInit()`

初始化参考仲裁器实例。

### `GimbalRefManagerReset()`

重置参考仲裁器状态。

### `GimbalRefManagerStep()`

执行一拍最小参考仲裁。

输入：

- 手操参考
- 自瞄参考
- 自瞄前馈
- 接管标志

输出：

- 当前拍应该给云台控制器的参考值与前馈

## 当前策略

当前采用**最小仲裁器**策略：

1. 默认输出手操参考
2. 当 `0 -> 1` 刚进入视觉接管时：
   - 当前拍先保持上一拍参考
   - 当前拍前馈清零
3. 当 `1 -> 0` 刚退出视觉接管时：
   - 当前拍先保持上一拍参考
   - 当前拍前馈清零
4. 仅在稳定视觉接管阶段输出视觉参考与视觉前馈
5. 对输出的 `pitch_ref_rad` 做统一限位

这意味着它**只负责 enter/exit 边沿的一拍过渡**，当前并不提供：

- 持续时间上的接管平滑
- 多拍缓入/缓释
- 更完整的接管状态机

这样做的目的是先把“参考仲裁”从 `gimbal.c` 主流程中抽出来，后续若要增加：

- 更长时间的接管缓入
- 更长时间的丢目标缓释
- Yaw 特殊约束
- SYSID 特殊参考注入

都可以继续在这个模块中扩展，而不必把逻辑重新塞回应用层。  
在这些能力真正实现之前，请把当前模块理解为“最小仲裁器”，不要把它误读成完整状态机。

## 使用示例

```c
Gimbal_Ref_Input_s ref_input = {
    .manual_yaw_ref_rad = gimbal_cmd_recv.yaw,
    .manual_pitch_ref_rad = gimbal_cmd_recv.pitch,
    .autoaim_mode = (gimbal_cmd_recv.gimbal_mode == GIMBAL_AUTOAIM_MODE),
    .vision_takeover = vision_data_recv.vision_takeover,
    .vision_yaw_ref_rad = vision_data_recv.yaw_ref_rad,
    .vision_pitch_ref_rad = vision_data_recv.pitch_ref_rad,
    .vision_yaw_rate_ff_rad_s = vision_data_recv.yaw_rate_ff_rad_s,
    .vision_pitch_rate_ff_rad_s = vision_data_recv.pitch_rate_ff_rad_s,
};

Gimbal_Ref_Output_s ref_output = {0};
GimbalRefManagerStep(&gimbal_ref_manager, &ref_input, &ref_output);
```

## 注意事项

1. `gimbal_ref_manager` 当前不做执行量计算，输出语义始终是“参考值/前馈”。
2. 当前只提供 enter/exit 过渡帧，不提供持续平滑；若文档中需要更强表述，必须先补真实状态机实现。
3. 若后续加入限位或平滑策略，必须保持输出语义不变。
4. 若需要接入系统辨识参考，优先在此模块扩展，而不是重新把仲裁逻辑写回 `gimbal.c`。
