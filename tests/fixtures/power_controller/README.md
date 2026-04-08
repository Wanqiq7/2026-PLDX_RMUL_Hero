# Power Controller Golden Samples

本目录用于冻结 `power_controller` 在 Phase 3 的“行为样本级”黄金回归基线。

## 样本契约

每个样本至少包含以下字段：

- `sample_id`：样本唯一标识。
- `purpose`：该样本覆盖的行为目的。
- `source`：样本来源。
- `input`：单步输入，或配合 `pre_steps` / `steps` 表示状态演化。
- `expected_output`：输出向量类结果。
- `expected_state`：状态快照类结果。
- `tolerance`：逐字段容差。
- `notes`：补充说明。

## 可选字段

- `pre_steps`：在最终输入前执行的预置步骤，用于构造最后一级状态。
- `steps`：多步样本序列，主要用于 RLS 连续更新轨迹。
- `input.config`：覆盖默认 `PowerControllerConfig_t`。
- `input.motor_disconnect_cycles`：模拟 `PowerUpdateMotorOnline(..., 0)` 的累计调用次数。

## 默认配置

若样本未显式覆盖 `input.config`，默认配置为：

```json
{
  "k1_init": 0.22,
  "k2_init": 1.2,
  "k3": 5.1,
  "rls_lambda": 0.98,
  "torque_constant": 1.0,
  "current_scale": 0.001,
  "robot_division": "infantry"
}
```

## 样本来源约束

每个样本的 `source.kind` 必须属于以下三类之一：

1. `baseline-current-implementation`：来自当前 `power_controller` 行为镜像的基线冻结值。
2. `bench-or-log-replay`：来自既有台架或实机日志回放。
3. `synthetic-boundary`：明确标注的人工边界样本。

禁止脱离实现与日志凭空填写期望值。

## Harness 约束

- `tests/harnesses/power_controller_sample_harness.*` 只定义样本适配层，不改控制逻辑。
- harness 只依赖 `power_controller` public API 和 `PowerGetStatus()`。
- 对于单实例静态实现，建议未来 host runner 按“单进程单样本”执行，确保样本间状态隔离。
