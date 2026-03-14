# UART libxr 对齐基线（2026-03-15）

## 目标

为 `Gimbal` 的 UART RX 链路冻结一套与 `libxr` 核心实现一致的基线，后续修复应优先满足这套前提，再考虑本地扩展。

## 必须对齐

- `HAL_UARTEx_ReceiveToIdle_DMA`
- `DMA_CIRCULAR`
- 使用 `__HAL_DMA_GET_COUNTER()` 计算当前 DMA 写入位置
- 在 RxEvent 中按 `last_pos -> curr_pos` 增量提取新字节
- 驱动层只负责把字节推进软件 FIFO，不在 HAL 回调内做协议解包

## 第一阶段必须消除的本地偏差

- `USART6_RX` 不再启用 `RX FIFO`
- `USART6_RX` 不再依赖 `FIFO FULL threshold`
- `vtm_input` 第一阶段不改协议接口，继续验证现有 21 字节整包读取是否恢复

## 第一阶段暂不修改

- `vtm_input` 的 21 字节协议格式
- `VTMInputParseFrame()` 的字段映射逻辑
- `robot_cmd.c` 对 `VTMInputGetData()` / `VTMInputProcess()` 的使用方式

## 第二阶段备选

如果第一阶段后 `queue_data.size` 仍无法稳定凑满 21 字节，再把 `vtm_input` 从整包等待改为流式拼帧解析。

## 结论

当前优化的唯一准绳是：

1. 先恢复 `libxr` 的 UART RX 运行前提；
2. 再验证本地 `vtm_input` 现有路径是否恢复；
3. 只有验证失败，才进入协议层重构。
