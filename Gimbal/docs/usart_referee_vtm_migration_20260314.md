# Gimbal USART / Referee / VTM 迁移说明

## 迁移目标

本次迁移在 `Gimbal` 现有 C 架构下等价适配以下能力：

- `BLOCK / CALLBACK / POLLING` 三种读写完成语义
- `DMA_CIRCULAR + IDLE/RxEvent` 的 RX 增量提取
- RX 软件 FIFO
- TX 数据队列 + 请求队列 + 双缓冲 DMA

## 与 Chassis 的差异

- `Gimbal` 的 legacy USART 回调原本就带 `size` 参数，因此兼容层继续保留 `module_callback(size)` 语义
- `Gimbal` 的图传链路输入 `vtm_input` 当前已改为显式 `USARTRead(..., POLLING)` 路径，不再走 legacy fixed-length callback
- `Gimbal` 的裁判模块重点承接 `0x0309 / 0x0310 / 0x0311` 图传链路命令，因此流式解析要覆盖这些扩展命令

## 当前模块状态

- 已保留兼容：
  `remote_control`、`HC05`、`servo_motor`
- 已迁移为流式解析：
  `referee`
- 已迁移为显式 read + polling：
  `vtm_input`

## 仍需关注

- `USART6_RX` 的运行前提需与 libxr 对齐：保留 `DMA_CIRCULAR + ReceiveToIdle + NDTR`，但不启用 RX FIFO
- 当前 BLOCK 机制使用任务通知完成等待与唤醒，后续如需要更强的超时/分离语义，可继续向 `libxr_rw` 的 detach/claim 状态机靠拢
- 全量构建时若出现与 `application` 业务代码有关的错误，应先判断是否为仓库既有问题，不应误判为本轮 USART/Referee/VTM 迁移引入
