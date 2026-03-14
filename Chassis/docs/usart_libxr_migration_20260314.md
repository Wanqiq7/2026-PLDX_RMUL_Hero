# Chassis USART libxr 风格迁移说明

## 迁移目标

本次迁移的目标不是直接引入 `libxr` 的 C++ 框架，而是在 `Chassis` 现有 C 架构下等价适配以下能力：

- `BLOCK / CALLBACK / POLLING` 三种读写完成语义
- `DMA_CIRCULAR + IDLE/RxEvent` 的 RX 增量提取
- RX 软件 FIFO
- TX 数据队列 + 请求队列 + 双缓冲 DMA

## 落地方式

- `bsp/usart/bsp_usart_async.[ch]`
  承担 C 版 `libxr_rw` 等价基础设施
- `bsp/usart/bsp_usart.c`
  保持 HAL 唯一落点，负责 DMA 与中断绑定
- legacy fixed-length 模块
  继续使用 `recv_buff/module_callback`
- 流式协议模块
  迁移到 `USARTRead()` / `USARTWrite()`

## 当前模块状态

- 已保留兼容：
  `remote_control`、`HC05`、`master_process`、`servo_motor`
- 已迁移为流式解析：
  `referee`

## 仍需关注

- 当前 BLOCK 机制使用任务通知完成等待与唤醒，后续如需要更强的超时/分离语义，可继续向 `libxr_rw` 的 detach/claim 状态机靠拢
- 全量构建时若出现 `WHEEL_BASE`、`REDUCTION_RATIO_LOADER` 等错误，应先判断是否为仓库既有问题，不应误判为本轮 USART 迁移引入
