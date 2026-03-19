# bsp_usart

> 当前版本的 USART BSP 已从“单次 DMA 接收 + 直接 `recv_buff` 解包”升级为“环形 DMA RX + 软件 FIFO + TX 队列/双缓冲”的异步架构，同时保留对 legacy fixed-length 模块的兼容层。对于 `Gimbal`，legacy 回调仍保留 `size` 参数，便于兼容 `vtm_input` 这类固定帧图传输入模块。

## 总体说明

- RX 架构：
  `DMA_CIRCULAR + HAL_UARTEx_ReceiveToIdle_DMA + NDTR 增量提取 + 软件 FIFO`
- RX 对齐基线：
  与 `libxr` 保持一致，RX 侧使用 direct mode，不启用 DMA FIFO
- TX 架构：
  `数据 FIFO + 请求元信息队列 + 双缓冲 DMA + TxCplt handoff`
- 兼容策略：
  旧模块仍可通过 `recv_buff + module_callback(size)` 工作，但这条路径只适用于**固定帧**协议。
- 推荐路径：
  新模块应使用 `USARTRead()` / `USARTWrite()` 与 `BLOCK/CALLBACK/POLLING` 操作模型，不再依赖直接访问 `recv_buff`。
  若协议层需要自行做流式拼帧，应使用只消费当前 FIFO 的非挂起接口，而不是对局部变量发起 pending read。

## 分层约束

- HAL 调用只允许出现在 `bsp/usart/bsp_usart.c`
- Module 层只能通过 `USARTRegister/USARTRead/USARTWrite/USARTSend/USARTServiceInit` 等 BSP 接口使用串口
- 禁止在临界区中延时
- RX/TX 共享状态必须使用 ISR-safe 机制保护

## 代码结构

- [bsp_usart.h](D:/RoboMaster/HeroCode/Code/Gimbal/bsp/usart/bsp_usart.h)
  对外公开接口、状态枚举、操作对象、实例定义与 legacy 包装声明
- [bsp_usart_async.h](D:/RoboMaster/HeroCode/Code/Gimbal/bsp/usart/bsp_usart_async.h)
  C 版 `libxr_rw` 等价基础设施声明：FIFO、请求队列、双缓冲、读写端口
- [bsp_usart_async.c](D:/RoboMaster/HeroCode/Code/Gimbal/bsp/usart/bsp_usart_async.c)
  队列、端口、operation、pending 完成路径等纯软件逻辑
- [bsp_usart.c](D:/RoboMaster/HeroCode/Code/Gimbal/bsp/usart/bsp_usart.c)
  HAL 绑定、DMA 启停、RxEvent 增量提取、legacy fixed-length 兼容分发、TX kick/完成回调

## 类型与接口

### 状态与操作类型

- `USART_Status_e`
  串口读写操作的统一返回状态，覆盖 `OK/PENDING/BUSY/FULL/EMPTY/TIMEOUT/FAILED`
- `USART_Operation_Type_e`
  对齐 `libxr_rw` 的三种模式：
  `BLOCK / CALLBACK / POLLING`
- `USART_Polling_Status_e`
  轮询模式状态：
  `READY / RUNNING / DONE / ERROR`

### 操作对象

`USART_Operation_s` 用于描述一次读/写操作的完成方式：

- `CALLBACK`
  完成时调用 `usart_operation_callback`
- `BLOCK`
  任务上下文阻塞等待完成
- `POLLING`
  外部通过状态变量轮询完成结果

### 实例与兼容边界

`USARTInstance` 同时包含两套语义：

- legacy 语义：
  `recv_buff`、`rx_data_len`、`recv_buff_size`、`module_callback(size)`
- 新架构语义：
  `owner_id`、`rx_fifo_size`、`tx_fifo_size`、`tx_queue_depth`、`read_port`、`write_port`、`driver_context`

其中：

- `owner_id` 参考 `bsp_can` 的 parent pointer 设计
- `read_port` / `write_port` 是唯一推荐的新接口入口
- `driver_context` 是 BSP 私有上下文，Module 层禁止访问

## RX 工作流程

1. `USARTRegister()` 初始化实例、软件 FIFO、TX 双缓冲和驱动上下文
2. `USARTServiceInit()` 将 `hdmarx` 设为 `DMA_CIRCULAR`，并启动一次 `HAL_UARTEx_ReceiveToIdle_DMA`
3. `HAL_UARTEx_RxEventCallback()` 通过 `NDTR` 计算当前 DMA 写指针
4. 根据 `curr_pos` 和 `last_pos` 提取线性区或回卷区新字节
5. 新字节被推入 `read_port` 的软件 FIFO
6. 若该实例仍在使用 legacy fixed-length 模式，BSP 会在 FIFO 中凑满 `recv_buff_size` 后复制到 `recv_buff` 并以 `recv_buff_size` 触发 `module_callback`
7. 新模块则通过 `USARTRead()` 主动消费 FIFO 中的数据

## TX 工作流程

1. `USARTWrite()` 将数据写入 TX 数据 FIFO，并把操作信息写入请求队列
2. `USARTWritePortKick()` 在 UART 空闲时装载 active buffer 并启动 DMA 发送
3. 若发送过程中有后续数据进入，则预装入 pending buffer
4. `HAL_UART_TxCpltCallback()` 完成 active request，并切换/推进下一帧发送

## legacy 兼容策略

当前仍可直接工作的模块类型：

- `remote_control`
- `HC05`
- `servo_motor`
- `vtm_input`

兼容前提：

- 协议是固定帧
- 初始化时必须把 `USART_Init_Config_s` **零初始化**
- 只依赖 `recv_buff` 和 `module_callback(size)`

不再推荐的旧假设：

- “每收到一包就重挂一次 DMA”
- “TX DMA 会天然打断 RX DMA，因此必须用 IT 发送”
- “在 UART 回调里直接做复杂协议解析”

## 推荐使用方式

### fixed-length 旧模块

```c
USART_Init_Config_s conf;
memset(&conf, 0, sizeof(conf));
conf.usart_handle = &huart1;
conf.recv_buff_size = 21U;
conf.module_callback = VTMInputRxCallback;
USARTRegister(&conf);
```

### 新模块使用 POLLING 读

```c
USART_Operation_s op;
volatile USART_Polling_Status_e status;

USARTOperationInitPolling(&op, &status);
if (USARTRead(usart_instance, recv_buf, expect_len, &op, 0U) == USART_STATUS_PENDING) {
    if (status == USART_POLLING_DONE) {
        // 任务上下文处理收到的数据
    }
}
```

### 新模块使用当前 FIFO 的非挂起消费接口

```c
uint8_t data[16];
uint16_t actual_size = 0U;

if (USARTReadAvailable(usart_instance, data, sizeof(data), &actual_size, 0U) ==
        USART_STATUS_OK &&
    actual_size > 0U) {
    // 任务上下文消费当前 FIFO 中已有字节
}
```

### 新模块发送

```c
USART_Operation_s op;
USARTOperationInitNone(&op);
USARTWrite(usart_instance, send_buf, send_len, &op, 0U);
```

## 模块迁移建议

- fixed-length 模块：
  短期保留 legacy callback，先把 `USART_Init_Config_s` 改为零初始化
- 图传官方输入：
  当前 `vtm_input` 已升级为流式拼帧解析，协议层使用持久化 parser context 自行找 `A9 53`、累积 21 字节并校验 CRC
- 裁判系统图传链路：
  已迁移为任务上下文中的流式状态机，不再在 UART ISR 内解包

## 调试与验收

最低验收建议：

1. 单独编译 `bsp_usart_async.o` 与 `bsp_usart.o`
2. 编译依赖 legacy 兼容层的固定帧模块对象
3. 编译 `rm_referee.o`、`referee_task.o` 与 `vtm_input.o`
4. 全量构建时，区分 USART 迁移新增问题与仓库既有问题

重点检查项：

- `HAL` 调用未外溢到 Module/APP
- 临界区中无延时
- FIFO/队列满时有安全检查或告警
- `USART_Init_Config_s` 的栈变量使用前均已清零
