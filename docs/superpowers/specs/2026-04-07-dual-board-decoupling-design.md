# 双板专用职责裁剪设计

## 背景

当前 `.worktrees/tau-ref-unified-local` 下的 `Gimbal` 与 `Chassis` 虽然已经分别定义为 `GIMBAL_BOARD` 与 `CHASSIS_BOARD`，但仍然保留了原始 `basic_framework` 的单板/双板混合兼容结构，主要表现为：

- 两个工程都通过递归方式收集 `application/*.c`，导致对侧板应用目录仍处于构建面之内。
- 云台工程仍保留底盘执行层代码，例如 `[Gimbal/application/chassis/chassis.c]` 的 `ChassisInit/ChassisTask` 与底盘运动解算。
- 底盘工程仍保留 `cmd/gimbal/shoot` 执行层代码，例如 `[Chassis/application/cmd/robot_cmd.c]`、`[Chassis/application/shoot/shoot.c]`。
- `ONE_BOARD`、`GIMBAL_BOARD`、`CHASSIS_BOARD` 三套条件编译同时存在，造成职责边界模糊、死代码增多、后续维护成本高。

本次目标不是重写框架，而是把两个工程收敛为“严格双板专用”架构：保留必要的跨板协议与链路层，删除不再承担职责的执行层。

## 目标

### Gimbal 最终职责

- 负责遥控器/键鼠/VTM/视觉输入汇总。
- 负责 `RobotCMD` 决策、云台控制、发射控制、视觉应用。
- 负责将底盘控制指令打包并通过 CAN 发给底盘板。
- 负责接收底盘板的状态回传，并将其转化为本板控制所需的摘要信息。
- 不再承担底盘执行、底盘运动解算、超级电容控制、裁判系统/UI 主逻辑。

### Chassis 最终职责

- 负责接收云台板下发的底盘控制指令。
- 负责底盘执行层、功率控制、超级电容、裁判系统/UI、底盘相关系统辨识。
- 负责将底盘状态按约定协议回传给云台板。
- 不再承担遥控解码、云台执行、发射执行、RobotCMD 主决策。

## 设计原则

### 1. 先收缩构建面，再删执行层

第一优先级是把两个工程的构建入口改成“显式白名单”，避免目录虽然已删除但未来又被递归收回构建面。

### 2. 协议层保留，执行层删除

以下内容视为必须保留的公共边界，不属于冗余代码：

- `Chassis_Ctrl_Cmd_s`
- `Chassis_Upload_Data_s`
- `Chassis_Ctrl_Fast_Pkt_s`
- `Chassis_Ctrl_State_Pkt_s`
- `Chassis_Ctrl_UI_Pkt_s`
- `Chassis_Ctrl_Event_Pkt_s`
- `Chassis_Feed_Fast_Pkt_s`
- `Chassis_Feed_State_Pkt_s`
- `modules/can_comm/chassis_can_link.*`
- Gimbal 侧 `RobotCMD` 中的底盘 CAN 打包/发送/接收逻辑

它们是双板协议面，不应随着执行层裁剪被删除。

### 3. `message_center` 不按“是否跨板”判断去留

`message_center` 的作用是板内应用之间解耦，不等同于双板链路本身。

- `Gimbal` 侧仍需要 `cmd/gimbal/shoot/vision/sysid` 的板内消息分发，因此 `modules/message_center` 必须保留。
- `Chassis` 侧如果保留 `sysid` 且其实现仍依赖消息中心，则 `modules/message_center` 也应保留。
- 真正要删除的是“不再执行的应用层逻辑”，不是盲删消息模块。

### 4. 第一轮不做大规模共享目录抽取

虽然从长期看可以把跨板协议抽到根目录 `shared/`，但这会扩大本次变更范围。本轮裁剪先以“协议定义一致 + 角色固定 + 构建显式化”为目标，避免任务膨胀成架构重写。

## 目标结构

### Gimbal 保留

- `application/cmd/*`
- `application/gimbal/*`
- `application/shoot/*`
- `application/vision/*`
- `application/sysid/*`
- `application/robot.c`
- `application/robot.h`
- `application/robot_def.h`
- `application/robot_task.h`
- `modules/message_center/*`
- `modules/can_comm/*`
- `modules/vision_comm/*`
- `modules/vtm_input/*`
- `modules/remote/*`
- 云台/发射/IMU/电机等本板依赖模块

### Gimbal 删除

- `application/chassis/*`
- 所有 `ONE_BOARD` 入口
- `RobotInit/RobotTask` 中的底盘执行分支
- `RobotCMD` 中基于本板 `Pub/Sub` 的底盘指令/反馈路径

### Chassis 保留

- `application/chassis/*`
- `application/sysid/*`
- `application/robot.c`
- `application/robot.h`
- `application/robot_def.h`
- `application/robot_task.h`
- `modules/can_comm/*`
- `modules/power_controller/*`
- `modules/referee/*`
- `modules/super_cap/*`
- `modules/vofa/*`
- `modules/master_machine/*`
- 底盘/IMU/电机等本板依赖模块
- 若 `sysid` 仍依赖，则保留 `modules/message_center/*`

### Chassis 删除

- `application/cmd/*`
- `application/gimbal/*`
- `application/shoot/*`
- 所有 `ONE_BOARD` 入口
- 底盘执行层中基于本板 `Pub/Sub` 的底盘控制路径

## 关键改动点

### A. 构建层

`Gimbal/CMakeLists.txt` 与 `Chassis/CMakeLists.txt` 当前都会递归收集 `application/*.c`。应改为：

- `modules/` 可以暂时继续递归收集。
- `application/` 改为显式列出本板允许编译的源文件。
- 删除目录后，构建面仍然稳定，不会因误加文件重新污染。

### B. 角色宏

两边都取消 `ONE_BOARD` 兼容，仅保留固定板型：

- `Gimbal/application/robot_def.h` 只保留 `#define GIMBAL_BOARD`
- `Chassis/application/robot_def.h` 只保留 `#define CHASSIS_BOARD`

对应的 `#ifdef ONE_BOARD` 逻辑应全部删除，而不是继续“留着以后也许会用”。

### C. Gimbal 控制链

Gimbal 控制链最终应为：

`Remote/VTM/Vision -> RobotCMD -> 本板 message_center -> Gimbal/Shoot/Vision`

并额外通过 CAN 完成：

- `RobotCMD -> Chassis CAN command`
- `Chassis CAN feedback -> RobotCMD`

也就是说，云台板保留“底盘控制指令生成”，但删除“底盘执行与解算”。

### D. Chassis 控制链

Chassis 控制链最终应为：

`Chassis CAN command -> ChassisTask -> 电机/功率/裁判/UI -> Chassis CAN feedback`

底盘板不再存在本板 `RobotCMD`、云台控制、发射控制三条执行主线。

## 风险与规避

### 风险 1：删掉执行层时顺手删掉协议定义

规避：

- 先列出双板通信结构体与链路文件白名单。
- 删除前用检索确认协议符号仍被双侧使用。

### 风险 2：两份 `robot_def.h` 协议区块逐渐漂移

规避：

- 第一轮裁剪时同步梳理两份头文件中“跨板协议区块”。
- 为关键结构体增加尺寸约束。
- 以后若继续演进，再考虑抽共享头文件。

### 风险 3：构建仍在递归收录残留文件

规避：

- 先改 CMake，再删目录。
- 删除后执行全文检索，确认没有残留引用。

### 风险 4：误删 `message_center`

规避：

- 按“板内应用依赖”而非“是否跨板”判断保留。
- `Gimbal` 必留。
- `Chassis` 在确认 `sysid` 是否仍依赖前，不删除。

## 完成标准

完成本轮裁剪后，应满足：

- `Gimbal` 工程不再编译 `application/chassis/*`
- `Chassis` 工程不再编译 `application/cmd/*`、`application/gimbal/*`、`application/shoot/*`
- 两个工程中 `ONE_BOARD` 不再出现
- `RobotInit/RobotTask` 只保留本板职责入口
- 双板 CAN 指令与反馈链路仍然完整
- 两边均能独立编译通过
- 与本次裁剪无关的公共模块不做额外重构

## 实际执行结果

### 已完成项

- `Gimbal` 已删除 `application/chassis/*`，并移除 `RobotInit/RobotTask` 中的底盘执行入口。
- `Gimbal/application/cmd/robot_cmd.c` 已删除本板 `chassis_cmd_pub` / `chassis_feed_sub` 兼容路径，仅保留 CAN 摘要链路。
- `Chassis` 已删除 `application/cmd/*`、`application/gimbal/*`、`application/shoot/*`，并移除入口中的 `RobotCMD/Gimbal/Shoot` 执行主线。
- `Chassis/application/chassis/chassis.c` 已删除单板 `Pub/Sub` 控制路径，仅保留 `ChassisCanLinkUpdateCommand()` 与 `ChassisCanLinkSendFeedbackIfDue()`。
- `Chassis/application/robot_task.h` 与 `Chassis/modules/imu/ins_task.c` 已移除残留的视觉发送钩子，避免底盘板继续承担视觉链路职责。
- 两侧 `CMakeLists.txt` 都已将 `application` 源文件改为显式白名单，不再递归吞掉整个 `application/*.c`。

### 保留项说明

- `Gimbal/modules/message_center/*` 保留。
原因：`cmd/gimbal/shoot/vision/sysid` 仍通过板内消息中心解耦，不属于跨板冗余。

- `Chassis/modules/message_center/*` 保留。
原因：`application/sysid/sysid_task.c` 仍依赖 `RegisterPublisher/RegisterSubscriber/SubGetMessage/PubPushMessage`。

- `Chassis/modules/can_comm/chassis_can_link.*` 保留。
原因：这是双板控制链路的服务端协议入口，不属于冗余代码。

### 验证结果

- `tests/dual_board_gimbal_boundary_regression.ps1`：通过
- `tests/dual_board_chassis_boundary_regression.ps1`：通过
- `Gimbal/compile.ps1 Debug`：通过
- `Chassis/compile.ps1 Debug`：通过
