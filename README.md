# RMUL_Hero

RoboMaster 英雄机器人（Hero）代码仓库，包含双板电控固件（云台板/底盘板）与视觉自瞄工程。

## 1. 项目概览

本仓库用于 RoboMaster 英雄机器人整机开发，主要包含：

- **Gimbal**：云台板固件，负责云台控制、发射机构、视觉通信与部分系统辨识能力。
- **Chassis**：底盘板固件，负责底盘运动控制、功率控制、裁判系统交互等。
- **Vision/Hero_Vision**：当前使用的视觉与自瞄工程（C++/CMake）。
- **Vision/HUST_HeroAim_2024-main**：参考视觉工程（保留用于对比与迁移）。

核心目标是形成“感知-决策-执行”闭环：

1. 视觉端识别装甲板并估计目标状态；
2. 云台板执行控制与火控策略；
3. 底盘板配合机动与功率管理；
4. 通过板间通信与裁判系统实现整机联动。

---

## 2. 仓库结构

```text
.
├─ Chassis/                       # 底盘板 STM32 固件
│  ├─ application/                # 任务与业务逻辑（chassis/cmd/sysid/...）
│  ├─ modules/                    # 算法、功率、消息中心、裁判系统等模块
│  ├─ bsp/ Drivers/ Middlewares/  # HAL、RTOS、底层驱动
│  ├─ Makefile                    # 主构建入口（目标: basic_framework）
│  └─ flash.sh / compile.sh       # 常用构建与烧录脚本
│
├─ Gimbal/                        # 云台板 STM32 固件
│  ├─ application/                # 云台、发射、视觉任务、系统辨识等
│  ├─ modules/                    # 电机、IMU、消息中心、视觉通信等
│  ├─ bsp/ Drivers/ Middlewares/  # HAL、RTOS、底层驱动
│  ├─ Makefile                    # 主构建入口（目标: basic_framework）
│  └─ flash.sh / compile.sh       # 常用构建与烧录脚本
│
├─ Vision/
│  ├─ Hero_Vision/                # 主视觉工程（OpenCV/OpenVINO 等）
│  └─ HUST_HeroAim_2024-main/     # 参考视觉工程
│
└─ *.md                           # 参数、调试、迁移与经验文档
```

---

## 3. 技术栈

### 3.1 嵌入式（Gimbal / Chassis）

- MCU：STM32F407 系列（见链接脚本与启动文件）
- 语言：C / C++（以 C 为主）
- 框架：STM32 HAL + FreeRTOS + CMSIS
- 构建：`make` / `mingw32-make`
- 调试/下载：OpenOCD、JLink（仓库内提供配置）

### 3.2 视觉（Hero_Vision）

- 语言标准：C++17
- 构建系统：CMake
- 主要依赖（见 `Vision/Hero_Vision/CMakeLists.txt`）：
  - OpenCV
  - OpenVINO
  - fmt
  - Eigen3
  - spdlog
  - yaml-cpp
  - nlohmann_json

---

## 4. 快速开始

> 建议先保证工具链可用，再分别编译三个子系统。

### 4.1 克隆仓库

```bash
git clone https://github.com/Wanqiq7/RMUL_Hero.git
cd RMUL_Hero
```

### 4.2 编译云台板固件（Gimbal）

```bash
cd Gimbal
make -j8
# Windows/MinGW 可使用
# mingw32-make -j8
```

输出产物位于：`Gimbal/build/basic_framework.elf`。

### 4.3 编译底盘板固件（Chassis）

```bash
cd Chassis
make -j8
# Windows/MinGW 可使用
# mingw32-make -j8
```

输出产物位于：`Chassis/build/basic_framework.elf`。

### 4.4 烧录（OpenOCD 示例）

仓库内已提供脚本：

```bash
cd Gimbal
bash flash.sh

cd ../Chassis
bash flash.sh
```

默认脚本使用 `openocd_dap.cfg`，如需 JLink，请按本地调试器修改配置。

### 4.5 编译视觉工程（Hero_Vision）

```bash
cd Vision/Hero_Vision
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j8
```

可执行程序名称与模式相关（如 `standard`、`mt_standard`、`auto_aim_debug_mpc` 等）。

---

## 5. 开发建议与关键文档

首次接手建议优先阅读：

- `功率控制参数调试指南.md`
- `功率控制调参指南.md`
- `Vision参数调整指南.md`
- `代码架构问题修复总结.md`
- `Parameter_means.md`

板级开发建议优先从 `application/robot.c`、`application/cmd/`、`modules/message_center/` 入手理解任务调度与信息流。

---

## 6. 注意事项

- 视觉工程中包含较大模型/样例文件，首次拉取与推送可能较慢。
- 仓库已忽略本地 IDE 与缓存目录；提交前请先执行 `git status` 检查无关文件。
- 涉及通信协议改动时，务必同步检查视觉端与云台端解析逻辑是否一致。

---

## 7. 许可证

本仓库含多个来源/阶段的工程，许可证可能按子目录区分。

- 请优先查看各子目录中的 `LICENSE` 文件；
- 二次分发或商用前，务必确认对应子项目授权条款。

---

## 8. 致谢

感谢 RoboMaster 社区与开源战队项目对算法与工程实践的持续贡献。

如果你正在接手本项目，建议从“先跑通、再测通、后优化”的节奏开始：

1. 先完成双板基础编译与烧录；
2. 再联通视觉-云台通信链路；
3. 最后进行参数整定与实车优化。
