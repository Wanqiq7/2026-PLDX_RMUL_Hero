# Repository Guidelines

## Project Structure & Module Organization
- `Src/` and `Inc/` hold CubeMX-generated FreeRTOS entry points; route new logic to `application/` or `modules/`.
- `application/{gimbal,chassis,shoot,vision,cmd,sysid}` implement high-level behaviors and communicate via `modules/message_center`.
- `modules/` covers algorithms, motors, sensors, and comms—extend modules before touching BSP.
- `bsp/`, `Drivers/`, and `Middlewares/` expose ST/SEGGER hardware layers shared across robots.
- `build/` is generated; `.Doc/` and `.assets/` keep design notes—never commit their binaries.

## Build, Flash, and Development Commands
- `make -j24` or `./compile.sh` builds `build/basic_framework.{elf,bin,hex}` with `arm-none-eabi-gcc`.
- `make clean` resets `build/` artifacts after branch switches or linker-script edits.
- `make download_dap` / `make download_jlink` flash via OpenOCD or JLink; confirm cables and COM ports first.
- `./flash.sh` chains build + OpenOCD verify/reset for a one-shot bring-up loop.

## Coding Style & Naming Conventions
- Use 2-space indentation, brace-on-same-line style, and include groups ordered HAL → modules → app.
- Files and topics stay lowercase snake_case (`bsp_dwt.c`, `gimbal_sysid_cmd`); macros remain SCREAMING_SNAKE_CASE (`ENABLE_GIMBAL_SYSID`, `CHASSIS_BOARD`).
- Keep HAL init inside `Src/`, schedule work in FreeRTOS tasks, and guard experiments with `#if`.
- Fix warnings proactively, but the current build no longer treats them as hard errors; still favor explicit widths, `const`, and checked return values.

## Testing Guidelines
- There is no automated test suite; validation equals building, flashing, and exercising FreeRTOS tasks on the STM32F407 target.
- Enable the relevant macros (e.g., `ENABLE_GIMBAL_SYSID`, `SYSID_AXIS_*`) and capture SEGGER RTT or CAN logs before merging.
- Record bench steps in PRs—sensor bias calibration, CAN ID checks, thermal or over-current observations—for reviewer replay.

## Commit & Pull Request Guidelines
- Match the current log style: short, present-tense subsystem summaries (e.g., `Adjust feed disk ratio to 51`); keep one feature per commit.
- Reference impacted boards or tasks in the subject and elaborate in the body when multiple domains shift.
- PRs must explain motivation, build/flash output, bench evidence (scope trace, Ozone capture), and link to tasks/issues; attach screenshots for tooling changes.

## Configuration & Safety Tips
- Declare the board role in `application/robot_def.h` (`ONE_BOARD`, `CHASSIS_BOARD`, `GIMBAL_BOARD`) before compiling.
- Avoid CAN conflicts by coordinating IDs in `bsp/can` and documenting new topics in `modules/message_center`.
- Keep `application/application.md` and APP-layer notes synchronized with code paths so other operators can reproduce setups.
