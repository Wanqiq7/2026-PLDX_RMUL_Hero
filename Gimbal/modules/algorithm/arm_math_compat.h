#ifndef ARM_MATH_COMPAT_H
#define ARM_MATH_COMPAT_H

/*
 * 统一通过设备头引入 CMSIS-DSP，确保 __FPU_PRESENT 等芯片能力宏
 * 在 arm_math.h 展开前已完成定义，避免因头文件顺序导致的 FPU 误判。
 */
#include "stm32f407xx.h"
#include "arm_math.h"

#endif // ARM_MATH_COMPAT_H
