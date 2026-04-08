cmake_minimum_required(VERSION 3.16)
# 指定编译平台/架构与语言标准, 推荐指定Ninja为构建工具,可以加快编译速度(相比make)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

# 指定工程名称和语言类型
project(basic_framework C ASM) 

# 选择构建类型
set(CMAKE_BUILD_TYPE Debug) # Debug Release RelWithDebInfo MinSizeRel

# board specific settings, arch/fpu/instruction
set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mthumb-interwork -mfloat-abi=hard -mfpu=fpv4-sp-d16)
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32F407IGHx_FLASH.ld") # 指定链接脚本
set(DSP_LIB_PATH "${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libCMSISDSP.a")

# Generic compiler settings for optimization and basic link lib
add_compile_options(-pipe ${MCU_FLAGS} -Wall -fmessage-length=0 # basic options
                    -ffunction-sections -fdata-sections -fno-common # optimize options 
                    )
add_link_options(-pipe ${MCU_FLAGS} -T${LINKER_SCRIPT} -Wl,--no-warn-rwx-segments # close RWX warning
                 -lm -lc # lib options
                 -Wl,--gc-sections -flto # optimize options
                 -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map -Wl,--cref -Wl,--print-memory-usage # map options
                 ) # if your executable is too large , try option '-s' to strip symbols

# add_compile_definitions() works for compile stage
# while add_definitions() works for both compile and link stage
add_definitions(
    -DUSE_HAL_DRIVER 
    -DSTM32F407xx 
    -DARM_MATH_CM4 
    ) # need -D<macro> to define macro

# add inc
# 显式声明 public include 根，避免递归暴露整仓目录结构
set(COMMON_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Inc
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Include
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/SEGGER/Config
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/SEGGER/RTT
    ${CMAKE_SOURCE_DIR}/Inc
    ${CMAKE_SOURCE_DIR}/bsp/adc
    ${CMAKE_SOURCE_DIR}/bsp/can
    ${CMAKE_SOURCE_DIR}/bsp/dwt
    ${CMAKE_SOURCE_DIR}/bsp/flash
    ${CMAKE_SOURCE_DIR}/bsp/gpio
    ${CMAKE_SOURCE_DIR}/bsp/iic
    ${CMAKE_SOURCE_DIR}/bsp/log
    ${CMAKE_SOURCE_DIR}/bsp/pwm
    ${CMAKE_SOURCE_DIR}/bsp/spi
    ${CMAKE_SOURCE_DIR}/bsp/usart
    ${CMAKE_SOURCE_DIR}/bsp/usb
    ${CMAKE_SOURCE_DIR}/modules/alarm
    ${CMAKE_SOURCE_DIR}/modules/algorithm
    ${CMAKE_SOURCE_DIR}/modules/BMI088
    ${CMAKE_SOURCE_DIR}/modules/can_comm
    ${CMAKE_SOURCE_DIR}/modules/daemon
    ${CMAKE_SOURCE_DIR}/modules/imu
    ${CMAKE_SOURCE_DIR}/modules/message_center
    ${CMAKE_SOURCE_DIR}/modules/motor
    ${CMAKE_SOURCE_DIR}/modules/motor/DJImotor
    ${CMAKE_SOURCE_DIR}/modules/motor/DMmotor
    ${CMAKE_SOURCE_DIR}/modules/motor/servo_motor
    ${CMAKE_SOURCE_DIR}/modules/motor/step_motor
    ${CMAKE_SOURCE_DIR}/modules/referee
)

set(BOARD_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/application/cmd
    ${CMAKE_SOURCE_DIR}/application/gimbal
    ${CMAKE_SOURCE_DIR}/application/shoot
    ${CMAKE_SOURCE_DIR}/application/sysid
    ${CMAKE_SOURCE_DIR}/application/vision
    ${CMAKE_SOURCE_DIR}/modules/remote
    ${CMAKE_SOURCE_DIR}/modules/vision_comm
    ${CMAKE_SOURCE_DIR}/modules/vtm_input
)

# audit:root-level-public-include-exceptions 根层 public header 例外白名单；新增项必须同步更新 tests/cmake_explicit_include_regression.ps1 并补充审计说明。
set(ROOT_LEVEL_PUBLIC_INCLUDE_EXCEPTIONS
    ${CMAKE_SOURCE_DIR}/bsp         # audit:bsp-root-public 保留板级驱动根层 public header 入口，例如 bsp_can.h、bsp_usart.h。
    ${CMAKE_SOURCE_DIR}/modules     # audit:modules-root-public 保留模块聚合根层 public header 入口，例如 referee.h、can_comm.h。
    ${CMAKE_SOURCE_DIR}/application # audit:application-root-public 保留应用层根层 public header 入口，例如 robot_def.h、robot_task.h。
)

set(APPLICATION_SOURCES
    application/robot.c
    application/cmd/robot_cmd.c
    application/gimbal/gimbal.c
    application/shoot/shoot.c
    application/sysid/sysid_task.c
    application/vision/vision.c
)

# add source, only surfix .c
# 递归收集 bsp/modules/application 下新增源文件,包括 bsp/usart/bsp_usart_async.c
file(GLOB_RECURSE SOURCES
    "Drivers/*.c"
    "Src/*.c"
    "Middlewares/*.c"
    "bsp/*.c"
    "modules/*.c"
)
list(APPEND SOURCES ${APPLICATION_SOURCES})

# 示例文件不参与正式固件构建，避免占位实现污染主 target
list(FILTER SOURCES EXCLUDE REGEX "modules/algorithm/.*/LQR_Example\\.c$")

# 汇编文件路径
# ENABLE_LANGUAGE(ASM)
set(ASM_SOURCES
    startup_stm32f407xx.s
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s
    )
set_source_files_properties(${ASM_SOURCES} PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

# Build types
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g -gdwarf-2)
    add_definitions(-DESC_DEBUG) # ESC Debug
endif ()

# build binary and hex file
add_executable(${PROJECT_NAME}  ${SOURCES} ${ASM_SOURCES} ${LINKER_SCRIPT})
target_include_directories(${PROJECT_NAME} PRIVATE
    ${COMMON_INCLUDE_DIRS}
    ${BOARD_INCLUDE_DIRS}
    ${ROOT_LEVEL_PUBLIC_INCLUDE_EXCEPTIONS}
)
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")
if (EXISTS "${DSP_LIB_PATH}")
    target_link_libraries(${PROJECT_NAME} "${DSP_LIB_PATH}" m) # link DSP lib and math lib
else ()
    message(FATAL_ERROR "Missing CMSIS-DSP static library: ${DSP_LIB_PATH}")
endif ()

add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMENT "Building hex & bin file..."
    COMMENT "EXCUTABLE SIZE:"
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
  )
