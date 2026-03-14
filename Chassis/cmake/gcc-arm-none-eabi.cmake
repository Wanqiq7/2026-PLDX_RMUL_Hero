set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings
set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(_arm_gnu_toolchain_hints)

if(DEFINED ENV{ARM_GNU_TOOLCHAIN_ROOT} AND NOT "$ENV{ARM_GNU_TOOLCHAIN_ROOT}" STREQUAL "")
  list(APPEND _arm_gnu_toolchain_hints "$ENV{ARM_GNU_TOOLCHAIN_ROOT}/bin")
endif()

if(DEFINED ENV{STM32CUBECLT_ROOT} AND NOT "$ENV{STM32CUBECLT_ROOT}" STREQUAL "")
  list(APPEND _arm_gnu_toolchain_hints "$ENV{STM32CUBECLT_ROOT}/GNU-tools-for-STM32/bin")
endif()

list(APPEND _arm_gnu_toolchain_hints
  "C:/ST/STM32CubeCLT_1.19.0/GNU-tools-for-STM32/bin"
)

find_program(ARM_NONE_EABI_GCC NAMES ${TOOLCHAIN_PREFIX}gcc HINTS ${_arm_gnu_toolchain_hints})
find_program(ARM_NONE_EABI_GXX NAMES ${TOOLCHAIN_PREFIX}g++ HINTS ${_arm_gnu_toolchain_hints})
find_program(ARM_NONE_EABI_AR NAMES ${TOOLCHAIN_PREFIX}ar HINTS ${_arm_gnu_toolchain_hints})
find_program(ARM_NONE_EABI_OBJCOPY NAMES ${TOOLCHAIN_PREFIX}objcopy HINTS ${_arm_gnu_toolchain_hints})
find_program(ARM_NONE_EABI_OBJDUMP NAMES ${TOOLCHAIN_PREFIX}objdump HINTS ${_arm_gnu_toolchain_hints})
find_program(ARM_NONE_EABI_SIZE NAMES ${TOOLCHAIN_PREFIX}size HINTS ${_arm_gnu_toolchain_hints})

if(NOT ARM_NONE_EABI_GCC OR NOT ARM_NONE_EABI_GXX OR NOT ARM_NONE_EABI_AR OR
   NOT ARM_NONE_EABI_OBJCOPY OR NOT ARM_NONE_EABI_OBJDUMP OR NOT ARM_NONE_EABI_SIZE)
  message(FATAL_ERROR
    "ARM GNU toolchain not found. Please add STM32CubeCLT GNU-tools-for-STM32/bin to PATH "
    "or set ARM_GNU_TOOLCHAIN_ROOT / STM32CUBECLT_ROOT.")
endif()

set(CMAKE_C_COMPILER                ${ARM_NONE_EABI_GCC})
set(CMAKE_ASM_COMPILER              ${ARM_NONE_EABI_GCC})
set(CMAKE_CXX_COMPILER              ${ARM_NONE_EABI_GXX})
set(CMAKE_LINKER                    ${ARM_NONE_EABI_GXX})
set(CMAKE_AR                        ${ARM_NONE_EABI_AR})
set(CMAKE_OBJCOPY                   ${ARM_NONE_EABI_OBJCOPY})
set(CMAKE_OBJDUMP                   ${ARM_NONE_EABI_OBJDUMP})
set(CMAKE_SIZE                      ${ARM_NONE_EABI_SIZE})

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
