#
# CMake toolchain file for the Arm GNU bare-metal toolchain.
#
# Selected with:
#   cmake -S . -B build-fw -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
#
# Requires arm-none-eabi-gcc on PATH, or ARM_TOOLCHAIN_DIR pointing at its
# bin directory.
#

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Test compilation only, since a bare-metal target cannot run a test binary.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if(DEFINED ARM_TOOLCHAIN_DIR)
    set(_prefix "${ARM_TOOLCHAIN_DIR}/arm-none-eabi-")
else()
    set(_prefix "arm-none-eabi-")
endif()

set(CMAKE_C_COMPILER   "${_prefix}gcc")
set(CMAKE_CXX_COMPILER "${_prefix}g++")
set(CMAKE_ASM_COMPILER "${_prefix}gcc")
set(CMAKE_OBJCOPY      "${_prefix}objcopy" CACHE FILEPATH "objcopy")
set(CMAKE_SIZE         "${_prefix}size"    CACHE FILEPATH "size")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# TMPM4KNF10AFG: Cortex-M4 with a single-precision FPU.
set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)
