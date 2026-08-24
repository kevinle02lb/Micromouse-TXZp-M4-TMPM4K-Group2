#
# Firmware build for the TMPM4KNF10AFG micromouse.
#
# Produces an ELF plus .hex and .bin, and prints the section sizes.
# Runs alongside the uVision project; neither affects the other.
#
# Usage:
#   cmake -S . -B build-fw -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
#   cmake --build build-fw
#

cmake_minimum_required(VERSION 3.16)

project(micromouse C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# --------------------------------------------------------------------------
# Sources
#
# Listed explicitly rather than globbed, so adding a file is a deliberate act
# and CMake does not need re-running to notice one.
# --------------------------------------------------------------------------

set(SRC_DIR       "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(VENDOR_DIR    "${CMAKE_CURRENT_SOURCE_DIR}/vendor")
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/TMPM4KNF10A.ld")

add_executable(${PROJECT_NAME}
    startup_TMPM4KNA.S

    # Clock and core setup from the Toshiba pack. Confirm the exact filename
    # against vendor/toshiba/Source; SystemInit() is defined here.
    ${VENDOR_DIR}/toshiba/Source/system_TMPM4KyA.c

    ${SRC_DIR}/main.c
    ${SRC_DIR}/ModuleTest.c

    ${SRC_DIR}/modules/Encoder.c
    ${SRC_DIR}/modules/FloodFill.c
    ${SRC_DIR}/modules/IrSensor.c
    ${SRC_DIR}/modules/Motion.c
    ${SRC_DIR}/modules/Motor.c
    ${SRC_DIR}/modules/Navigator.c
    ${SRC_DIR}/modules/Odometry.c
    ${SRC_DIR}/modules/PID.c
    ${SRC_DIR}/modules/Profile.c
    ${SRC_DIR}/modules/Timebase.c

    ${SRC_DIR}/drivers/adc.c
    ${SRC_DIR}/drivers/encoder32A.c
    ${SRC_DIR}/drivers/gpio.c
    ${SRC_DIR}/drivers/systick.c
    ${SRC_DIR}/drivers/timer32A.c
    ${SRC_DIR}/drivers/uart.c
)

# Mirrors the uVision include paths. The two vendor directories are copies of
# the Toshiba DFP and the ARM CMSIS Core pack, kept in-tree so the project
# builds without those packs installed.
target_include_directories(${PROJECT_NAME} PRIVATE
    ${SRC_DIR}
    ${SRC_DIR}/modules
    ${SRC_DIR}/drivers
    ${VENDOR_DIR}/toshiba/Include              # TMPM4KyA.h, TMPM4KNA.h
    ${VENDOR_DIR}/cmsis/Include                # core_cm4.h, cmsis_gcc.h
)

# --------------------------------------------------------------------------
# Flags
# --------------------------------------------------------------------------

target_compile_options(${PROJECT_NAME} PRIVATE
    ${MCU_FLAGS}
    -Wall -Wextra
    -Werror=implicit-function-declaration       # a missing prototype is a bug
    -ffunction-sections -fdata-sections         # let the linker drop unused code
    -fno-common
    $<$<CONFIG:Debug>:-Og -g3 -gdwarf-4>
    $<$<CONFIG:Release>:-O2 -g>
)

target_link_options(${PROJECT_NAME} PRIVATE
    ${MCU_FLAGS}
    -T${LINKER_SCRIPT}
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--print-memory-usage
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    SUFFIX ".elf"
    LINK_DEPENDS "${LINKER_SCRIPT}"
)

# --------------------------------------------------------------------------
# Outputs
# --------------------------------------------------------------------------

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMENT "Generating .hex and .bin"
)

# --------------------------------------------------------------------------
# Flashing and debugging over CMSIS-DAP
#
# pyocd needs the device's flash algorithm, which ships in Toshiba's CMSIS
# pack. Either install it from the online index:
#
#   pyocd pack install TMPM4KNF10A
#
# or point at the copy uVision already downloaded, which avoids the index
# entirely:
#
#   cmake -S . -B build-fw -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
#         -DPYOCD_TARGET=tmpm4knf10a \
#         -DPYOCD_PACK="C:/Keil_v5/ARM/PACK/Toshiba/TMPM4K_DFP/<ver>/<name>.pack"
#
# Confirm the exact target name with:  pyocd list --targets
# --------------------------------------------------------------------------

if(DEFINED PYOCD_TARGET)
    set(_pyocd_common --target ${PYOCD_TARGET})
    if(DEFINED PYOCD_PACK)
        list(APPEND _pyocd_common --pack "${PYOCD_PACK}")
    endif()

    add_custom_target(flash
        COMMAND pyocd flash --erase auto ${_pyocd_common} ${PROJECT_NAME}.hex
        DEPENDS ${PROJECT_NAME}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Flashing over CMSIS-DAP"
    )

    # Leaves a GDB server on :3333. Attach with arm-none-eabi-gdb, or point
    # the VS Code cortex-debug extension at it.
    add_custom_target(gdbserver
        COMMAND pyocd gdbserver ${_pyocd_common}
        COMMENT "pyocd gdbserver on port 3333"
    )
endif()