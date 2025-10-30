cmake_minimum_required(VERSION 3.13)

# Use the import file you already have in the root
include(pico_sdk_import.cmake)

project(imu_orientation_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

add_executable(imu_orientation
    src/main.c
    src/imu_orientation.c
)

# Make headers in src/ visible as "imu_orientation.h"
target_include_directories(imu_orientation PRIVATE src)

pico_enable_stdio_usb(imu_orientation 1)
pico_enable_stdio_uart(imu_orientation 0)

target_link_libraries(imu_orientation PRIVATE pico_stdlib hardware_i2c)

pico_add_extra_outputs(imu_orientation)
