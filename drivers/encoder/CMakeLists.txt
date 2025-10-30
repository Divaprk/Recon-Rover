cmake_minimum_required(VERSION 3.13)

# Import the Pico SDK BEFORE project() so the toolchain is set up.
include(pico_sdk_import.cmake)

project(wheel_encoder C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Build for Pico W is fine even if Wi-Fi isn't used
set(PICO_BOARD pico_w)

# (Optional but handy on quirky Windows setups: avoid host link in try-compile)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

# Initialize SDK
pico_sdk_init()

# Target
add_executable(wheel_encoder
    main.c
)

# USB stdio on, UART off
pico_enable_stdio_usb(wheel_encoder 1)
pico_enable_stdio_uart(wheel_encoder 0)

# Link only what we use
target_link_libraries(wheel_encoder
    pico_stdlib
    hardware_gpio
)

# If printf with floats prints nothing, uncomment:
# target_link_options(wheel_encoder PRIVATE -Wl,-u,_printf_float)

# Produce UF2/bin/map
pico_add_extra_outputs(wheel_encoder)
