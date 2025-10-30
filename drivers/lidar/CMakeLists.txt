cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(rover_map C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Ensure Pico W (has WiFi)
set(PICO_BOARD pico_w)

pico_sdk_init()

add_executable(rover_map
    main.c
)

# Link everything we need
target_link_libraries(rover_map
    pico_stdlib
    hardware_uart
    hardware_pwm
    hardware_i2c
    hardware_gpio
    pico_lwip_nosys                     # ✅ adds lwipopts.h automatically
    pico_cyw43_arch_lwip_poll           # ✅ simpler version of WiFi
)

# Add include directory to be safe
target_include_directories(rover_map PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})


pico_enable_stdio_usb(rover_map 1)
pico_enable_stdio_uart(rover_map 0)
pico_add_extra_outputs(rover_map)
