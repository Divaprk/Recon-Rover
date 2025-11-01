cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(Recon-Rover C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)
pico_sdk_init()

# --- MODIFICATION 1 ---
# Add all your .c files here.
# CMake needs to know about main.c AND the new driver files.
add_executable(Recon-Rover
    main.c
    drivers/motor.c
    drivers/encoder.c
    drivers/ultrasonic.c 
)

# --- MODIFICATION 2 ---
# Add the 'drivers' directory to the include path.
# This lets main.c find #include "drivers/motor.h"
target_include_directories(Recon-Rover BEFORE PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/drivers
    $ENV{PICO_SDK_PATH}/src/rp2_common/pico_lwip/include
)

# --- MERGED library list (No changes here) ---
target_link_libraries(Recon-Rover
    pico_stdlib
    hardware_gpio
    pico_lwip
    pico_cyw43_arch_lwip_threadsafe_background
)

# --- Stdio / Float / UF2 (No changes here) ---
pico_enable_stdio_usb(Recon-Rover 1)
pico_enable_stdio_uart(Recon-Rover 0)
target_link_options(Recon-Rover PRIVATE -Wl,-u,_printf_float)
pico_add_extra_outputs(Recon-Rover)