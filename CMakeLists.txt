cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(Recon-Rover C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w)
pico_sdk_init()

# --- Add all source files including new LIDAR driver ---
add_executable(Recon-Rover
    main.c
    drivers/motor.c
    drivers/encoder.c
    drivers/ultrasonic.c 
    drivers/imu.c
    drivers/lidar.c
)

# --- Add the 'drivers' directory to the include path ---
target_include_directories(Recon-Rover BEFORE PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/drivers
    $ENV{PICO_SDK_PATH}/src/rp2_common/pico_lwip/include
)

# --- Link all required libraries ---
target_link_libraries(Recon-Rover
    pico_stdlib
    hardware_gpio
    hardware_uart        # NEW: UART for LIDAR
    hardware_pwm         # NEW: PWM for LIDAR motor
    pico_lwip
    pico_cyw43_arch_lwip_threadsafe_background
    hardware_i2c
)

# --- Stdio / Float / UF2 ---
pico_enable_stdio_usb(Recon-Rover 1)
pico_enable_stdio_uart(Recon-Rover 0)
target_link_options(Recon-Rover PRIVATE -Wl,-u,_printf_float)
pico_add_extra_outputs(Recon-Rover)