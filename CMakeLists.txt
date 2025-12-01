# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(PICO_BOARD pico2_w) 
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(Recon-Rover C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
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
)

# --- Link all required libraries ---
target_link_libraries(Recon-Rover
    pico_stdlib
    hardware_uart
    hardware_pwm
    hardware_i2c
    hardware_gpio
    pico_lwip_nosys
    pico_cyw43_arch_lwip_poll
)

# --- Stdio / Float / UF2 ---
pico_enable_stdio_usb(Recon-Rover 1)
pico_enable_stdio_uart(Recon-Rover 0)
target_link_options(Recon-Rover PRIVATE -Wl,-u,_printf_float)
pico_add_extra_outputs(Recon-Rover)