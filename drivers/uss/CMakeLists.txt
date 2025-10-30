cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(functions C CXX ASM)
pico_sdk_init()

add_executable(functions
    main.c
    ultrasonic.c
    ultrasonicObject.cpp
)

# headers live in the same folder
target_include_directories(functions PRIVATE .)

# link what you need
target_link_libraries(functions
    pico_stdlib
    hardware_uart
)
pico_enable_stdio_usb(functions 1)
pico_enable_stdio_uart(functions 0)
pico_add_extra_outputs(functions)

add_library(pico_ultrasonic ultrasonic.h ultrasonic.c)

target_link_libraries(pico_ultrasonic pico_stdlib hardware_gpio hardware_timer)

target_include_directories(pico_ultrasonic PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")


add_library(pico_ultrasonic_object ultrasonic.h ultrasonic.c ultrasonicObject.cpp ultrasonicObject.h)

target_link_libraries(pico_ultrasonic_object pico_stdlib hardware_gpio hardware_timer)

target_include_directories(pico_ultrasonic_object PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

