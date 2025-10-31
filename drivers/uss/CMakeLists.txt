add_library(uss STATIC
    ultrasonic.c
)

target_include_directories(uss PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(uss PUBLIC
    pico_stdlib
    hardware_gpio
    hardware_timer
)