# drivers/teleop/CMakeLists.txt
add_library(teleop_driver
  teleop.c          # your implementation file
)

target_include_directories(teleop_driver PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/../../include
)

target_link_libraries(teleop_driver
  pico_stdlib
  hardware_gpio
  hardware_timer
  hardware_pwm       # add if your teleop drives motors with PWM
)
