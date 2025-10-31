cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w)
include(pico_sdk_import.cmake)

project(recon_rover C CXX ASM)
pico_sdk_init()

add_subdirectory(drivers/teleop)
add_subdirectory(drivers/uss)          

add_executable(recon_rover
  src/main.c
  src/util.c
  src/net_io.c
)                         

target_include_directories(recon_rover PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(recon_rover
  pico_stdlib
  hardware_gpio
  hardware_timer
  hardware_i2c
  hardware_uart
  hardware_pwm
  pico_lwip
  pico_cyw43_arch_lwip_threadsafe_background
  pico_async_context
  pico_rand
  teleop_driver
  uss                                   
)

pico_enable_stdio_usb(recon_rover 1)
pico_enable_stdio_uart(recon_rover 0)
pico_add_extra_outputs(recon_rover)
