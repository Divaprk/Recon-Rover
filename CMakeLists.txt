cmake_minimum_required(VERSION 3.13)

# 0) Board & SDK
# (Do this before pico_sdk_init so toolchain/defines are correct)
set(PICO_BOARD pico_w)  # <-- IMPORTANT for Wi-Fi/cyw43
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(recon_rover C CXX ASM)
pico_sdk_init()

# 1) Sub-libs
add_subdirectory(drivers/teleop)

# 2) App
add_executable(recon_rover
  src/main.c
  src/util.c
  src/net_io.c
)

# Only your project headers (and where lwipopts.h lives)
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
)

pico_enable_stdio_usb(recon_rover 1)
pico_enable_stdio_uart(recon_rover 0)
pico_add_extra_outputs(recon_rover)
