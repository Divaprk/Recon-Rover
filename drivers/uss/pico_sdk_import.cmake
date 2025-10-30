# Uses PICO_SDK_PATH to locate the SDK.

if (DEFINED ENV{PICO_SDK_PATH} AND NOT PICO_SDK_PATH)
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()

if (NOT PICO_SDK_PATH)
    message(FATAL_ERROR
        "PICO_SDK_PATH is not set. Set it to your pico-sdk folder, e.g. C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk")
endif()

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
