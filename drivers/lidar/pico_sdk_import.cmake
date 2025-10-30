if (NOT PICO_SDK_PATH)
    set(PICO_SDK_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk" CACHE PATH "Path to the Pico SDK")
endif()

if (NOT EXISTS ${PICO_SDK_PATH})
    message(FATAL_ERROR "Pico SDK not found at ${PICO_SDK_PATH}")
endif()

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
