// include/cyw43_configport.h
#pragma once

// We're using the Pico SDK's CYW43 + LwIP integration
#define CYW43_LWIP 1

// Match the lib you linked: pico_cyw43_arch_lwip_threadsafe_background
#define CYW43_ARCH_THREADSAFE 1

// Optional: use MAC from OTP if available (good default)
#define CYW43_USE_OTP_MAC 1

// Verbose driver logs (0 = off). Set to 1 if you need deep Wi-Fi debugging.
#ifndef CYW43_DEBUG
#define CYW43_DEBUG 0
#endif
