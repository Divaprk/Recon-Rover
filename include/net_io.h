#pragma once
#include <stdbool.h>
#include <stdint.h>

// Signature for your UDP text-command callback
typedef void (*udp_text_cb_t)(const char* msg);

// Init Wi-Fi (STA) and connect. Returns true on success.
bool net_init_and_connect(const char* ssid, const char* pass);

// Start a UDP listener on `port` and call `on_text` for each packet.
// Returns true on success.
bool net_start_udp(uint16_t port, udp_text_cb_t on_text);
