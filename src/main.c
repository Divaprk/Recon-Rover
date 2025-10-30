#include <stdio.h>
#include "pico/stdlib.h"
#include "drivers/teleop.h"
#include "net_io.h"

// include per-arch header ONLY
#include "pico/cyw43_arch/arch_threadsafe_background.h"

#define WIFI_SSID "Diva iPhone"
#define WIFI_PASS "91902017"
#define CTRL_PORT 5000

static void on_udp_text(const char* msg) {
    teleop_apply_text(msg);
}

int main(void) {
    stdio_init_all();

    if (cyw43_arch_init()) {
        while (1) tight_loop_contents();
    }

    // (optional) actually connect & start UDP
    // if (!net_init_and_connect(WIFI_SSID, WIFI_PASS)) return -1;
    // net_start_udp(CTRL_PORT, on_udp_text);

    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(250);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(250);
    }
}
