#include <stdio.h>
#include "pico/stdlib.h"
#include "drivers/teleop.h"
#include "net_io.h"
#include "ultrasonic.h"

// include per-arch header ONLY
#include "pico/cyw43_arch/arch_threadsafe_background.h"

#define WIFI_SSID "Diva iPhone"
#define WIFI_PASS "91902017"
#define CTRL_PORT 5000

#define TRIG_PIN        5
#define ECHO_PIN        4
#define FRONT_STOP_CM   25          // stop if closer than this
#define LOOP_MS         40          // ~25 Hz safety checks (USS)

static void on_udp_text(const char* msg) {
    teleop_apply_text(msg);
}

int main(void) {
    stdio_init_all();

    if (cyw43_arch_init()) {
        while (1) tight_loop_contents();
    }

    teleop_init();
    setupUltrasonicPins(TRIG_PIN, ECHO_PIN);

    // (optional) actually connect & start UDP
    // if (!net_init_and_connect(WIFI_SSID, WIFI_PASS)) return -1;
    // net_start_udp(CTRL_PORT, on_udp_text);
 
    absolute_time_t next_led = make_timeout_time_ms(0);
    bool led_on = false;
    
    while (true) {
        // autonomous 
        int d = getCm(TRIG_PIN, ECHO_PIN);       // USS distance in cm
        if (d > 0 && d < FRONT_STOP_CM) {
            teleop_stop();                       // emergency brake
            // full avoidance
            teleop_apply_text("reverse 30");  sleep_ms(300);
            teleop_apply_text("right 35");    sleep_ms(350);
            teleop_apply_text("forward 35");
        }

        if (absolute_time_diff_us(get_absolute_time(), next_led) <= 0) {
            led_on = !led_on;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
            next_led = delayed_by_ms(get_absolute_time(), 500);
        }

        sleep_ms(LOOP_MS);
    }
}
