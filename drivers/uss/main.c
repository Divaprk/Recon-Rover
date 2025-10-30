#include "pico/stdlib.h"
#include <stdio.h>
#include "ultrasonic.h"

#define ECHO_PIN 4
#define TRIG_PIN 5

void hcsr04p_init() {
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, true);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, false);
}

float hcsr04p_get_distance_cm() {
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    while (gpio_get(ECHO_PIN) == 0);
    uint32_t start = time_us_32();

    while (gpio_get(ECHO_PIN) == 1);
    uint32_t end = time_us_32();

    uint32_t duration = end - start;
    return duration / 58.0f;
}

int main() {
    stdio_init_all();
    hcsr04p_init();
    sleep_ms(2000);

    while (1) {
        float d = hcsr04p_get_distance_cm();
        printf("Distance: %.2f cm\n", d);
        emergencyStopIfTooClose(TRIG_PIN, ECHO_PIN, 10);
        sleep_ms(100); // change to 100ms
    }
}