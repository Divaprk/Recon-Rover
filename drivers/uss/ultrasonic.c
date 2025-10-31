#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"

static const uint32_t PULSE_TIMEOUT_US = 26100; 

static uint32_t getPulseUs(uint trigPin, uint echoPin) {
    gpio_put(trigPin, 0);
    sleep_us(2);
    gpio_put(trigPin, 1);
    sleep_us(10);
    gpio_put(trigPin, 0);

    absolute_time_t t0 = get_absolute_time();
    while (gpio_get(echoPin) == 0) {
        if (absolute_time_diff_us(t0, get_absolute_time()) > PULSE_TIMEOUT_US)
            return 0;
        tight_loop_contents();
    }

    absolute_time_t start = get_absolute_time();
    while (gpio_get(echoPin) == 1) {
        if (absolute_time_diff_us(start, get_absolute_time()) > PULSE_TIMEOUT_US)
            return 0;
        tight_loop_contents();
    }
    absolute_time_t end = get_absolute_time();

    return (uint32_t)absolute_time_diff_us(start, end);
}

void setupUltrasonicPins(uint trigPin, uint echoPin) {
    gpio_init(trigPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_put(trigPin, 0);

    gpio_init(echoPin);
    gpio_set_dir(echoPin, GPIO_IN);
}

int getCm(uint trigPin, uint echoPin) {
    uint32_t us = getPulseUs(trigPin, echoPin);
    if (us == 0) return -1;              
    return (int)(us / 58u);              
}

int getInch(uint trigPin, uint echoPin) {
    uint32_t us = getPulseUs(trigPin, echoPin);
    if (us == 0) return -1;
    return (int)(us / 148u);          
}

bool emergencyStopIfTooClose(uint trigPin, uint echoPin, int threshold_cm) {
    int d = getCm(trigPin, echoPin);
    if (d > 0 && d <= threshold_cm) {
        printf("EMERGENCY: object at %d cm\n", d);
        return true;
    }
    return false;
}
