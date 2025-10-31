#include "motor.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// --- Pin Definitions ---
// These are now private to the motor driver
#define M1A 8
#define M1B 9
#define M2A 10
#define M2B 11
#define STBY 15

// --- Function Definitions ---

void motor_init_pins(void) {
    const uint pins[] = {M1A, M1B, M2A, M2B, STBY};
    for (int i = 0; i < 5; ++i) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
        gpio_put(pins[i], 0);
    }
    gpio_put(STBY, 1);
}

void motor_stop(void) {
    gpio_put(M1A, 0); gpio_put(M1B, 0);
    gpio_put(M2A, 0); gpio_put(M2B, 0);
}

void motor_forward(void) {
    gpio_put(M1A, 1); gpio_put(M1B, 0);
    gpio_put(M2A, 1); gpio_put(M2B, 0);
}

void motor_backward(void) {
    gpio_put(M1A, 0); gpio_put(M1B, 1);
    gpio_put(M2A, 0); gpio_put(M2B, 1);
}

void motor_left(void) {
    gpio_put(M1A, 0); gpio_put(M1B, 1);
    gpio_put(M2A, 1); gpio_put(M2B, 0);
}

void motor_right(void) {
    gpio_put(M1A, 1); gpio_put(M1B, 0);
    gpio_put(M2A, 0); gpio_put(M2B, 1);
}

void motor_forward_left(void) {
    gpio_put(M1A, 0); gpio_put(M1B, 0);
    gpio_put(M2A, 1); gpio_put(M2B, 0);
}

void motor_forward_right(void) {
    gpio_put(M1A, 1); gpio_put(M1B, 0);
    gpio_put(M2A, 0); gpio_put(M2B, 0);
}

void motor_backward_left(void) {
    gpio_put(M1A, 0); gpio_put(M1B, 0);
    gpio_put(M2A, 0); gpio_put(M2B, 1);
}

void motor_backward_right(void) {
    gpio_put(M1A, 0); gpio_put(M1B, 1);
    gpio_put(M2A, 0); gpio_put(M2B, 0);
}