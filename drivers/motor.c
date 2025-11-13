#include "motor.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

// --- Pin Definitions ---
#define M1A 8
#define M1B 9
#define M2A 10
#define M2B 11
#define STBY 15

// --- PWM Configuration ---
static uint slice_m1a, slice_m1b, slice_m2a, slice_m2b;
static uint chan_m1a, chan_m1b, chan_m2a, chan_m2b;

// --- Function Definitions ---

void motor_init_pins(void) {
    const uint pins[] = {M1A, M1B, M2A, M2B, STBY};
    
    // Initialize all pins as GPIO first
    for (int i = 0; i < 5; ++i) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
        gpio_put(pins[i], 0);
    }
    gpio_put(STBY, 1);
    
    // Configure PWM for motor pins
    gpio_set_function(M1A, GPIO_FUNC_PWM);
    gpio_set_function(M1B, GPIO_FUNC_PWM);
    gpio_set_function(M2A, GPIO_FUNC_PWM);
    gpio_set_function(M2B, GPIO_FUNC_PWM);
    
    slice_m1a = pwm_gpio_to_slice_num(M1A);
    slice_m1b = pwm_gpio_to_slice_num(M1B);
    slice_m2a = pwm_gpio_to_slice_num(M2A);
    slice_m2b = pwm_gpio_to_slice_num(M2B);
    
    chan_m1a = pwm_gpio_to_channel(M1A);
    chan_m1b = pwm_gpio_to_channel(M1B);
    chan_m2a = pwm_gpio_to_channel(M2A);
    chan_m2b = pwm_gpio_to_channel(M2B);
    
    // Set PWM frequency to ~20kHz (good for motors)
    // System clock is 125MHz, divider of 1, wrap of 6249 gives ~20kHz
    pwm_set_wrap(slice_m1a, 6249);
    pwm_set_wrap(slice_m1b, 6249);
    pwm_set_wrap(slice_m2a, 6249);
    pwm_set_wrap(slice_m2b, 6249);
    
    pwm_set_clkdiv(slice_m1a, 1.0f);
    pwm_set_clkdiv(slice_m1b, 1.0f);
    pwm_set_clkdiv(slice_m2a, 1.0f);
    pwm_set_clkdiv(slice_m2b, 1.0f);
    
    // Start all PWM slices
    pwm_set_enabled(slice_m1a, true);
    pwm_set_enabled(slice_m1b, true);
    pwm_set_enabled(slice_m2a, true);
    pwm_set_enabled(slice_m2b, true);
}

static inline void set_pwm(uint slice, uint chan, uint8_t duty_percent) {
    uint16_t level = (duty_percent * 6249) / 100;
    pwm_set_chan_level(slice, chan, level);
}

void motor_stop(void) {
    set_pwm(slice_m1a, chan_m1a, 0);
    set_pwm(slice_m1b, chan_m1b, 0);
    set_pwm(slice_m2a, chan_m2a, 0);
    set_pwm(slice_m2b, chan_m2b, 0);
}

void motor_forward(void) {
    set_pwm(slice_m1a, chan_m1a, 100);
    set_pwm(slice_m1b, chan_m1b, 0);
    set_pwm(slice_m2a, chan_m2a, 100);
    set_pwm(slice_m2b, chan_m2b, 0);
}

void motor_backward(void) {
    set_pwm(slice_m1a, chan_m1a, 0);
    set_pwm(slice_m1b, chan_m1b, 100);
    set_pwm(slice_m2a, chan_m2a, 0);
    set_pwm(slice_m2b, chan_m2b, 100);
}

void motor_left(void) {
    set_pwm(slice_m1a, chan_m1a, 0);
    set_pwm(slice_m1b, chan_m1b, 100);
    set_pwm(slice_m2a, chan_m2a, 100);
    set_pwm(slice_m2b, chan_m2b, 0);
}

void motor_right(void) {
    set_pwm(slice_m1a, chan_m1a, 100);
    set_pwm(slice_m1b, chan_m1b, 0);
    set_pwm(slice_m2a, chan_m2a, 0);
    set_pwm(slice_m2b, chan_m2b, 100);
}

void motor_forward_left(void) {
    set_pwm(slice_m1a, chan_m1a, 0);
    set_pwm(slice_m1b, chan_m1b, 0);
    set_pwm(slice_m2a, chan_m2a, 100);
    set_pwm(slice_m2b, chan_m2b, 0);
}

void motor_forward_right(void) {
    set_pwm(slice_m1a, chan_m1a, 100);
    set_pwm(slice_m1b, chan_m1b, 0);
    set_pwm(slice_m2a, chan_m2a, 0);
    set_pwm(slice_m2b, chan_m2b, 0);
}

void motor_backward_left(void) {
    set_pwm(slice_m1a, chan_m1a, 0);
    set_pwm(slice_m1b, chan_m1b, 0);
    set_pwm(slice_m2a, chan_m2a, 0);
    set_pwm(slice_m2b, chan_m2b, 100);
}

void motor_backward_right(void) {
    set_pwm(slice_m1a, chan_m1a, 0);
    set_pwm(slice_m1b, chan_m1b, 100);
    set_pwm(slice_m2a, chan_m2a, 0);
    set_pwm(slice_m2b, chan_m2b, 0);
}

// PWM turn functions with variable power
void motor_left_pwm(uint8_t duty_percent) {
    if (duty_percent > 100) duty_percent = 100;
    set_pwm(slice_m1a, chan_m1a, 0);
    set_pwm(slice_m1b, chan_m1b, duty_percent);
    set_pwm(slice_m2a, chan_m2a, duty_percent);
    set_pwm(slice_m2b, chan_m2b, 0);
}

void motor_right_pwm(uint8_t duty_percent) {
    if (duty_percent > 100) duty_percent = 100;
    set_pwm(slice_m1a, chan_m1a, duty_percent);
    set_pwm(slice_m1b, chan_m1b, 0);
    set_pwm(slice_m2a, chan_m2a, 0);
    set_pwm(slice_m2b, chan_m2b, duty_percent);
}