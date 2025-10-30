#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "drivers/teleop.h"
#include "pins.h"


// === Motor primitives (from your working code) ===
static inline void motor_stop(void) {
    gpio_put(M1A, 0); gpio_put(M1B, 0);
    gpio_put(M2A, 0); gpio_put(M2B, 0);
}
static inline void motor_forward(void) {
    gpio_put(M1A, 1); gpio_put(M1B, 0);
    gpio_put(M2A, 1); gpio_put(M2B, 0);
}
static inline void motor_backward(void) {
    gpio_put(M1A, 0); gpio_put(M1B, 1);
    gpio_put(M2A, 0); gpio_put(M2B, 1);
}
static inline void motor_left(void) {
    gpio_put(M1A, 0); gpio_put(M1B, 1);
    gpio_put(M2A, 1); gpio_put(M2B, 0);
}
static inline void motor_right(void) {
    gpio_put(M1A, 1); gpio_put(M1B, 0);
    gpio_put(M2A, 0); gpio_put(M2B, 1);
}
static inline void motor_forward_left(void) {
    gpio_put(M1A, 0); gpio_put(M1B, 0);
    gpio_put(M2A, 1); gpio_put(M2B, 0);
}
static inline void motor_forward_right(void) {
    gpio_put(M1A, 1); gpio_put(M1B, 0);
    gpio_put(M2A, 0); gpio_put(M2B, 0);
}
static inline void motor_backward_left(void) {
    gpio_put(M1A, 0); gpio_put(M1B, 0);
    gpio_put(M2A, 0); gpio_put(M2B, 1);
}
static inline void motor_backward_right(void) {
    gpio_put(M1A, 0); gpio_put(M1B, 1);
    gpio_put(M2A, 0); gpio_put(M2B, 0);
}

// Case-insensitive substring match (based on your contains_cmd)
static bool contains_cmd(const char *buf, const char *tok) {
    size_t n = strlen(buf), m = strlen(tok);
    if (!m || n < m) return false;
    for (size_t i = 0; i + m <= n; ++i) {
        bool match = true;
        for (size_t j = 0; j < m; ++j) {
            char a = buf[i + j], b = tok[j];
            if (a >= 'A' && a <= 'Z') a += 32;
            if (b >= 'A' && b <= 'Z') b += 32;
            if (a != b) { match = false; break; }
        }
        if (match) return true;
    }
    return false;
}

void teleop_init(void) {
    const uint pins[] = {M1A, M1B, M2A, M2B, STBY};
    for (int i = 0; i < 5; ++i) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
        gpio_put(pins[i], 0);
    }
    gpio_put(STBY, 1);
    motor_stop();
}

void teleop_apply_text(const char *cmd) {
    if (!cmd) return;
    if      (contains_cmd(cmd, "forward_left"))  motor_forward_left();
    else if (contains_cmd(cmd, "forward_right")) motor_forward_right();
    else if (contains_cmd(cmd, "backward_left")) motor_backward_left();
    else if (contains_cmd(cmd, "backward_right"))motor_backward_right();
    else if (contains_cmd(cmd, "forward"))       motor_forward();
    else if (contains_cmd(cmd, "backward"))      motor_backward();
    else if (contains_cmd(cmd, "left"))          motor_left();
    else if (contains_cmd(cmd, "right"))         motor_right();
    else                                         motor_stop();
}

void teleop_stop(void) { motor_stop(); }

// Placeholder for future PWM/arcade mix if you send structured values
void teleop_apply(const teleop_cmd_t *cmd) {
    (void)cmd;
    // For now, text API drives the motors. Later we can map throttle/steer to PWM here.
}
