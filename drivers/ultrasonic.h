#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>

// ===== Configure your sensor pins here =====
#define ULTRA_TRIG_PIN 2
#define ULTRA_ECHO_PIN 3

// ===== Drive command type (maps to your motor.h functions) =====
typedef enum {
    CMD_STOP = 0,
    CMD_FORWARD,
    CMD_BACKWARD,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_FWD_LEFT,
    CMD_FWD_RIGHT,
    CMD_BWD_LEFT,
    CMD_BWD_RIGHT
} DriveCmd;

// Initialize GPIO for ultrasonic
void ultra_init(void);

// Read distance (cm); 0 means invalid/timeout
uint32_t ultra_read_cm(void);

// Call this every loop with your *desired* command.
// If path is clear, it forwards to motor_*().
// If blocked (within STOP_CM) *and* you're trying to go forward,
// it runs avoidance then hands control back automatically.
void ultra_obstacle_aware_apply(DriveCmd desired);

// (Optional) Quick manual passthrough if you don’t need avoidance:
void ultra_apply_direct(DriveCmd cmd);

#endif // ULTRASONIC_H
