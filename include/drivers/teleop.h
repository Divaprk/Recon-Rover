#pragma once
#include <stdbool.h>
#include <stdint.h>

// Command struct (optional if you prefer text commands only)
typedef struct {
  int16_t throttle;   // -1000..1000 (future PWM version)
  int16_t steer;      // -1000..1000
  uint8_t mode;       // 0=stop, 1=arcade
} teleop_cmd_t;

// Initialize motor GPIO pins (M1A/M1B/M2A/M2B/STBY from pins.h).
void teleop_init(void);

// Immediate text-command API (matches your Python/pygame send strings)
void teleop_apply_text(const char *cmd);

// Emergency stop
void teleop_stop(void);

// Optional structured command API for later (PWM/arcade mix)
void teleop_apply(const teleop_cmd_t *cmd);
