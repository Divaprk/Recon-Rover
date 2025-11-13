#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

// Call this once in main() to set up the motor pins
void motor_init_pins(void);

// Motor action commands (digital on/off)
void motor_stop(void);
void motor_forward(void);
void motor_backward(void);
void motor_left(void);
void motor_right(void);
void motor_forward_left(void);
void motor_forward_right(void);
void motor_backward_left(void);
void motor_backward_right(void);

// PWM motor control (0-100% duty cycle)
void motor_left_pwm(uint8_t duty_percent);
void motor_right_pwm(uint8_t duty_percent);

#endif // MOTOR_H