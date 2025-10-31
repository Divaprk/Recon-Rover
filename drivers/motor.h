#ifndef MOTOR_H
#define MOTOR_H

// Call this once in main() to set up the motor pins
void motor_init_pins(void);

// Motor action commands
void motor_stop(void);
void motor_forward(void);
void motor_backward(void);
void motor_left(void);
void motor_right(void);
void motor_forward_left(void);
void motor_forward_right(void);
void motor_backward_left(void);
void motor_backward_right(void);

#endif // MOTOR_H