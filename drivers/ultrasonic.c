#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "motor.h"
#include "imu.h"  // <-- ADDED for IMU-based turns

/* ========== THRESHOLDS ========== */
#define OBSTACLE_DETECT_CM  30      // Trigger avoidance
#define OBSTACLE_CLEAR_CM   30      // Path is clear

/* ========== MOVEMENT CALIBRATION ========== */
#define DRIVE_20CM_MS       400     // Time to drive 20cm forward
#define DRIVE_70CM_MS       1400    // Time to drive 70cm forward
#define PAUSE_MS            200     // Pause between actions

/* ========== IMU TURN SETTINGS ========== */
#define TURN_TARGET_DEG     90.0f   // Target turn angle
#define TURN_TOLERANCE_DEG  2.0f    // ±2° tolerance (slightly looser to prevent oscillation)
#define TURN_TIMEOUT_MS     15000   // Max time for a turn (safety, increased for incremental)
#define TURN_BURST_MS       80      // Short burst when far from target
#define TURN_BURST_FINE_MS  40      // Even shorter burst when close to target
#define TURN_SETTLE_MS      250     // Settle time after each burst to read IMU
#define TURN_FINE_THRESHOLD 15.0f   // Switch to fine control when within 15° of target

/* ========== SAFETY LIMITS ========== */
#define MAX_SIDESTEP_CM     200     // Max lateral displacement

/* ========== ULTRASONIC SAMPLING ========== */
#define TIMEOUT_ECHO_US     26000
#define SAMPLE_COUNT        5

/* ========== ULTRASONIC HELPERS ========== */

static inline uint32_t pulse_us(void) {
    gpio_put(ULTRA_TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(ULTRA_TRIG_PIN, 0);
    
    absolute_time_t t0 = get_absolute_time();
    while (gpio_get(ULTRA_ECHO_PIN) == 0) {
        if (absolute_time_diff_us(t0, get_absolute_time()) > TIMEOUT_ECHO_US)
            return 0;
        tight_loop_contents();
    }
    
    absolute_time_t start = get_absolute_time();
    while (gpio_get(ULTRA_ECHO_PIN) == 1) {
        if (absolute_time_diff_us(start, get_absolute_time()) > TIMEOUT_ECHO_US)
            return 0;
        tight_loop_contents();
    }
    
    return (uint32_t)absolute_time_diff_us(start, get_absolute_time());
}

static uint32_t median5(uint32_t a[5]) {
    for (int i = 1; i < 5; i++) {
        uint32_t k = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > k) {
            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = k;
    }
    return a[2];
}

/* ========== PUBLIC API ========== */

void ultra_init(void) {
    gpio_init(ULTRA_TRIG_PIN);
    gpio_set_dir(ULTRA_TRIG_PIN, GPIO_OUT);
    gpio_put(ULTRA_TRIG_PIN, 0);
    
    gpio_init(ULTRA_ECHO_PIN);
    gpio_set_dir(ULTRA_ECHO_PIN, GPIO_IN);
}

uint32_t ultra_read_cm(void) {
    uint32_t v[SAMPLE_COUNT];
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        uint32_t us = pulse_us();
        v[i] = us ? (us / 58) : 0;
        sleep_ms(20);
    }
    
    uint32_t m = median5(v);
    if (m == 0) {
        uint32_t us2 = pulse_us();
        m = us2 ? (us2 / 58) : 0;
    }
    return m;
}

void ultra_apply_direct(DriveCmd cmd) {
    switch (cmd) {
        case CMD_FORWARD:    motor_forward(); break;
        case CMD_BACKWARD:   motor_backward(); break;
        case CMD_LEFT:       motor_left(); break;
        case CMD_RIGHT:      motor_right(); break;
        case CMD_FWD_LEFT:   motor_forward_left(); break;
        case CMD_FWD_RIGHT:  motor_forward_right(); break;
        case CMD_BWD_LEFT:   motor_backward_left(); break;
        case CMD_BWD_RIGHT:  motor_backward_right(); break;
        case CMD_STOP:
        default:             motor_stop(); break;
    }
}

/* ========== BOX NAVIGATION STATE MACHINE ========== */

typedef enum {
    MODE_MANUAL = 0,
    MODE_AVOID
} Mode;

typedef enum {
    // Phase 1: Sidestep left until clear
    AV_IDLE = 0,
    AV_P1_TURN_LEFT,
    AV_P1_TURN_LEFT_WAIT,      // <-- ADDED: Wait for turn to complete
    AV_P1_DRIVE_20CM,
    AV_P1_TURN_RIGHT,
    AV_P1_TURN_RIGHT_WAIT,     // <-- ADDED: Wait for turn to complete
    AV_P1_CHECK_CLEAR,
    
    // Phase 2: Final left clearance
    AV_P2_TURN_LEFT,
    AV_P2_TURN_LEFT_WAIT,      // <-- ADDED
    AV_P2_DRIVE_20CM,
    AV_P2_TURN_RIGHT,
    AV_P2_TURN_RIGHT_WAIT,     // <-- ADDED
    
    // Phase 3: Pass alongside object
    AV_P3_DRIVE_70CM,
    
    // Phase 4: Sidestep right until aligned
    AV_P4_TURN_RIGHT,
    AV_P4_TURN_RIGHT_WAIT,     // <-- ADDED
    AV_P4_CHECK_ALIGNED,
    AV_P4_TURN_LEFT,
    AV_P4_TURN_LEFT_WAIT,      // <-- ADDED
    AV_P4_DRIVE_20CM,
    AV_P4_TURN_RIGHT_BACK,
    AV_P4_TURN_RIGHT_BACK_WAIT, // <-- ADDED
    
    // Phase 5: Return to original path
    AV_P5_TURN_LEFT,
    AV_P5_TURN_LEFT_WAIT,      // <-- ADDED
    AV_P5_DRIVE_20CM,
    AV_P5_TURN_RIGHT,
    AV_P5_TURN_RIGHT_WAIT,     // <-- ADDED
    AV_P5_DRIVE_X_CM,
    AV_COMPLETE
} AvoidState;

typedef struct {
    Mode mode;
    AvoidState state;
    uint32_t lateral_displacement_cm;  // The "x" variable
    absolute_time_t action_until;
    
    // IMU turn tracking
    float turn_start_heading;          // Heading when turn started
    float turn_target_heading;         // Target heading to reach
    absolute_time_t turn_timeout;      // Safety timeout for turns
    bool turning_active;               // Is the motor currently spinning?
} Avoider;

static Avoider A = {0};

/* ========== MOVEMENT HELPERS ========== */

static inline void set_timer_ms(int ms) {
    A.action_until = delayed_by_ms(get_absolute_time(), ms);
}

static inline bool timer_expired(void) {
    return absolute_time_diff_us(get_absolute_time(), A.action_until) <= 0;
}

/* ========== IMU TURN HELPERS ========== */

// Normalize angle to 0-360 range
static inline float normalize_angle(float angle) {
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

// Calculate shortest angular difference between two headings
static inline float angle_difference(float target, float current) {
    float diff = target - current;
    // Normalize to -180 to +180
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return fabsf(diff);
}

// Start a left turn (counterclockwise)
static inline void start_turn_left_90(void) {
    // Read current IMU heading
    imu_vector_t mag;
    imu_read_mag(&mag);
    A.turn_start_heading = imu_calculate_heading(&mag);
    
    // Calculate target (90° counterclockwise = subtract 90°)
    A.turn_target_heading = normalize_angle(A.turn_start_heading - TURN_TARGET_DEG);
    
    // Set timeout
    A.turn_timeout = delayed_by_ms(get_absolute_time(), TURN_TIMEOUT_MS);
    
    // Start first turning burst
    motor_left();
    A.turning_active = true;
    set_timer_ms(TURN_BURST_MS);
    
    printf("  IMU Turn Left: Start=%.1f° Target=%.1f°\n", 
           A.turn_start_heading, A.turn_target_heading);
}

// Start a right turn (clockwise)
static inline void start_turn_right_90(void) {
    // Read current IMU heading
    imu_vector_t mag;
    imu_read_mag(&mag);
    A.turn_start_heading = imu_calculate_heading(&mag);
    
    // Calculate target (90° clockwise = add 90°)
    A.turn_target_heading = normalize_angle(A.turn_start_heading + TURN_TARGET_DEG);
    
    // Set timeout
    A.turn_timeout = delayed_by_ms(get_absolute_time(), TURN_TIMEOUT_MS);
    
    // Start first turning burst
    motor_right();
    A.turning_active = true;
    set_timer_ms(TURN_BURST_MS);
    
    printf("  IMU Turn Right: Start=%.1f° Target=%.1f°\n", 
           A.turn_start_heading, A.turn_target_heading);
}

// Check if turn is complete (incremental turning with pauses and overshoot correction)
static inline bool turn_complete(void) {
    // Safety timeout check
    if (absolute_time_diff_us(get_absolute_time(), A.turn_timeout) <= 0) {
        printf("  [WARNING] Turn timeout! Stopping.\n");
        motor_stop();
        return true;
    }
    
    // If timer hasn't expired, we're either turning or settling
    if (!timer_expired()) {
        return false;
    }
    
    // Timer expired - check what we were doing
    if (A.turning_active) {
        // We just finished a turning burst - STOP and settle
        motor_stop();
        A.turning_active = false;
        set_timer_ms(TURN_SETTLE_MS);  // Wait for IMU to settle
        return false;
    } else {
        // We just finished settling - READ the IMU and decide next action
        imu_vector_t mag;
        imu_read_mag(&mag);
        float current_heading = imu_calculate_heading(&mag);
        
        // Calculate error and direction
        float diff = A.turn_target_heading - current_heading;
        
        // Normalize to -180 to +180
        while (diff > 180.0f) diff -= 360.0f;
        while (diff < -180.0f) diff += 360.0f;
        
        float error = fabsf(diff);  // Absolute error
        bool need_turn_left = (diff < 0.0f);   // Negative = need to turn left
        bool need_turn_right = (diff > 0.0f);  // Positive = need to turn right
        
        printf("  Current=%.1f° Target=%.1f° Error=%.1f° | ", 
               current_heading, A.turn_target_heading, error);
        
        // Check if within tolerance
        if (error <= TURN_TOLERANCE_DEG) {
            printf("DONE!\n");
            motor_stop();
            return true;  // Turn complete!
        }
        
        // Not there yet - decide burst length based on error
        int burst_time;
        if (error < TURN_FINE_THRESHOLD) {
            // Close to target - use fine control (shorter bursts)
            burst_time = TURN_BURST_FINE_MS;
            printf("Fine ");
        } else {
            // Far from target - use normal bursts
            burst_time = TURN_BURST_MS;
            printf("Coarse ");
        }
        
        // SMART: Turn in the direction that reduces error (can reverse if overshot!)
        if (need_turn_left) {
            printf("turning LEFT (%dms)...\n", burst_time);
            motor_left();
        } else if (need_turn_right) {
            printf("turning RIGHT (%dms)...\n", burst_time);
            motor_right();
        }
        
        A.turning_active = true;
        set_timer_ms(burst_time);  // Turn for calculated burst time
        return false;
    }
}

/* ========== MOVEMENT HELPERS ========== */

static inline void do_drive_20cm(void) {
    motor_forward();
    set_timer_ms(DRIVE_20CM_MS);
}

static inline void do_drive_70cm(void) {
    motor_forward();
    set_timer_ms(DRIVE_70CM_MS);
}

static inline void do_drive_x_cm(uint32_t x_cm) {
    motor_forward();
    // Calculate time based on x_cm: (x_cm / 20) * DRIVE_20CM_MS
    uint32_t time_ms = (x_cm * DRIVE_20CM_MS) / 20;
    set_timer_ms(time_ms);
}

static inline void do_pause(void) {
    motor_stop();
    set_timer_ms(PAUSE_MS);
}

static inline void start_avoidance(void) {
    printf("\n=== OBSTACLE DETECTED - STARTING BOX NAVIGATION ===\n");
    A.mode = MODE_AVOID;
    A.state = AV_P1_TURN_LEFT;
    A.lateral_displacement_cm = 0;
    do_pause();
}

/* ========== STATE MACHINE TICK ========== */

static void avoider_tick(void) {
    if (!timer_expired()) return;  // Wait for current action to finish
    
    uint32_t dist;
    
    switch (A.state) {
    
    /* ===== PHASE 1: SIDESTEP LEFT UNTIL CLEAR ===== */
    
    case AV_P1_TURN_LEFT:
        printf("[P1] Turning left 90°\n");
        start_turn_left_90();
        A.state = AV_P1_TURN_LEFT_WAIT;
        break;
        
    case AV_P1_TURN_LEFT_WAIT:
        if (turn_complete()) {
            A.state = AV_P1_DRIVE_20CM;
        }
        break;
        
    case AV_P1_DRIVE_20CM:
        printf("[P1] Driving 20cm left (x = %lu cm)\n", 
               (unsigned long)(A.lateral_displacement_cm + 20));
        A.lateral_displacement_cm += 20;
        do_drive_20cm();
        A.state = AV_P1_TURN_RIGHT;
        break;
        
    case AV_P1_TURN_RIGHT:
        printf("[P1] Turning right 90° (facing object)\n");
        start_turn_right_90();
        A.state = AV_P1_TURN_RIGHT_WAIT;
        break;
        
    case AV_P1_TURN_RIGHT_WAIT:
        if (turn_complete()) {
            A.state = AV_P1_CHECK_CLEAR;
        }
        break;
        
    case AV_P1_CHECK_CLEAR:
        do_pause();
        dist = ultra_read_cm();
        printf("[P1] USS check: %lu cm | ", (unsigned long)dist);
        
        if (dist > OBSTACLE_CLEAR_CM) {
            printf("Path clear! Moving to Phase 2\n");
            A.state = AV_P2_TURN_LEFT;
        } else {
            printf("Still blocked. Continuing left sidestep\n");
            
            // Safety check
            if (A.lateral_displacement_cm >= MAX_SIDESTEP_CM) {
                printf("[ERROR] Max sidestep reached! Aborting.\n");
                A.state = AV_COMPLETE;
            } else {
                A.state = AV_P1_TURN_LEFT;  // Repeat sidestep
            }
        }
        break;
    
    /* ===== PHASE 2: FINAL LEFT CLEARANCE ===== */
    
    case AV_P2_TURN_LEFT:
        printf("[P2] Final left turn 90°\n");
        start_turn_left_90();
        A.state = AV_P2_TURN_LEFT_WAIT;
        break;
        
    case AV_P2_TURN_LEFT_WAIT:
        if (turn_complete()) {
            A.state = AV_P2_DRIVE_20CM;
        }
        break;
        
    case AV_P2_DRIVE_20CM:
        printf("[P2] Final 20cm safety margin (x = %lu cm total)\n",
               (unsigned long)(A.lateral_displacement_cm + 20));
        A.lateral_displacement_cm += 20;
        do_drive_20cm();
        A.state = AV_P2_TURN_RIGHT;
        break;
        
    case AV_P2_TURN_RIGHT:
        printf("[P2] Turning right 90° (facing forward)\n");
        start_turn_right_90();
        A.state = AV_P2_TURN_RIGHT_WAIT;
        break;
        
    case AV_P2_TURN_RIGHT_WAIT:
        if (turn_complete()) {
            A.state = AV_P3_DRIVE_70CM;
        }
        break;
    
    /* ===== PHASE 3: PASS ALONGSIDE OBJECT ===== */
    
    case AV_P3_DRIVE_70CM:
        printf("[P3] Driving 70cm forward (passing object)\n");
        do_drive_70cm();
        A.state = AV_P4_TURN_RIGHT;
        break;
    
    /* ===== PHASE 4: SIDESTEP RIGHT UNTIL ALIGNED ===== */
    
    case AV_P4_TURN_RIGHT:
        printf("[P4] Turning right 90° (facing left side of object)\n");
        start_turn_right_90();
        A.state = AV_P4_TURN_RIGHT_WAIT;
        break;
        
    case AV_P4_TURN_RIGHT_WAIT:
        if (turn_complete()) {
            A.state = AV_P4_CHECK_ALIGNED;
        }
        break;
        
    case AV_P4_CHECK_ALIGNED:
        do_pause();
        dist = ultra_read_cm();
        printf("[P4] USS check: %lu cm | x = %lu cm | ",
               (unsigned long)dist, (unsigned long)A.lateral_displacement_cm);
        
        if (dist > A.lateral_displacement_cm) {
            printf("Not aligned yet. Sidestepping right\n");
            A.state = AV_P4_TURN_LEFT;
        } else {
            printf("Aligned! Moving to Phase 5\n");
            A.state = AV_P5_TURN_LEFT;
        }
        break;
        
    case AV_P4_TURN_LEFT:
        printf("[P4] Turning left 90°\n");
        start_turn_left_90();
        A.state = AV_P4_TURN_LEFT_WAIT;
        break;
        
    case AV_P4_TURN_LEFT_WAIT:
        if (turn_complete()) {
            A.state = AV_P4_DRIVE_20CM;
        }
        break;
        
    case AV_P4_DRIVE_20CM:
        printf("[P4] Driving 20cm right\n");
        do_drive_20cm();
        A.state = AV_P4_TURN_RIGHT_BACK;
        break;
        
    case AV_P4_TURN_RIGHT_BACK:
        printf("[P4] Turning right 90° (facing object again)\n");
        start_turn_right_90();
        A.state = AV_P4_TURN_RIGHT_BACK_WAIT;
        break;
        
    case AV_P4_TURN_RIGHT_BACK_WAIT:
        if (turn_complete()) {
            A.state = AV_P4_CHECK_ALIGNED;
        }
        break;
    
    /* ===== PHASE 5: RETURN TO ORIGINAL PATH ===== */
    
    case AV_P5_TURN_LEFT:
        printf("[P5] Turning left 90° (facing forward)\n");
        start_turn_left_90();
        A.state = AV_P5_TURN_LEFT_WAIT;
        break;
        
    case AV_P5_TURN_LEFT_WAIT:
        if (turn_complete()) {
            A.state = AV_P5_DRIVE_20CM;
        }
        break;
        
    case AV_P5_DRIVE_20CM:
        printf("[P5] Driving 20cm forward\n");
        do_drive_20cm();
        A.state = AV_P5_TURN_RIGHT;
        break;
        
    case AV_P5_TURN_RIGHT:
        printf("[P5] Turning right 90° (facing original path)\n");
        start_turn_right_90();
        A.state = AV_P5_TURN_RIGHT_WAIT;
        break;
        
    case AV_P5_TURN_RIGHT_WAIT:
        if (turn_complete()) {
            A.state = AV_P5_DRIVE_X_CM;
        }
        break;
        
    case AV_P5_DRIVE_X_CM:
        printf("[P5] Driving %lu cm to return to original path\n",
               (unsigned long)A.lateral_displacement_cm);
        do_drive_x_cm(A.lateral_displacement_cm);
        A.state = AV_COMPLETE;
        break;
    
    /* ===== COMPLETE ===== */
    
    case AV_COMPLETE:
        motor_stop();
        printf("=== BOX NAVIGATION COMPLETE - RETURNING TO MANUAL CONTROL ===\n\n");
        A.mode = MODE_MANUAL;
        A.state = AV_IDLE;
        A.lateral_displacement_cm = 0;
        break;
        
    case AV_IDLE:
    default:
        motor_stop();
        A.mode = MODE_MANUAL;
        break;
    }
}

/* ========== MAIN CONTROL FUNCTION ========== */

void ultra_obstacle_aware_apply(DriveCmd desired) {
    if (A.mode == MODE_MANUAL) {
        // Check if user wants to go forward
        bool wants_forward = (desired == CMD_FORWARD || 
                             desired == CMD_FWD_LEFT || 
                             desired == CMD_FWD_RIGHT);
        
        if (wants_forward) {
            uint32_t dist = ultra_read_cm();
            if (dist > 0 && dist <= OBSTACLE_DETECT_CM) {
                // Obstacle detected! Start avoidance
                start_avoidance();
                return;
            }
        }
        
        // Normal manual control
        ultra_apply_direct(desired);
        
    } else {
        // In autonomous avoidance mode
        avoider_tick();
    }
}