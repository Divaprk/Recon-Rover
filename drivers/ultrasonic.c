#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "motor.h"
#include "imu.h"

/* ========== THRESHOLDS ========== */
#define OBSTACLE_DETECT_CM  30
#define OBSTACLE_CLEAR_CM   30

/* ========== MOVEMENT CALIBRATION ========== */
#define DRIVE_20CM_MS       400
#define DRIVE_70CM_MS       2500
#define PAUSE_MS            200
#define TURN_TARGET_DEG     90.0f
#define TURN_TOLERANCE_DEG  2.0f
#define TURN_TIMEOUT_MS     15000
#define IMU_SAMPLE_MS       50      // Sample IMU every 50ms while turning

// Power levels for different error ranges
#define TURN_POWER_HIGH     70      // Far from target (>30°)
#define TURN_POWER_MED      45      // Medium distance (15-30°)
#define TURN_POWER_LOW      30      // Close to target (5-15°)
#define TURN_POWER_FINE     20      // Very close (<5°)

#define TURN_ERROR_HIGH     30.0f
#define TURN_ERROR_MED      15.0f
#define TURN_ERROR_LOW      5.0f

/* ========== SAFETY LIMITS ========== */
#define MAX_SIDESTEP_CM     200

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

/* ========== SIMPLIFIED AVOIDANCE STATE MACHINE ========== */

typedef enum {
    MODE_MANUAL = 0,
    MODE_AVOID
} Mode;

typedef enum {
    AV_IDLE = 0,
    
    // Step 1: Stop and turn left 90°
    AV_TURN_LEFT,
    
    // Step 2: Move forward, stop, turn right 90°
    AV_DRIVE_FORWARD_STEP2,
    AV_TURN_RIGHT_STEP2,
    
    // Step 3: Check if obstacle is still there
    AV_CHECK_CLEAR,
    
    // Step 4: If clear - turn left, drive slightly, turn right, drive forward
    AV_TURN_LEFT_STEP4,
    AV_DRIVE_SLIGHTLY_STEP4,
    AV_TURN_RIGHT_STEP4,
    AV_DRIVE_FORWARD_FINAL,
    
    AV_COMPLETE
} AvoidState;

typedef struct {
    Mode mode;
    AvoidState state;
    absolute_time_t action_until;

    // IMU turn tracking
    float turn_start_heading;
    float turn_target_heading;
    absolute_time_t turn_timeout;
    absolute_time_t next_imu_sample;
} Avoider;

static Avoider A = {0};

/* ========== MOVEMENT HELPERS ========== */

static inline void set_timer_ms(int ms) {
    A.action_until = delayed_by_ms(get_absolute_time(), ms);
}

static inline bool timer_expired(void) {
    return absolute_time_diff_us(get_absolute_time(), A.action_until) <= 0;
}

<<<<<<< Updated upstream
=======
/* ========== IMU TURN HELPERS ========== */

static inline float normalize_angle(float angle) {
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

static inline float angle_difference(float target, float current) {
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return fabsf(diff);
}

static inline void start_turn_left_90(void) {
    imu_vector_t mag;
    imu_read_mag(&mag);
    A.turn_start_heading = imu_calculate_heading(&mag);
    A.turn_target_heading = normalize_angle(A.turn_start_heading - TURN_TARGET_DEG);
    A.turn_timeout = delayed_by_ms(get_absolute_time(), TURN_TIMEOUT_MS);
    A.next_imu_sample = get_absolute_time();
    
    motor_left_pwm(TURN_POWER_HIGH);
    
    printf("  PWM Turn Left: Start=%.1f° Target=%.1f°\n", 
           A.turn_start_heading, A.turn_target_heading);
}

static inline void start_turn_right_90(void) {
    imu_vector_t mag;
    imu_read_mag(&mag);
    A.turn_start_heading = imu_calculate_heading(&mag);
    A.turn_target_heading = normalize_angle(A.turn_start_heading + TURN_TARGET_DEG);
    A.turn_timeout = delayed_by_ms(get_absolute_time(), TURN_TIMEOUT_MS);
    A.next_imu_sample = get_absolute_time();
    
    motor_right_pwm(TURN_POWER_HIGH);
    
    printf("  PWM Turn Right: Start=%.1f° Target=%.1f°\n", 
           A.turn_start_heading, A.turn_target_heading);
}

static inline bool turn_complete(bool turn_right) {
    // Safety timeout
    if (absolute_time_diff_us(get_absolute_time(), A.turn_timeout) <= 0) {
        printf("  [WARNING] Turn timeout! Stopping.\n");
        motor_stop();
        return true;
    }
    
    // Only sample IMU at specified intervals
    if (absolute_time_diff_us(get_absolute_time(), A.next_imu_sample) > 0) {
        return false;
    }
    A.next_imu_sample = delayed_by_ms(get_absolute_time(), IMU_SAMPLE_MS);
    
    // Read current heading
    imu_vector_t mag;
    imu_read_mag(&mag);
    float current_heading = imu_calculate_heading(&mag);
    
    // Calculate error
    float diff = A.turn_target_heading - current_heading;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    
    float error = fabsf(diff);
    bool need_turn_left = (diff < 0.0f);
    bool need_turn_right = (diff > 0.0f);
    
    // Check if we've reached target
    if (error <= TURN_TOLERANCE_DEG) {
        printf("  Current=%.1f° Target=%.1f° Error=%.1f° - DONE!\n", 
               current_heading, A.turn_target_heading, error);
        motor_stop();
        return true;
    }
    
    // Adjust power based on error (proportional control)
    uint8_t power;
    if (error > TURN_ERROR_HIGH) {
        power = TURN_POWER_HIGH;
    } else if (error > TURN_ERROR_MED) {
        power = TURN_POWER_MED;
    } else if (error > TURN_ERROR_LOW) {
        power = TURN_POWER_LOW;
    } else {
        power = TURN_POWER_FINE;
    }
    
    // Apply turn in correct direction (can reverse if overshot)
    if (need_turn_left) {
        motor_left_pwm(power);
        printf("  Current=%.1f° Target=%.1f° Error=%.1f° - LEFT @ %d%%\n", 
               current_heading, A.turn_target_heading, error, power);
    } else if (need_turn_right) {
        motor_right_pwm(power);
        printf("  Current=%.1f° Target=%.1f° Error=%.1f° - RIGHT @ %d%%\n", 
               current_heading, A.turn_target_heading, error, power);
    }
    
    return false;
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

static inline void do_pause(void) {
    motor_stop();
    set_timer_ms(PAUSE_MS);
}

static inline void start_avoidance(void) {
    printf("\n=== OBSTACLE DETECTED - STARTING AVOIDANCE ===\n");
    printf("[Step 1] Stopping and preparing to turn left\n");
    A.mode = MODE_AVOID;
    A.state = AV_TURN_LEFT;
    set_timer_ms(PAUSE_MS);  // Changed from do_pause() to explicit timer
}

/* ========== STATE MACHINE TICK ========== */

static void avoider_tick(void) {
    uint32_t dist;

    switch (A.state) {
    
    /* ===== STEP 1: STOP AND TURN LEFT 90° ===== */
    
    case AV_TURN_LEFT:
        if (!timer_expired()) return;
        printf("[Step 1] Turning left 90°\n");
        start_turn_left_90();
        A.state = AV_DRIVE_FORWARD_STEP2;
        break;
    
    /* ===== STEP 2: MOVE FORWARD, STOP, TURN RIGHT 90° ===== */
    
    case AV_DRIVE_FORWARD_STEP2:
        if (!turn_complete(false)) return;
        printf("[Step 2] Moving forward\n");
        do_drive_20cm();
        A.state = AV_TURN_RIGHT_STEP2;
        break;
        
    case AV_TURN_RIGHT_STEP2:
        if (!timer_expired()) return;
        printf("[Step 2] Stopping and turning right 90°\n");
        start_turn_right_90();
        A.state = AV_CHECK_CLEAR;
        break;
    
    /* ===== STEP 3: CHECK IF OBSTACLE IS STILL THERE ===== */
    
    case AV_CHECK_CLEAR:
        if (!turn_complete(true)) return;
        
        // Quick 1 second check - no waiting
        dist = ultra_read_cm();
        printf("[Step 3] Checking if obstacle is clear: %lu cm | ", (unsigned long)dist);
        
        if (dist > OBSTACLE_CLEAR_CM) {
            printf("Clear! Moving to Step 4\n");
            set_timer_ms(PAUSE_MS);
            A.state = AV_TURN_LEFT_STEP4;
        } else {
            printf("Still blocked. Repeating from Step 1\n");
            set_timer_ms(PAUSE_MS);
            A.state = AV_TURN_LEFT;  // Go back to Step 1
        }
        break;
    
    /* ===== STEP 4: TURN LEFT, DRIVE SLIGHTLY, TURN RIGHT, DRIVE FORWARD ===== */
    
    case AV_TURN_LEFT_STEP4:
        if (!timer_expired()) return;
        printf("[Step 4] Turning left 90°\n");
        start_turn_left_90();
        A.state = AV_DRIVE_SLIGHTLY_STEP4;
        break;
        
    case AV_DRIVE_SLIGHTLY_STEP4:
        if (!turn_complete(false)) return;
        printf("[Step 4] Moving forward slightly\n");
        do_drive_20cm();
        A.state = AV_TURN_RIGHT_STEP4;
        break;
        
    case AV_TURN_RIGHT_STEP4:
        if (!timer_expired()) return;
        printf("[Step 4] Turning right 90°\n");
        start_turn_right_90();
        A.state = AV_DRIVE_FORWARD_FINAL;
        break;
        
    case AV_DRIVE_FORWARD_FINAL:
        if (!turn_complete(true)) return;
        printf("[Step 4] Moving forward to pass obstacle\n");
        do_drive_70cm();
        A.state = AV_COMPLETE;
        break;
    
>>>>>>> Stashed changes
    /* ===== COMPLETE ===== */
    case AV_COMPLETE:
        if (!timer_expired()) return;
        motor_stop();
        printf("=== AVOIDANCE COMPLETE - RETURNING CONTROL TO USER ===\n\n");
        A.mode = MODE_MANUAL;
        A.state = AV_IDLE;
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
        bool wants_forward = (desired == CMD_FORWARD || 
                             desired == CMD_FWD_LEFT || 
                             desired == CMD_FWD_RIGHT);
        
>>>>>>> Stashed changes
        if (wants_forward) {
            uint32_t dist = ultra_read_cm();
            if (dist > 0 && dist <= OBSTACLE_DETECT_CM) {
                start_avoidance();
                return;
            }
        }
        
        ultra_apply_direct(desired);

    } else {
        // In autonomous avoidance mode - keep calling avoider_tick()
        avoider_tick();
    }
}
