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
#define TURN_TOLERANCE_DEG  2.0f
#define TURN_TIMEOUT_MS     5000
#define IMU_SAMPLE_MS       50      // Sample IMU every 50ms while turning
#define SETTLE_TIME_MS      500     // Wait time to verify heading

// --- Averaged heading samples ---
#define START_HEADING_SAMPLES 10
#define START_HEADING_DELAY_MS 10

// Power levels for different error ranges
#define TURN_POWER_HIGH     70      // Far from target (>30°)
#define TURN_POWER_MED      45      // Medium distance (15-30°)
#define TURN_POWER_LOW      45      // Close to target (5-15°)
#define TURN_POWER_FINE     45      // Very close (<5°)

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

    // Step 5: Pause for stability
    AV_RETURN_PAUSE_BEFORE_TURN,
    
    // Step 6: Return maneuver
    AV_RETURN_TURN_RIGHT,
    AV_RETURN_DRIVE_LATERAL,
    AV_RETURN_TURN_LEFT,
    
    AV_COMPLETE
} AvoidState;

typedef struct {
    Mode mode;
    AvoidState state;
    absolute_time_t action_until;

    // IMU turn tracking
    float original_heading;     // Captured when obstacle first detected
    float turn_target_heading;  // The absolute compass angle we want right now
    absolute_time_t turn_timeout;
    absolute_time_t next_imu_sample;
    absolute_time_t settle_deadline; // For verify logic

    // Tracks how many sidesteps we've made
    uint8_t lateral_step_count;
} Avoider;

static Avoider A = {0};

/* ========== MOVEMENT HELPERS ========== */

static inline void set_timer_ms(int ms) {
    A.action_until = delayed_by_ms(get_absolute_time(), ms);
}

static inline bool timer_expired(void) {
    return absolute_time_diff_us(get_absolute_time(), A.action_until) <= 0;
}

// Helper to normalize angle to 0-360 range
static inline float normalize_angle(float angle) {
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

/**
 * @brief Reads the IMU multiple times to get a stable, averaged heading.
 */
static float imu_get_averaged_heading(void) {
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    imu_vector_t mag;

    for (int i = 0; i < START_HEADING_SAMPLES; i++) {
        imu_read_mag(&mag);
        float heading_deg = imu_calculate_heading(&mag);
        
        float heading_rad = heading_deg * M_PI / 180.0f;
        sum_x += cosf(heading_rad);
        sum_y += sinf(heading_rad);
        
        sleep_ms(START_HEADING_DELAY_MS);
    }
    
    float avg_x = sum_x / START_HEADING_SAMPLES;
    float avg_y = sum_y / START_HEADING_SAMPLES;
    
    float avg_heading_rad = atan2f(avg_y, avg_x);
    float avg_heading_deg = avg_heading_rad * 180.0f / M_PI;
    
    if (avg_heading_deg < 0.0f) {
        avg_heading_deg += 360.0f;
    }
    return avg_heading_deg;
}

/* ========== IMU TURN LOGIC (ABSOLUTE) ========== */

// Generic function to start turning toward a specific absolute compass heading
static void start_turn_to_heading(float target_deg) {
    A.turn_target_heading = normalize_angle(target_deg);
    A.turn_timeout = delayed_by_ms(get_absolute_time(), TURN_TIMEOUT_MS);
    A.next_imu_sample = get_absolute_time();
    A.settle_deadline = nil_time; // Reset verify timer

    // Decide which way to start turning
    float current = imu_get_averaged_heading();
    float diff = A.turn_target_heading - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    if (diff < 0) {
        motor_left_pwm(TURN_POWER_HIGH);
        printf("  Turn LEFT to Abs Target %.1f° (Curr=%.1f°)\n", A.turn_target_heading, current);
    } else {
        motor_right_pwm(TURN_POWER_HIGH);
        printf("  Turn RIGHT to Abs Target %.1f° (Curr=%.1f°)\n", A.turn_target_heading, current);
    }
}

static inline bool turn_complete(void) {
    // 1. Safety Timeout
    if (absolute_time_diff_us(get_absolute_time(), A.turn_timeout) <= 0) {
        printf("  [WARNING] Turn timeout! Stopping.\n");
        motor_stop();
        return true;
    }
    
    // 2. Rate Limiter
    if (absolute_time_diff_us(get_absolute_time(), A.next_imu_sample) > 0) {
        return false;
    }
    A.next_imu_sample = delayed_by_ms(get_absolute_time(), IMU_SAMPLE_MS);
    
    // 3. Read Heading
    imu_vector_t mag;
    imu_read_mag(&mag);
    float current_heading = imu_calculate_heading(&mag);
    
    // 4. Calculate Error
    float diff = A.turn_target_heading - current_heading;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    float error = fabsf(diff);

    // ============================================================
    //                 VERIFICATION LOGIC
    // ============================================================

    if (error <= TURN_TOLERANCE_DEG) {
        // We hit the target range.
        if (is_nil_time(A.settle_deadline)) {
            // Start verification timer
            printf("  Target hit (Err=%.1f). Stopping to verify...\n", error);
            motor_stop();
            A.settle_deadline = delayed_by_ms(get_absolute_time(), SETTLE_TIME_MS);
            return false;
        }
        
        // Wait for timer
        if (absolute_time_diff_us(get_absolute_time(), A.settle_deadline) > 0) {
            return false;
        }
        
        // Timer expired and we are still good!
        printf("  Verified! Final Heading=%.1f (Err=%.1f). DONE.\n", current_heading, error);
        return true;
    }
    
    // ============================================================
    //                 ADJUSTMENT LOGIC
    // ============================================================

    if (!is_nil_time(A.settle_deadline)) {
        printf("  Drift detected (Err=%.1f). Re-adjusting...\n", error);
        A.settle_deadline = nil_time; 
    }

    bool need_turn_left = (diff < 0.0f);
    
    uint8_t power;
    if (error > TURN_ERROR_HIGH)      power = TURN_POWER_HIGH;
    else if (error > TURN_ERROR_MED)  power = TURN_POWER_MED;
    else if (error > TURN_ERROR_LOW)  power = TURN_POWER_LOW;
    else                              power = TURN_POWER_FINE; 

    if (need_turn_left) {
        motor_left_pwm(power);
        if (error > 2.0f) printf("  Adj Left: Err=%.1f Pwr=%d\n", error, power);
    } else {
        motor_right_pwm(power);
        if (error > 2.0f) printf("  Adj Right: Err=%.1f Pwr=%d\n", error, power);
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
    printf("\n=== OBSTACLE DETECTED - STARTING ABSOLUTE AVOIDANCE ===\n");
    A.mode = MODE_AVOID;
    A.state = AV_TURN_LEFT;
    A.lateral_step_count = 0; 
    
    // --- CAPTURE THE "FORWARD" HEADING ---
    A.original_heading = imu_get_averaged_heading();
    printf("  [ORIGIN LOCKED] Heading: %.1f°\n", A.original_heading);

    set_timer_ms(PAUSE_MS);
}

/* ========== STATE MACHINE TICK ========== */

static void avoider_tick(void) {
    uint32_t dist;
    
    // Pre-calculate target headings based on Origin
    float heading_left   = normalize_angle(A.original_heading - 90.0f);
    float heading_right  = normalize_angle(A.original_heading + 90.0f);
    float heading_fwd    = A.original_heading;

    switch (A.state) {
    
    /* ===== STEP 1: STOP AND TURN LEFT (Away from Origin) ===== */
    case AV_TURN_LEFT:
        if (!timer_expired()) return;
        printf("[Step 1] Turning Left to %.1f°\n", heading_left);
        start_turn_to_heading(heading_left);
        A.state = AV_DRIVE_FORWARD_STEP2;
        break;
    
    /* ===== STEP 2: DRIVE 20CM, TURN RIGHT (Back to Forward) ===== */
    case AV_DRIVE_FORWARD_STEP2:
        if (!turn_complete()) return;
        printf("[Step 2] Moving forward 20cm\n");
        do_drive_20cm();
        A.lateral_step_count++;
        A.state = AV_TURN_RIGHT_STEP2;
        break;
        
    case AV_TURN_RIGHT_STEP2:
        if (!timer_expired()) return;
        printf("[Step 2] Aligning Forward (%.1f°)\n", heading_fwd);
        start_turn_to_heading(heading_fwd);
        A.state = AV_CHECK_CLEAR;
        break;
    
    /* ===== STEP 3: CHECK ===== */
    case AV_CHECK_CLEAR:
        if (!turn_complete()) return;
        
        dist = ultra_read_cm();
        printf("[Step 3] Dist: %lu cm. ", (unsigned long)dist);
        
        if (dist > OBSTACLE_CLEAR_CM) {
            printf("Clear! Proceeding.\n");
            set_timer_ms(PAUSE_MS);
            A.state = AV_TURN_LEFT_STEP4;
        } else {
            printf("Blocked. Repeating Step 1.\n");
            set_timer_ms(PAUSE_MS);
            // Rover is currently facing FWD. It will turn LEFT again in next state.
            A.state = AV_TURN_LEFT; 
        }
        break;
    
    /* ===== STEP 4: WIDEN AND PASS ===== */
    case AV_TURN_LEFT_STEP4:
        if (!timer_expired()) return;
        printf("[Step 4] Turning Left to %.1f°\n", heading_left);
        start_turn_to_heading(heading_left);
        A.state = AV_DRIVE_SLIGHTLY_STEP4;
        break;
        
    case AV_DRIVE_SLIGHTLY_STEP4:
        if (!turn_complete()) return;
        printf("[Step 4] Extra width 20cm\n");
        do_drive_20cm();
        A.lateral_step_count++;
        A.state = AV_TURN_RIGHT_STEP4;
        break;
        
    case AV_TURN_RIGHT_STEP4:
        if (!timer_expired()) return;
        printf("[Step 4] Aligning Forward to %.1f°\n", heading_fwd);
        start_turn_to_heading(heading_fwd);
        A.state = AV_DRIVE_FORWARD_FINAL;
        break;
    
    case AV_DRIVE_FORWARD_FINAL:
        if (!turn_complete()) return;
        printf("[Step 4] Passing Obstacle (70cm)\n");
        do_drive_70cm();
        A.state = AV_RETURN_PAUSE_BEFORE_TURN; 
        break;
    
    /* ===== STEP 5: RETURN LOGIC ===== */
    case AV_RETURN_PAUSE_BEFORE_TURN:
        if (!timer_expired()) return;
        printf("[Step 5] Pausing...\n");
        do_pause();
        A.state = AV_RETURN_TURN_RIGHT;
        break;

    case AV_RETURN_TURN_RIGHT:
        if (!timer_expired()) return;
        // Turn toward the return path (Original + 90)
        printf("[Step 5] Turning Right to %.1f°\n", heading_right);
        start_turn_to_heading(heading_right);
        A.state = AV_RETURN_DRIVE_LATERAL;
        break;

    case AV_RETURN_DRIVE_LATERAL:
        if (!turn_complete()) return;
        
        uint32_t return_drive_ms = A.lateral_step_count * DRIVE_20CM_MS;
        printf("[Step 6] Returning Lateral (N=%u, %lu ms)\n",
               A.lateral_step_count, (unsigned long)return_drive_ms);
               
        motor_forward();
        set_timer_ms(return_drive_ms);
        A.state = AV_RETURN_TURN_LEFT;
        break;

    case AV_RETURN_TURN_LEFT:
        if (!timer_expired()) return;
        // Final turn: Face Original Heading
        printf("[Step 7] Re-aligning to Origin %.1f°\n", heading_fwd);
        start_turn_to_heading(heading_fwd);
        A.state = AV_COMPLETE;
        break;
    
    /* ===== COMPLETE ===== */
    case AV_COMPLETE:
        if (!turn_complete()) return; 
        motor_stop();
        printf("=== AVOIDANCE COMPLETE ===\n\n");
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
        
        if (wants_forward) {
            uint32_t dist = ultra_read_cm();
            if (dist > 0 && dist <= OBSTACLE_DETECT_CM) {
                start_avoidance();
                return;
            }
        }
        ultra_apply_direct(desired);
    } else {
        avoider_tick();
    }
}