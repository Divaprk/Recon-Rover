#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>

#include "motor.h"

#define STOP_CM         20      // if object <= STOP_CM, start avoid
#define CLEAR_CM        28      // consider path clear if >= CLEAR_CM
#define TIMEOUT_ECHO_US 26000   // ~4.5m round-trip timeout
#define SAMPLE_COUNT    5       // median smoothing
#define STEP_DEG        18      // micro-pivot step (time-based)
#define PIVOT_MS_90     700     // time to pivot ≈90° at current motor power (calibrate!)
#define STEP_MS         220     // creep-forward duration per step
#define PUSH_MS         800     // push ahead when clear
#define BACK_MS         400     // backup if stuck
#define MAX_STEPS       8       // max micro-steps before trying other side

// Internals 
static inline int ms_for_deg(int deg) { return (deg * PIVOT_MS_90) / 90; }

static inline uint32_t pulse_us(void) {
    // 10us trigger
    gpio_put(ULTRA_TRIG_PIN, 1); sleep_us(10); gpio_put(ULTRA_TRIG_PIN, 0);

    // Wait for echo HIGH
    absolute_time_t t0 = get_absolute_time();
    while (gpio_get(ULTRA_ECHO_PIN) == 0) {
        if (absolute_time_diff_us(t0, get_absolute_time()) > TIMEOUT_ECHO_US) return 0;
        tight_loop_contents();
    }
    absolute_time_t start = get_absolute_time();

    // Measure HIGH width
    while (gpio_get(ULTRA_ECHO_PIN) == 1) {
        if (absolute_time_diff_us(start, get_absolute_time()) > TIMEOUT_ECHO_US) return 0;
        tight_loop_contents();
    }
    return (uint32_t)absolute_time_diff_us(start, get_absolute_time());
}

static uint32_t median5(uint32_t a[5]) {
    for (int i=1;i<5;i++){ uint32_t k=a[i]; int j=i-1; while(j>=0 && a[j]>k){a[j+1]=a[j]; j--;} a[j+1]=k; }
    return a[2];
}

// Public 
void ultra_init(void) {
    gpio_init(ULTRA_TRIG_PIN);
    gpio_set_dir(ULTRA_TRIG_PIN, GPIO_OUT);
    gpio_put(ULTRA_TRIG_PIN, 0);

    gpio_init(ULTRA_ECHO_PIN);
    gpio_set_dir(ULTRA_ECHO_PIN, GPIO_IN);
}

uint32_t ultra_read_cm(void) {
    uint32_t v[SAMPLE_COUNT];
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        uint32_t us = pulse_us();
        v[i] = (us == 0) ? 0 : (us / 58); // approx cm
        sleep_ms(8);
    }
    // If too many zeros, median may be 0 → treat as invalid
    uint32_t m = median5(v);
    return m;
}

void ultra_apply_direct(DriveCmd cmd) {
    switch (cmd) {
        case CMD_FORWARD:       motor_forward(); break;
        case CMD_BACKWARD:      motor_backward(); break;
        case CMD_LEFT:          motor_left(); break;
        case CMD_RIGHT:         motor_right(); break;
        case CMD_FWD_LEFT:      motor_forward_left(); break;
        case CMD_FWD_RIGHT:     motor_forward_right(); break;
        case CMD_BWD_LEFT:      motor_backward_left(); break;
        case CMD_BWD_RIGHT:     motor_backward_right(); break;
        case CMD_STOP:
        default:                motor_stop(); break;
    }
}

// Avoidance FSM 
typedef enum { MODE_MANUAL=0, MODE_AVOID } Mode;
typedef enum { AV_IDLE=0, AV_PIVOT, AV_STEP_FWD, AV_RECHECK, AV_RECENTER, AV_BACKUP } AvState;
typedef enum { SIDE_LEFT=0, SIDE_RIGHT=1 } Side;

typedef struct {
    Mode mode;
    AvState st;
    Side side;
    int net_heading_deg;       // +left, -right
    int step_idx;
    absolute_time_t until;     // time that current action ends
} Avoidor;

static Avoidor A = {0};

static inline void set_until_ms(int ms) { A.until = delayed_by_ms(get_absolute_time(), ms); }
static inline bool due(void) { return absolute_time_diff_us(get_absolute_time(), A.until) <= 0; }
static inline void pivot_left_ms(int ms){ motor_left(); set_until_ms(ms); }
static inline void pivot_right_ms(int ms){ motor_right(); set_until_ms(ms); }
static inline void forward_ms(int ms){ motor_forward(); set_until_ms(ms); }
static inline void backward_ms(int ms){ motor_backward(); set_until_ms(ms); }
static inline void brake_ms(int ms){ motor_stop(); set_until_ms(ms); }

static void start_avoid(Side first) {
    A.mode = MODE_AVOID;
    A.st = AV_PIVOT;
    A.side = first;
    A.net_heading_deg = 0;
    A.step_idx = 0;
    brake_ms(1);
}

static void switch_side(void){ A.side = (A.side==SIDE_LEFT)?SIDE_RIGHT:SIDE_LEFT; }

static void recenter_start(void) {
    if (A.net_heading_deg > 0)      pivot_right_ms(ms_for_deg(A.net_heading_deg));
    else if (A.net_heading_deg < 0) pivot_left_ms(ms_for_deg(-A.net_heading_deg));
    else                             brake_ms(1);
}

static void avoidor_tick(void) {
    switch (A.st) {
        case AV_PIVOT: {
            int ms = ms_for_deg(STEP_DEG);
            if (A.side == SIDE_LEFT) { pivot_left_ms(ms);  A.net_heading_deg += STEP_DEG; }
            else                     { pivot_right_ms(ms); A.net_heading_deg -= STEP_DEG; }
            A.st = AV_STEP_FWD;
        } break;

        case AV_STEP_FWD:
            if (!due()) break;
            forward_ms(STEP_MS);
            A.st = AV_RECHECK;
            break;

        case AV_RECHECK:
            if (!due()) break;
            motor_stop();

            {
                uint32_t d = ultra_read_cm();
                printf("[avoid] step %d dist=%lucm\n", A.step_idx+1, (unsigned long)d);

                if (d >= CLEAR_CM) {
                    A.st = AV_RECENTER;
                    recenter_start();
                } else {
                    A.step_idx++;
                    if (A.step_idx < MAX_STEPS) {
                        A.st = AV_PIVOT; // repeat same side
                    } else {
                        // give up on this side → recenter then back up and try other side
                        A.st = AV_BACKUP;
                        recenter_start();
                    }
                }
            }
            break;

        case AV_RECENTER:
            if (!due()) break;
            // push through
            forward_ms(PUSH_MS);
            A.st = AV_IDLE; // after push, let AV_IDLE hand back to manual
            break;

        case AV_BACKUP:
            if (!due()) break;
            backward_ms(BACK_MS);
            A.st = AV_IDLE; // after backup we’ll re-evaluate
            switch_side();
            break;

        case AV_IDLE:
        default:
            if (!due()) break;
            motor_stop();
            if (ultra_read_cm() >= CLEAR_CM) {
                // done → manual again
                A.mode = MODE_MANUAL;
                A.st = AV_IDLE;
            } else {
                // try avoidance again from the top with current side
                A.net_heading_deg = 0;
                A.step_idx = 0;
                A.st = AV_PIVOT;
            }
            break;
    }
}

void ultra_obstacle_aware_apply(DriveCmd desired) {
    uint32_t d = ultra_read_cm();

    if (A.mode == MODE_MANUAL) {
        // Only trigger avoidance if user intends to go forward
        bool wants_forward = (desired == CMD_FORWARD || desired == CMD_FWD_LEFT || desired == CMD_FWD_RIGHT);
        if (wants_forward && d > 0 && d <= STOP_CM) {
            printf("Obstacle at %lucm → AUTO AVOID\n", (unsigned long)d);
            start_avoid(SIDE_LEFT); // start by trying left; it will alternate if stuck
            return; // motors now controlled by FSM
        }
        // Safe or not going forward → pass through
        ultra_apply_direct(desired);
    } else {
        // Avoidance in progress
        avoidor_tick();
    }
}
