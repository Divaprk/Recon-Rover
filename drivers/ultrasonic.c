#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include "motor.h"

#define STOP_CM         20
#define CLEAR_CM        28
#define TIMEOUT_ECHO_US 26000
#define SAMPLE_COUNT    5
#define PIVOT_MS_90     700     
#define DRIVE_MS        450     // how long to move during the side detour
#define CHECK_PAUSE_MS  120     // pause before measuring distance
#define PUSH_MS         800
#define BACK_MS         400

// Pins come from ultrasonic.h
static inline int ms_for_deg(int deg) { return (deg * PIVOT_MS_90) / 90; }

/* ---------------- Ultrasonic sampling ---------------- */
static inline uint32_t pulse_us(void) {
    gpio_put(ULTRA_TRIG_PIN, 1); sleep_us(10); gpio_put(ULTRA_TRIG_PIN, 0);
    absolute_time_t t0 = get_absolute_time();
    while (gpio_get(ULTRA_ECHO_PIN) == 0) {
        if (absolute_time_diff_us(t0, get_absolute_time()) > TIMEOUT_ECHO_US) return 0;
        tight_loop_contents();
    }
    absolute_time_t start = get_absolute_time();
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

/* ---------------- Public API ---------------- */
void ultra_init(void) {
    gpio_init(ULTRA_TRIG_PIN); gpio_set_dir(ULTRA_TRIG_PIN, GPIO_OUT); gpio_put(ULTRA_TRIG_PIN, 0);
    gpio_init(ULTRA_ECHO_PIN); gpio_set_dir(ULTRA_ECHO_PIN, GPIO_IN);
}

uint32_t ultra_read_cm(void) {
    uint32_t v[SAMPLE_COUNT];
    for (int i=0;i<SAMPLE_COUNT;i++){ uint32_t us = pulse_us(); v[i] = us ? (us/58) : 0; sleep_ms(8); }
    return median5(v);
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

/* ---------------- Deterministic sidestep FSM ---------------- */
typedef enum { MODE_MANUAL=0, MODE_AVOID } Mode;
typedef enum {
    AV_IDLE=0,
    AV_TURN_90,        // turn left/right 90°
    AV_DRIVE_SIDE,     // drive forward a bit while sideways
    AV_TURN_BACK_90,   // turn back 90° to re-center
    AV_PAUSE_CHECK,    // short stop -> recheck distance
    AV_DECIDE,         // choose next action based on distance
    AV_PUSH_THROUGH,   // run forward a bit then hand back to manual
    AV_BACKUP,         // back up after failing both sides
    AV_TRY_OTHER_SIDE  // switch side and repeat
} AvState;
typedef enum { SIDE_LEFT=0, SIDE_RIGHT=1 } Side;

typedef struct {
    Mode mode;
    AvState st;
    Side side;              // which side we are trying now
    bool tried_left;        // have we already tried left?
    bool tried_right;       // have we already tried right?
    absolute_time_t until;  // when current timed action ends
} Avoidor;

static Avoidor A = {0};

static inline void set_until_ms(int ms){ A.until = delayed_by_ms(get_absolute_time(), ms); }
static inline bool due(void){ return absolute_time_diff_us(get_absolute_time(), A.until) <= 0; }

/* timed motor helpers */
static inline void do_left_90(void){ motor_left();     set_until_ms(ms_for_deg(90)); }
static inline void do_right_90(void){ motor_right();   set_until_ms(ms_for_deg(90)); }
static inline void do_drive_side(void){ motor_forward(); set_until_ms(DRIVE_MS); }
static inline void do_turnback_90(Side s){
    if (s==SIDE_LEFT)  do_right_90();
    else               do_left_90();
}
static inline void do_pause_check(void){ motor_stop(); set_until_ms(CHECK_PAUSE_MS); }
static inline void do_push(void){ motor_forward(); set_until_ms(PUSH_MS); }
static inline void do_backup(void){ motor_backward(); set_until_ms(BACK_MS); }
static inline void do_stop1(void){ motor_stop(); set_until_ms(1); }

static inline void start_avoid(Side first){
    A.mode = MODE_AVOID;
    A.st = AV_TURN_90;
    A.side = first;
    A.tried_left  = (first==SIDE_RIGHT); // if starting right, mark left as not tried; vice versa
    A.tried_right = (first==SIDE_LEFT);
    do_stop1();
}

static inline void switch_side(void){
    A.side = (A.side==SIDE_LEFT)?SIDE_RIGHT:SIDE_LEFT;
}

/* main FSM step */
static void avoidor_tick(void){
    switch (A.st){
    case AV_TURN_90:
        if (!due()) { /* waiting from previous stop */ break; }
        if (A.side==SIDE_LEFT) do_left_90(); else do_right_90();
        A.st = AV_DRIVE_SIDE;
        break;

    case AV_DRIVE_SIDE:
        if (!due()) break;
        do_drive_side();
        A.st = AV_TURN_BACK_90;
        break;

    case AV_TURN_BACK_90:
        if (!due()) break;
        do_turnback_90(A.side);
        A.st = AV_PAUSE_CHECK;
        break;

    case AV_PAUSE_CHECK:
        if (!due()) break;
        do_pause_check();   // settle before reading
        A.st = AV_DECIDE;
        break;

    case AV_DECIDE:
        if (!due()) break;
        {
            uint32_t d = ultra_read_cm();
            printf("[avoid] after %s detour: dist=%lucm\n",
                   A.side==SIDE_LEFT?"LEFT":"RIGHT", (unsigned long)d);

            if (d >= CLEAR_CM){
                A.st = AV_PUSH_THROUGH;
                do_push();
            } else {
                // mark this side as tried
                if (A.side==SIDE_LEFT) A.tried_left = true; else A.tried_right = true;

                // try the other side if not yet tried
                if (!(A.tried_left && A.tried_right)){
                    A.st = AV_TRY_OTHER_SIDE;
                    do_stop1();
                } else {
                    // both failed -> back up
                    A.st = AV_BACKUP;
                    do_backup();
                }
            }
        }
        break;

    case AV_TRY_OTHER_SIDE:
        if (!due()) break;
        switch_side();
        A.st = AV_TURN_90;
        do_stop1();
        break;

    case AV_PUSH_THROUGH:
        if (!due()) break;
        // done -> manual again
        motor_stop();
        A.mode = MODE_MANUAL;
        A.st = AV_IDLE;
        break;

    case AV_BACKUP:
        if (!due()) break;
        motor_stop();
        A.mode = MODE_MANUAL; // give control back; user can try again
        A.st = AV_IDLE;
        break;

    case AV_IDLE:
    default:
        // should not sit here in MODE_AVOID, but keep safe
        motor_stop();
        A.mode = MODE_MANUAL;
        break;
    }
}

/* ------------ external entry point ------------ */
void ultra_obstacle_aware_apply(DriveCmd desired) {
    uint32_t d = ultra_read_cm();

    if (A.mode == MODE_MANUAL) {
        bool wants_forward = (desired == CMD_FORWARD || desired == CMD_FWD_LEFT || desired == CMD_FWD_RIGHT);
        if (wants_forward && d > 0 && d <= STOP_CM) {
            printf("Obstacle at %lucm → deterministic sidestep\n", (unsigned long)d);
            start_avoid(SIDE_LEFT); // start left; it will try right if left fails
            return;
        }
        // pass-through when clear or not moving forward
        ultra_apply_direct(desired);
    } else {
        // avoidance in progress
        avoidor_tick();
    }
}
