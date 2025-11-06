#include "ultrasonic.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <stdbool.h>
#include "motor.h"

/* ---------- Clear/Stop thresholds ---------- */
#define STOP_CM           30
#define CLEAR_CM          31

/* ---------- Ultrasonic timing ---------- */
#define TIMEOUT_ECHO_US   26000
#define SAMPLE_COUNT      5

/* ---------- Turning/drive calibration ---------- */
#define PIVOT_MS_90_LEFT   317   // ms that gives ~90° when pivoting LEFT
#define PIVOT_MS_90_RIGHT  540   // ms that gives ~90° when pivoting RIGHT
#define DRIVE_MS           500   // how far to slide per side step
#define CHECK_PAUSE_MS     120   // settle before reading
#define FORWARD_CLEAR_MS   650   // forward time once clear
#define MAX_SIDE_STEPS     20    // safety cap

/* ---------- New: distances by time ---------- */
#define GO_AROUND_MS       1200  // ~50 cm forward after first clear (tune)
#define OFFSET_10CM_MS      400  // ~10 cm nudge left (tune)

/* ---------- helpers to convert degrees -> ms per side ---------- */
static inline int ms_for_deg_left(int deg)  { return (deg * PIVOT_MS_90_LEFT)  / 90; }
static inline int ms_for_deg_right(int deg) { return (deg * PIVOT_MS_90_RIGHT) / 90; }

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
    for (int i=0;i<SAMPLE_COUNT;i++){
        uint32_t us = pulse_us();
        v[i] = us ? (us/58) : 0;
        sleep_ms(20);
    }
    uint32_t m = median5(v);
    if (m == 0) {                  // retry once if timeout/no echo
        uint32_t us2 = pulse_us();
        m = us2 ? (us2/58) : 0;
    }
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

/* ---------------- Side-step-until-clear FSM ---------------- */
typedef enum { MODE_MANUAL=0, MODE_AVOID } Mode;

typedef enum {
    AV_IDLE=0,
    AV_TURN_90,
    AV_DRIVE_SIDE,
    AV_TURN_BACK_90,
    AV_PAUSE_CHECK,
    AV_DECIDE,
    AV_GO_FORWARD,

    /* Existing second pass */
    AV_GO_AROUND_LONG,     // drive forward ~50 cm after first clear
    AV_TURN_OTHER_SIDE,    // then turn right 90° and start flank scanning

    /* New: extra offset before second pass */
    AV_OFFSET_LEFT,        // turn left 90°
    AV_OFFSET_DRIVE_SHORT, // drive ~10 cm
    AV_OFFSET_TURNBACK     // turn right 90° (face forward again)
} AvState;

typedef enum { SIDE_LEFT=0, SIDE_RIGHT=1 } Side;

typedef struct {
    Mode mode;
    AvState st;
    Side side;
    uint8_t side_steps;
    absolute_time_t until;
    bool second_pass_pending;   // run the “other side” once after first clear
    bool right_flank_scan;      // scanning while facing right flank
} Avoidor;

static Avoidor A = {0};

static inline void set_until_ms(int ms){ A.until = delayed_by_ms(get_absolute_time(), ms); }
static inline bool due(void){ return absolute_time_diff_us(get_absolute_time(), A.until) <= 0; }

/* degree-based timed motor helpers using per-side calibration */
static inline void turn_left_deg(int deg){  motor_left();  set_until_ms(ms_for_deg_left(deg)); }
static inline void turn_right_deg(int deg){ motor_right(); set_until_ms(ms_for_deg_right(deg)); }

static inline void do_left_90(void){  turn_left_deg(90); }
static inline void do_right_90(void){ turn_right_deg(90); }
static inline void do_drive_side(void){ motor_forward(); set_until_ms(DRIVE_MS); }
static inline void do_turnback_90(Side s){
    /* exact 90° back (no bias) */
    if (s==SIDE_LEFT)  turn_right_deg(90);
    else               turn_left_deg(90);
}
static inline void do_pause_check(void){ motor_stop(); set_until_ms(CHECK_PAUSE_MS); }
static inline void do_forward_clear(void){ motor_forward(); set_until_ms(FORWARD_CLEAR_MS); }
static inline void do_forward_long(void){ motor_forward(); set_until_ms(GO_AROUND_MS); }
static inline void do_forward_short(void){ motor_forward(); set_until_ms(OFFSET_10CM_MS); }
static inline void do_stop1(void){ motor_stop(); set_until_ms(1); }

static inline void start_avoid(Side first){
    A.mode = MODE_AVOID;
    A.st = AV_TURN_90;
    A.side = first;
    A.side_steps = 0;
    A.second_pass_pending = true;     // after first clear, go-around+other-side scan
    A.right_flank_scan = false;
    do_stop1();
}

/* main FSM step */
static void avoidor_tick(void){
    switch (A.st){
    case AV_TURN_90:
        if (!due()) break;
        if (A.right_flank_scan) {
            /* In right-flank mode: start micro-cycle with LEFT 90° */
            turn_left_deg(90);
        } else {
            if (A.side==SIDE_LEFT) do_left_90(); else do_right_90();
        }
        A.st = AV_DRIVE_SIDE;
        break;

    case AV_DRIVE_SIDE:
        if (!due()) break;
        do_drive_side();
        A.st = AV_TURN_BACK_90;
        break;

    case AV_TURN_BACK_90:
        if (!due()) break;
        if (A.right_flank_scan) {
            /* Complete micro-cycle: return to facing RIGHT */
            turn_right_deg(90);
        } else {
            do_turnback_90(A.side);
        }
        A.st = AV_PAUSE_CHECK;
        break;

    case AV_PAUSE_CHECK:
        if (!due()) break;
        do_pause_check();
        A.st = AV_DECIDE;
        break;

    case AV_DECIDE: {
        if (!due()) break;
        uint32_t d = ultra_read_cm();
        printf("[avoid] mode=%s side=%s step=%u dist=%lucm\n",
               A.right_flank_scan ? "RIGHT_FLANK" : "LEFT_PASS",
               A.side==SIDE_LEFT?"LEFT":"RIGHT",
               (unsigned)A.side_steps, (unsigned long)d);

        if (d == 0 || d <= STOP_CM) {
            /* still blocked → repeat */
            A.side_steps++;
            if (A.side_steps >= MAX_SIDE_STEPS) {
                /* optional safety flip */
                A.side = (A.side==SIDE_LEFT)?SIDE_RIGHT:SIDE_LEFT;
                A.side_steps = 0;
            }
            A.st = AV_TURN_90;     // kick off next micro-cycle
            do_stop1();
        }
        else if (d < CLEAR_CM) {
            /* marginal → settle and recheck */
            do_pause_check();
        }
        else {
            /* clear */
            if (A.second_pass_pending) {
                /* NEW: before the 50cm go-around, do a left 90 + 10cm + right 90 */
                A.st = AV_OFFSET_LEFT;
                do_left_90();                 // step (1): turn left again
            } else if (A.right_flank_scan) {
                /* finished scanning other side → short push then return to manual */
                A.right_flank_scan = false;
                A.st = AV_GO_FORWARD;
                do_forward_clear();           // change to immediate handoff if preferred
            } else {
                /* normal behavior */
                A.st = AV_GO_FORWARD;
                do_forward_clear();
            }
        }
    } break;

    /* --- NEW offset mini-sequence: left 90 → ~10cm → right 90 --- */
    case AV_OFFSET_LEFT:
        if (!due()) break;
        A.st = AV_OFFSET_DRIVE_SHORT;
        do_forward_short();                   // step (2): move ~10 cm
        break;

    case AV_OFFSET_DRIVE_SHORT:
        if (!due()) break;
        A.st = AV_OFFSET_TURNBACK;
        turn_right_deg(90);                   // step (3): face forward again
        break;

    case AV_OFFSET_TURNBACK:
        if (!due()) break;
        /* continue with the existing second part: ~50 cm forward, then turn right and scan */
        A.st = AV_GO_AROUND_LONG;
        do_forward_long();
        break;

    /* --- existing second pass --- */
    case AV_GO_AROUND_LONG:
        if (!due()) break;
        motor_stop();
        A.st = AV_TURN_OTHER_SIDE;
        do_stop1();
        break;

    case AV_TURN_OTHER_SIDE:
        if (!due()) break;
        do_right_90();                  // face right
        A.side = SIDE_RIGHT;
        A.side_steps = 0;
        A.second_pass_pending = false;  // entering second pass now
        A.right_flank_scan = true;      // enable flank micro-cycles
        A.st = AV_PAUSE_CHECK;          // check first while facing right
        break;

    case AV_GO_FORWARD:
        if (!due()) break;
        motor_stop();
        A.mode = MODE_MANUAL;
        A.st = AV_IDLE;
        break;

    case AV_IDLE:
    default:
        motor_stop();
        A.mode = MODE_MANUAL;
        break;
    }
}

void ultra_obstacle_aware_apply(DriveCmd desired) {
    uint32_t d = ultra_read_cm();

    if (A.mode == MODE_MANUAL) {
        bool wants_forward = (desired == CMD_FORWARD || desired == CMD_FWD_LEFT || desired == CMD_FWD_RIGHT);
        if (wants_forward && (d == 0 || d <= STOP_CM)) {
            printf("Obstacle at %lucm → side-step until clear\n", (unsigned long)d);
            start_avoid(SIDE_LEFT);   // start left; then offset+go-around+right-flank scan
            return;
        }
        ultra_apply_direct(desired);
    } else {
        avoidor_tick();
    }
}
