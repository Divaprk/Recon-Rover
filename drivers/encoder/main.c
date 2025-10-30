// if both sides go opposite way, distance should have positive and negative. need circumference of tank wheel
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// ========== USER SETTINGS ==========
#define SENSOR_PIN        28          // Encoder output pin (GP0 here)
#define PRINT_MS          500        // Report every 500 ms

// How many counts per one full wheel revolution *as your code counts them*.
// If you count only rising edges, set this to the number of slots/notches.
// If you count both edges (rise+fall), set this to 2 * slots.
#define COUNTS_PER_REV    20         // <-- CHANGE to your disc (e.g., 20 slots)
#define WHEEL_CIRCUM_MM   185.17f     // <-- CHANGE to your wheel circumference in mm
// ===================================

volatile uint32_t tick_count = 0;    // ISR increments this
static double distance_mm_total = 0; // accumulated distance

static inline double mm_per_tick(void) {
    return (double)WHEEL_CIRCUM_MM / (double)COUNTS_PER_REV;
}

static void sensor_isr(uint gpio, uint32_t events) {
    // Count on rising edges only (one count per slot/transition)
    if (events & GPIO_IRQ_EDGE_RISE) {
        tick_count++;
    }
}

static bool print_cb(repeating_timer_t *t) {
    // Take a snapshot of ticks since last print and reset the counter
    uint32_t ticks = tick_count;
    tick_count = 0;

    // Convert ticks in the last PRINT_MS to RPM, speed, and distance
    const double interval_s = PRINT_MS / 1000.0;          // seconds per interval
    const double revs       = (double)ticks / COUNTS_PER_REV;
    const double rpm        = (revs / interval_s) * 60.0; // revs/s -> revs/min

    const double mm_this    = (double)ticks * mm_per_tick();
    const double mm_per_s   = mm_this / interval_s;
    distance_mm_total      += mm_this;

    // Report
    printf("ticks=%lu | rpm=%.2f | speed=%.1f mm/s | +%.1f mm | total=%.1f mm\n",
           (unsigned long)ticks, rpm, mm_per_s, mm_this, distance_mm_total);

    return true; // keep repeating
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000); // give USB serial time to enumerate

    // Configure sensor input
    gpio_init(SENSOR_PIN);
    gpio_set_dir(SENSOR_PIN, GPIO_IN);
    gpio_pull_up(SENSOR_PIN); // safe for open-collector encoder modules

    // ISR on rising edges (use RISE | FALL if you want to double counts)
    gpio_set_irq_enabled_with_callback(
        SENSOR_PIN,
        GPIO_IRQ_EDGE_RISE,
        true,
        &sensor_isr
    );

    // Periodic reporting
    repeating_timer_t timer;
    add_repeating_timer_ms(PRINT_MS, print_cb, NULL, &timer);

    while (true) {
        tight_loop_contents();
    }
}
