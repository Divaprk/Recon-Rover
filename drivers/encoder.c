#include "encoder.h"

#include <stdio.h>

#include <string.h> // <-- ADD THIS for memcpy

#include "pico/stdlib.h"

#include "hardware/gpio.h"

// --- ADD THESE LWIP INCLUDES ---

#include "lwip/udp.h"

#include "lwip/pbuf.h"

// --- END ADD ---



// ========== ENCODER SETTINGS ==========

#define SENSOR_PIN_LEFT   28      

#define SENSOR_PIN_RIGHT  4      

#define PRINT_MS          500    

#define COUNTS_PER_REV    80      

#define WHEEL_CIRCUM_MM   58.94f  



// ========== GLOBAL VARIABLES (ENCODER) ==========

static volatile uint32_t tick_count_left = 0;

static volatile uint32_t tick_count_right = 0;

static double distance_mm_total_left = 0;

static double distance_mm_total_right = 0;



// --- ADD THESE TELEMETRY GLOBALS ---

static struct udp_pcb *telemetry_pcb = NULL;

static ip_addr_t telemetry_addr;

static u16_t telemetry_port;

static bool telemetry_target_set = false;

// --- END ADD ---





// ========== INTERNAL HELPER FUNCTIONS ==========



static inline double mm_per_tick(void) {

    return (double)WHEEL_CIRCUM_MM / (double)COUNTS_PER_REV;

}



static void sensor_isr(uint gpio, uint32_t events) {

    if (events & GPIO_IRQ_EDGE_RISE) {

        if (gpio == SENSOR_PIN_LEFT) {

            tick_count_left++;

        } else if (gpio == SENSOR_PIN_RIGHT) {

            tick_count_right++;

        }

    }

}



// This timer callback now PRINTS *and* SENDS UDP

static bool print_cb(repeating_timer_t *t) {

    // --- 1. Snapshot and Reset ---

    uint32_t ticks_l = tick_count_left;

    tick_count_left = 0;

    uint32_t ticks_r = tick_count_right;

    tick_count_right = 0;



    // --- 2. Calculations ---

    const double interval_s = PRINT_MS / 1000.0;

    const double mm_per_tick_val = mm_per_tick();



    // Left Wheel

    const double revs_l     = (double)ticks_l / COUNTS_PER_REV;

    const double rpm_l      = (revs_l / interval_s) * 60.0;

    const double mm_this_l  = (double)ticks_l * mm_per_tick_val;

    const double mm_per_s_l = mm_this_l / interval_s;

    distance_mm_total_left += mm_this_l;



    // Right Wheel

    const double revs_r     = (double)ticks_r / COUNTS_PER_REV;

    const double rpm_r      = (revs_r / interval_s) * 60.0;

    const double mm_this_r  = (double)ticks_r * mm_per_tick_val;

    const double mm_per_s_r = mm_this_r / interval_s;

    distance_mm_total_right += mm_this_r;



    // --- 3. Report to USB Serial ---

    printf("L: ticks=%-4lu | rpm=%-6.1f | speed=%-5.1f mm/s | total=%.1f mm\n",

           (unsigned long)ticks_l, rpm_l, mm_per_s_l, distance_mm_total_left);

    printf("R: ticks=%-4lu | rpm=%-6.1f | speed=%-5.1f mm/s | total=%.1f mm\n",

           (unsigned long)ticks_r, rpm_r, mm_per_s_r, distance_mm_total_right);

    printf("---\n");



    // --- 4. ADD THIS: Send Report over UDP ---

    if (telemetry_target_set) {

        // A. Format the *exact same* string

        // (Use a static buffer to avoid stack overflow)

        static char telemetry_buf[256];

        int len = snprintf(telemetry_buf, sizeof(telemetry_buf),

            "L: ticks=%-4lu | rpm=%-6.1f | speed=%-5.1f mm/s | total=%.1f mm\r\n"

            "R: ticks=%-4lu | rpm=%-6.1f | speed=%-5.1f mm/s | total=%.1f mm\r\n---\r\n",

            (unsigned long)ticks_l, rpm_l, mm_per_s_l, distance_mm_total_left,

            (unsigned long)ticks_r, rpm_r, mm_per_s_r, distance_mm_total_right);



        // B. Allocate a packet buffer

        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);

        if (p != NULL) {

            // C. Copy the string into the buffer

            memcpy(p->payload, telemetry_buf, len);

           

            // D. Send the UDP packet

            udp_sendto(telemetry_pcb, p, &telemetry_addr, telemetry_port);

           

            // E. Free the buffer

            pbuf_free(p);

        }

    }

    // --- END ADD ---



    return true; // keep repeating

}





// ========== PUBLIC FUNCTIONS ==========



// --- ADD THIS FUNCTION DEFINITION ---

void encoder_set_remote_udp_target(struct udp_pcb *pcb, const ip_addr_t *addr, u16_t port) {

    telemetry_pcb = pcb;

    ip_addr_copy(telemetry_addr, *addr);

    telemetry_port = port;

    telemetry_target_set = true;

}

// --- END ADD ---





void encoder_init(void) {

    // --- 1. Configure Encoder Pins ---

    gpio_init(SENSOR_PIN_LEFT);

    gpio_set_dir(SENSOR_PIN_LEFT, GPIO_IN);

    gpio_pull_up(SENSOR_PIN_LEFT);



    gpio_init(SENSOR_PIN_RIGHT);

    gpio_set_dir(SENSOR_PIN_RIGHT, GPIO_IN);

    gpio_pull_up(SENSOR_PIN_RIGHT);

    printf("Encoders initialized on pins %d and %d.\n", SENSOR_PIN_LEFT, SENSOR_PIN_RIGHT);



    // --- 2. Configure Interrupts ---

    gpio_set_irq_enabled_with_callback(

        SENSOR_PIN_LEFT,

        GPIO_IRQ_EDGE_RISE,

        true,

        &sensor_isr

    );

    gpio_set_irq_enabled(

        SENSOR_PIN_RIGHT,

        GPIO_IRQ_EDGE_RISE,

        true

    );

    printf("Encoder interrupts configured.\n");



    // --- 3. Configure Reporting Timer ---

    static repeating_timer_t timer;

    add_repeating_timer_ms(PRINT_MS, print_cb, NULL, &timer);

    printf("Reporting timer configured for every %d ms.\n", PRINT_MS);

}