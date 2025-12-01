#include "encoder.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"

// ========== ENCODER SETTINGS ==========
#define SENSOR_PIN_LEFT   28      
#define SENSOR_PIN_RIGHT  2      
#define COUNTS_PER_REV    80      
#define WHEEL_CIRCUM_MM   (58.94f * 1.20588f)

// ========== VARIABLES ==========
static volatile uint32_t tick_count_left = 0;
static volatile uint32_t tick_count_right = 0;
static double distance_mm_total_left = 0;
static double distance_mm_total_right = 0;

static struct udp_pcb *telemetry_pcb = NULL;
static ip_addr_t telemetry_addr;
static u16_t telemetry_port;
static bool telemetry_target_set = false;

// Externs from main.c
extern volatile uint32_t g_current_distance_cm;
extern volatile float g_current_heading;
extern volatile float g_current_tilt_x;
extern volatile float g_current_tilt_y;

static inline double mm_per_tick(void) {
    return (double)WHEEL_CIRCUM_MM / (double)COUNTS_PER_REV;
}

// Keep the GPIO Interrupt (This is safe!)
static void sensor_isr(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        if (gpio == SENSOR_PIN_LEFT) tick_count_left++;
        else if (gpio == SENSOR_PIN_RIGHT) tick_count_right++;
    }
}

// ========== PUBLIC FUNCTIONS ==========

void encoder_set_remote_udp_target(struct udp_pcb *pcb, const ip_addr_t *addr, u16_t port) {
    telemetry_pcb = pcb;
    ip_addr_copy(telemetry_addr, *addr);
    telemetry_port = port;
    telemetry_target_set = true;
}

void encoder_init(void) {
    gpio_init(SENSOR_PIN_LEFT);
    gpio_set_dir(SENSOR_PIN_LEFT, GPIO_IN);
    gpio_pull_up(SENSOR_PIN_LEFT);

    gpio_init(SENSOR_PIN_RIGHT);
    gpio_set_dir(SENSOR_PIN_RIGHT, GPIO_IN);
    gpio_pull_up(SENSOR_PIN_RIGHT);

    gpio_set_irq_enabled_with_callback(SENSOR_PIN_LEFT, GPIO_IRQ_EDGE_RISE, true, &sensor_isr);
    gpio_set_irq_enabled(SENSOR_PIN_RIGHT, GPIO_IRQ_EDGE_RISE, true);
    
    printf("Encoders initialized (GPIO Interrupts Only).\n");
}

// New Manual Report Function
void encoder_update_and_report(void) {
    static absolute_time_t last_time = {0};
    absolute_time_t now = get_absolute_time();
    
    // First run initialization
    if (to_us_since_boot(last_time) == 0) {
        last_time = now;
        return;
    }

    int64_t dt_us = absolute_time_diff_us(last_time, now);
    if (dt_us < 1000) return; // Prevent divide by zero if called too fast
    double interval_s = dt_us / 1000000.0;
    last_time = now;

    // Snapshot atomic counters
    uint32_t ticks_l = tick_count_left;
    tick_count_left = 0;
    uint32_t ticks_r = tick_count_right;
    tick_count_right = 0;

    // Snapshot globals
    uint32_t uss_dist = g_current_distance_cm;
    float heading = g_current_heading;
    float tilt_x = g_current_tilt_x;
    float tilt_y = g_current_tilt_y;

    // Math
    double mm_val = mm_per_tick();
    
    double revs_l = (double)ticks_l / COUNTS_PER_REV;
    double rpm_l  = (revs_l / interval_s) * 60.0;
    double mm_l   = (double)ticks_l * mm_val;
    double spd_l  = mm_l / interval_s;
    distance_mm_total_left += mm_l;

    double revs_r = (double)ticks_r / COUNTS_PER_REV;
    double rpm_r  = (revs_r / interval_s) * 60.0;
    double mm_r   = (double)ticks_r * mm_val;
    double spd_r  = mm_r / interval_s;
    distance_mm_total_right += mm_r;

    // Print
    printf("USS: %-3lu cm | HDG: %.1f | TX: %.1f TY: %.1f\n", 
           uss_dist, heading, tilt_x, tilt_y);
           
    // UDP Send (Now safe because it's in the main thread!)
    if (telemetry_target_set) {
        static char t_buf[384];
        int len = snprintf(t_buf, sizeof(t_buf),
            "USS: %-3lu cm | HDG: %.1f | Tilt X: %.1f Y: %.1f\r\n"
            "L: ticks=%-3lu | rpm=%-5.1f | mm/s=%-5.1f | tot=%.0f\r\n"
            "R: ticks=%-3lu | rpm=%-5.1f | mm/s=%-5.1f | tot=%.0f\r\n---\r\n",
            uss_dist, heading, tilt_x, tilt_y,
            ticks_l, rpm_l, spd_l, distance_mm_total_left,
            ticks_r, rpm_r, spd_r, distance_mm_total_right);

        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
        if (p != NULL) {
            memcpy(p->payload, t_buf, len);
            udp_sendto(telemetry_pcb, p, &telemetry_addr, telemetry_port);
            pbuf_free(p);
        }
    }
}