#include "lidar.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"

// ----------------- CONFIGURATION -----------------
#define UART_ID      uart1
#define TX_PIN       4 
#define RX_PIN       5 
#define PWM_PIN      0 
#define UART_BAUD    115200

// ----------------- MAPPING CONSTANTS -----------------
#define MAP_W        240
#define MAP_H        240
#define MAP_RES_MM   80
#define LOG_ODDS_FREE   -2
#define LOG_ODDS_OCC     18
#define LOG_ODDS_MIN   -100
#define LOG_ODDS_MAX    100

// ----------------- GLOBALS -----------------
static int8_t grid[MAP_H][MAP_W];       
static float SIN_LUT[360 * 64]; 
#define ANG_STEPS (360 * 64)

static float pose_x_mm = 0.0f;
static float pose_y_mm = 0.0f;
static float pose_th = 0.0f;

static struct udp_pcb *lidar_pcb = NULL;
static ip_addr_t lidar_target_ip;
static u16_t lidar_target_port = 0;
static uint16_t frame_counter = 0;

// --- IMPORTANT: STRIPE_H = 4 (Preserved Fix) ---
#define STRIPE_H       4
#define STRIPES_TOTAL  (MAP_H / STRIPE_H)
#define STRIPE_BYTES   (MAP_W * STRIPE_H)

typedef struct __attribute__((packed)) {
    uint32_t magic; 
    uint16_t frame;
    uint8_t  stripe;
    uint8_t  total;
} stripe_hdr_t;

// ----------------- HELPERS -----------------
static inline void clamp_i8(int *v, int lo, int hi) {
    if (*v < lo) *v = lo; else if (*v > hi) *v = hi;
}

static inline bool world_to_cell(float x_mm, float y_mm, int *cx, int *cy) {
    const int ox = MAP_W/2, oy = MAP_H/2;
    *cx = ox + (int)lroundf(y_mm / MAP_RES_MM);
    *cy = oy - (int)lroundf(x_mm / MAP_RES_MM);
    return (*cx >= 0 && *cx < MAP_W && *cy >= 0 && *cy < MAP_H);
}

static void bresenham_free(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    int x = x0, y = y0;
    while (true) {
        if (x >= 0 && x < MAP_W && y >= 0 && y < MAP_H) {
            int v = grid[y][x] + LOG_ODDS_FREE;
            clamp_i8(&v, LOG_ODDS_MIN, LOG_ODDS_MAX);
            grid[y][x] = (int8_t)v;
        }
        if (x == x1 && y == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
}

static inline void integrate_ray(float hit_ang_deg, float hit_dist_mm, bool mark_endpoint) {
    const float min_hit_mm = MAP_RES_MM * 1.5f;
    if (hit_dist_mm < min_hit_mm) return;

    int rx, ry;
    if (!world_to_cell(pose_x_mm, pose_y_mm, &rx, &ry)) return;

    float th_deg = (pose_th * 180.0f / (float)M_PI) + hit_ang_deg;
    int idx = (int)lroundf(fmodf(th_deg * 64.0f, (float)ANG_STEPS));
    if (idx < 0) idx += ANG_STEPS;
    int idx_cos = idx + (ANG_STEPS / 4);
    if (idx_cos >= ANG_STEPS) idx_cos -= ANG_STEPS;

    float c = SIN_LUT[idx_cos];
    float s = SIN_LUT[idx];
    float hx = pose_x_mm + hit_dist_mm * c;
    float hy = pose_y_mm + hit_dist_mm * s;
    int hx_c, hy_c;
    if (!world_to_cell(hx, hy, &hx_c, &hy_c)) return;

    bresenham_free(rx, ry, hx_c, hy_c);
    if (mark_endpoint && !(hx_c == rx && hy_c == ry)) {
        int v = grid[hy_c][hx_c] + LOG_ODDS_OCC;
        clamp_i8(&v, LOG_ODDS_MIN, LOG_ODDS_MAX);
        grid[hy_c][hx_c] = (int8_t)v;
    }
}

static void motor_pwm_start(float duty_percent) {
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PWM_PIN);
    uint chan  = pwm_gpio_to_channel(PWM_PIN);
    pwm_set_wrap(slice, 9999);
    pwm_set_clkdiv(slice, 6.25f); 
    uint16_t level = (uint16_t)(duty_percent * 9999.0f / 100.0f);
    pwm_set_chan_level(slice, chan, level);
    pwm_set_enabled(slice, true);
}

static inline void send_command(uint8_t cmd) {
    uint8_t pkt[2] = {0xA5, cmd};
    uart_write_blocking(UART_ID, pkt, 2);
}

// ----------------- PUBLIC API -----------------

void lidar_set_udp_target(struct udp_pcb *pcb, const ip_addr_t *addr, u16_t port) {
    lidar_pcb = pcb;
    ip_addr_copy(lidar_target_ip, *addr);
    lidar_target_port = port;
}

void lidar_set_pose(float x_mm, float y_mm, float theta_rad) {
    pose_x_mm = x_mm;
    pose_y_mm = y_mm;
    pose_th = theta_rad;
}

void lidar_init(void) {
    for (int i = 0; i < ANG_STEPS; i++) {
        float a = (i / 64.0f) * (float)M_PI / 180.0f;
        SIN_LUT[i] = sinf(a);
    }
    uart_init(UART_ID, UART_BAUD);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_fifo_enabled(UART_ID, true);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
    memset(grid, 0, sizeof(grid));
    printf("LIDAR Driver Initialized.\n");
}

void lidar_start(void) {
    // --- SLOW STARTUP FIX ---
    motor_pwm_start(100.0f);
    sleep_ms(1200); 

    send_command(0x25); // STOP
    sleep_ms(100);
    while(uart_is_readable(UART_ID)) uart_getc(UART_ID);
    
    send_command(0x40); // RESET
    sleep_ms(2000);     // --- WAIT FIX ---
    while(uart_is_readable(UART_ID)) uart_getc(UART_ID);

    send_command(0x20); // START SCAN
    printf("LIDAR Scanning Started.\n");
}

// --- NEW FUNCTION ---
void lidar_stop(void) {
    send_command(0x25); // STOP SCAN
    sleep_ms(50);
    motor_pwm_start(0.0f); // STOP MOTOR
    printf("LIDAR Stopped.\n");
}

void lidar_update(void) {
    static uint8_t buf[5];
    static int idx = 0;
    static bool synced = false;
    
    while (uart_is_readable(UART_ID)) {
        uint8_t b = uart_getc(UART_ID);
        if (!synced) {
            bool b0 = (b & 0x01) != 0;
            bool b1 = (b & 0x02) != 0;
            if (b0 != b1) { buf[0] = b; idx = 1; synced = true; }
            continue;
        }
        buf[idx++] = b;
        if (idx < 5) continue; 
        bool b0 = (buf[0] & 0x01) != 0;
        bool b1 = (buf[0] & 0x02) != 0;
        if (b0 == b1) { idx = 0; synced = false; continue; }

        uint8_t quality = buf[0] >> 2;
        uint16_t angle_q6 = ((uint16_t)buf[2] << 7) | ((uint16_t)buf[1] >> 1);
        float angle_deg = angle_q6 / 64.0f;
        uint16_t dist_q2 = ((uint16_t)buf[4] << 8) | buf[3];
        float dist_mm = dist_q2 / 4.0f;

        if (dist_q2 != 0) {
            if (quality > 0 && dist_mm > 50.0f && dist_mm < 8000.0f) {
                bool near_max = (dist_mm >= 7000.0f);
                integrate_ray(angle_deg, dist_mm, !near_max);
            }
        }
        idx = 0; synced = false;
    }
}

void lidar_send_map_chunked(void) {
    if (!lidar_pcb || lidar_target_port == 0) return;
    uint16_t frame = frame_counter++;
    for (uint8_t s = 0; s < STRIPES_TOTAL; s++) {
        stripe_hdr_t hdr = {0x524D4150u, frame, s, STRIPES_TOTAL};
        const int y0 = s * STRIPE_H;
        const u16_t payload_len = sizeof(hdr) + STRIPE_BYTES;
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, payload_len, PBUF_RAM);
        if (!p) return;
        uint8_t *q = (uint8_t*)p->payload;
        memcpy(q, &hdr, sizeof(hdr));
        q += sizeof(hdr);
        for (int yy = 0; yy < STRIPE_H; yy++) {
            int y = y0 + yy;
            for (int x = 0; x < MAP_W; x++) {
                int8_t lo = grid[y][x];
                uint8_t v = 127; 
                if (lo > 0)      v = (uint8_t)(127 + (lo * 128) / LOG_ODDS_MAX);
                else if (lo < 0) v = (uint8_t)(127 + (lo * 127) / (-LOG_ODDS_MIN));
                *q++ = v;
            }
        }
        udp_sendto(lidar_pcb, p, &lidar_target_ip, lidar_target_port);
        pbuf_free(p);
        
        // --- DELAY FIX (2ms) ---
        busy_wait_us(2000);
    }
}