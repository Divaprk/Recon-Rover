#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"

#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/dhcp.h"

#include "pico/binary_info.h"

// ----------------- LiDAR / UART pins -----------------
#define UART_ID      uart1
#define TX_PIN       4
#define RX_PIN       5
#define PWM_PIN      0
#define UART_BAUD    115200
#define T_HDR_US     500000
#define T_NODE_US    300000

// ----------------- Wi-Fi / UDP -----------------
#define WIFI_SSID    "Diva iPhone"
#define WIFI_PASS    "91902017"
#define LAPTOP_IP    "172.20.10.8"
#define UDP_PORT     5005

static struct udp_pcb *g_udp = NULL;
static ip_addr_t g_dst;

// ----------------- Occupancy Grid -----------------
#define MAP_W        240
#define MAP_H        240
#define MAP_RES_MM   80

#define LOG_ODDS_FREE   -2
#define LOG_ODDS_OCC     18
#define LOG_ODDS_MIN   -100
#define LOG_ODDS_MAX    100

static int8_t grid[MAP_H][MAP_W]; // ~57 KB

// Stripe streaming (MTU-safe)
#define STRIPE_H       6
#define STRIPES_TOTAL  (MAP_H / STRIPE_H)
#define STRIPE_BYTES   (MAP_W * STRIPE_H)

// LiDAR gating
#define LIDAR_MIN_MM   50.0f
#define LIDAR_MAX_MM   8000.0f
#define LIDAR_Q_MIN    10
#define MAX_MARGIN_MM  2000.0f

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t frame;
    uint8_t  stripe;
    uint8_t  total;
} stripe_hdr_t;

static uint16_t frame_counter = 0;

// ----------------- Encoders -----------------
#define ENC_L_PIN_A   6
#define ENC_L_PIN_B   7
#define ENC_R_PIN_A   14
#define ENC_R_PIN_B   15
#define TICKS_PER_REV   600
#define WHEEL_RADIUS_MM 33.0f
#define AXLE_TRACK_MM   120.0f
#define USE_QUADRATURE  1

static float pose_x_mm = 0.0f, pose_y_mm = 0.0f, pose_th = 0.0f;
static volatile int32_t enc_l_ticks = 0;
static volatile int32_t enc_r_ticks = 0;

// ----------------- Fast trig lookup table (single SIN table) -----------------
#define ANG_STEPS (360 * 64)          // 23,040 entries
static float SIN_LUT[ANG_STEPS];      // ~90 KB (vs ~180 KB for sin+cos)

// ----------------- Helpers -----------------
static inline void clamp_i8(int *v, int lo, int hi) {
    if (*v < lo) *v = lo; else if (*v > hi) *v = hi;
}

static inline bool world_to_cell(float x_mm, float y_mm, int *cx, int *cy) {
    int ox = MAP_W/2, oy = MAP_H/2;
    *cx = ox + (int)lroundf(x_mm / MAP_RES_MM);
    *cy = oy - (int)lroundf(y_mm / MAP_RES_MM);
    return (*cx >= 0 && *cx < MAP_W && *cy >= 0 && *cy < MAP_H);
}

// ---- A: full free-space carving (no checkerboard) ----
static void bresenham_free(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    int x = x0, y = y0;

    while (true) {
        if (x >= 0 && x < MAP_W && y >= 0 && y < MAP_H) {
            int v = grid[y][x] + LOG_ODDS_FREE;   // no skipping
            clamp_i8(&v, LOG_ODDS_MIN, LOG_ODDS_MAX);
            grid[y][x] = (int8_t)v;
        }
        if (x == x1 && y == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
}

// ---- B: skip very short hits + don't paint the robot cell ----
static inline void integrate_ray(float hit_ang_deg, float hit_dist_mm, bool mark_endpoint) {
    // ignore ultra-short returns that quantize into robot cell
    float min_hit_mm = MAP_RES_MM * 1.5f;
    if (hit_dist_mm < min_hit_mm) return;

    int rx, ry;
    if (!world_to_cell(pose_x_mm, pose_y_mm, &rx, &ry)) return;

    float th_deg = pose_th * 180.0f / (float)M_PI + hit_ang_deg;
    int idx = (int)lroundf(fmodf(th_deg * 64.0f, (float)ANG_STEPS)); if (idx < 0) idx += ANG_STEPS;

    int idx_cos = idx + (ANG_STEPS / 4); if (idx_cos >= ANG_STEPS) idx_cos -= ANG_STEPS;
    float c = SIN_LUT[idx_cos], s = SIN_LUT[idx];

    float hx = pose_x_mm + hit_dist_mm * c;
    float hy = pose_y_mm + hit_dist_mm * s;

    int hx_c, hy_c;
    if (!world_to_cell(hx, hy, &hx_c, &hy_c)) return;

    // carve free space along the ray always
    bresenham_free(rx, ry, hx_c, hy_c);

    // optionally mark the endpoint occupied
    if (mark_endpoint && !(hx_c == rx && hy_c == ry)) {
        int v = grid[hy_c][hx_c] + LOG_ODDS_OCC;
        clamp_i8(&v, LOG_ODDS_MIN, LOG_ODDS_MAX);
        grid[hy_c][hx_c] = (int8_t)v;
    }
}



// ----------------- Wi-Fi helpers -----------------
static bool wifi_up_and_udp_target(void) {
    if (cyw43_arch_init()) { printf("WiFi init failed\n"); return false; }
    cyw43_arch_enable_sta_mode();
    printf("WiFi connecting...\n");
    int rc = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS,
                                                CYW43_AUTH_WPA2_AES_PSK, 15000);
    if (rc) { printf("WiFi connect failed (%d)\n", rc); return false; }
    printf("WiFi connected\n");

    sleep_ms(1000);
    cyw43_arch_lwip_begin();
    struct netif *sta = netif_list;
    const ip4_addr_t *ip = sta ? netif_ip4_addr(sta) : NULL;
    cyw43_arch_lwip_end();
    if (ip) printf("My IP: %s\n", ip4addr_ntoa(ip));

    g_udp = udp_new();
    if (!g_udp) { printf("udp_new failed\n"); return false; }
    ipaddr_aton(LAPTOP_IP, &g_dst);
    printf("UDP target: %s:%d\n", LAPTOP_IP, UDP_PORT);
    return true;
}

static void send_grid_udp_chunked(void) {
    if (!g_udp) return;
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
                if (lo > 0) v = (uint8_t)(127 + (lo * 128) / LOG_ODDS_MAX);
                else if (lo < 0) v = (uint8_t)(127 + (lo * 127) / (-LOG_ODDS_MIN));
                *q++ = v;
            }
        }

        cyw43_arch_lwip_begin();
        udp_sendto(g_udp, p, &g_dst, UDP_PORT);
        cyw43_arch_lwip_end();
        pbuf_free(p);
        cyw43_arch_poll();
        sleep_us(120);
    }
}

// ----------------- PWM for LiDAR motor -----------------
static void motor_start(float duty_percent) {
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PWM_PIN);
    uint chan  = pwm_gpio_to_channel(PWM_PIN);
    pwm_set_wrap(slice, 9999);
    pwm_set_clkdiv(slice, 6.25f);
    uint16_t level = (uint16_t)(duty_percent * 9999.0f / 100.0f);
    pwm_set_chan_level(slice, chan, level);
    pwm_set_enabled(slice, true);
    printf("LIDAR motor PWM %.1f%%\n", duty_percent);
}

// ----------------- LiDAR UART -----------------
static inline void send_command(uint8_t cmd) {
    uint8_t pkt[2] = {0xA5, cmd};
    uart_write_blocking(UART_ID, pkt, 2);
}

static bool read_exact(uint8_t *buf, int len, uint32_t timeout_us) {
    for (int i = 0; i < len; i++) {
        if (!uart_is_readable_within_us(UART_ID, timeout_us)) return false;
        buf[i] = uart_getc(UART_ID);
    }
    return true;
}

static bool get_info(void) {
    send_command(0x50);
    uint8_t hdr[7];
    if (!read_exact(hdr, 7, T_HDR_US)) return false;
    uint8_t payload[20];
    return read_exact(payload, 20, T_HDR_US);
}

static bool start_scan(void) {
    send_command(0x20);
    uint8_t hdr[7];
    return read_exact(hdr, 7, T_HDR_US);
}

static inline bool is_start_byte(uint8_t b) {
    bool b0 = (b & 0x01) != 0;
    bool b1 = (b & 0x02) != 0;
    return b0 != b1;
}

static bool read_scan_node(float *angle_deg, float *dist_mm, uint8_t *quality) {
    static uint8_t buf[5];
    static int idx = 0;
    static bool synced = false;
    const uint32_t PER_BYTE_US = 1000;
    const uint32_t MAX_WAIT_US = 8000;
    uint32_t waited = 0;

    while (waited < MAX_WAIT_US) {
        if (!uart_is_readable_within_us(UART_ID, PER_BYTE_US)) {
            waited += PER_BYTE_US;
            continue;
        }
        uint8_t b = uart_getc(UART_ID);

        if (!synced) {
            if (!is_start_byte(b)) continue;
            buf[0] = b; idx = 1; synced = true; continue;
        }

        buf[idx++] = b;
        if (idx < 5) continue;
        if (!is_start_byte(buf[0])) { idx = 0; synced = false; continue; }

        *quality = buf[0] >> 2;
        uint16_t angle_q6 = ((uint16_t)buf[2] << 7) | ((uint16_t)buf[1] >> 1);
        *angle_deg = angle_q6 / 64.0f;
        uint16_t dist_q2 = ((uint16_t)buf[4] << 8) | buf[3];
        *dist_mm = dist_q2 / 4.0f;
        idx = 0; synced = false;
        return dist_q2 != 0;
    }
    return false;
}

// ----------------- Odometry -----------------
static inline float ticks_to_mm(int32_t dticks) {
    float revs = (float)dticks / (float)TICKS_PER_REV;
    return (2.0f * (float)M_PI * WHEEL_RADIUS_MM) * revs;
}

static void odom_update_from_encoders(void) {
    static int32_t last_l = 0, last_r = 0;
    int32_t cur_l = enc_l_ticks, cur_r = enc_r_ticks;
    int32_t dL_ticks = cur_l - last_l;
    int32_t dR_ticks = cur_r - last_r;
    last_l = cur_l; last_r = cur_r;

    float dL = ticks_to_mm(dL_ticks);
    float dR = ticks_to_mm(dR_ticks);
    float dC = 0.5f * (dL + dR);
    float dTh = (dR - dL) / AXLE_TRACK_MM;

    float th_mid = pose_th + 0.5f * dTh;
    pose_x_mm += dC * cosf(th_mid);
    pose_y_mm += dC * sinf(th_mid);
    pose_th   += dTh;
    if (pose_th > (float)M_PI)  pose_th -= 2.0f*(float)M_PI;
    if (pose_th < -(float)M_PI) pose_th += 2.0f*(float)M_PI;
}

// ----------------- Encoder IRQs -----------------
static inline int rd(uint pin) { return gpio_get(pin) ? 1 : 0; }

static void gpio_irq_callback(uint gpio, uint32_t events) {
#if USE_QUADRATURE
    if (gpio == ENC_L_PIN_A) {
        int a = rd(ENC_L_PIN_A), b = (ENC_L_PIN_B == 255) ? 0 : rd(ENC_L_PIN_B);
        enc_l_ticks += (a == b) ? +1 : -1;
    } else if (gpio == ENC_R_PIN_A) {
        int a = rd(ENC_R_PIN_A), b = (ENC_R_PIN_B == 255) ? 0 : rd(ENC_R_PIN_B);
        enc_r_ticks += (a == b) ? +1 : -1;
    }
#else
    if (gpio == ENC_L_PIN_A) enc_l_ticks++;
    else if (gpio == ENC_R_PIN_A) enc_r_ticks++;
#endif
}

static void encoders_init(void) {
    if (ENC_L_PIN_B != 255) { gpio_init(ENC_L_PIN_B); gpio_set_dir(ENC_L_PIN_B, GPIO_IN); gpio_pull_up(ENC_L_PIN_B); }
    if (ENC_R_PIN_B != 255) { gpio_init(ENC_R_PIN_B); gpio_set_dir(ENC_R_PIN_B, GPIO_IN); gpio_pull_up(ENC_R_PIN_B); }

    gpio_init(ENC_L_PIN_A); gpio_set_dir(ENC_L_PIN_A, GPIO_IN); gpio_pull_up(ENC_L_PIN_A);
    gpio_init(ENC_R_PIN_A); gpio_set_dir(ENC_R_PIN_A, GPIO_IN); gpio_pull_up(ENC_R_PIN_A);

    gpio_set_irq_enabled_with_callback(ENC_L_PIN_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_irq_callback);
    gpio_set_irq_enabled(ENC_R_PIN_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

// ----------------- Timer helper -----------------
static inline bool due_ms(absolute_time_t *t, uint32_t period_ms){
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(*t, now) >= (int64_t)period_ms * 1000) {
        *t = now; return true;
    }
    return false;
}

// ----------------- Main -----------------
int main(void) {
    stdio_init_all();
    sleep_ms(500);
    printf("---- Teleop Mapping: LiDAR + Encoders -> Occupancy Grid ----\n");

    // Precompute single SIN LUT; use phase shift for COS
    for (int i = 0; i < ANG_STEPS; i++) {
        float a = (i / 64.0f) * (float)M_PI / 180.0f;
        SIN_LUT[i] = sinf(a);
    }
    printf("Precomputed SIN LUT (%d)\n", ANG_STEPS);

    bool wifi_ok = wifi_up_and_udp_target();
    encoders_init();

    uart_init(UART_ID, UART_BAUD);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_fifo_enabled(UART_ID, true);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
    while (uart_is_readable(UART_ID)) (void)uart_getc(UART_ID);

    motor_start(100.0f);
    sleep_ms(1200);
    get_info(); start_scan();

    printf("Mapping + Streaming...\n");
    absolute_time_t t_stream = get_absolute_time();

    while (true) {
        sleep_ms(1);
        if (wifi_ok) cyw43_arch_poll();

        float ang_deg, dist_mm;
        uint8_t q;
        if (read_scan_node(&ang_deg, &dist_mm, &q)) {
            if (q >= LIDAR_Q_MIN && dist_mm >= LIDAR_MIN_MM && dist_mm <= LIDAR_MAX_MM) {
                bool near_max = (dist_mm >= (LIDAR_MAX_MM - MAX_MARGIN_MM)) || (q < 15);
                if (!near_max) {
                    integrate_ray(ang_deg, dist_mm, true);   // solid hit
                } else {
                    integrate_ray(ang_deg, dist_mm, false);  // clear ray only
                }
            }
        }

        if (wifi_ok && due_ms(&t_stream, 140)) {  // ~7 fps
            send_grid_udp_chunked();
        }
    }
}
