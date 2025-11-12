#include "lidar.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"

// ========== LIDAR PROTOCOL CONSTANTS ==========
#define UART_BAUD        115200
#define T_HDR_US         500000

// ========== OCCUPANCY GRID ==========
#define LOG_ODDS_FREE    -2
#define LOG_ODDS_OCC     18
#define LOG_ODDS_MIN     -100
#define LOG_ODDS_MAX     100

static int8_t grid[MAP_H][MAP_W];  // ~57 KB

// ========== LIDAR GATING ==========
#define LIDAR_MIN_MM     50.0f
#define LIDAR_MAX_MM     8000.0f
#define LIDAR_Q_MIN      10
#define MAX_MARGIN_MM    1000.0f  // Don't mark endpoint if near max range

// ========== STRIPE STREAMING ==========
#define STRIPE_H         6
#define STRIPES_TOTAL    (MAP_H / STRIPE_H)
#define STRIPE_BYTES     (MAP_W * STRIPE_H)

typedef struct __attribute__((packed)) {
    uint32_t magic;   // 'RMAP' = 0x524D4150
    uint16_t frame;
    uint8_t  stripe;
    uint8_t  total;
} stripe_hdr_t;

static uint16_t frame_counter = 0;

// ========== UDP STREAMING ==========
static struct udp_pcb *map_pcb = NULL;
static ip_addr_t map_addr;
static bool map_target_set = false;

// ========== POSE (start at origin) ==========
static float pose_x_mm = 0.0f;
static float pose_y_mm = 0.0f;
static float pose_th_rad = 0.0f;

// ========== FAST TRIG LOOKUP ==========
#define ANG_STEPS (360 * 64)  // 23,040 entries
static float SIN_LUT[ANG_STEPS];

// ========== TIMING ==========
static absolute_time_t last_stream_time;

// ========== INTERNAL HELPER FUNCTIONS ==========

static inline void clamp_i8(int *v, int lo, int hi) {
    if (*v < lo) *v = lo;
    else if (*v > hi) *v = hi;
}

static inline bool world_to_cell(float x_mm, float y_mm, int *cx, int *cy) {
    const int ox = MAP_W / 2;
    const int oy = MAP_H / 2;
    *cx = ox + (int)lroundf(x_mm / MAP_RES_MM);
    *cy = oy - (int)lroundf(y_mm / MAP_RES_MM);
    return (*cx >= 0 && *cx < MAP_W && *cy >= 0 && *cy < MAP_H);
}

// Bresenham line algorithm - mark all cells along ray as free
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

// Integrate a single laser ray into the occupancy grid
static void integrate_ray(float hit_ang_deg, float hit_dist_mm, bool mark_endpoint) {
    // Ignore ultra-short returns
    const float min_hit_mm = MAP_RES_MM * 1.5f;
    if (hit_dist_mm < min_hit_mm) return;

    int rx, ry;
    if (!world_to_cell(pose_x_mm, pose_y_mm, &rx, &ry)) return;

    // Convert pose heading from radians to degrees
    float th_deg = (pose_th_rad * 180.0f / M_PI) + hit_ang_deg;
    
    // Use lookup table for fast trig
    int idx = (int)lroundf(fmodf(th_deg * 64.0f, (float)ANG_STEPS));
    if (idx < 0) idx += ANG_STEPS;

    // cos(θ) = sin(θ + 90°)
    int idx_cos = idx + (ANG_STEPS / 4);
    if (idx_cos >= ANG_STEPS) idx_cos -= ANG_STEPS;

    float c = SIN_LUT[idx_cos];
    float s = SIN_LUT[idx];

    float hx = pose_x_mm + hit_dist_mm * c;
    float hy = pose_y_mm + hit_dist_mm * s;

    int hx_c, hy_c;
    if (!world_to_cell(hx, hy, &hx_c, &hy_c)) return;

    // Always carve free space along the ray
    bresenham_free(rx, ry, hx_c, hy_c);

    // Mark endpoint as occupied only if requested
    if (mark_endpoint && !(hx_c == rx && hy_c == ry)) {
        int v = grid[hy_c][hx_c] + LOG_ODDS_OCC;
        clamp_i8(&v, LOG_ODDS_MIN, LOG_ODDS_MAX);
        grid[hy_c][hx_c] = (int8_t)v;
    }
}

// ========== LIDAR UART HELPERS ==========

static inline void send_command(uint8_t cmd) {
    uint8_t pkt[2] = {0xA5, cmd};
    uart_write_blocking(LIDAR_UART_ID, pkt, 2);
}

static bool read_exact(uint8_t *buf, int len, uint32_t timeout_us) {
    for (int i = 0; i < len; i++) {
        if (!uart_is_readable_within_us(LIDAR_UART_ID, timeout_us)) return false;
        buf[i] = uart_getc(LIDAR_UART_ID);
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

static bool start_scan_internal(void) {
    send_command(0x20);
    uint8_t hdr[7];
    return read_exact(hdr, 7, T_HDR_US);
}

static inline bool is_start_byte(uint8_t b) {
    bool b0 = (b & 0x01) != 0;
    bool b1 = (b & 0x02) != 0;
    return b0 != b1;
}

// Robust streaming node reader (non-blocking)
static bool read_scan_node(float *angle_deg, float *dist_mm, uint8_t *quality) {
    static uint8_t buf[5];
    static int idx = 0;
    static bool synced = false;

    const uint32_t PER_BYTE_US = 1000;
    const uint32_t MAX_WAIT_US = 8000;
    uint32_t waited = 0;

    while (waited < MAX_WAIT_US) {
        if (!uart_is_readable_within_us(LIDAR_UART_ID, PER_BYTE_US)) {
            waited += PER_BYTE_US;
            continue;
        }
        uint8_t b = uart_getc(LIDAR_UART_ID);

        if (!synced) {
            if (!is_start_byte(b)) continue;
            buf[0] = b;
            idx = 1;
            synced = true;
            continue;
        }

        buf[idx++] = b;
        if (idx < 5) continue;

        if (!is_start_byte(buf[0])) {
            idx = 0;
            synced = false;
            continue;
        }

        *quality = buf[0] >> 2;

        uint16_t angle_q6 = ((uint16_t)buf[2] << 7) | ((uint16_t)buf[1] >> 1);
        *angle_deg = angle_q6 / 64.0f;

        uint16_t dist_q2 = ((uint16_t)buf[4] << 8) | buf[3];
        *dist_mm = dist_q2 / 4.0f;

        idx = 0;
        synced = false;
        return dist_q2 != 0;
    }
    return false;
}

// ========== MAP STREAMING ==========

static void send_grid_udp_chunked(void) {
    if (!map_target_set || !map_pcb) return;

    uint16_t frame = frame_counter++;

    for (uint8_t s = 0; s < STRIPES_TOTAL; s++) {
        stripe_hdr_t hdr = {0x524D4150u, frame, s, STRIPES_TOTAL};

        const int y0 = s * STRIPE_H;
        const u16_t payload_len = sizeof(hdr) + STRIPE_BYTES;

        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, payload_len, PBUF_RAM);
        if (!p) {
            printf("LIDAR: pbuf_alloc failed (stripe %u)\n", s);
            return;
        }

        uint8_t *q = (uint8_t *)p->payload;
        memcpy(q, &hdr, sizeof(hdr));
        q += sizeof(hdr);

        // Convert log-odds to grayscale
        for (int yy = 0; yy < STRIPE_H; yy++) {
            int y = y0 + yy;
            for (int x = 0; x < MAP_W; x++) {
                int8_t lo = grid[y][x];
                uint8_t v = 127;  // unknown
                if (lo > 0)
                    v = (uint8_t)(127 + (lo * 128) / LOG_ODDS_MAX);
                else if (lo < 0)
                    v = (uint8_t)(127 + (lo * 127) / (-LOG_ODDS_MIN));
                *q++ = v;
            }
        }

        err_t e = udp_sendto(map_pcb, p, &map_addr, LIDAR_MAP_PORT);
        pbuf_free(p);

        if (e != ERR_OK) {
            printf("LIDAR: udp_sendto ERR=%d stripe=%u\n", e, s);
            return;
        }

        // Gentle pacing to avoid overwhelming the network
        sleep_us(120);
    }
}

// ========== PWM MOTOR CONTROL ==========

static void motor_start(float duty_percent) {
    gpio_set_function(LIDAR_PWM_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(LIDAR_PWM_PIN);
    uint chan = pwm_gpio_to_channel(LIDAR_PWM_PIN);
    pwm_set_wrap(slice, 9999);
    pwm_set_clkdiv(slice, 6.25f);  // ~2 kHz PWM
    uint16_t level = (uint16_t)(duty_percent * 9999.0f / 100.0f);
    pwm_set_chan_level(slice, chan, level);
    pwm_set_enabled(slice, true);
    printf("LIDAR motor PWM: %.1f%%\n", duty_percent);
}

// ========== PUBLIC FUNCTIONS ==========

void lidar_init(void) {
    // Precompute sine lookup table
    for (int i = 0; i < ANG_STEPS; i++) {
        float a = (i / 64.0f) * M_PI / 180.0f;
        SIN_LUT[i] = sinf(a);
    }
    printf("LIDAR: Precomputed SIN LUT (%d entries)\n", ANG_STEPS);

    // Initialize UART
    uart_init(LIDAR_UART_ID, UART_BAUD);
    uart_set_format(LIDAR_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(LIDAR_UART_ID, false, false);
    uart_set_fifo_enabled(LIDAR_UART_ID, true);
    gpio_set_function(LIDAR_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LIDAR_RX_PIN, GPIO_FUNC_UART);
    
    // Clear any junk in the UART buffer
    while (uart_is_readable(LIDAR_UART_ID)) {
        (void)uart_getc(LIDAR_UART_ID);
    }

    printf("LIDAR: UART1 initialized on GP4/GP5 at %d baud\n", UART_BAUD);

    // Initialize timing
    last_stream_time = get_absolute_time();
    
    printf("LIDAR: Occupancy grid initialized (%dx%d, %d mm/cell)\n", 
           MAP_W, MAP_H, MAP_RES_MM);
}

void lidar_start_scan(void) {
    // Start motor
    motor_start(100.0f);  // Adjust 65-85% to lock at ~5-10 Hz
    sleep_ms(1200);  // Let motor spin up
    
    // Get device info
    get_info();
    
    // Start scanning
    start_scan_internal();
    printf("LIDAR: Scanning started\n");
}

void lidar_set_map_target(struct udp_pcb *pcb, const ip_addr_t *addr) {
    map_pcb = pcb;
    ip_addr_copy(map_addr, *addr);
    map_target_set = true;
    printf("LIDAR: Map streaming target set (port %d)\n", LIDAR_MAP_PORT);
}

void lidar_process(void) {
    // Read and process LIDAR data points
    float ang_deg, dist_mm;
    uint8_t q;
    
    if (read_scan_node(&ang_deg, &dist_mm, &q)) {
        if (q >= LIDAR_Q_MIN && dist_mm >= LIDAR_MIN_MM && dist_mm <= LIDAR_MAX_MM) {
            bool near_max = (dist_mm >= (LIDAR_MAX_MM - MAX_MARGIN_MM));
            integrate_ray(ang_deg, dist_mm, !near_max);
        }
    }

    // Stream map at ~7 fps
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(last_stream_time, now) >= 140000) {  // 140 ms
        last_stream_time = now;
        send_grid_udp_chunked();
    }
}

void lidar_update_pose(float x_mm, float y_mm, float heading_rad) {
    pose_x_mm = x_mm;
    pose_y_mm = y_mm;
    pose_th_rad = heading_rad;
}