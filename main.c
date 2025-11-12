#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

// --- Application Includes ---
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

// --- DRIVER INCLUDES ---
#include "drivers/motor.h"
#include "drivers/encoder.h"
#include "drivers/ultrasonic.h"
#include "drivers/imu.h"
#include "drivers/lidar.h"  // <-- NEW: LIDAR driver

// ========== APPLICATION SETTINGS ==========
#define WIFI_SSID "Diva iPhone"
#define WIFI_PASS "91902017"
#define CTRL_PORT 5000
#define TELEMETRY_PORT 5001
// LIDAR uses port 5005 (defined in lidar.h)

// ========== APPLICATION GLOBALS ==========
static struct udp_pcb *udp_server = NULL;

// This is the ONLY command that should be consumed by the motor layer.
// Ultrasonic will either forward it or temporarily override to avoid obstacles.
static volatile DriveCmd g_desired_cmd = CMD_STOP;

// --- Global variable to share sensor data with other modules ---
volatile uint32_t g_current_distance_cm = 0;

// --- Global variables for IMU data ---
volatile float g_current_heading = 0.0f;
volatile float g_current_tilt_x = 0.0f;
volatile float g_current_tilt_y = 0.0f;

// ==========================================================
//               TELEOP UDP FUNCTIONS
// ==========================================================

static bool contains_cmd(const char *buf, const char *tok) {
    size_t n = strlen(buf), m = strlen(tok);
    if (m == 0 || n < m) return false;
    for (size_t i = 0; i + m <= n; ++i) {
        bool match = true;
        for (size_t j = 0; j < m; ++j) {
            char a = buf[i + j], b = tok[j];
            if (a >= 'A' && a <= 'Z') a += 32;
            if (b >= 'A' && b <= 'Z') b += 32;
            if (a != b) { match = false; break; }
        }
        if (match) return true;
    }
    return false;
}

static void udp_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                        const ip_addr_t *addr, u16_t port) {
    if (!p) return;

    char buf[128];
    size_t len = (p->len < sizeof(buf) - 1) ? p->len : sizeof(buf) - 1;
    memcpy(buf, p->payload, len);
    buf[len] = '\0';

    // Map incoming text to our desired command (do NOT call motor_* here).
    if      (contains_cmd(buf, "forward_left"))     g_desired_cmd = CMD_FWD_LEFT;
    else if (contains_cmd(buf, "forward_right"))    g_desired_cmd = CMD_FWD_RIGHT;
    else if (contains_cmd(buf, "backward_left"))    g_desired_cmd = CMD_BWD_LEFT;
    else if (contains_cmd(buf, "backward_right"))   g_desired_cmd = CMD_BWD_RIGHT;
    else if (contains_cmd(buf, "forward"))          g_desired_cmd = CMD_FORWARD;
    else if (contains_cmd(buf, "backward"))         g_desired_cmd = CMD_BACKWARD;
    else if (contains_cmd(buf, "left"))             g_desired_cmd = CMD_LEFT;
    else if (contains_cmd(buf, "right"))            g_desired_cmd = CMD_RIGHT;
    else                                            g_desired_cmd = CMD_STOP;

    // Set telemetry target (remote IP, fixed TELEMETRY_PORT)
    encoder_set_remote_udp_target(pcb, addr, TELEMETRY_PORT);
    
    // NEW: Set LIDAR map streaming target (same IP, port 5005)
    lidar_set_map_target(pcb, addr);

    pbuf_free(p);
}

// ==========================================================
//                      MAIN FUNCTION
// ==========================================================

int main(void) {
    // --- 1. System Init ---
    stdio_init_all();
    sleep_ms(2000); // Wait for USB serial
    printf("=================================================\n");
    printf("     RECON ROVER - Autonomous Mapping System    \n");
    printf("=================================================\n");
    printf("Initializing systems...\n\n");

    // --- 2. Wi-Fi Init ---
    if (cyw43_arch_init()) {
        printf("ERROR: CYW43 init failed\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();
    printf("[WiFi] Connecting to SSID: %s\n", WIFI_SSID);

    int rc = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 30000
    );
    if (rc) {
        printf("ERROR: Wi-Fi connect failed, rc=%d\n", rc);
        return -1;
    }
    printf("[WiFi] Connected!\n");
    
    const ip4_addr_t *ip = netif_ip4_addr(netif_default);
    printf("[WiFi] IP Address: %s\n\n", ip4addr_ntoa(ip));

    // --- 3. DRIVER Init ---
    printf("[Drivers] Initializing motor controller...\n");
    motor_init_pins();
    motor_stop();
    
    printf("[Drivers] Initializing encoders...\n");
    encoder_init();
    
    printf("[Drivers] Initializing ultrasonic sensor...\n");
    ultra_init();
    
    printf("[Drivers] Initializing IMU...\n");
    imu_init();
    
    printf("[Drivers] Initializing LIDAR...\n");  // <-- NEW
    lidar_init();
    
    printf("[Drivers] All drivers initialized.\n\n");

    // --- 4. UDP Server Init ---
    udp_server = udp_new();
    if (!udp_server) {
        printf("ERROR: Failed to create UDP PCB\n");
        return -1;
    }
    err_t err = udp_bind(udp_server, IP_ADDR_ANY, CTRL_PORT);
    if (err != ERR_OK) {
        printf("ERROR: UDP bind failed: %d\n", err);
        return -1;
    }
    udp_recv(udp_server, udp_recv_cb, NULL);
    printf("[UDP] Command server listening on port %d\n", CTRL_PORT);
    printf("[UDP] Telemetry will be sent to port %d\n", TELEMETRY_PORT);
    printf("[UDP] LIDAR map will be sent to port %d\n", LIDAR_MAP_PORT);

    // --- 5. Start LIDAR Scanning ---
    printf("\n[LIDAR] Starting LIDAR scan...\n");
    lidar_start_scan();  // <-- NEW
    printf("[LIDAR] Mapping active!\n\n");

    // --- 6. Main Loop ---
    printf("=================================================\n");
    printf("     ALL SYSTEMS OPERATIONAL - READY TO DRIVE   \n");
    printf("=================================================\n");
    printf("Send commands on UDP port %d to control rover\n", CTRL_PORT);
    printf("Run viewer_map.py on your laptop to see the map\n");
    printf("Run telemetry_listener.py to see sensor data\n\n");
    
    // Variables for IMU data
    imu_vector_t accel, mag;
    
    while (true) {
        // 1. Ultrasonic obstacle avoidance and motor control
        ultra_obstacle_aware_apply(g_desired_cmd);
        g_current_distance_cm = ultra_read_cm();

        // 2. Read IMU data
        imu_read_accel(&accel);
        imu_read_mag(&mag);
        g_current_heading = imu_calculate_heading(&mag);
        g_current_tilt_x = imu_calculate_tilt_x(&accel);
        g_current_tilt_y = imu_calculate_tilt_y(&accel);

        // 3. NEW: Process LIDAR data and update map
        lidar_process();

        // 4. Service the background tasks (WiFi polling, etc.)
        tight_loop_contents();
    }
}