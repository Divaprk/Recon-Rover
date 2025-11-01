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

// ========== APPLICATION SETTINGS ==========
#define WIFI_SSID "Diva iPhone"
#define WIFI_PASS "91902017"
#define CTRL_PORT 5000
#define TELEMETRY_PORT 5001

// ========== APPLICATION GLOBALS ==========
static struct udp_pcb *udp_server = NULL;

// This is the ONLY command that should be consumed by the motor layer.
// Ultrasonic will either forward it or temporarily override to avoid obstacles.
static volatile DriveCmd g_desired_cmd = CMD_STOP;

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

    // Keep your telemetry pairing (remote IP, fixed TELEMETRY_PORT)
    encoder_set_remote_udp_target(pcb, addr, TELEMETRY_PORT);

    pbuf_free(p);
}

// ==========================================================
//                      MAIN FUNCTION
// ==========================================================

int main(void) {
    // --- 1. System Init ---
    stdio_init_all();
    sleep_ms(2000); // Wait for USB serial
    printf("Recon Rover Systems Initializing...\n");

    // --- 2. Wi-Fi Init ---
    if (cyw43_arch_init()) {
        printf("CYW43 init failed\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi SSID: %s\n", WIFI_SSID);

    int rc = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 30000
    );
    if (rc) {
        printf("Wi-Fi connect failed, rc=%d\n", rc);
        return -1;
    }
    printf("Wi-Fi connected!\n");
    const ip4_addr_t *ip = netif_ip4_addr(netif_default);
    printf("IP: %s\n", ip4addr_ntoa(ip));

    // --- 3. DRIVER Init ---
    motor_init_pins();
    motor_stop();
    printf("Motor controller initialized.\n");

    encoder_init();
    ultra_init();        

    // --- 4. UDP Server Init ---
    udp_server = udp_new();
    if (!udp_server) {
        printf("Failed to create UDP PCB\n");
        return -1;
    }
    err_t err = udp_bind(udp_server, IP_ADDR_ANY, CTRL_PORT);
    if (err != ERR_OK) {
        printf("UDP bind failed: %d\n", err);
        return -1;
    }
    udp_recv(udp_server, udp_recv_cb, NULL);
    printf("UDP server listening for commands on port %d\n", CTRL_PORT);
    printf("UDP server will send telemetry to port %d\n", TELEMETRY_PORT);

    // --- 5. Main Loop ---
    printf("Initialization complete. Entering main loop.\n\n");
    while (true) {
        // Decide what actually goes to the motors:
        // - If path is clear: pass-through g_desired_cmd
        // - If obstacle ahead (forward-ish intent): auto avoid, then hand back
        ultra_obstacle_aware_apply(g_desired_cmd);

        tight_loop_contents();
    }
}
