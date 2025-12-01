#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"

// Drivers
#include "drivers/motor.h"
#include "drivers/ultrasonic.h"
#include "drivers/imu.h"
#include "drivers/encoder.h"
#include "drivers/lidar.h" 

// --- WIFI CONFIG ---
#define WIFI_SSID "Diva iPhone"
#define WIFI_PASS "91902017"
#define LAPTOP_IP "172.20.10.8" 
#define UDP_PORT  5005          

// --- TIMING CONSTANTS ---
#define TELEMETRY_MS    500     
#define SENSOR_READ_MS  50      
#define MAP_UPDATE_MS   140     

// --- GLOBAL VARIABLES ---
volatile uint32_t g_current_distance_cm = 0;
volatile float g_current_heading = 0.0f;
volatile float g_current_tilt_x = 0.0f;
volatile float g_current_tilt_y = 0.0f;

// --- STATE FLAG ---
bool lidar_active = false; // <--- Starts FALSE

struct udp_pcb *udp_socket = NULL;
ip_addr_t dest_addr;

// --- HELPER ---
bool cmd_is(const char *data, int len, const char *cmd_str) {
    if (len != strlen(cmd_str)) return false;
    return (strncmp(data, cmd_str, len) == 0);
}

void udp_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    if (p == NULL) return;
    char *data = (char *)p->payload;
    int len = p->len;
    
    // --- WASD COMMANDS ---
    DriveCmd cmd = CMD_STOP;
    if      (cmd_is(data, len, "forward_left"))   cmd = CMD_FWD_LEFT;
    else if (cmd_is(data, len, "forward_right"))  cmd = CMD_FWD_RIGHT;
    else if (cmd_is(data, len, "backward_left"))  cmd = CMD_BWD_LEFT;
    else if (cmd_is(data, len, "backward_right")) cmd = CMD_BWD_RIGHT;
    else if (cmd_is(data, len, "forward"))        cmd = CMD_FORWARD;
    else if (cmd_is(data, len, "backward"))       cmd = CMD_BACKWARD;
    else if (cmd_is(data, len, "left"))           cmd = CMD_LEFT;
    else if (cmd_is(data, len, "right"))          cmd = CMD_RIGHT;
    else if (cmd_is(data, len, "stop"))           cmd = CMD_STOP;
    
    // --- LIDAR TOGGLE COMMAND ---
    else if (cmd_is(data, len, "toggle_lidar")) {
        if (!lidar_active) {
            printf("CMD: Start Lidar\n");
            lidar_start();
            lidar_active = true;
        } else {
            printf("CMD: Stop Lidar\n");
            lidar_stop();
            lidar_active = false;
        }
    }

    ultra_obstacle_aware_apply(cmd);
    pbuf_free(p);
}

void update_sensors(void) {
    g_current_distance_cm = ultra_read_cm();
    imu_vector_t accel, mag;
    imu_read_accel(&accel);
    imu_read_mag(&mag);
    g_current_heading = imu_calculate_heading(&mag);
    g_current_tilt_x  = imu_calculate_tilt_x(&accel);
    g_current_tilt_y  = imu_calculate_tilt_y(&accel);
}

int main() {
    stdio_init_all();
    sleep_ms(2000); 
    printf("--- Recon Rover Final: TOGGLE MODE ---\n");

    if (cyw43_arch_init()) { printf("Wi-Fi Init failed!\n"); return 1; }
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 15000)) {
        printf("Wi-Fi connection failed.\n");
        return 1;
    }
    printf("Wi-Fi Connected! IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));

    udp_socket = udp_new();
    ipaddr_aton(LAPTOP_IP, &dest_addr);
    udp_bind(udp_socket, IP_ADDR_ANY, UDP_PORT);
    udp_recv(udp_socket, udp_recv_cb, NULL);

    imu_init();
    motor_init_pins();
    ultra_init();
    encoder_init();
    lidar_init(); // Init memory/UART, but DO NOT START yet.

    encoder_set_remote_udp_target(udp_socket, &dest_addr, UDP_PORT); 
    lidar_set_udp_target(udp_socket, &dest_addr, UDP_PORT); 
    
    // Note: lidar_start() is REMOVED from here.

    absolute_time_t next_telemetry_time = get_absolute_time();
    absolute_time_t next_sensor_time = get_absolute_time();
    absolute_time_t next_map_time = get_absolute_time();

    while (true) {
        cyw43_arch_poll();
        
        // ONLY process LIDAR if active
        if (lidar_active) {
            lidar_update(); 
        }

        if (absolute_time_diff_us(get_absolute_time(), next_telemetry_time) < 0) {
            encoder_update_and_report(); 
            next_telemetry_time = delayed_by_ms(get_absolute_time(), TELEMETRY_MS);
        }

        if (absolute_time_diff_us(get_absolute_time(), next_sensor_time) < 0) {
            update_sensors();
            next_sensor_time = delayed_by_ms(get_absolute_time(), SENSOR_READ_MS);
        }
        
        // ONLY stream map if active
        if (lidar_active && absolute_time_diff_us(get_absolute_time(), next_map_time) < 0) {
            lidar_send_map_chunked(); 
            next_map_time = delayed_by_ms(get_absolute_time(), MAP_UPDATE_MS);
        }
        
        sleep_us(50); 
    }
    return 0;
}