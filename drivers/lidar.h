#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>
#include <stdbool.h>
#include "lwip/udp.h"
#include "lwip/ip_addr.h"

// ========== LIDAR CONFIGURATION ==========
// UART pins for RPLIDAR A1
#define LIDAR_UART_ID    uart1
#define LIDAR_TX_PIN     4     // GP4 -> LiDAR RX
#define LIDAR_RX_PIN     5     // GP5 <- LiDAR TX
#define LIDAR_PWM_PIN    0     // GP0 -> LiDAR motor PWM

// Map streaming port (different from telemetry port 5001)
#define LIDAR_MAP_PORT   5005

// ========== OCCUPANCY GRID SETTINGS ==========
#define MAP_W            240
#define MAP_H            240
#define MAP_RES_MM       80    // 8 cm per cell

// ========== PUBLIC FUNCTIONS ==========

/**
 * @brief Initialize LIDAR hardware (UART, PWM motor control)
 * Call this once in main() during initialization
 */
void lidar_init(void);

/**
 * @brief Start the LIDAR spinning and begin scanning
 * Call this after lidar_init() and after WiFi is connected
 */
void lidar_start_scan(void);

/**
 * @brief Set the UDP target for map streaming
 * @param pcb UDP control block (can be the same one used for telemetry)
 * @param addr Target IP address (typically your laptop)
 * Call this from your UDP receive callback when first command is received
 */
void lidar_set_map_target(struct udp_pcb *pcb, const ip_addr_t *addr);

/**
 * @brief Process LIDAR data and update occupancy grid
 * Call this frequently in main loop (it's non-blocking)
 * Automatically streams map data to laptop at ~7 fps
 */
void lidar_process(void);

/**
 * @brief Update rover pose for mapping
 * @param x_mm X position in millimeters
 * @param y_mm Y position in millimeters  
 * @param heading_rad Heading in radians
 * Call this whenever you update odometry (future enhancement)
 */
void lidar_update_pose(float x_mm, float y_mm, float heading_rad);

#endif // LIDAR_H