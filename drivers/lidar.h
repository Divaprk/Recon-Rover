#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>
#include <stdbool.h>
#include "lwip/udp.h"
#include "lwip/ip_addr.h"

void lidar_init(void);
void lidar_start(void);
void lidar_stop(void); // <--- NEW FUNCTION
void lidar_update(void);
void lidar_set_udp_target(struct udp_pcb *pcb, const ip_addr_t *addr, u16_t port);
void lidar_send_map_chunked(void);
void lidar_set_pose(float x_mm, float y_mm, float theta_rad);

#endif // LIDAR_H