#ifndef ENCODER_H
#define ENCODER_H

#include "lwip/ip_addr.h"
struct udp_pcb;

// Initialize pins and GPIO interrupts (NO timer anymore)
void encoder_init(void);

// Configure where to send UDP data
void encoder_set_remote_udp_target(struct udp_pcb *pcb, const ip_addr_t *addr, u16_t port);

// New function: Call this from main loop to calc speeds and send UDP
void encoder_update_and_report(void);

#endif // ENCODER_H