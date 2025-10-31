#ifndef ENCODER_H
#define ENCODER_H

// --- ADD THESE ---
#include "lwip/ip_addr.h" // For ip_addr_t
struct udp_pcb; // Forward declaration is fine for a pointer
// --- END ADD ---

// Call this once in main() to set up the encoders,
// interrupts, and reporting timer.
void encoder_init(void);

// --- ADD THIS FUNCTION PROTOTYPE ---
// Call this from udp_recv_cb in main.c to tell the encoder
// where to send telemetry data.
void encoder_set_remote_udp_target(struct udp_pcb *pcb, const ip_addr_t *addr, u16_t port);
// --- END ADD ---

#endif // ENCODER_H