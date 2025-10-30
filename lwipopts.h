#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

// --- system model ---
#define NO_SYS                          1     // no RTOS
#define MEM_ALIGNMENT                   4

// --- core protocol enables ---
#define LWIP_IPV4                       1
#define LWIP_IPV6                       0
#define LWIP_ICMP                       1
#define LWIP_UDP                        1
#define LWIP_TCP                        1

// --- network interface / DHCP ---
#define LWIP_SINGLE_NETIF               0     // needed by cyw43 driver (uses netif_list)
#define LWIP_NETIF_HOSTNAME             1     // needed for netif_set_hostname()
#define LWIP_DHCP                       1     // get IP from your Wi-Fi router

// --- sockets/api layers (we use raw/udp only) ---
#define LWIP_RAW                        1
#define LWIP_NETCONN                    0
#define LWIP_SOCKET                     0

// --- buffers (keep modest; pico_w examples use similar) ---
#define TCP_MSS                         (1500 - 20 - 20)
#define TCP_SND_BUF                     (2 * TCP_MSS)
#define TCP_WND                         (TCP_MSS)
#define PBUF_POOL_SIZE                  6
#define MEMP_NUM_SYS_TIMEOUT            8

// --- misc ---
#define LWIP_STATS                      0
#define LWIP_PROVIDE_ERRNO              1
#define ETH_PAD_SIZE                    0

#endif /* __LWIPOPTS_H__ */
