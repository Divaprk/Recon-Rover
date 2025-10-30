#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

// ---- System model ----
#define NO_SYS                 1          // no OS
#define LWIP_TCPIP_CORE_LOCKING 0

// ---- Protocols we use ----
#define LWIP_IPV4              1
#define LWIP_ICMP              1
#define LWIP_UDP               1
#define LWIP_TCP               0          // off (we only use UDP)
#define LWIP_RAW               1

// ---- Disable Sequential API completely (required for NO_SYS=1) ----
#define LWIP_NETCONN           0
#define LWIP_SOCKET            0
#define LWIP_NETIF_API         0
#define LWIP_COMPAT_SOCKETS    0
#define MEMP_NUM_NETCONN       0

// ---- Netif and hostname (needed by cyw43_lwip.c) ----
#define LWIP_NETIF_HOSTNAME    1

// ---- DHCP (ok with NO_SYS when used from poll/begin/end) ----
#define LWIP_DHCP              1

// ---- Memory ----
#define MEM_ALIGNMENT          4
#define MEM_SIZE               8192
#define MEMP_NUM_PBUF          16
#define PBUF_POOL_SIZE         8

// ---- Ethernet/ARP (harmless) ----
#define LWIP_ETHERNET          1
#define LWIP_ARP               1

#endif /* __LWIPOPTS_H__ */
