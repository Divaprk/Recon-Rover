#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

// per-arch header ONLY
#include "pico/cyw43_arch/arch_threadsafe_background.h"

#include "lwip/udp.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "net_io.h"


static struct udp_pcb *udp_server = NULL;
static udp_text_cb_t user_cb = NULL;

static void udp_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                        const ip_addr_t *addr, u16_t port) {
    if (!p) return;
    char buf[256];
    size_t n = p->len < (sizeof buf - 1) ? p->len : (sizeof buf - 1);
    memcpy(buf, p->payload, n);
    buf[n] = '\0';
    if (user_cb) user_cb(buf);
    pbuf_free(p);
}

bool net_init_and_connect(const char* ssid, const char* pass) {
    if (cyw43_arch_init()) {
        printf("CYW43 init failed\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi SSID: %s\n", ssid);
    int rc = cyw43_arch_wifi_connect_timeout_ms(
        ssid, pass, CYW43_AUTH_WPA2_AES_PSK, 30000
    );
    if (rc) {
        printf("Wi-Fi connect failed, rc=%d\n", rc);
        return false;
    }
    const ip4_addr_t *ip = netif_ip4_addr(netif_default);
    printf("Wi-Fi connected. IP: %s\n", ip4addr_ntoa(ip));
    return true;
}

bool net_start_udp(uint16_t port, udp_text_cb_t on_text) {
    user_cb = on_text;

    // (Optional but recommended for thread-safe arch)
    cyw43_arch_lwip_begin();
    udp_server = udp_new();
    if (!udp_server) {
        cyw43_arch_lwip_end();
        printf("udp_new failed\n");
        return false;
    }
    err_t err = udp_bind(udp_server, IP_ADDR_ANY, port);
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        printf("udp_bind failed: %d\n", err);
        return false;
    }

    cyw43_arch_lwip_begin();
    udp_recv(udp_server, udp_recv_cb, NULL);
    cyw43_arch_lwip_end();

    printf("UDP listening on %u\n", port);
    return true;
}
