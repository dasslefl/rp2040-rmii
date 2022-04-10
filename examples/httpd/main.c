
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"

#include "eth_lwip.h"

#include "lwip/dhcp.h"
#include "lwip/init.h"
#include "lwip/apps/httpd.h"


void netif_link_callback(struct netif *netif) {
    printf("netif link status changed %s\n", netif_is_link_up(netif) ? "up" : "down");
}

void netif_status_callback(struct netif *netif) {
    printf("netif status changed %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
}

// Debug für HTTP
#if LWIP_DEBUG
const char *lwip_strerr(err_t err) {
    return "";
}
#endif

int main() {

    struct netif netif;

    eth_lwip_init(&netif);

    // assign callbacks for link and status
    netif_set_link_callback(&netif, netif_link_callback);
    netif_set_status_callback(&netif, netif_status_callback);

    // set the default interface and bring it up
    netif_set_default(&netif);
    netif_set_up(&netif);

    // Start DHCP client
    dhcp_start(&netif);
    
    httpd_init();
    
    while (true) {
        eth_lwip_poll();
    }
}