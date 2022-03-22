
#ifndef __ETH_LWIP_H__
#define __ETH_LWIP_H__

#include "lwip/netif.h"

#define ETH_LWIP_CONN_POLL_INTERVAL 200 // Alle 200 ms Verbindungsstatus pollen

void eth_lwip_init(struct netif * netif);
void eth_lwip_poll();

#endif