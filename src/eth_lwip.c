
// eth_lwip.h
// Bindet lwip an das phy-Library an

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "eth.h"
#include "eth_lwip.h"

#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "lwip/init.h"

#include "lwip/etharp.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"

static eth_tx_buf_t tx_buffer;
static struct netif * eth_netif;

static uint status_poll_timer;

static err_t eth_lwip_output(struct netif *netif, struct pbuf *p) {
    // Puffer zusammenführen
    uint tot_len = 0;
    
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        memcpy(tx_buffer.data + tot_len, q->payload, q->len);

        tot_len += q->len;

        if(tot_len >= ETH_TX_MAX_LEN) {
            return ERR_BUF;
        }

        if (q->len == q->tot_len) {
            break;
        }
    }

    eth_wait_for_transmit_done();
    eth_transmit(&tx_buffer, tot_len);

    return ERR_OK;
}

static err_t eth_lwip_low_init(struct netif *netif) {
    // Setzt unter anderem Systemtakt
    eth_init();

    eth_netif = netif;

    netif->linkoutput = eth_lwip_output;
    netif->output     = etharp_output;
    netif->mtu        = 1500; 
    netif->flags      = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET | NETIF_FLAG_IGMP | NETIF_FLAG_MLD6;

    // generate MAC from unique board id
    pico_unique_board_id_t board_id;

    pico_get_unique_board_id(&board_id);

    netif->hwaddr[0] = 0xb8;
    netif->hwaddr[1] = 0x27;
    netif->hwaddr[2] = 0xeb;
    memcpy(&netif->hwaddr[3], &board_id.id[5], 3);
    
    netif->hwaddr_len = ETH_HWADDR_LEN;

    return ERR_OK;
}

void eth_lwip_init(struct netif * netif) {
    lwip_init();

    eth_init_tx_buf(&tx_buffer);

    netif_add(netif, IP4_ADDR_ANY, IP4_ADDR_ANY, IP4_ADDR_ANY, NULL, eth_lwip_low_init, netif_input);

    netif->name[0] = 'e';
    netif->name[1] = '0';
}

void eth_lwip_poll() {
    // Verbindungsstatus
    if(eth_every_ms(&status_poll_timer, ETH_LWIP_CONN_POLL_INTERVAL)) {
        bool link_state = eth_is_connected();

        if(link_state != netif_is_link_up(eth_netif)) {
            if (link_state) netif_set_link_up(eth_netif);
            else            netif_set_link_down(eth_netif);
        }
    }
    // RX
    uint rx_frame_length = eth_get_rx_buf_count();
    if(rx_frame_length > 0) {
        uint8_t * rx_frame = eth_get_rx_buf();

        struct pbuf* p = pbuf_alloc(PBUF_RAW, rx_frame_length, PBUF_POOL);

        pbuf_take(p, rx_frame, rx_frame_length);

        if (eth_netif->input(p, eth_netif) != ERR_OK) {
            pbuf_free(p);
        }

        eth_reset_rx_buf();
    }

    sys_check_timeouts();
}