
#if !defined(__ETH_H__)
#define __ETH_H__

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

#define ETH_PREAMBLE ((uint8_t[]){ 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xd5 })
#define ETH_PREAMBLE_SIZE 8 // Länge Ethernet Präambel in Bytes

#define ETH_MIN_PAYLOAD_LEN 60 // +4 Byte CRC -> 64er Paket
#define ETH_TX_MAX_LEN 1540

#define ETH_MAC_LEN 6

typedef struct __attribute__((__packed__)) {
    uint8_t preamble[ETH_PREAMBLE_SIZE];
    uint8_t data[ETH_TX_MAX_LEN + 4]; // DMA überträgt eventuell 1 Word mehr
} eth_tx_buf_t;

bool eth_is_connected();
void eth_init();
void eth_init_tx_buf(eth_tx_buf_t * buffer);

void eth_wait_for_transmit_done();

bool eth_transmit(eth_tx_buf_t * buf, uint len);
//void eth_transmit_bytes(uint8_t * bytes, uint len);

uint8_t * eth_get_rx_buf();
uint eth_get_rx_buf_count();
void eth_reset_rx_buf();

uint eth_get_time_millis();
bool eth_every_ms(uint * timer, uint interval);

void eth_hexdump(const void* data, uint size);

#endif // __ETH_H__
