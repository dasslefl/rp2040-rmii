
// hardware.h
// definition of PIO's, pins, ...

#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#include "hardware/pio.h"

// 3*50MHz = 150MHz sysclk
#define CLK_MULTIPLIER  3

// PINS
#define PIN_ETH_CLK     21
#define PIN_ETH_RST     22

#define PIN_ETH_MDC     20
#define PIN_ETH_MDIO    19

#define PIN_ETH_RX_EN   1
#define PIN_ETH_RX_0    2
#define PIN_ETH_RX_1    3

#define PIN_ETH_TX_EN   4
#define PIN_ETH_TX_0    5
#define PIN_ETH_TX_1    6

#define ETH_PIO         pio0
#define ETH_PIO_HW      pio0_hw
#define ETH_PIO_IRQ     PIO0_IRQ_0

// Adresse des PHY
#define PHY_ADDR 0x01

// Empfangspuffer
#define RX_BUF_SIZE_BYTES 1560
#define RX_BUF_SIZE_WORDS (RX_BUF_SIZE_BYTES / sizeof(uint))

#endif