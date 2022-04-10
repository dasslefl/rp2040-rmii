
// smi.c
// implements functions for acessing the serial management interface

#include "pico/stdlib.h"

#include "include/hardware.h"
#include "include/smi.h"

#define SMI_OPCODE_READ 0b10
#define SMI_OPCODE_WRITE 0b01

#define SMI_ADRESS_MASK 0b11111 // 5 BIT

void smi_init() {
    gpio_init(PIN_ETH_MDC);
    gpio_set_dir(PIN_ETH_MDC, GPIO_OUT);

    gpio_init(PIN_ETH_MDIO);
    gpio_set_dir(PIN_ETH_MDIO, GPIO_IN);
}

// Empfangen -> MDIO Input
static inline void smi_set_receive() {
    gpio_put(PIN_ETH_MDIO, 0);
    gpio_set_dir(PIN_ETH_MDIO, GPIO_IN);
}

static inline void smi_set_transmit() {
    gpio_set_dir(PIN_ETH_MDIO, GPIO_OUT);
}

static inline void smi_clock() {
    sleep_us(1);
    gpio_put(PIN_ETH_MDC, 1);
    sleep_us(1);
    gpio_put(PIN_ETH_MDC, 0);
}

static void smi_transmit(uint16_t data) {
    smi_set_transmit();
  
    // MSB first
    for(uint8_t i = 0; i < 16; i++) {
        gpio_put(PIN_ETH_MDIO, data & (1 << 15));
        smi_clock();
        data = data << 1;
    }
}

static uint16_t smi_receive() {
    smi_set_receive();

    uint16_t received_data = 0;
    // MSB first
    for(uint8_t i = 0; i < 16; i++) {
        received_data = received_data << 1;
        // IO High?
        received_data |= (gpio_get(PIN_ETH_MDIO) > 0);

        smi_clock();
    }
    return received_data;
}

static void smi_transmit_header(uint8_t opcode, uint8_t phy_addr, uint8_t reg_addr) {
    // Präambel
    smi_transmit(0xFFFF);
    smi_transmit(0xFFFF);
    // Kopf
    smi_transmit(
      (1 << 14)       | // Start of Frame
      (opcode << 12)  | // Opcode
      ((phy_addr & SMI_ADRESS_MASK) << 7) | // PHY Adresse
      ((reg_addr & SMI_ADRESS_MASK) << 2)   // Register-Adresse, restliche Bits null (Turn Around)
    );
}

uint16_t smi_reg_read(uint8_t phy_addr, uint8_t reg_addr) {
    smi_transmit_header(SMI_OPCODE_READ, phy_addr, reg_addr);
    return smi_receive();
}

uint16_t smi_reg_read_bits(uint8_t phy_addr, uint8_t reg_addr, uint16_t bits) {
    smi_transmit_header(SMI_OPCODE_READ, phy_addr, reg_addr);
    return smi_receive() & bits;
}

void smi_reg_write(uint8_t phy_addr, uint8_t reg_addr, uint16_t data) {
    smi_transmit_header(SMI_OPCODE_WRITE, phy_addr, reg_addr);
    smi_transmit(data);
}

// Einzelne Bits setzen
void smi_reg_set_bits(uint8_t phy_addr, uint8_t reg_addr, uint16_t bits) {
    uint16_t reg_val = smi_reg_read(phy_addr, reg_addr);

    reg_val |= bits;

    smi_reg_write(phy_addr, reg_addr, reg_val);
}

// Einzelne Bits löschen
void smi_reg_clear_bits(uint8_t phy_addr, uint8_t reg_addr, uint16_t bits) {
    uint16_t reg_val = smi_reg_read(phy_addr, reg_addr);

    reg_val &= ~bits;

    smi_reg_write(phy_addr, reg_addr, reg_val);
}