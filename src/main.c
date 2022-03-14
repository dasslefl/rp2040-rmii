
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"

#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include "hardware/structs/bus_ctrl.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"

#include "eth.pio.h"

#include "crc32.h"

// 3*50MHz = 150MHz sysclk
#define CLK_MULTIPLIER 3

// PINS
#define PIN_ETH_CLK 21
#define PIN_ETH_RST 22

#define PIN_ETH_MDC 20
#define PIN_ETH_MDIO 19

#define PIN_ETH_RX_EN 1
#define PIN_ETH_RX_0 2
#define PIN_ETH_RX_1 3

#define PIN_ETH_TX_EN 4
#define PIN_ETH_TX_0 5
#define PIN_ETH_TX_1 6


// SMI Interface
#define PHY_ADDR 0x01

#define PHY_REG_BCR 0
#define PHY_REG_BCR_SOFT_RESET (1 << 15)
#define PHY_REG_BCR_LOOPBACK (1 << 14)
#define PHY_REG_BCR_SPEED_100 (1 << 13)
#define PHY_REG_BCR_AUTO_NEGOTIATION_ENABLE (1 << 12)
#define PHY_REG_BCR_FULL_DUPLEX (1 << 8)

#define PHY_REG_BSR 1
#define PHY_REG_BSR_AUTO_NEGOTIATION_COMPLETE (1 << 5)
#define PHY_REG_BSR_LINK_UP (1 << 2)

#define PHY_REG_PIR1 2
#define PHY_REG_PIR1_DEFAULT_VALUE 0x0007

#define PHY_REG_SECR 26

#define SMI_OPCODE_READ 0b10
#define SMI_OPCODE_WRITE 0b01

#define SMI_ADRESS_MASK 0b11111 // 5 BIT

#define RX_BUF_SIZE_BYTES 1560
#define RX_BUF_SIZE_WORDS (RX_BUF_SIZE_BYTES / sizeof(uint))

// Puffer

uint8_t rx_buf[RX_BUF_SIZE_BYTES]; // 382 * uint32_t
/*
uint8_t tx_data[] = { 
0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xd5, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xd4, 0x3d, 
0x7e, 0xdd, 0x00, 0xd7, 0x84, 0x00, 0xde, 0xad, 
0xbe, 0xef, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x93, 0x8a, 0x36, 0xf0, 0x00, 0x00, 0x00, 0x00 };*/

#define ETH_PREAMBLE ((uint8_t[]){ 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xd5 })
#define ETH_PREAMBLE_SIZE 8 // Länge Ethernet Präambel in Bytes

#define ETH_MIN_PAYLOAD_LEN 60 // +4 Byte CRC -> 64er Paket

typedef struct __attribute__((__packed__)) {

    uint8_t preamble[ETH_PREAMBLE_SIZE];
    uint8_t data[1500];

} eth_tx_buf_t;

static uint tx_sm_entry_point;

void smi_init() {
    gpio_init(PIN_ETH_MDC);
    gpio_set_dir(PIN_ETH_MDC, GPIO_OUT);

    gpio_init(PIN_ETH_MDIO);
    gpio_set_dir(PIN_ETH_MDIO, GPIO_IN);
}

// Empfangen -> MDIO Input
void smi_set_receive() {
    gpio_put(PIN_ETH_MDIO, 0);
    gpio_set_dir(PIN_ETH_MDIO, GPIO_IN);
}

void smi_set_transmit() {
    gpio_set_dir(PIN_ETH_MDIO, GPIO_OUT);
}

void smi_clock() {
    // Hold beim TX
    sleep_us(1);
  
    gpio_put(PIN_ETH_MDC, 1);
    sleep_us(1);
    gpio_put(PIN_ETH_MDC, 0);
}

void smi_transmit(uint16_t data) {
    smi_set_transmit();
  
    // MSB first
    for(uint8_t i = 0; i < 16; i++) {
    
        gpio_put(PIN_ETH_MDIO, data & (1 << 15));
        smi_clock();
        data = data << 1;
    }
}

uint16_t smi_receive() {
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

void smi_transmit_header(uint8_t opcode, uint8_t phy_addr, uint8_t reg_addr) {
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

void bindump16(uint16_t data) {
    // MSB first
    for(uint8_t i = 0; i < 16; i++) {

        if(data & (1 << 15)) 
            putchar('1');
        else
            putchar('0');

        data = data << 1;
    }
}

// Ende SMI

// RX

void hexdump(const void* data, size_t size) {
	
	for(uint i = 0; i < size; i++) {
		printf("%02hhx ", ((unsigned char*) data)[i]);
        if((i + 1) % 8 == 0)
            putchar(' ');
        if((i + 1) % 16 == 0)
            putchar('\n');
	}
    putchar('\n');
}

void eth_rx_dma_init(uint8_t * target, size_t len_words) {
    // FIFOs löschen (reine Vorsicht)
    pio_sm_clear_fifos(pio0, 0);

    dma_channel_config c = dma_channel_get_default_config(0);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio0, 0, false));

    dma_channel_configure(0, &c,
        target,        // Destination pointer
        &(pio0_hw->rxf[0]),      // Source pointer
        len_words, // Number of transfers
        true                // Start immediately
    );

    pio0_hw->irq = 0x01; // ersten IRQ leeren, damit Loop weiter läuft
}

void pio_interrupt_handler() {
    if(pio0_hw->irq == 0x01) { // RX IRQ
        uint dma_transfer_count = RX_BUF_SIZE_WORDS - dma_hw->ch[0].transfer_count;
        // DMA abbrechen
        dma_channel_abort(0);

        // Paketlänge auslesen
        uint * rx_buf_32 = (uint *) rx_buf;
        // Anzahl empfangener 2 Bit Worte letztes Objekt in Speicher -> Auslesen und durch 4 teilen für Bytes
        uint packet_len = rx_buf_32[dma_transfer_count - 1] / 4; 

        printf("\nDMA %u packet %u\n", dma_transfer_count, packet_len);

        hexdump(rx_buf, packet_len);

        // DMA re-arm
        eth_rx_dma_init(rx_buf, RX_BUF_SIZE_WORDS);
    }
}

void eth_rx_init() {
    const PIO pio = pio0;
    const uint sm = 0;

    // SM initialisieren
    // Programm laden
    uint offset = pio_add_program(pio, &eth_rx_program);
    pio_sm_config config = eth_rx_program_get_default_config(offset);
    // RX0 und RX1 sind IN-Pins
    sm_config_set_in_pins(&config, PIN_ETH_RX_0); 
    // RX-Fifo -> Auto push
    sm_config_set_in_shift(&config, true, true, 32);
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_RX);

	pio_sm_init(pio, sm, offset, &config);

    // Pin für JMP (Ende inner loop) auf RX En setzen: Muss nach init sein weil init das Register überschreibt
    pio0_hw->sm[sm].execctrl |= (PIN_ETH_RX_EN << 24);

    // INT
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_interrupt_handler);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio0_hw->inte0 = PIO_IRQ0_INTE_SM0_BITS /*| PIO_IRQ0_INTE_SM1_BITS*/;

    eth_rx_dma_init(rx_buf, RX_BUF_SIZE_WORDS);
	pio_sm_set_enabled(pio, sm, true);
}

// TX
void eth_tx_init() {
    const PIO pio = pio0;
    const uint sm = 1;

    // Pins als Output
    pio_gpio_init(pio, PIN_ETH_TX_EN);
    pio_gpio_init(pio, PIN_ETH_TX_0);
    pio_gpio_init(pio, PIN_ETH_TX_1);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_ETH_TX_EN, 3, true);

    uint offset = pio_add_program(pio0, &eth_tx_program);
    tx_sm_entry_point = offset;
    pio_sm_config c = eth_tx_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, PIN_ETH_TX_EN); // TX_EN Sideset
    sm_config_set_out_pins(&c, PIN_ETH_TX_0, 2);   // TX_0 und TX_1 out

    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    pio_sm_init(pio, sm, offset, &c);
}

void eth_transmit_raw(uint8_t * data, uint len) {

    uint8_t len_words = (len / sizeof(uint) + 1); // 1 Word mehr damit im Fall der Teilbarkeit die Schleife die den Fifo liest was zum Arbeiten hat
    const PIO pio = pio0;
    const uint sm = 1;
    const uint dma_channel = 1;

    // SM anhalten, FIFOs löschen
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);

    // Länge des zu sendenden Pakets hinterlegen
    pio_sm_put_blocking(pio, sm, (len * 4) - 1); // len - 1 weil am Ende der Schleife mindestens ein Word gesendet wird

    dma_channel_config c = dma_channel_get_default_config(dma_channel);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));

    dma_channel_configure(dma_channel, &c,
        &(pio0_hw->txf[sm]),        // Destination pointer
        data,      // Source pointer
        len_words, // Number of transfers
        true       // Start immediately
    );

    // SM wieder starten, an Anfang springen
    pio_sm_set_enabled(pio, sm, true);
    pio_sm_exec(pio, sm, pio_encode_jmp(tx_sm_entry_point));
}

void eth_transmit(eth_tx_buf_t * buf, uint len) {

    // Muss Paket auf 64 Byte aufgefüllt werden?
    if(len < ETH_MIN_PAYLOAD_LEN) {
        memset(buf->data + len, 0, ETH_MIN_PAYLOAD_LEN - len);
        len = ETH_MIN_PAYLOAD_LEN;
    }

    // CRC berechnen, an Ende anfügen
    uint32_t crc = crc32(buf->data, len);
    memcpy(buf->data + len, (uint8_t *) &crc, CRC32_SIZE);

    len += CRC32_SIZE + ETH_PREAMBLE_SIZE;

    eth_transmit_raw((uint8_t *) buf, len);
}

void eth_init_tx_buf(eth_tx_buf_t * buffer) {
    // Präambel setzen
    memcpy(buffer->preamble, ETH_PREAMBLE, ETH_PREAMBLE_SIZE);
}

eth_tx_buf_t buf;

uint8_t tx_data[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xd4, 0x3d, 0x7e, 0xde, 0xad, 0x07, 
    0x84, 0x00, 
    0xde, 0xad, 0xbe, 0xef
};

int main() {
    // Overclock auf 50 MHz * CLK_MULTIPLIER (150 MHz)
    set_sys_clock_khz(50000 * CLK_MULTIPLIER, false);
    // 50 MHz Clock Output
    clock_gpio_init(PIN_ETH_CLK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, CLK_MULTIPLIER);

    stdio_init_all();

    // LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    // SMI
    smi_init();
    // Reset ETH frei geben
    sleep_ms(250);
    gpio_init(PIN_ETH_RST);
    gpio_set_dir(PIN_ETH_RST, GPIO_OUT);
    gpio_put(PIN_ETH_RST, 1);

    // PHY konfigurieren
    // Phy initialisieren
    // Soft reset
    smi_reg_write(PHY_ADDR, PHY_REG_BCR, PHY_REG_BCR_SOFT_RESET);
    sleep_ms(100);
    // Überprüfen, ob mit PHY kommuniziert werden kann
    if(smi_reg_read(PHY_ADDR, PHY_REG_PIR1) != PHY_REG_PIR1_DEFAULT_VALUE) {
        while(1) {
            printf("Verbindung mit PHY fehlgeschlagen.\n");
            sleep_ms(1000);
        }
    }

    // Grundeinstellung: Auto Negotiation an
    smi_reg_set_bits(PHY_ADDR, PHY_REG_BCR, PHY_REG_BCR_AUTO_NEGOTIATION_ENABLE);
    //smi_reg_set_bits(PHY_ADDR, PHY_REG_BCR, PHY_REG_BCR_LOOPBACK | PHY_REG_BCR_SPEED_100 | PHY_REG_BCR_FULL_DUPLEX);

    // Auf Verbindung warten
    while(!smi_reg_read_bits(PHY_ADDR, PHY_REG_BSR, PHY_REG_BSR_LINK_UP)) {
        printf("Warte auf Verbindung...\n");
        sleep_ms(500);
    }

    // Auto Negotiation abwarten
    while(!smi_reg_read_bits(PHY_ADDR, PHY_REG_BSR, PHY_REG_BSR_AUTO_NEGOTIATION_COMPLETE)) {
        printf("Warte auf Handshake...\n");
        sleep_ms(500);
    };

    printf("Verbindung hergestellt.\n");

    eth_rx_init();
    eth_tx_init();

    eth_init_tx_buf(&buf);

    memcpy(buf.data, tx_data, sizeof(tx_data));
    
    while (true) {
        
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        eth_transmit(&buf, sizeof(tx_data));
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(500);
    }
}