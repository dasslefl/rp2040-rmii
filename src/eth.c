
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
#include "hardware.h"
#include "smi.h"
#include "eth.h"

static uint8_t rx_buf[RX_BUF_SIZE_BYTES]; 

static eth_tx_buf_t tx_buf;

static uint rx_sm_num;
static uint tx_sm_num;

static uint rx_sm_offset;
static uint tx_sm_offset;

static int rx_dma_chan;
static int tx_dma_chan;

void eth_hexdump(const void* data, uint size) {
	for(uint i = 0; i < size; i++) {
		printf("%02hhx ", ((unsigned char*) data)[i]);
        if((i + 1) % 8 == 0)
            putchar(' ');
        if((i + 1) % 16 == 0)
            putchar('\n');
	}
    putchar('\n');
}

static void eth_rx_dma_init(uint8_t * target, uint len_words) {
    // FIFOs löschen (reine Vorsicht)
    pio_sm_clear_fifos(ETH_PIO, rx_sm_num);

    dma_channel_config c = dma_channel_get_default_config(rx_dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(ETH_PIO, rx_sm_num, false));

    dma_channel_configure(0, &c,
        target,                             // Destination pointer
        &(ETH_PIO_HW->rxf[rx_sm_num]),      // Source pointer
        len_words,                          // Number of transfers
        true                                // Start immediately
    );

    ETH_PIO_HW->irq = 0x01; // ersten IRQ leeren, damit Loop weiter läuft
}

void eth_pio_interrupt_handler() {
    if(ETH_PIO_HW->irq == 0x01) { // RX IRQ
        uint dma_transfer_count = RX_BUF_SIZE_WORDS - dma_hw->ch[rx_dma_chan].transfer_count;
        // DMA abbrechen
        dma_channel_abort(rx_dma_chan);

        // Paketlänge auslesen
        uint * rx_buf_32 = (uint *) rx_buf;
        // Anzahl empfangener 2 Bit Worte letztes Objekt in Speicher -> Auslesen und durch 4 teilen für Bytes
        uint packet_len = rx_buf_32[dma_transfer_count - 1] / 4; 

        printf("\nDMA %u packet %u\n", dma_transfer_count, packet_len);

        eth_hexdump(rx_buf, packet_len);

        // DMA re-arm
        eth_rx_dma_init(rx_buf, RX_BUF_SIZE_WORDS);
    }
}

// RX initialisieren, only to be called once
static void eth_rx_init() {
    // Resourcen klären
    rx_sm_num = pio_claim_unused_sm(ETH_PIO, true);
    rx_dma_chan = dma_claim_unused_channel(true);
    rx_sm_offset = pio_add_program(ETH_PIO, &eth_rx_program);

    pio_sm_config config = eth_rx_program_get_default_config(rx_sm_offset);
    // RX0 und RX1 sind IN-Pins
    sm_config_set_in_pins(&config, PIN_ETH_RX_0); 
    // RX-Fifo -> Auto push
    sm_config_set_in_shift(&config, true, true, 32);
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_RX);

	pio_sm_init(ETH_PIO, rx_sm_num, rx_sm_offset, &config);

    // Pin für JMP (Ende inner loop) auf RX En setzen: Muss nach init sein weil init das Register überschreibt
    ETH_PIO_HW->sm[rx_sm_num].execctrl |= (PIN_ETH_RX_EN << 24);

    // INT
    irq_set_exclusive_handler(ETH_PIO_IRQ, eth_pio_interrupt_handler);
    irq_set_enabled(ETH_PIO_IRQ, true);
    ETH_PIO_HW->inte0 = PIO_IRQ0_INTE_SM0_BITS /*| PIO_IRQ0_INTE_SM1_BITS*/;

    eth_rx_dma_init(rx_buf, RX_BUF_SIZE_WORDS);
	pio_sm_set_enabled(ETH_PIO, rx_sm_num, true);
}

// TX initialisieren, only to be called once
static void eth_tx_init() {
    // Resourcen
    tx_sm_num = pio_claim_unused_sm(ETH_PIO, true);
    tx_dma_chan = dma_claim_unused_channel(true);
    tx_sm_offset = pio_add_program(ETH_PIO, &eth_tx_program);

    // Pins als Output
    pio_gpio_init(ETH_PIO, PIN_ETH_TX_EN);
    pio_gpio_init(ETH_PIO, PIN_ETH_TX_0);
    pio_gpio_init(ETH_PIO, PIN_ETH_TX_1);
    pio_sm_set_consecutive_pindirs(ETH_PIO, tx_sm_num, PIN_ETH_TX_EN, 3, true);

    pio_sm_config c = eth_tx_program_get_default_config(tx_sm_offset);
    sm_config_set_sideset_pins(&c, PIN_ETH_TX_EN); // TX_EN Sideset
    sm_config_set_out_pins(&c, PIN_ETH_TX_0, 2);   // TX_0 und TX_1 out

    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    pio_sm_init(ETH_PIO, tx_sm_num, tx_sm_offset, &c);
}

static void eth_transmit_raw(uint8_t * data, uint len) {

    uint8_t len_words = (len / sizeof(uint) + 1); // 1 Word mehr damit im Fall der Teilbarkeit die Schleife die den Fifo liest was zum Arbeiten hat

    // SM anhalten, FIFOs löschen
    pio_sm_set_enabled(ETH_PIO, tx_sm_num, false);
    pio_sm_clear_fifos(ETH_PIO, tx_sm_num);

    // Länge des zu sendenden Pakets hinterlegen
    pio_sm_put_blocking(ETH_PIO, tx_sm_num, (len * 4) - 1); // len - 1 weil am Ende der Schleife mindestens ein Word gesendet wird

    dma_channel_config c = dma_channel_get_default_config(tx_dma_chan);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(ETH_PIO, tx_sm_num, true));

    dma_channel_configure(tx_dma_chan, &c,
        &(ETH_PIO_HW->txf[tx_sm_num]),        // Destination pointer
        data,      // Source pointer
        len_words, // Number of transfers
        true       // Start immediately
    );

    // SM wieder starten, an Anfang springen
    pio_sm_set_enabled(ETH_PIO, tx_sm_num, true);
    pio_sm_exec(ETH_PIO, tx_sm_num, pio_encode_jmp(tx_sm_offset));
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

void eth_transmit_bytes(uint8_t * bytes, uint len) {
    memcpy(tx_buf.data, bytes, len);
    eth_transmit(&tx_buf, len);
}

void eth_init_tx_buf(eth_tx_buf_t * buffer) {
    // Präambel setzen
    memcpy(buffer->preamble, ETH_PREAMBLE, ETH_PREAMBLE_SIZE);
}

void eth_init() {
    // Overclock auf 50 MHz * CLK_MULTIPLIER (150 MHz)
    set_sys_clock_khz(50000 * CLK_MULTIPLIER, false);
    // 50 MHz Clock Output
    clock_gpio_init(PIN_ETH_CLK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, CLK_MULTIPLIER);

    sleep_ms(250);
    stdio_init_all();

    // SMI
    smi_init();
    // Reset ETH frei geben
    gpio_init(PIN_ETH_RST);
    gpio_set_dir(PIN_ETH_RST, GPIO_OUT);
    gpio_put(PIN_ETH_RST, 1);

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

    // Sendepuffer vorbereiten
    eth_init_tx_buf(&tx_buf);

    eth_rx_init();
    eth_tx_init();
}

bool eth_is_connected() {
    return (!!smi_reg_read_bits(PHY_ADDR, PHY_REG_BSR, PHY_REG_BSR_LINK_UP)) &&
           (!!smi_reg_read_bits(PHY_ADDR, PHY_REG_BSR, PHY_REG_BSR_AUTO_NEGOTIATION_COMPLETE));
}