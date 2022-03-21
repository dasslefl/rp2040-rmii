
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
#include "dma_accel.h"

uint8_t tx_data[] = "Lorem ipsum sit dolo8 amed";

uint timer_led;

int main() {
    eth_init();

    // LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    /*
    while (true) {

        printf("%08x\n\n", crc32(tx_data, sizeof(tx_data)));

        uint seed = 0x00000000;

        printf("%08x\n", dma_accel_crc32(tx_data, sizeof(tx_data), 0x00, seed));
        printf("%08x\n", dma_accel_crc32(tx_data, sizeof(tx_data), 0x00, ~seed));
        printf("%08x\n", dma_accel_crc32(tx_data, sizeof(tx_data), 0x01, seed));
        printf("%08x\n", dma_accel_crc32(tx_data, sizeof(tx_data), 0x01, ~seed));

        printf("%08x\n", ~dma_accel_crc32(tx_data, sizeof(tx_data), 0x00, seed));
        printf("%08x\n", ~dma_accel_crc32(tx_data, sizeof(tx_data), 0x00, ~seed));
        printf("%08x\n", ~dma_accel_crc32(tx_data, sizeof(tx_data), 0x01, seed));
        printf("%08x\n", ~dma_accel_crc32(tx_data, sizeof(tx_data), 0x01, ~seed));

        printf("\n");
        sleep_ms(500);
    }*/
    
    
    // Auf Verbindung warten
    while(!eth_is_connected()) {
        printf("Warte auf Verbindung...\n");
        sleep_ms(500);
    }

    printf("Verbindung hergestellt.\n");
    
    while (true) {
        
        if(eth_every_ms(&timer_led, 500)) {
            gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));
            eth_transmit_bytes(tx_data, sizeof(tx_data));
        }

        uint rx_count = eth_get_rx_buf_count();
        uint8_t * rx_buf = eth_get_rx_buf();
        if(rx_count > 0) {
            printf("\nLen %u\n", rx_count);
            eth_hexdump(rx_buf, rx_count);

            eth_reset_rx_buf();
        }
    }
}