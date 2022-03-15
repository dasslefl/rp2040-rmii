
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

eth_tx_buf_t buf;

uint8_t tx_data[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
    0xd4, 0x3d, 0x7e, 0xde, 0xad, 0x07, 
    0x84, 0x00, 
    0xde, 0xad, 0xbe, 0xef
};

int main() {

    eth_init();

    // LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    // Auf Verbindung warten
    while(!eth_is_connected()) {
        printf("Warte auf Verbindung...\n");
        sleep_ms(500);
    }

    printf("Verbindung hergestellt.\n");

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