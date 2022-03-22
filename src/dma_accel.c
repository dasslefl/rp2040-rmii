
// dma_accel.c
// Beschleunigung von memcpy und CRC via DMA

#include <stdint.h>

#include "pico/stdlib.h"

#include "hardware/dma.h"
#include "hardware/pio.h"

/*
    while (true) {

        printf("%08x\n\n", crc32(tx_data, sizeof(tx_data)));

        uint seed = 0x00000000;

        printf("%08x\n", dma_accel_crc32(tx_data, sizeof(tx_data), 0x00, seed, false));
        printf("%08x\n", dma_accel_crc32(tx_data, sizeof(tx_data), 0x00, ~seed, false));
        printf("%08x\n", dma_accel_crc32(tx_data, sizeof(tx_data), 0x01, seed, false));
        printf("%08x\n", dma_accel_crc32(tx_data, sizeof(tx_data), 0x01, ~seed, false));

        printf("%08x\n", ~dma_accel_crc32(tx_data, sizeof(tx_data), 0x00, seed, false));
        printf("%08x\n", ~dma_accel_crc32(tx_data, sizeof(tx_data), 0x00, ~seed, false));
        printf("%08x\n", ~dma_accel_crc32(tx_data, sizeof(tx_data), 0x01, seed, false));
        printf("%08x\n", ~dma_accel_crc32(tx_data, sizeof(tx_data), 0x01, ~seed, false));

        printf("\n");

        printf("%08x\n", dma_accel_crc32(tx_data, sizeof(tx_data), 0x00, seed, true));
        printf("%08x\n", dma_accel_crc32(tx_data, sizeof(tx_data), 0x00, ~seed, true));
        printf("%08x\n", dma_accel_crc32(tx_data, sizeof(tx_data), 0x01, seed, true));
        printf("%08x\n", dma_accel_crc32(tx_data, sizeof(tx_data), 0x01, ~seed, true));

        printf("%08x\n", ~dma_accel_crc32(tx_data, sizeof(tx_data), 0x00, seed, true));
        printf("%08x\n", ~dma_accel_crc32(tx_data, sizeof(tx_data), 0x00, ~seed, true));
        printf("%08x\n", ~dma_accel_crc32(tx_data, sizeof(tx_data), 0x01, seed, true));
        printf("%08x\n", ~dma_accel_crc32(tx_data, sizeof(tx_data), 0x01, ~seed, true));

        printf("\n");
        sleep_ms(500);
    }*/

inline static uint32_t reverseBits(uint32_t n) {
    n = (n >> 1) & 0x55555555 | (n << 1) & 0xaaaaaaaa;
    n = (n >> 2) & 0x33333333 | (n << 2) & 0xcccccccc;
    n = (n >> 4) & 0x0f0f0f0f | (n << 4) & 0xf0f0f0f0;
    n = (n >> 8) & 0x00ff00ff | (n << 8) & 0xff00ff00;
    n = (n >> 16) & 0x0000ffff | (n << 16) & 0xffff0000;
    return n;
}

uint dma_accel_crc32(uint8_t * buf, uint size, uint seed, uint algo, bool byte_swap) {

    uint transfer_count = size / sizeof(uint); // Anzahl 32 Bit Transfers

    // Sollte ab Werk thread-sicher sein
    uint channel = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(channel);

    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_sniff_enable(&c, true);

    dma_sniffer_enable(channel, algo, true); 
    dma_sniffer_set_byte_swap_enabled(byte_swap);

    dma_hw->sniff_data = seed;

    dma_channel_configure(channel, &c,
        &(pio0_hw->txf[3]),        // Destination pointer, FIXME
        buf,                    // Source pointer
        transfer_count,                   // Number of transfers
        true                    // Start immediately
    );

    dma_channel_wait_for_finish_blocking(channel);
    uint digest = dma_hw->sniff_data;

    dma_channel_unclaim(channel);

    return digest;
}