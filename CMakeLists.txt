cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(rp2040-rmii)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_executable(main
        src/main.c
        src/crc32.c
        )

# PIO kompilieren
pico_generate_pio_header(main ${CMAKE_CURRENT_LIST_DIR}/src/eth.pio)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(main pico_stdlib hardware_pio hardware_dma hardware_irq)

# enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(main)
