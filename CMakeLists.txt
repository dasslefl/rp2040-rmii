cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(rp2040-rmii)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# LWIP Library

set(LWIP_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/lwip)

add_library(pico_lwip INTERFACE)

target_sources(pico_lwip INTERFACE
    ${LWIP_PATH}/src/core/altcp.c
    ${LWIP_PATH}/src/core/altcp_alloc.c
    ${LWIP_PATH}/src/core/altcp_tcp.c
    ${LWIP_PATH}/src/core/def.c
    ${LWIP_PATH}/src/core/dns.c
    ${LWIP_PATH}/src/core/inet_chksum.c
    ${LWIP_PATH}/src/core/init.c
    ${LWIP_PATH}/src/core/ip.c
    ${LWIP_PATH}/src/core/mem.c
    ${LWIP_PATH}/src/core/memp.c
    ${LWIP_PATH}/src/core/netif.c
    ${LWIP_PATH}/src/core/pbuf.c
    ${LWIP_PATH}/src/core/raw.c
    ${LWIP_PATH}/src/core/stats.c
    ${LWIP_PATH}/src/core/sys.c
    ${LWIP_PATH}/src/core/tcp.c
    ${LWIP_PATH}/src/core/tcp_in.c
    ${LWIP_PATH}/src/core/tcp_out.c
    ${LWIP_PATH}/src/core/timeouts.c
    ${LWIP_PATH}/src/core/udp.c
    ${LWIP_PATH}/src/core/ipv4/autoip.c
    ${LWIP_PATH}/src/core/ipv4/dhcp.c
    ${LWIP_PATH}/src/core/ipv4/etharp.c
    ${LWIP_PATH}/src/core/ipv4/icmp.c
    ${LWIP_PATH}/src/core/ipv4/igmp.c
    ${LWIP_PATH}/src/core/ipv4/ip4.c
    ${LWIP_PATH}/src/core/ipv4/ip4_addr.c
    ${LWIP_PATH}/src/core/ipv4/ip4_frag.c
    ${LWIP_PATH}/src/netif/ethernet.c

    ${LWIP_PATH}/src/apps/http/httpd.c
    ${LWIP_PATH}/src/apps/http/fs.c

    ${CMAKE_CURRENT_LIST_DIR}/src/lwip/sys_arch.c
)

target_include_directories(pico_lwip INTERFACE
	${LWIP_PATH}/src/include
	${CMAKE_CURRENT_LIST_DIR}/src/lwip
)

# rest of your project
add_executable(main
        src/main.c
        src/crc32.c
        src/smi.c
        src/eth.c
        src/dma_accel.c
        src/eth_lwip.c
        )

# PIO kompilieren
pico_generate_pio_header(main ${CMAKE_CURRENT_LIST_DIR}/src/eth.pio)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(main hardware_pio hardware_dma pico_stdlib pico_unique_id pico_lwip hardware_irq)

# enable usb output, disable uart output
pico_enable_stdio_usb(main 0)
pico_enable_stdio_uart(main 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(main)
