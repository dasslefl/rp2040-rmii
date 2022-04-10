cmake_minimum_required(VERSION 3.12)

project(rp2040_rmii_httpd)

# rest of your project
add_executable(rp2040_rmii_httpd
    main.c
)

target_link_libraries(rp2040_rmii_httpd pico_stdlib rp2040_rmii)

# enable usb output, disable uart output
pico_enable_stdio_usb(rp2040_rmii_httpd 1)
pico_enable_stdio_uart(rp2040_rmii_httpd 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(rp2040_rmii_httpd)
