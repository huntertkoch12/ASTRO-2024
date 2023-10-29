# CMakeLists file for ASTRO-2024 test.c source file
cmake_minimum_required(VERSION 3.13)

include(pico-sdk/pico_sdk_init.cmake)

project(ASTRO2024 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(test
    test.c
)

# Enable USB for serial monitor stdio (1) but disable UART stdio (will be used for GPS TX/RX) for Target File
pico_enable_stdio_usb(test 1)
pico_enable_stdio_uart(test 0) 

# Create map/bin/hex file etc. for disassembly
pico_add_extra_outputs(test)

# Tells linker which libraries to include after compilation (pulls in common dependencies)
target_link_libraries(test pico_stdlib)

