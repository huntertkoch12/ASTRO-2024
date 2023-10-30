cmake_minimum_required(VERSION 3.12)

include ($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(ASTRO-PICO-SDK CXX C ASM)

pico_sdk_init()
add_executable(main
  main.c
)
pico_enable_stdio_uart(main 1)
pico_add_extra_outputs(main)
target_link_libraries(main pico_stdlib)
