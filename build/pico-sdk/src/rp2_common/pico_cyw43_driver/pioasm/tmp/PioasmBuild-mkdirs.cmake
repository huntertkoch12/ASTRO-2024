# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/daniel/_UniversityofNotreDame/Clubs/NDRT/Payload_2024/ASTRO-2024/pico-sdk/tools/pioasm"
  "/Users/daniel/_UniversityofNotreDame/Clubs/NDRT/Payload_2024/ASTRO-2024/build/pioasm"
  "/Users/daniel/_UniversityofNotreDame/Clubs/NDRT/Payload_2024/ASTRO-2024/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm"
  "/Users/daniel/_UniversityofNotreDame/Clubs/NDRT/Payload_2024/ASTRO-2024/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/tmp"
  "/Users/daniel/_UniversityofNotreDame/Clubs/NDRT/Payload_2024/ASTRO-2024/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
  "/Users/daniel/_UniversityofNotreDame/Clubs/NDRT/Payload_2024/ASTRO-2024/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src"
  "/Users/daniel/_UniversityofNotreDame/Clubs/NDRT/Payload_2024/ASTRO-2024/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/daniel/_UniversityofNotreDame/Clubs/NDRT/Payload_2024/ASTRO-2024/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/daniel/_UniversityofNotreDame/Clubs/NDRT/Payload_2024/ASTRO-2024/build/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/PioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
