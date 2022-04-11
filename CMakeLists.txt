cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(ft8_xcvr C CXX ASM)
project(ft8_xcvr VERSION 0.1.0)
#set(CMAKE_C_STANDARD 11)
#set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_executable(run_ft8 run_ft8.cpp)

# Overclocking hacks
pico_define_boot_stage2(slower_boot2 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
target_compile_definitions(slower_boot2 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
pico_set_boot_stage2(run_ft8 slower_boot2)

add_subdirectory(ft8)
add_subdirectory(fft)
add_subdirectory(util)
add_subdirectory(peripheral_util)
add_subdirectory(../pico-arduino-compat/libs/tinygpsplus build_pac-tinygpsplus)
add_subdirectory(../pico-arduino-compat/libs/time build_pac-time)

#target_include_directories(run_ft8 PUBLIC ft8 fft util)

#target_link_directories(run_ft8 PRIVATE ft8 fft util)

#target_link_libraries(run_ft8 ft8 fft util)

target_link_libraries(run_ft8 ft8 fft util pico_si5351 hardware_adc hardware_dma hardware_rtc pac-tinygpsplus pac-time pico_multicore pico_stdlib)

pico_enable_stdio_usb(run_ft8 1)
pico_enable_stdio_uart(run_ft8 0)
pico_add_extra_outputs(run_ft8)
