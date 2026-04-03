cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH $ENV{HOME}/Development/privat/pico-sdk)
set(PICO_BOARD pga2350)
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/../board)
set(PICO_PLATFORM rp2350-arm-s)

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(psram_test C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(psram_test
    main.c
    ${CMAKE_CURRENT_LIST_DIR}/../psram.c
)

target_include_directories(psram_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)

target_link_libraries(psram_test
    pico_stdlib
    hardware_clocks
    hardware_sync
    hardware_dma
    hardware_xip_cache
)

pico_enable_stdio_usb(psram_test 1)
pico_enable_stdio_uart(psram_test 0)

pico_add_extra_outputs(psram_test)
