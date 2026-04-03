cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH $ENV{HOME}/Development/privat/pico-sdk)
set(PICO_BOARD pga2350)
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/../board)
set(PICO_PLATFORM rp2350-arm-s)

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(usb_bench C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(usb_bench main.c)

target_link_libraries(usb_bench
    pico_stdlib
    hardware_clocks
)

# USB CDC via pico-sdk stdio, same as the tracer
pico_enable_stdio_usb(usb_bench 1)
pico_enable_stdio_uart(usb_bench 0)

pico_add_extra_outputs(usb_bench)
