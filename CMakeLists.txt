cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH $ENV{HOME}/Development/privat/pico-sdk)
set(PICO_BOARD pga2350)
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/board)
set(PICO_PLATFORM rp2350-arm-s)

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(z80_tracer C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(z80_tracer main.c)

pico_generate_pio_header(z80_tracer ${CMAKE_CURRENT_LIST_DIR}/z80_trace.pio)

target_include_directories(z80_tracer PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(z80_tracer
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_gpio
)

# Enable stdio USB so pico-sdk configures TinyUSB CDC for us
pico_enable_stdio_usb(z80_tracer 1)
pico_enable_stdio_uart(z80_tracer 0)

pico_add_extra_outputs(z80_tracer)
