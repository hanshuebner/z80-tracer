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

# Generate PIO headers
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

# --- Test firmware: raw console output ---
add_executable(test_raw test_raw.c)
pico_generate_pio_header(test_raw ${CMAKE_CURRENT_LIST_DIR}/z80_trace.pio)
target_include_directories(test_raw PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(test_raw
    pico_stdlib
    hardware_pio
    hardware_gpio
)
pico_enable_stdio_usb(test_raw 1)
pico_enable_stdio_uart(test_raw 0)
pico_add_extra_outputs(test_raw)

# --- Test firmware: /WAIT toggle ---
add_executable(test_wait test_wait.c)
target_include_directories(test_wait PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(test_wait
    pico_stdlib
    hardware_gpio
)
pico_enable_stdio_usb(test_wait 1)
pico_enable_stdio_uart(test_wait 0)
pico_add_extra_outputs(test_wait)
