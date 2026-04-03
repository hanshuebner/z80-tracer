cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH $ENV{HOME}/Development/privat/pico-sdk)
set(PICO_BOARD pga2350)
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/../board)
set(PICO_PLATFORM rp2350-arm-s)

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(wait_test C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(wait_test main.c)

target_include_directories(wait_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)

target_link_libraries(wait_test
    pico_stdlib
    hardware_gpio
    hardware_clocks
)

pico_enable_stdio_usb(wait_test 1)
pico_enable_stdio_uart(wait_test 0)

pico_add_extra_outputs(wait_test)
