#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#include "z80_trace.h"
#include "z80_trace.pio.h"

// ---- GPIO setup ----

static void gpio_setup(void) {
    // Address bus (A0-A15): inputs
    for (int i = PIN_ADDR_BASE; i < PIN_ADDR_BASE + 16; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_disable_pulls(i);
    }
    // Data bus (D0-D7): inputs
    for (int i = PIN_DATA_BASE; i < PIN_DATA_BASE + 8; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_disable_pulls(i);
    }
    // Control signals: inputs
    int ctrl_pins[] = { PIN_M1, PIN_MREQ, PIN_IORQ, PIN_RD, PIN_WR,
                        PIN_RFSH, PIN_CLK, PIN_HALT,
                        PIN_INT, PIN_NMI, PIN_RESET };
    for (int i = 0; i < (int)(sizeof(ctrl_pins)/sizeof(ctrl_pins[0])); i++) {
        gpio_init(ctrl_pins[i]);
        gpio_set_dir(ctrl_pins[i], GPIO_IN);
        gpio_disable_pulls(ctrl_pins[i]);
    }
    // /WAIT: output, active-low, start released (high)
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 1);
}

// ---- PIO setup ----

static PIO pio = pio0;
static uint sm_read;
static uint sm_write;

static void pio_setup(void) {
    // Load read capture program
    uint offset_rd = pio_add_program(pio, &z80_read_capture_program);
    sm_read = pio_claim_unused_sm(pio, true);

    pio_sm_config cfg_rd = z80_read_capture_program_get_default_config(offset_rd);
    sm_config_set_in_pins(&cfg_rd, PIN_IN_BASE);
    sm_config_set_jmp_pin(&cfg_rd, PIN_RFSH);  // jmp pin checks /RFSH
    sm_config_set_in_shift(&cfg_rd, false, false, 31); // shift left, no autopush
    pio_sm_init(pio, sm_read, offset_rd, &cfg_rd);

    // Load write capture program
    uint offset_wr = pio_add_program(pio, &z80_write_capture_program);
    sm_write = pio_claim_unused_sm(pio, true);

    pio_sm_config cfg_wr = z80_write_capture_program_get_default_config(offset_wr);
    sm_config_set_in_pins(&cfg_wr, PIN_IN_BASE);
    sm_config_set_in_shift(&cfg_wr, false, false, 31);
    pio_sm_init(pio, sm_write, offset_wr, &cfg_wr);
}

// ---- USB output ----
// Binary protocol: 4-byte packets with bit-7 sync framing.
// Sent via putchar_raw() which bypasses newline translation.

static void emit_sample(uint32_t sample) {
    cycle_type_t ct = classify_sample(sample);
    if (ct == CYCLE_UNKNOWN) return;

    uint16_t addr = sample_addr(sample);
    uint8_t data = sample_data(sample);
    uint8_t pkt[USB_PACKET_SIZE];
    pkt[0] = 0x80 | ((uint8_t)ct << 3) | ((addr >> 13) & 0x07);
    pkt[1] = (addr >> 6) & 0x7F;
    pkt[2] = ((addr & 0x3F) << 1) | ((data >> 7) & 0x01);
    pkt[3] = data & 0x7F;

    for (int i = 0; i < USB_PACKET_SIZE; i++) {
        putchar_raw(pkt[i]);
    }
}

// ---- Entry point ----

int main(void) {
    stdio_init_all();

    // Wait for USB serial connection before starting capture
    while (!stdio_usb_connected())
        sleep_ms(100);

    gpio_setup();
    pio_setup();

    // Start PIO state machines
    pio_sm_set_enabled(pio, sm_read, true);
    pio_sm_set_enabled(pio, sm_write, true);

    // Main loop: poll PIO FIFOs directly and emit binary packets
    while (true) {
        while (!pio_sm_is_rx_fifo_empty(pio, sm_read))
            emit_sample(pio_sm_get(pio, sm_read));

        while (!pio_sm_is_rx_fifo_empty(pio, sm_write))
            emit_sample(pio_sm_get(pio, sm_write));

        stdio_flush();
    }
}
