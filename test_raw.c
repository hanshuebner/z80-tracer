#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#include "z80_trace.h"
#include "z80_trace.pio.h"

static const char *cycle_name(cycle_type_t ct) {
    switch (ct) {
        case CYCLE_OPCODE_FETCH: return "FETCH";
        case CYCLE_MEM_READ:     return "MREAD";
        case CYCLE_MEM_WRITE:    return "MWRIT";
        case CYCLE_IO_READ:      return "IORD ";
        case CYCLE_IO_WRITE:     return "IOWR ";
        default:                 return "???? ";
    }
}

static void decode_ctrl(uint32_t sample, char *buf, int buflen) {
    // Show signal names when ACTIVE (active-low: bit=0 means asserted)
    buf[0] = '\0';
    if (!(sample & SAMPLE_M1_BIT))   strncat(buf, "M1 ", buflen - strlen(buf) - 1);
    if (!(sample & SAMPLE_MREQ_BIT)) strncat(buf, "MREQ ", buflen - strlen(buf) - 1);
    if (!(sample & SAMPLE_IORQ_BIT)) strncat(buf, "IORQ ", buflen - strlen(buf) - 1);
    if (!(sample & SAMPLE_RD_BIT))   strncat(buf, "RD ", buflen - strlen(buf) - 1);
    if (!(sample & SAMPLE_WR_BIT))   strncat(buf, "WR ", buflen - strlen(buf) - 1);
    if (!(sample & SAMPLE_RFSH_BIT)) strncat(buf, "RFSH ", buflen - strlen(buf) - 1);
    if (sample & SAMPLE_CLK_BIT)     strncat(buf, "CLK ", buflen - strlen(buf) - 1);
}

static void gpio_setup(void) {
    for (int i = PIN_ADDR_BASE; i < PIN_ADDR_BASE + 16; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_disable_pulls(i);
    }
    for (int i = PIN_DATA_BASE; i < PIN_DATA_BASE + 8; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_disable_pulls(i);
    }
    int ctrl_pins[] = { PIN_M1, PIN_MREQ, PIN_IORQ, PIN_RD, PIN_WR,
                        PIN_RFSH, PIN_CLK, PIN_HALT,
                        PIN_INT, PIN_NMI, PIN_RESET };
    for (int i = 0; i < (int)(sizeof(ctrl_pins)/sizeof(ctrl_pins[0])); i++) {
        gpio_init(ctrl_pins[i]);
        gpio_set_dir(ctrl_pins[i], GPIO_IN);
        gpio_disable_pulls(ctrl_pins[i]);
    }
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 1);
}

static PIO pio = pio0;
static uint sm_read;
static uint sm_write;

static void pio_setup(void) {
    uint offset_rd = pio_add_program(pio, &z80_read_capture_program);
    sm_read = pio_claim_unused_sm(pio, true);
    pio_sm_config cfg_rd = z80_read_capture_program_get_default_config(offset_rd);
    sm_config_set_in_pins(&cfg_rd, PIN_IN_BASE);
    sm_config_set_jmp_pin(&cfg_rd, PIN_RFSH);
    sm_config_set_in_shift(&cfg_rd, false, false, 31);
    pio_sm_init(pio, sm_read, offset_rd, &cfg_rd);

    uint offset_wr = pio_add_program(pio, &z80_write_capture_program);
    sm_write = pio_claim_unused_sm(pio, true);
    pio_sm_config cfg_wr = z80_write_capture_program_get_default_config(offset_wr);
    sm_config_set_in_pins(&cfg_wr, PIN_IN_BASE);
    sm_config_set_in_shift(&cfg_wr, false, false, 31);
    pio_sm_init(pio, sm_write, offset_wr, &cfg_wr);
}

int main(void) {
    stdio_init_all();

    // Wait for USB serial connection
    while (!stdio_usb_connected())
        sleep_ms(100);
    sleep_ms(500);

    printf("\n=== Z80 Bus Tracer - Raw Test ===\n");

    gpio_setup();
    pio_setup();

    // Print initial GPIO state before starting PIO
    uint32_t all_pins = gpio_get_all();
    printf("GPIO snapshot: %08lX\n", (unsigned long)all_pins);
    printf("  Addr: %04X  Data: %02X\n",
           (unsigned)(all_pins & 0xFFFF),
           (unsigned)((all_pins >> 16) & 0xFF));
    printf("  /M1=%d /MREQ=%d /IORQ=%d /RD=%d /WR=%d /RFSH=%d CLK=%d /HALT=%d\n",
           (all_pins >> PIN_M1) & 1,
           (all_pins >> PIN_MREQ) & 1,
           (all_pins >> PIN_IORQ) & 1,
           (all_pins >> PIN_RD) & 1,
           (all_pins >> PIN_WR) & 1,
           (all_pins >> PIN_RFSH) & 1,
           (all_pins >> PIN_CLK) & 1,
           (all_pins >> PIN_HALT) & 1);
    printf("\nStarting PIO capture...\n");
    printf("%-6s %-6s %s\n", "TYPE", "ADDR", "DATA");
    printf("------ ------ ----\n");

    pio_sm_set_enabled(pio, sm_read, true);
    pio_sm_set_enabled(pio, sm_write, true);

    uint32_t count = 0;
    while (true) {
        char ctrl[64];

        // Poll read SM FIFO
        while (!pio_sm_is_rx_fifo_empty(pio, sm_read)) {
            uint32_t sample = pio_sm_get(pio, sm_read);
            cycle_type_t ct = classify_sample(sample);
            decode_ctrl(sample, ctrl, sizeof(ctrl));
            printf("RD %s  %04X   %02X  [%08lX]  %s\n",
                   cycle_name(ct),
                   sample_addr(sample),
                   sample_data(sample),
                   (unsigned long)sample,
                   ctrl);
            count++;
        }

        // Poll write SM FIFO
        while (!pio_sm_is_rx_fifo_empty(pio, sm_write)) {
            uint32_t sample = pio_sm_get(pio, sm_write);
            cycle_type_t ct = classify_sample(sample);
            decode_ctrl(sample, ctrl, sizeof(ctrl));
            printf("WR %s  %04X   %02X  [%08lX]  %s\n",
                   cycle_name(ct),
                   sample_addr(sample),
                   sample_data(sample),
                   (unsigned long)sample,
                   ctrl);
            count++;
        }

        // Periodic status if nothing captured
        if (count == 0) {
            static uint32_t last_status = 0;
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if (now - last_status >= 2000) {
                all_pins = gpio_get_all();
                printf("[no captures] GPIO: %08lX  /RD=%d /WR=%d /MREQ=%d CLK=%d /RFSH=%d\n",
                       (unsigned long)all_pins,
                       (all_pins >> PIN_RD) & 1,
                       (all_pins >> PIN_WR) & 1,
                       (all_pins >> PIN_MREQ) & 1,
                       (all_pins >> PIN_CLK) & 1,
                       (all_pins >> PIN_RFSH) & 1);
                last_status = now;
            }
        }
    }
}
