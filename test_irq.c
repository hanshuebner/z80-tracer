#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#include "z80_trace.h"
#include "z80_trace.pio.h"

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
    gpio_set_dir(PIN_WAIT, GPIO_IN);
    gpio_disable_pulls(PIN_WAIT);
}

static void decode_ctrl(uint32_t sample, char *buf, int buflen) {
    buf[0] = '\0';
    if (!(sample & SAMPLE_M1_BIT))   strncat(buf, "M1 ", buflen - 1);
    if (!(sample & SAMPLE_MREQ_BIT)) strncat(buf, "MREQ ", buflen - strlen(buf) - 1);
    if (!(sample & SAMPLE_IORQ_BIT)) strncat(buf, "IORQ ", buflen - strlen(buf) - 1);
    if (!(sample & SAMPLE_RD_BIT))   strncat(buf, "RD ", buflen - strlen(buf) - 1);
    if (!(sample & SAMPLE_WR_BIT))   strncat(buf, "WR ", buflen - strlen(buf) - 1);
    if (!(sample & SAMPLE_RFSH_BIT)) strncat(buf, "RFSH ", buflen - strlen(buf) - 1);
    if (sample & SAMPLE_CLK_BIT)     strncat(buf, "CLK ", buflen - strlen(buf) - 1);
}

static PIO pio = pio0;
static uint sm_read;
static uint sm_write;
static uint sm_intack;

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

    uint offset_ia = pio_add_program(pio, &z80_intack_capture_program);
    sm_intack = pio_claim_unused_sm(pio, true);
    pio_sm_config cfg_ia = z80_intack_capture_program_get_default_config(offset_ia);
    sm_config_set_in_pins(&cfg_ia, PIN_IN_BASE);
    sm_config_set_jmp_pin(&cfg_ia, PIN_M1);
    sm_config_set_in_shift(&cfg_ia, false, false, 31);
    pio_sm_init(pio, sm_intack, offset_ia, &cfg_ia);
}

int main(void) {
    stdio_init_all();
    while (!stdio_usb_connected())
        sleep_ms(100);
    sleep_ms(500);

    printf("\n=== IRQ/NMI detector ===\n");
    printf("Watching for:\n");
    printf("  INT_ACK: M1 + IORQ (no MREQ)\n");
    printf("  NMI:     FETCH at $0066\n");
    printf("  Also showing all non-FETCH cycles for context\n\n");

    gpio_setup();
    pio_setup();

    pio_sm_set_enabled(pio, sm_read, true);
    pio_sm_set_enabled(pio, sm_write, true);
    pio_sm_set_enabled(pio, sm_intack, true);

    char ctrl[64];
    uint32_t last_fetch_addr = 0;
    uint32_t count = 0;
    uint32_t intack_count = 0;

    while (true) {
        // Read SM captures
        while (!pio_sm_is_rx_fifo_empty(pio, sm_read)) {
            uint32_t sample = pio_sm_get(pio, sm_read);
            uint16_t addr = sample_addr(sample);
            uint8_t data = sample_data(sample);
            cycle_type_t ct = classify_sample(sample);
            decode_ctrl(sample, ctrl, sizeof(ctrl));

            if (ct == CYCLE_INT_ACK) {
                printf("*** INT_ACK  addr=%04X data=%02X  [%s] raw=%08lX\n", addr, data, ctrl, (unsigned long)sample);
            } else if (ct == CYCLE_OPCODE_FETCH) {
                // Check if IORQ is also active (shouldn't happen for normal fetch)
                int iorq_active = !(sample & SAMPLE_IORQ_BIT);
                if (iorq_active) {
                    printf("*** FETCH+IORQ  addr=%04X data=%02X  [%s] raw=%08lX\n", addr, data, ctrl, (unsigned long)sample);
                }
                if (addr == 0x0066) {
                    printf("*** NMI      FETCH at $0066 (prev fetch was $%04X)\n",
                           last_fetch_addr);
                }
                last_fetch_addr = addr;
            } else {
                // Non-fetch read cycle — show if it looks interesting
                // (IO reads might be INT_ACK that we're misclassifying)
                if (ct == CYCLE_IO_READ) {
                    // Check if M1 is active — this would be an INT ack
                    // that classify_sample is miscategorizing
                    int m1_active = !(sample & SAMPLE_M1_BIT);
                    if (m1_active) {
                        printf("*** HIDDEN INT_ACK  addr=%04X data=%02X  [%s] raw=%08lX\n",
                               addr, data, ctrl, (unsigned long)sample);
                    } else {
                        printf("    IO_READ  addr=%04X data=%02X  [%s]\n", addr, data, ctrl);
                    }
                }
            }
            count++;
        }

        // Write SM captures
        while (!pio_sm_is_rx_fifo_empty(pio, sm_write)) {
            uint32_t sample = pio_sm_get(pio, sm_write);
            uint16_t addr = sample_addr(sample);
            uint8_t data = sample_data(sample);
            cycle_type_t ct = classify_sample(sample);
            decode_ctrl(sample, ctrl, sizeof(ctrl));

            if (ct == CYCLE_MEM_WRITE) {
                // Stack writes near NMI would go to high memory
                // Show all writes for context
            }
            if (ct == CYCLE_IO_WRITE) {
                printf("    IO_WRITE addr=%04X data=%02X  [%s]\n", addr, data, ctrl);
            }
        }

        // Poll INT acknowledge SM
        while (!pio_sm_is_rx_fifo_empty(pio, sm_intack)) {
            uint32_t sample = pio_sm_get(pio, sm_intack);
            uint16_t addr = sample_addr(sample);
            uint8_t data = sample_data(sample);
            decode_ctrl(sample, ctrl, sizeof(ctrl));
            printf("*** INT_ACK(PIO) addr=%04X data=%02X  [%s] raw=%08lX\n",
                   addr, data, ctrl, (unsigned long)sample);
            intack_count++;
        }

        // Periodic heartbeat
        static uint32_t last_report = 0;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_report >= 5000) {
            printf("[%lu cycles captured, %lu intacks, last fetch=$%04X]\n",
                   (unsigned long)count, (unsigned long)intack_count, last_fetch_addr);
            last_report = now;
        }
    }
}
