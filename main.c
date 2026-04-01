#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "tusb.h"

#include "z80_trace.h"
#include "z80_trace.pio.h"

// Ring buffer: DMA writes PIO samples here, ARM reads and classifies
static uint32_t __attribute__((aligned(CAPTURE_BUF_SIZE_BYTES)))
    sample_buf[CAPTURE_BUF_SIZE_WORDS];
static uint32_t sample_tail = 0;

// Flow control
static bool wait_asserted = false;

// ---- GPIO setup ----

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
    gpio_put(PIN_WAIT, 0);
}

// ---- PIO setup ----

static PIO pio = pio0;
static uint sm;
static uint pio_offset;

static void pio_setup(void) {
    pio_offset = pio_add_program(pio, &z80_bus_sample_program);
    sm = pio_claim_unused_sm(pio, true);
    pio_sm_config cfg = z80_bus_sample_program_get_default_config(pio_offset);
    sm_config_set_in_pins(&cfg, PIN_IN_BASE);
    sm_config_set_in_shift(&cfg, false, false, 31);
    pio_sm_init(pio, sm, pio_offset, &cfg);
}

// ---- DMA setup ----

static int dma_chan;

static void dma_setup(void) {
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, CAPTURE_BUF_RING_BITS);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
    dma_channel_configure(dma_chan, &c,
        sample_buf, &pio->rxf[sm], 0xFFFFFFFF, false);
}

// ---- Helpers ----

static inline uint32_t dma_write_index(void) {
    uint32_t write_addr = dma_channel_hw_addr(dma_chan)->write_addr;
    return (write_addr - (uint32_t)(uintptr_t)sample_buf) / sizeof(uint32_t);
}

static inline uint32_t ring_available(uint32_t head, uint32_t tail) {
    return (head - tail) & (CAPTURE_BUF_SIZE_WORDS - 1);
}

static void flow_control_update(uint32_t pending) {
    if (!wait_asserted && pending >= FLOW_CTRL_ASSERT_THRESHOLD) {
        gpio_set_dir(PIN_WAIT, GPIO_OUT);
        wait_asserted = true;
    } else if (wait_asserted && pending <= FLOW_CTRL_RELEASE_THRESHOLD) {
        gpio_set_dir(PIN_WAIT, GPIO_IN);
        wait_asserted = false;
    }
}

// ---- Bus cycle classifier (runs on ARM) ----
//
// Processes raw CLK-edge samples and detects bus cycle boundaries
// by watching for falling edges of control signals. Captures bus
// data one CLK cycle after the trigger (when data is valid).

static uint32_t prev_sample = 0xFFFFFFFF;  // all signals inactive

// Track active bus cycles waiting for full signal setup
static bool rd_active = false;   // /RD is low, waiting to classify
static bool wr_active = false;   // /WR is low, waiting to classify
static bool intack_active = false;

// Classifies bus cycles from CLK-edge samples.
//
// On Z80, /RD and /WR may go active before /MREQ settles (signal
// skew at CLK↓). So we track "in a read/write cycle" and classify
// on the first sample where ALL required signals are present.
// Each bus cycle is emitted exactly once.
static cycle_type_t process_sample(uint32_t sample) {
    uint32_t fell = prev_sample & ~sample;  // bits that went 1→0
    prev_sample = sample;

    int m1   = !(sample & SAMPLE_M1_BIT);
    int mreq = !(sample & SAMPLE_MREQ_BIT);
    int iorq = !(sample & SAMPLE_IORQ_BIT);
    int rd   = !(sample & SAMPLE_RD_BIT);
    int wr   = !(sample & SAMPLE_WR_BIT);
    int rfsh = !(sample & SAMPLE_RFSH_BIT);

    // Reset tracking when signals go inactive
    if (!rd)   rd_active = false;
    if (!wr)   wr_active = false;
    if (!iorq || !m1) intack_active = false;

    // INT acknowledge: /IORQ active with /M1 active, not yet classified
    if (m1 && iorq && !intack_active) {
        intack_active = true;
        return CYCLE_INT_ACK;
    }

    // Read cycle: /RD active, classify when /MREQ or /IORQ also active
    // Suppress during INT acknowledge (intack_active) to avoid
    // misclassifying the ack's /RD as an OPCODE_FETCH.
    if (rd && !rd_active && !rfsh && !intack_active) {
        if (mreq || iorq) {
            rd_active = true;  // classified — don't repeat
            if (m1 && mreq)  return CYCLE_OPCODE_FETCH;
            if (mreq)        return CYCLE_MEM_READ;
            if (iorq)        return CYCLE_IO_READ;
        }
        // /RD active but /MREQ and /IORQ not yet — will retry next sample
    }

    // Write cycle: /WR active, classify when /MREQ or /IORQ also active
    if (wr && !wr_active && !intack_active) {
        if (mreq || iorq) {
            wr_active = true;
            if (mreq)        return CYCLE_MEM_WRITE;
            if (iorq)        return CYCLE_IO_WRITE;
        }
    }

    return CYCLE_UNKNOWN;
}

static void reset_classifier(void) {
    prev_sample = 0xFFFFFFFF;
    rd_active = false;
    wr_active = false;
    intack_active = false;
}

// ---- USB output ----

static bool emit_cycle(cycle_type_t ct, uint32_t sample) {
    if (ct == CYCLE_UNKNOWN) return true;
    if (tud_cdc_write_available() < USB_PACKET_SIZE) return false;

    uint16_t addr = sample_addr(sample);
    uint8_t data = sample_data(sample);
    uint8_t pkt[USB_PACKET_SIZE];
    pkt[0] = 0x80 | ((uint8_t)ct << 3) | ((addr >> 13) & 0x07);
    pkt[1] = (addr >> 6) & 0x7F;
    pkt[2] = ((addr & 0x3F) << 1) | ((data >> 7) & 0x01);
    pkt[3] = data & 0x7F;

    tud_cdc_write(pkt, USB_PACKET_SIZE);
    return true;
}

// ---- USB text I/O ----

static void usb_puts(const char *s) {
    tud_cdc_write_str(s);
    tud_cdc_write_flush();
}

// ---- Capture start/stop ----

static void capture_start(void) {
    sample_tail = 0;
    reset_classifier();

    dma_channel_set_write_addr(dma_chan, sample_buf, false);
    dma_channel_set_trans_count(dma_chan, 0xFFFFFFFF, false);

    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_jmp(pio_offset));

    dma_channel_start(dma_chan);
    pio_sm_set_enabled(pio, sm, true);
}

static void capture_stop(void) {
    pio_sm_set_enabled(pio, sm, false);
    dma_channel_abort(dma_chan);

    if (wait_asserted) {
        gpio_set_dir(PIN_WAIT, GPIO_IN);
        wait_asserted = false;
    }
}

// ---- Entry point ----

int main(void) {
    stdio_init_all();

    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_IN);
    gpio_disable_pulls(PIN_WAIT);

    gpio_setup();
    pio_setup();
    dma_setup();

    char cmd_buf[32];
    int cmd_len = 0;
    bool tracing = false;

    while (true) {
        tud_task();

        if (!tud_cdc_connected()) {
            if (tracing) {
                capture_stop();
                tracing = false;
            }
            cmd_len = 0;
            continue;
        }

        if (tracing) {
            if (tud_cdc_available()) {
                uint8_t dummy[64];
                tud_cdc_read(dummy, sizeof(dummy));
                capture_stop();
                tracing = false;
                tud_cdc_write_flush();
                usb_puts("stopped\r\n");
                continue;
            }

            // Flow control
            uint32_t head = dma_write_index();
            uint32_t pending = ring_available(head, sample_tail);
            flow_control_update(pending);

            // Process samples from ring buffer
            while (sample_tail != head) {
                uint32_t sample = sample_buf[sample_tail];
                cycle_type_t ct = process_sample(sample);

                if (ct != CYCLE_UNKNOWN) {
                    if (!emit_cycle(ct, sample))
                        break;  // USB full — back-pressure
                }

                sample_tail = (sample_tail + 1) & (CAPTURE_BUF_SIZE_WORDS - 1);
            }

            tud_cdc_write_flush();

        } else {
            while (tud_cdc_available()) {
                uint8_t ch;
                tud_cdc_read(&ch, 1);
                if (ch == '\r' || ch == '\n') {
                    if (cmd_len > 0) {
                        cmd_buf[cmd_len] = '\0';
                        if (strcmp(cmd_buf, "trace") == 0) {
                            usb_puts("tracing\r\n");
                            capture_start();
                            tracing = true;
                        } else {
                            usb_puts("error: unknown command\r\n");
                        }
                        cmd_len = 0;
                    }
                } else if (cmd_len < (int)sizeof(cmd_buf) - 1) {
                    cmd_buf[cmd_len++] = ch;
                }
            }
        }
    }
}
