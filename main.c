#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "tusb.h"

#include "z80_trace.h"
#include "z80_trace.pio.h"

// Ring buffer shared between DMA (writer) and main loop (reader)
static uint32_t __attribute__((aligned(CAPTURE_BUF_SIZE_BYTES)))
    read_buf[CAPTURE_BUF_SIZE_WORDS];
static uint32_t __attribute__((aligned(CAPTURE_BUF_SIZE_BYTES)))
    write_buf[CAPTURE_BUF_SIZE_WORDS];

static uint32_t read_buf_tail = 0;
static uint32_t write_buf_tail = 0;

// Flow control state
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
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 1);
}

// ---- PIO setup ----

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

// ---- DMA setup ----

static int dma_chan_read;
static int dma_chan_write;

static void dma_setup(void) {
    dma_chan_read = dma_claim_unused_channel(true);
    dma_channel_config c_rd = dma_channel_get_default_config(dma_chan_read);
    channel_config_set_transfer_data_size(&c_rd, DMA_SIZE_32);
    channel_config_set_read_increment(&c_rd, false);
    channel_config_set_write_increment(&c_rd, true);
    channel_config_set_ring(&c_rd, true, CAPTURE_BUF_RING_BITS);
    channel_config_set_dreq(&c_rd, pio_get_dreq(pio, sm_read, false));
    dma_channel_configure(dma_chan_read, &c_rd,
        read_buf, &pio->rxf[sm_read], 0xFFFFFFFF, false);

    dma_chan_write = dma_claim_unused_channel(true);
    dma_channel_config c_wr = dma_channel_get_default_config(dma_chan_write);
    channel_config_set_transfer_data_size(&c_wr, DMA_SIZE_32);
    channel_config_set_read_increment(&c_wr, false);
    channel_config_set_write_increment(&c_wr, true);
    channel_config_set_ring(&c_wr, true, CAPTURE_BUF_RING_BITS);
    channel_config_set_dreq(&c_wr, pio_get_dreq(pio, sm_write, false));
    dma_channel_configure(dma_chan_write, &c_wr,
        write_buf, &pio->rxf[sm_write], 0xFFFFFFFF, false);
}

// ---- Helpers ----

static inline uint32_t dma_write_index(int chan, uint32_t *buf) {
    uint32_t write_addr = dma_channel_hw_addr(chan)->write_addr;
    return (write_addr - (uint32_t)(uintptr_t)buf) / sizeof(uint32_t);
}

static inline uint32_t ring_available(uint32_t head, uint32_t tail) {
    return (head - tail) & (CAPTURE_BUF_SIZE_WORDS - 1);
}

static void flow_control_update(uint32_t total_pending) {
    if (!wait_asserted && total_pending >= FLOW_CTRL_ASSERT_THRESHOLD) {
        gpio_put(PIN_WAIT, 0);
        wait_asserted = true;
    } else if (wait_asserted && total_pending <= FLOW_CTRL_RELEASE_THRESHOLD) {
        gpio_put(PIN_WAIT, 1);
        wait_asserted = false;
    }
}

// ---- USB output ----
// Uses TinyUSB CDC directly — all on core 0, no thread safety issues.

// Try to write one sample. Returns false if USB has no room.
static bool emit_sample(uint32_t sample) {
    cycle_type_t ct = classify_sample(sample);
    if (ct == CYCLE_UNKNOWN) return true;

    if (tud_cdc_write_available() < USB_PACKET_SIZE)
        return false;

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

// ---- Entry point ----

int main(void) {
    // Init stdio_usb so pico-sdk configures TinyUSB CDC.
    // We call tud_* directly for data output but need the stack initialized.
    stdio_init_all();

    // Release /WAIT immediately so Z80 runs during USB enumeration
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_OUT);
    gpio_put(PIN_WAIT, 1);

    // Wait for USB connection
    while (!tud_cdc_connected())
        tud_task();

    gpio_setup();
    pio_setup();
    dma_setup();

    dma_channel_start(dma_chan_read);
    dma_channel_start(dma_chan_write);
    pio_sm_set_enabled(pio, sm_read, true);
    pio_sm_set_enabled(pio, sm_write, true);

    while (true) {
        tud_task();  // service USB on this core

        // Flow control: check fill level before draining
        uint32_t rd_pending = ring_available(
            dma_write_index(dma_chan_read, read_buf), read_buf_tail);
        uint32_t wr_pending = ring_available(
            dma_write_index(dma_chan_write, write_buf), write_buf_tail);
        flow_control_update(rd_pending + wr_pending);

        // Drain read buffer — stop when USB is full
        uint32_t rd_head = dma_write_index(dma_chan_read, read_buf);
        while (read_buf_tail != rd_head) {
            if (!emit_sample(read_buf[read_buf_tail]))
                break;
            read_buf_tail = (read_buf_tail + 1) & (CAPTURE_BUF_SIZE_WORDS - 1);
        }

        // Drain write buffer — stop when USB is full
        uint32_t wr_head = dma_write_index(dma_chan_write, write_buf);
        while (write_buf_tail != wr_head) {
            if (!emit_sample(write_buf[write_buf_tail]))
                break;
            write_buf_tail = (write_buf_tail + 1) & (CAPTURE_BUF_SIZE_WORDS - 1);
        }

        tud_cdc_write_flush();
    }
}
