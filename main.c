#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

#include "z80_trace.h"
#include "z80_trace.pio.h"

// Ring buffer shared between DMA (writer) and core 1 (reader)
static uint32_t __attribute__((aligned(CAPTURE_BUF_SIZE_BYTES)))
    read_buf[CAPTURE_BUF_SIZE_WORDS];
static uint32_t __attribute__((aligned(CAPTURE_BUF_SIZE_BYTES)))
    write_buf[CAPTURE_BUF_SIZE_WORDS];

// Software read pointers (DMA write pointer is inferred from DMA read address)
static volatile uint32_t read_buf_tail = 0;
static volatile uint32_t write_buf_tail = 0;

// Flow control state
static volatile bool wait_asserted = false;

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

// ---- DMA setup ----
// Uses ring-mode DMA: write address wraps within the buffer automatically.

static int dma_chan_read;
static int dma_chan_write;

static void dma_setup(void) {
    // DMA channel for read captures (PIO SM RX FIFO -> read_buf)
    dma_chan_read = dma_claim_unused_channel(true);
    dma_channel_config c_rd = dma_channel_get_default_config(dma_chan_read);
    channel_config_set_transfer_data_size(&c_rd, DMA_SIZE_32);
    channel_config_set_read_increment(&c_rd, false);   // read from fixed FIFO addr
    channel_config_set_write_increment(&c_rd, true);    // write pointer advances
    channel_config_set_ring(&c_rd, true, 17);           // wrap write addr at 128KB (2^17)
    channel_config_set_dreq(&c_rd, pio_get_dreq(pio, sm_read, false));
    dma_channel_configure(dma_chan_read, &c_rd,
        read_buf,                               // write address
        &pio->rxf[sm_read],                     // read address (PIO RX FIFO)
        0xFFFFFFFF,                             // transfer count (run forever)
        false);                                 // don't start yet

    // DMA channel for write captures
    dma_chan_write = dma_claim_unused_channel(true);
    dma_channel_config c_wr = dma_channel_get_default_config(dma_chan_write);
    channel_config_set_transfer_data_size(&c_wr, DMA_SIZE_32);
    channel_config_set_read_increment(&c_wr, false);
    channel_config_set_write_increment(&c_wr, true);
    channel_config_set_ring(&c_wr, true, 17);
    channel_config_set_dreq(&c_wr, pio_get_dreq(pio, sm_write, false));
    dma_channel_configure(dma_chan_write, &c_wr,
        write_buf,
        &pio->rxf[sm_write],
        0xFFFFFFFF,
        false);
}

// Get DMA write pointer as an index into the ring buffer
static inline uint32_t dma_write_index(int chan, uint32_t *buf) {
    uint32_t write_addr = dma_channel_hw_addr(chan)->write_addr;
    return (write_addr - (uint32_t)(uintptr_t)buf) / sizeof(uint32_t);
}

// Number of entries available to read in a ring buffer
static inline uint32_t ring_available(uint32_t head, uint32_t tail) {
    return (head - tail) & (CAPTURE_BUF_SIZE_WORDS - 1);
}

// ---- Flow control ----

static void flow_control_update(uint32_t total_pending) {
    if (!wait_asserted && total_pending >= FLOW_CTRL_ASSERT_THRESHOLD) {
        gpio_put(PIN_WAIT, 0);  // assert /WAIT - pause Z80
        wait_asserted = true;
    } else if (wait_asserted && total_pending <= FLOW_CTRL_RELEASE_THRESHOLD) {
        gpio_put(PIN_WAIT, 1);  // release /WAIT - resume Z80
        wait_asserted = false;
    }
}

// ---- USB output (runs on core 1) ----
// Uses pico-sdk stdio_usb (CDC).  Output is raw binary: 4-byte packets
// sent via putchar_raw() which bypasses newline translation.

static void emit_sample(uint32_t sample) {
    cycle_type_t ct = classify_sample(sample);
    if (ct == CYCLE_UNKNOWN) return;

    uint8_t pkt[USB_PACKET_SIZE];
    pkt[0] = (uint8_t)ct;
    pkt[1] = sample_addr(sample) & 0xFF;
    pkt[2] = sample_addr(sample) >> 8;
    pkt[3] = sample_data(sample);

    for (int i = 0; i < USB_PACKET_SIZE; i++) {
        putchar_raw(pkt[i]);
    }
}

static void core1_main(void) {
    while (true) {
        // Drain read capture buffer
        uint32_t rd_head = dma_write_index(dma_chan_read, read_buf);
        while (read_buf_tail != rd_head) {
            emit_sample(read_buf[read_buf_tail]);
            read_buf_tail = (read_buf_tail + 1) & (CAPTURE_BUF_SIZE_WORDS - 1);
        }

        // Drain write capture buffer
        uint32_t wr_head = dma_write_index(dma_chan_write, write_buf);
        while (write_buf_tail != wr_head) {
            emit_sample(write_buf[write_buf_tail]);
            write_buf_tail = (write_buf_tail + 1) & (CAPTURE_BUF_SIZE_WORDS - 1);
        }

        // Flush USB
        stdio_flush();

        // Update flow control based on total pending data
        uint32_t total_pending = ring_available(rd_head, read_buf_tail)
                               + ring_available(wr_head, write_buf_tail);
        flow_control_update(total_pending);
    }
}

// ---- Entry point ----

int main(void) {
    stdio_init_all();

    gpio_setup();
    pio_setup();
    dma_setup();

    // Start core 1 (USB drain loop)
    multicore_launch_core1(core1_main);

    // Start DMA channels
    dma_channel_start(dma_chan_read);
    dma_channel_start(dma_chan_write);

    // Start PIO state machines
    pio_sm_set_enabled(pio, sm_read, true);
    pio_sm_set_enabled(pio, sm_write, true);

    // Core 0: idle (stdio_usb services USB in the background)
    while (true) {
        tight_loop_contents();
    }
}
