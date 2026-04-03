#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include "tusb.h"

#include "z80_trace.h"
#include "z80_trace.pio.h"

// ---- DMA ring buffer: PIO samples land here ----

static uint32_t __attribute__((aligned(CAPTURE_BUF_SIZE_BYTES)))
    sample_buf[CAPTURE_BUF_SIZE_WORDS];

// ---- Trace record queue (core 1 → core 0) ----

static queue_t trace_queue;

// ---- PIO / DMA handles ----

static PIO pio = pio0;
static uint sm;
static uint pio_offset;
static int dma_chan;

// ---- Shared state between cores ----

static volatile bool needs_reset = false;  // core 0 signals core 1 to reset

// ---- GPIO setup ----

static void gpio_setup(void) {
    // Address bus (GPIO 0-15)
    for (int i = PIN_ADDR_BASE; i < PIN_ADDR_BASE + 16; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_disable_pulls(i);
    }
    // Data bus (GPIO 16-23)
    for (int i = PIN_DATA_BASE; i < PIN_DATA_BASE + 8; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_disable_pulls(i);
    }
    // Control signals
    int ctrl_pins[] = { PIN_M1, PIN_MREQ, PIN_IORQ, PIN_RD, PIN_WR,
                        PIN_RFSH, PIN_CLK, PIN_HALT,
                        PIN_INT, PIN_NMI, PIN_RESET };
    for (int i = 0; i < (int)(sizeof(ctrl_pins)/sizeof(ctrl_pins[0])); i++) {
        gpio_init(ctrl_pins[i]);
        gpio_set_dir(ctrl_pins[i], GPIO_IN);
        gpio_disable_pulls(ctrl_pins[i]);
    }
    // /WAIT: initially input (released), output value preset to 0 for open-drain
    gpio_init(PIN_WAIT);
    gpio_set_dir(PIN_WAIT, GPIO_IN);
    gpio_disable_pulls(PIN_WAIT);
    gpio_put(PIN_WAIT, 0);
}

// ---- PIO setup ----

static void pio_setup(void) {
    pio_offset = pio_add_program(pio, &z80_bus_sample_program);
    sm = pio_claim_unused_sm(pio, true);
    pio_sm_config cfg = z80_bus_sample_program_get_default_config(pio_offset);
    sm_config_set_in_pins(&cfg, PIN_IN_BASE);
    // 32-bit capture with autopush — no manual push in PIO program
    sm_config_set_in_shift(&cfg, false, true, 32);
    pio_sm_init(pio, sm, pio_offset, &cfg);
}

// ---- DMA setup ----

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

// ========================================================================
// Z80 Bus Analyzer State Machine — runs on core 1
// ========================================================================
//
// Optimized for throughput: each sample must be processed in ~19 ARM
// cycles at 150 MHz to keep up with a 4 MHz Z80 (8M samples/sec).
//
// Key optimizations:
//  - Lazy signal extraction: each state only reads the signals it needs
//  - Flow control (queue_get_level with spinlock) only runs when emitting
//    records (~1.3M/sec), not on every sample (8M/sec)
//  - /RESET checked only on rising edges (~4M/sec)
//  - gpio_hi_in read only in states that need /WAIT, /INT, /NMI
//
// Synchronization: in sync mode, IDLE only accepts M1 cycles (/M1 low
// at T1↑). Internal operation T-states are handled by re-interpreting
// the current rising edge as a new potential T1.

// ---- Analyzer states ----

typedef enum {
    // Top-level / between cycles
    ST_IDLE,
    ST_RESET,

    // T1 — cycle type not yet determined
    ST_T1_M1,           // T1↑ saw /M1 low, awaiting T1↓
    ST_T1_NONM1,        // T1↑ saw /M1 high, awaiting T1↓

    // M1 opcode fetch (§4.1)
    ST_M1_T2R,          // awaiting T2↑
    ST_M1_T2F,          // awaiting T2↓ — /WAIT check
    ST_M1_TWR,          // wait state ↑
    ST_M1_TWF,          // wait state ↓ — /WAIT check
    ST_M1_T3R,          // T3↑ — capture opcode
    ST_M1_T3F,          // T3↓ — capture refresh addr
    ST_M1_T4R,          // T4↑ — end of M1, sample interrupts

    // Memory read (§4.2)
    ST_MR_T2R,
    ST_MR_T2F,          // /WAIT check
    ST_MR_TWR,
    ST_MR_TWF,          // /WAIT check
    ST_MR_T3R,
    ST_MR_T3F,          // capture data

    // Memory write (§4.3)
    ST_MW_T2R,          // capture data at ↑
    ST_MW_T2F,          // /WAIT check
    ST_MW_TWR,
    ST_MW_TWF,          // /WAIT check
    ST_MW_T3R,
    ST_MW_T3F,          // done

    // I/O — waiting for T2↑ to determine read vs write (§4.4, §4.5)
    ST_IO_T2R,

    // I/O read (§4.4)
    ST_IR_T2F,          // after T2↑ confirmed IO read
    ST_IR_TWR,          // auto-wait or additional wait ↑
    ST_IR_TWF,          // auto-wait or additional wait ↓ — /WAIT check
    ST_IR_T3R,
    ST_IR_T3F,          // capture data

    // I/O write (§4.5)
    ST_IW_T2F,          // after T2↑ confirmed IO write
    ST_IW_TWR,
    ST_IW_TWF,          // /WAIT check
    ST_IW_T3R,
    ST_IW_T3F,

    // Interrupt acknowledge (§4.6)
    ST_IA_T2R,
    ST_IA_T2F,          // /IORQ goes low at T2↓
    ST_IA_TW1R,         // first auto-wait ↑
    ST_IA_TW1F,         // first auto-wait ↓
    ST_IA_TW2R,         // second auto-wait ↑
    ST_IA_TW2F,         // second auto-wait ↓ — /WAIT check
    ST_IA_TWR,          // additional wait ↑
    ST_IA_TWF,          // additional wait ↓ — /WAIT check
    ST_IA_T3R,
    ST_IA_T3F,          // capture vector
    ST_IA_T4R,          // refresh, emit record
} state_t;

// ---- Analyzer context ----

static struct {
    state_t  state;

    // Current cycle being assembled
    uint16_t address;
    uint8_t  data;
    uint8_t  refresh;
    uint8_t  wait_count;
    bool     halt;

    // Interrupt edge detection (sampled at M1/INTACK T4↑)
    bool     nmi_prev;
    bool     nmi_pending;
    bool     int_pending;

    // Pending output record (when queue is full)
    trace_record_t pending;
    bool     has_pending;

    // Synchronization: when true, IDLE only accepts M1 cycles
    bool     syncing;

    // Sequence counter for gap detection (7-bit, wraps)
    uint8_t  seq;
} az;

// ---- Flow control ----

static inline void wait_assert(void) {
    gpio_set_dir(PIN_WAIT, GPIO_OUT);  // drive low
}

static inline void wait_release(void) {
    gpio_set_dir(PIN_WAIT, GPIO_IN);   // release (open-drain)
}

#define HI_WAIT_BIT   (1u << (PIN_WAIT  - 32))
#define HI_INT_BIT    (1u << (PIN_INT   - 32))
#define HI_NMI_BIT    (1u << (PIN_NMI   - 32))
#define HI_RESET_BIT  (1u << (PIN_RESET - 32))

// ---- Enqueue a trace record with flow control ----

static void enqueue_record(trace_record_t *rec) {
    if (!queue_try_add(&trace_queue, rec)) {
        az.pending = *rec;
        az.has_pending = true;
        wait_assert();
    } else {
        uint32_t level = queue_get_level(&trace_queue);
        if (level >= FLOW_CTRL_ASSERT_THRESHOLD)
            wait_assert();
    }
}

// ---- Emit a completed trace record ----

static void emit_record(uint8_t cycle_type) {
    trace_record_t rec;
    rec.address    = az.address;
    rec.data       = az.data;
    rec.cycle_type = cycle_type;
    rec.refresh    = az.refresh;
    rec.wait_count = az.wait_count;
    rec.flags      = 0;
    rec.seq        = az.seq++ & 0x7F;
    if (az.halt)        rec.flags |= TRACE_FLAG_HALT;
    if (az.int_pending) rec.flags |= TRACE_FLAG_INT;
    if (az.nmi_pending) rec.flags |= TRACE_FLAG_NMI;

    enqueue_record(&rec);
    az.nmi_pending = false;
}

// ---- Emit a diagnostic status record ----
// subtype in byte 0 bits 2:0 (via addr[15:13]), seq in bytes 1-2, count in bytes 3-4.

static void emit_status(uint8_t subtype, uint16_t count) {
    trace_record_t rec;
    rec.cycle_type = CYCLE_STATUS;
    rec.data       = subtype;
    rec.address    = az.seq;  // current 14-bit seq snapshot
    rec.wait_count = count & 0x7F;         // count low 7 bits
    rec.refresh    = (count >> 7) & 0x7F;  // count high 7 bits
    rec.flags      = 0;
    rec.seq        = 0;
    enqueue_record(&rec);
}

// ---- Begin a new cycle at T1↑ ----

static inline void begin_cycle(uint32_t sample) {
    az.address    = sample_addr(sample);
    az.data       = 0;
    az.refresh    = 0;
    az.wait_count = 0;
    az.halt       = !(sample & SAMPLE_HALT_BIT);
}

// ---- Process one PIO sample ----
// Each state extracts only the signals it needs (lazy extraction).

static void process_sample(uint32_t sample) {

    // /RESET check on rising edges only (~4 cycles overhead, 4M/sec)
    if ((sample & SAMPLE_CLK_BIT) &&
        __builtin_expect(!(sio_hw->gpio_hi_in & HI_RESET_BIT), 0)) {
        az.state = ST_RESET;
        return;
    }

    switch (az.state) {

    // ================================================================
    // IDLE — waiting for T1 rising edge
    // ================================================================
    case ST_IDLE:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        if (!(sample & SAMPLE_M1_BIT)) {
            begin_cycle(sample);
            az.state = ST_T1_M1;
            az.syncing = false;
        } else if (!az.syncing) {
            begin_cycle(sample);
            az.state = ST_T1_NONM1;
        }
        break;

    // ================================================================
    // RESET
    // ================================================================
    case ST_RESET:
        // Wait for /RESET to release (checked on both edges for responsiveness)
        if (!(sio_hw->gpio_hi_in & HI_RESET_BIT)) return;  // still in reset
        // Exiting reset — flush any pending record first
        if (az.has_pending) {
            if (!queue_try_add(&trace_queue, &az.pending)) return;
            az.has_pending = false;
        }
        az.address = 0; az.data = 0; az.refresh = 0;
        az.wait_count = 0; az.halt = false;
        az.nmi_prev = !(sio_hw->gpio_hi_in & HI_NMI_BIT);
        az.nmi_pending = false;
        az.int_pending = false;
        emit_record(CYCLE_RESET);
        az.state = ST_IDLE;
        az.syncing = true;
        break;

    // ================================================================
    // T1 — cycle type discrimination at falling edge
    // ================================================================
    case ST_T1_M1:
        if (sample & SAMPLE_CLK_BIT) return;
        if (!(sample & SAMPLE_MREQ_BIT)) {
            az.state = ST_M1_T2R;
        } else {
            az.wait_count = 2;
            az.state = ST_IA_T2R;
        }
        break;

    case ST_T1_NONM1:
        if (sample & SAMPLE_CLK_BIT) return;
        if (!(sample & SAMPLE_MREQ_BIT)) {
            az.state = (!(sample & SAMPLE_RD_BIT)) ? ST_MR_T2R : ST_MW_T2R;
        } else {
            az.state = ST_IO_T2R;
        }
        break;

    // ================================================================
    // M1 OPCODE FETCH — §4.1
    // ================================================================
    case ST_M1_T2R:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.state = ST_M1_T2F;
        break;

    case ST_M1_T2F:
        if (sample & SAMPLE_CLK_BIT) return;
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
            az.state = ST_M1_TWR;
        } else {
            az.state = ST_M1_T3R;
        }
        break;

    case ST_M1_TWR:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.state = ST_M1_TWF;
        break;

    case ST_M1_TWF:
        if (sample & SAMPLE_CLK_BIT) return;
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
            az.state = ST_M1_TWR;
        } else {
            az.state = ST_M1_T3R;
        }
        break;

    case ST_M1_T3R:     // capture opcode
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.data = sample_data(sample);
        az.state = ST_M1_T3F;
        break;

    case ST_M1_T3F:     // capture refresh address
        if (sample & SAMPLE_CLK_BIT) return;
        az.refresh = sample_addr(sample) & 0x7F;
        az.state = ST_M1_T4R;
        break;

    case ST_M1_T4R: {   // sample interrupts, emit
        if (!(sample & SAMPLE_CLK_BIT)) return;
        uint32_t hi = sio_hw->gpio_hi_in;
        bool nmi = !(hi & HI_NMI_BIT);
        if (nmi && !az.nmi_prev) az.nmi_pending = true;
        az.nmi_prev = nmi;
        az.int_pending = !(hi & HI_INT_BIT);
        emit_record(CYCLE_M1_FETCH);
        az.state = ST_IDLE;
        break;
    }

    // ================================================================
    // MEMORY READ — §4.2
    // ================================================================
    case ST_MR_T2R:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.state = ST_MR_T2F;
        break;

    case ST_MR_T2F:
        if (sample & SAMPLE_CLK_BIT) return;
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
            az.state = ST_MR_TWR;
        } else {
            az.state = ST_MR_T3R;
        }
        break;

    case ST_MR_TWR:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.state = ST_MR_TWF;
        break;

    case ST_MR_TWF:
        if (sample & SAMPLE_CLK_BIT) return;
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
            az.state = ST_MR_TWR;
        } else {
            az.state = ST_MR_T3R;
        }
        break;

    case ST_MR_T3R:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.state = ST_MR_T3F;
        break;

    case ST_MR_T3F:     // capture read data
        if (sample & SAMPLE_CLK_BIT) return;
        az.data = sample_data(sample);
        emit_record(CYCLE_MEM_READ);
        az.state = ST_IDLE;
        break;

    // ================================================================
    // MEMORY WRITE — §4.3
    // ================================================================
    case ST_MW_T2R:     // capture write data
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.data = sample_data(sample);
        az.state = ST_MW_T2F;
        break;

    case ST_MW_T2F:
        if (sample & SAMPLE_CLK_BIT) return;
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
            az.state = ST_MW_TWR;
        } else {
            az.state = ST_MW_T3R;
        }
        break;

    case ST_MW_TWR:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.state = ST_MW_TWF;
        break;

    case ST_MW_TWF:
        if (sample & SAMPLE_CLK_BIT) return;
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
            az.state = ST_MW_TWR;
        } else {
            az.state = ST_MW_T3R;
        }
        break;

    case ST_MW_T3R:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.state = ST_MW_T3F;
        break;

    case ST_MW_T3F:
        if (sample & SAMPLE_CLK_BIT) return;
        emit_record(CYCLE_MEM_WRITE);
        az.state = ST_IDLE;
        break;

    // ================================================================
    // I/O PENDING — wait for T2↑ to confirm I/O type
    // ================================================================
    case ST_IO_T2R:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        if (!(sample & SAMPLE_IORQ_BIT)) {
            if (!(sample & SAMPLE_RD_BIT)) {
                az.wait_count = 1;
                az.state = ST_IR_T2F;
            } else if (!(sample & SAMPLE_WR_BIT)) {
                az.data = sample_data(sample);
                az.wait_count = 1;
                az.state = ST_IW_T2F;
            } else {
                // /IORQ active but neither /RD nor /WR — shouldn't happen
                begin_cycle(sample);
                az.state = ST_T1_NONM1;
            }
        } else {
            // Not I/O — internal operation. Re-interpret as T1↑.
            if (!(sample & SAMPLE_M1_BIT)) {
                begin_cycle(sample);
                az.state = ST_T1_M1;
            } else {
                begin_cycle(sample);
                az.state = ST_T1_NONM1;
            }
        }
        break;

    // ================================================================
    // I/O READ — §4.4
    // ================================================================
    case ST_IR_T2F:
        if (sample & SAMPLE_CLK_BIT) return;
        az.state = ST_IR_TWR;
        break;

    case ST_IR_TWR:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.state = ST_IR_TWF;
        break;

    case ST_IR_TWF:
        if (sample & SAMPLE_CLK_BIT) return;
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
            az.state = ST_IR_TWR;
        } else {
            az.state = ST_IR_T3R;
        }
        break;

    case ST_IR_T3R:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.state = ST_IR_T3F;
        break;

    case ST_IR_T3F:
        if (sample & SAMPLE_CLK_BIT) return;
        az.data = sample_data(sample);
        emit_record(CYCLE_IO_READ);
        az.state = ST_IDLE;
        break;

    // ================================================================
    // I/O WRITE — §4.5
    // ================================================================
    case ST_IW_T2F:
        if (sample & SAMPLE_CLK_BIT) return;
        az.state = ST_IW_TWR;
        break;

    case ST_IW_TWR:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.state = ST_IW_TWF;
        break;

    case ST_IW_TWF:
        if (sample & SAMPLE_CLK_BIT) return;
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
            az.state = ST_IW_TWR;
        } else {
            az.state = ST_IW_T3R;
        }
        break;

    case ST_IW_T3R:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.state = ST_IW_T3F;
        break;

    case ST_IW_T3F:
        if (sample & SAMPLE_CLK_BIT) return;
        emit_record(CYCLE_IO_WRITE);
        az.state = ST_IDLE;
        break;

    // ================================================================
    // INTERRUPT ACKNOWLEDGE — §4.6
    // ================================================================
    case ST_IA_T2R:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.state = ST_IA_T2F;
        break;

    case ST_IA_T2F:
        if (sample & SAMPLE_CLK_BIT) return;
        az.state = ST_IA_TW1R;
        break;

    case ST_IA_TW1R:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.state = ST_IA_TW1F;
        break;

    case ST_IA_TW1F:
        if (sample & SAMPLE_CLK_BIT) return;
        az.state = ST_IA_TW2R;
        break;

    case ST_IA_TW2R:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.state = ST_IA_TW2F;
        break;

    case ST_IA_TW2F:
        if (sample & SAMPLE_CLK_BIT) return;
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
            az.state = ST_IA_TWR;
        } else {
            az.state = ST_IA_T3R;
        }
        break;

    case ST_IA_TWR:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.state = ST_IA_TWF;
        break;

    case ST_IA_TWF:
        if (sample & SAMPLE_CLK_BIT) return;
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
            az.state = ST_IA_TWR;
        } else {
            az.state = ST_IA_T3R;
        }
        break;

    case ST_IA_T3R:
        if (!(sample & SAMPLE_CLK_BIT)) return;
        az.state = ST_IA_T3F;
        break;

    case ST_IA_T3F:     // capture interrupt vector
        if (sample & SAMPLE_CLK_BIT) return;
        az.data = sample_data(sample);
        az.state = ST_IA_T4R;
        break;

    case ST_IA_T4R: {   // refresh done, emit
        if (!(sample & SAMPLE_CLK_BIT)) return;
        uint32_t hi = sio_hw->gpio_hi_in;
        bool nmi = !(hi & HI_NMI_BIT);
        if (nmi && !az.nmi_prev) az.nmi_pending = true;
        az.nmi_prev = nmi;
        az.int_pending = !(hi & HI_INT_BIT);
        emit_record(CYCLE_INT_ACK);
        az.state = ST_IDLE;
        break;
    }

    } // switch
}

// ---- Core 1 entry point ----

static void core1_main(void) {
    uint32_t tail = 0;
    bool wait_is_asserted = false;

    memset(&az, 0, sizeof(az));
    az.state = ST_IDLE;
    az.syncing = true;

    while (true) {
        if (needs_reset) {
            tail = dma_write_index();
            memset(&az, 0, sizeof(az));
            az.state = ST_IDLE;
            az.syncing = true;
            wait_release();
            wait_is_asserted = false;
            needs_reset = false;
        }

        uint32_t head = dma_write_index();

        // DMA ring buffer overflow detection: if head has lapped tail,
        // samples were overwritten before core 1 could process them.
        uint32_t distance = (head - tail) & (CAPTURE_BUF_SIZE_WORDS - 1);
        if (__builtin_expect(distance > (CAPTURE_BUF_SIZE_WORDS * 3 / 4), 0)
            && distance != 0) {
            // Jump tail forward to head, losing the overwritten samples
            uint32_t lost = distance;
            tail = head;
            az.state = ST_IDLE;
            az.syncing = true;
            emit_status(STATUS_DMA_OVERFLOW, lost > 0x3FFF ? 0x3FFF : lost);
        }

        while (tail != head) {
            // Flush pending record (only involves spinlock when pending)
            if (__builtin_expect(az.has_pending, 0)) {
                if (queue_try_add(&trace_queue, &az.pending)) {
                    az.has_pending = false;
                    uint32_t level = queue_get_level(&trace_queue);
                    if (level <= FLOW_CTRL_RELEASE_THRESHOLD) {
                        wait_release();
                        if (wait_is_asserted) {
                            wait_is_asserted = false;
                            emit_status(STATUS_WAIT_RELEASE, level);
                        }
                    }
                } else {
                    // Queue still full, /WAIT is asserted but CLK keeps
                    // running — PIO/DMA keep filling the ring buffer with
                    // repeated wait-state samples.  Skip them all to
                    // prevent DMA ring overflow.
                    tail = head;
                    break;
                }
            }
            process_sample(sample_buf[tail]);
            tail = (tail + 1) & (CAPTURE_BUF_SIZE_WORDS - 1);

            // Detect new /WAIT assertion from emit_record
            if (__builtin_expect(az.has_pending && !wait_is_asserted, 0)) {
                wait_is_asserted = true;
            }
        }
    }
}

// ---- USB output ----

static bool emit_usb_packet(const trace_record_t *rec) {
    if (tud_cdc_write_available() < USB_PACKET_SIZE) return false;

    uint8_t pkt[USB_PACKET_SIZE + 1];  // +1 for optional refresh byte

    if (rec->cycle_type == CYCLE_STATUS) {
        // Status packet: subtype in byte 0 addr bits, seq in 1-2, count in 3-4
        uint8_t subtype = rec->data;
        uint16_t seq   = rec->address & 0x3FFF;
        uint16_t count = (rec->refresh << 7) | rec->wait_count;
        pkt[0] = 0x80 | (CYCLE_STATUS << 3) | (subtype & 0x07);
        pkt[1] = (seq >> 7) & 0x7F;
        pkt[2] = seq & 0x7F;
        pkt[3] = (count >> 7) & 0x7F;
        pkt[4] = count & 0x7F;
        tud_cdc_write(pkt, 5);
        return true;
    }

    uint16_t addr = rec->address;
    uint8_t  data = rec->data;

    pkt[0] = 0x80 | ((rec->cycle_type & 0x0F) << 3) | ((addr >> 13) & 0x07);
    pkt[1] = (addr >> 6) & 0x7F;
    pkt[2] = ((addr & 0x3F) << 1) | ((data >> 7) & 0x01);
    pkt[3] = data & 0x7F;
    pkt[4] = ((rec->flags & TRACE_FLAG_HALT) ? 0x40 : 0)
           | (rec->wait_count & 0x3F);

    int len = USB_PACKET_SIZE;
    if (rec->cycle_type == CYCLE_M1_FETCH) {
        pkt[5] = rec->refresh & 0x7F;
        len = 6;
    }
    tud_cdc_write(pkt, len);
    return true;
}

// ---- USB text I/O ----

static void usb_puts(const char *s) {
    tud_cdc_write_str(s);
    tud_cdc_write_flush();
}

// ---- Capture start / stop ----

static void capture_start(void) {
    // Signal core 1 to reset state
    needs_reset = true;
    while (needs_reset) tight_loop_contents();

    // Reset DMA
    dma_channel_set_write_addr(dma_chan, sample_buf, false);
    dma_channel_set_trans_count(dma_chan, 0xFFFFFFFF, false);

    // Reset PIO
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_jmp(pio_offset));

    // Start capture
    dma_channel_start(dma_chan);
    pio_sm_set_enabled(pio, sm, true);
}

static void capture_stop(void) {
    pio_sm_set_enabled(pio, sm, false);
    dma_channel_abort(dma_chan);
    wait_release();
}

// ---- Entry point (core 0) ----

int main(void) {
    // Overclock to 200 MHz for comfortable margin on state machine throughput.
    // At 4 MHz Z80 (8M samples/sec), this gives 25 cycles/sample vs ~23 needed.
    set_sys_clock_khz(200000, true);

    stdio_init_all();

    queue_init(&trace_queue, sizeof(trace_record_t), TRACE_QUEUE_SIZE);

    gpio_setup();
    pio_setup();
    dma_setup();

    // Launch analyzer state machine on core 1
    multicore_launch_core1(core1_main);

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
            // Any incoming byte stops tracing
            if (tud_cdc_available()) {
                uint8_t dummy[64];
                tud_cdc_read(dummy, sizeof(dummy));
                capture_stop();
                tracing = false;

                // Drain remaining records from queue
                trace_record_t rec;
                while (queue_try_remove(&trace_queue, &rec)) {
                    emit_usb_packet(&rec);
                }
                tud_cdc_write_flush();
                usb_puts("stopped\r\n");
                continue;
            }

            // Drain trace queue to USB
            trace_record_t rec;
            while (queue_try_remove(&trace_queue, &rec)) {
                if (!emit_usb_packet(&rec)) break;  // USB full
            }
            tud_cdc_write_flush();

        } else {
            // Command parsing
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
