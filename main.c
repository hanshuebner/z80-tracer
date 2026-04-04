#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include "tusb.h"

#include "z80_trace.h"
#include "z80_trace.pio.h"
#include "psram.h"

// ---- ARM DWT Cycle Counter (Cortex-M33) ----
// Direct register access — avoids header dependency on CMSIS.
#define DWT_CTRL_REG    (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT_REG  (*(volatile uint32_t *)0xE0001004)
#define DEMCR_REG       (*(volatile uint32_t *)0xE000EDF0)
#define DEMCR_TRCENA    (1u << 24)
#define DWT_CYCCNTENA   (1u << 0)

// Poll timeout: 600 ARM cycles (~2 µs at 300 MHz).
// Handles Z80 clocks down to ~0.5 MHz; catches stuck CLK.
#define POLL_TIMEOUT_CYCLES 600

// ---- DMA ring buffer: PIO samples land here ----

static uint32_t __attribute__((aligned(CAPTURE_BUF_SIZE_BYTES)))
    sample_buf[CAPTURE_BUF_SIZE_WORDS];

// ---- PSRAM trace ring buffer ----
// Core 1 writes records here.  Core 0 reads for trigger eval / readout.
// Using uncached address so writes go straight to PSRAM (no cache coherency issues).

static volatile trace_record_t *psram_ring =
    (volatile trace_record_t *)PSRAM_BASE_UNCACHED;

// Written by core 1 only, read by core 0.
static volatile uint32_t psram_write_idx;

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
// Rising-edge-only architecture (per z80-bus-analyzer-statemachine.md):
//   - PIO samples GPIO 0-31 at CLK rising edges only (one sample per T-state)
//   - Falling-edge data (/WAIT, read data, refresh addr) captured via
//     inline GPIO polling (busy-wait for CLK low, then read GPIO)
//   - Each process_sample() call handles exactly one rising edge
//   - Budget: ~70 ARM cycles per rising edge at 300 MHz / 4 MHz Z80
//
// Key optimizations:
//   - Lazy signal extraction: each state reads only the signals it needs
//   - PSRAM writes only on record emission (~1.3M/sec), not every sample
//   - /RESET checked in IDLE only (once per bus cycle)
//   - GPIO hi read only in states that need /WAIT, /INT, /NMI
//
// Synchronization: in sync mode, IDLE only accepts M1 cycles (/M1 low
// at T1↑). Internal operation T-states are handled by re-interpreting
// the current rising edge as a new potential T1.

// ---- Analyzer states ----

typedef enum {
    // Top-level / between cycles
    ST_IDLE,            // expecting T1↑
    ST_RESET,           // /RESET active

    // After T1↑, expecting T2↑
    ST_T2_M1,          // M1 class cycle — will discriminate fetch vs INT ACK
    ST_T2_NONM1,      // Non-M1 cycle — will discriminate mem/io read/write

    // M1 opcode fetch (§4.1)
    ST_M1_TW,          // expecting TW↑ (wait state)
    ST_M1_T3,          // expecting T3↑ — capture opcode + refresh
    ST_M1_T4,          // expecting T4↑ — sample interrupts, emit

    // Memory read (§4.2)
    ST_MR_TW,          // expecting TW↑
    ST_MR_T3,          // expecting T3↑ — capture read data

    // Memory write (§4.3)
    ST_MW_TW,          // expecting TW↑
    ST_MW_T3,          // expecting T3↑ — emit

    // I/O read (§4.4)
    ST_IR_TW,          // expecting TW*↑ (auto) or additional TW↑
    ST_IR_T3,          // expecting T3↑ — capture read data

    // I/O write (§4.5)
    ST_IW_TW,          // expecting TW*↑ (auto) or additional TW↑
    ST_IW_T3,          // expecting T3↑ — emit

    // Interrupt acknowledge (§4.6)
    ST_IA_TW1,         // expecting TW1*↑ (first auto-wait)
    ST_IA_TW2,         // expecting TW2*↑ (second auto-wait)
    ST_IA_TW,          // expecting additional TW↑
    ST_IA_T3,          // expecting T3↑ — capture vector
    ST_IA_T4,          // expecting T4↑ — refresh, emit
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

    // Synchronization: when true, IDLE only accepts M1 cycles
    bool     syncing;

    // Sequence counter for gap detection (7-bit, wraps)
    uint8_t  seq;
} az;

#define HI_WAIT_BIT   (1u << (PIN_WAIT  - 32))
#define HI_INT_BIT    (1u << (PIN_INT   - 32))
#define HI_NMI_BIT    (1u << (PIN_NMI   - 32))
#define HI_RESET_BIT  (1u << (PIN_RESET - 32))

// ---- Performance tracking (core 1 only, read by core 0 via status) ----

static volatile uint32_t perf_sample_max;   // worst-case cycles per process_sample
static volatile uint32_t perf_sample_min;   // best-case cycles per process_sample
static volatile uint32_t perf_pio_overflows; // PIO FIFO stall count
static volatile uint32_t perf_staging_overflows; // staging buffer overrun count
static volatile uint32_t perf_poll_timeouts; // GPIO poll timeout count
static volatile uint32_t perf_data_loss_records; // CYCLE_DATA_LOSS records emitted

// ---- SRAM staging buffer (core 1 → PSRAM) ----
// emit_record() writes here (fast SRAM write, ~3 cycles).
// stage_drain_one() is called from the sample processing loop to
// interleave PSRAM writes with sample processing.
#define STAGE_BUF_SIZE  256  // power of 2
#define STAGE_BUF_MASK  (STAGE_BUF_SIZE - 1)

static trace_record_t stage_buf[STAGE_BUF_SIZE];
static uint32_t stage_wr;  // written by emit_record (core 1 only)
static uint32_t stage_rd;  // read by stage_drain_one (core 1 only)

// ---- Emit a completed trace record into SRAM staging buffer ----

static void __always_inline emit_record(uint8_t cycle_type) {
    if (__builtin_expect((stage_wr - stage_rd) >= STAGE_BUF_SIZE, 0)) {
        perf_staging_overflows++;
        return;  // drop record — staging buffer full
    }
    trace_record_t *dst = &stage_buf[stage_wr & STAGE_BUF_MASK];
    dst->address    = az.address;
    dst->data       = az.data;
    dst->cycle_type = cycle_type;
    dst->refresh    = az.refresh;
    dst->wait_count = az.wait_count;
    dst->flags      = 0;
    dst->seq        = az.seq++ & 0x7F;
    if (az.halt)        dst->flags |= TRACE_FLAG_HALT;
    if (az.int_pending) dst->flags |= TRACE_FLAG_INT;
    if (az.nmi_pending) dst->flags |= TRACE_FLAG_NMI;

    stage_wr++;
    az.nmi_pending = false;
}

// Drain one staged record to PSRAM (amortized: ~42 cycles every few samples)
static void __not_in_flash_func(stage_drain_one)(void) {
    if (stage_rd != stage_wr) {
        psram_ring[psram_write_idx & PSRAM_RING_MASK] =
            stage_buf[stage_rd & STAGE_BUF_MASK];
        psram_write_idx++;
        stage_rd++;
    }
}

// Drain all remaining staged records (called between batches)
static void __not_in_flash_func(stage_drain_all)(void) {
    while (stage_rd != stage_wr) {
        psram_ring[psram_write_idx & PSRAM_RING_MASK] =
            stage_buf[stage_rd & STAGE_BUF_MASK];
        psram_write_idx++;
        stage_rd++;
    }
}

// ---- Emit a data loss marker into the trace stream ----

static void emit_data_loss(uint8_t cause, uint16_t lost_count) {
    if ((stage_wr - stage_rd) >= STAGE_BUF_SIZE)
        return;  // staging buffer full — can't even report the loss
    trace_record_t *dst = &stage_buf[stage_wr & STAGE_BUF_MASK];
    dst->address    = lost_count;
    dst->data       = cause;
    dst->cycle_type = CYCLE_DATA_LOSS;
    dst->refresh    = 0;
    dst->wait_count = 0;
    dst->flags      = 0;
    dst->seq        = az.seq++ & 0x7F;
    stage_wr++;
    perf_data_loss_records++;
}

// ---- Falling-edge GPIO polling helpers ----
// After processing a rising edge, the ARM core busy-waits for CLK to go
// low, then reads the bus.  At 4 MHz Z80 / 300 MHz ARM, the spin is
// ~37 cycles (125 ns half-period).  Acceptable since core 1 is dedicated.
// A timeout catches stuck CLK or unexpectedly slow clocks.

// Poll for CLK falling edge, return true if /WAIT is active (low).
// Returns false (no wait) on timeout.
static bool __always_inline poll_wait(void) {
    uint32_t t0 = DWT_CYCCNT_REG;
    while (sio_hw->gpio_in & (1u << PIN_CLK)) {
        if (__builtin_expect((DWT_CYCCNT_REG - t0) > POLL_TIMEOUT_CYCLES, 0)) {
            perf_poll_timeouts++;
            return false;
        }
    }
    return !(sio_hw->gpio_hi_in & HI_WAIT_BIT);
}

// Poll for CLK falling edge, return GPIO 0-31 snapshot (bus state).
// Returns 0 on timeout.
static uint32_t __always_inline poll_bus(void) {
    uint32_t t0 = DWT_CYCCNT_REG;
    while (sio_hw->gpio_in & (1u << PIN_CLK)) {
        if (__builtin_expect((DWT_CYCCNT_REG - t0) > POLL_TIMEOUT_CYCLES, 0)) {
            perf_poll_timeouts++;
            return 0;
        }
    }
    return sio_hw->gpio_in;
}

// ---- Begin a new cycle at T1↑ ----

static void __always_inline begin_cycle(uint32_t sample) {
    az.address    = sample_addr(sample);
    az.data       = 0;
    az.refresh    = 0;
    az.wait_count = 0;
    az.halt       = !(sample & SAMPLE_HALT_BIT);
}

// ---- Process one PIO sample (one CLK rising edge) ----
// Each state handles exactly one rising edge.  States that need
// falling-edge data call poll_wait() or poll_bus() inline before
// returning — the GPIO poll completes within the same T-state,
// well before the next PIO sample arrives.

static void __always_inline process_sample(uint32_t sample) {

    switch (az.state) {

    // ================================================================
    // IDLE — expecting T1 rising edge
    // ================================================================
    case ST_IDLE:
        // Check /RESET once per bus cycle
        if (__builtin_expect(!(sio_hw->gpio_hi_in & HI_RESET_BIT), 0)) {
            az.state = ST_RESET;
            return;
        }
        // T1↑: capture address, check /M1
        if (!(sample & SAMPLE_M1_BIT)) {
            begin_cycle(sample);
            az.syncing = false;
            az.state = ST_T2_M1;
        } else if (!az.syncing) {
            begin_cycle(sample);
            az.state = ST_T2_NONM1;
        }
        break;

    // ================================================================
    // RESET
    // ================================================================
    case ST_RESET:
        // Wait for /RESET to release
        if (!(sio_hw->gpio_hi_in & HI_RESET_BIT)) return;  // still in reset
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
    // T2↑ — cycle type discrimination
    // ================================================================

    case ST_T2_M1:
        // T2↑ of M1 cycle: /MREQ low → opcode fetch, high → INT ACK
        if (!(sample & SAMPLE_MREQ_BIT)) {
            // OPCODE FETCH confirmed
            // Poll T2↓: check /WAIT
            if (poll_wait()) {
                az.wait_count++;
                az.state = ST_M1_TW;
            } else {
                az.state = ST_M1_T3;
            }
        } else {
            // INTERRUPT ACKNOWLEDGE — /MREQ stayed high
            az.wait_count = 2;  // two auto-waits always present
            az.state = ST_IA_TW1;
        }
        break;

    case ST_T2_NONM1:
        // T2↑ of non-M1 cycle: determine exact type from control signals
        if (!(sample & SAMPLE_MREQ_BIT)) {
            // Memory cycle — /MREQ asserted
            if (!(sample & SAMPLE_RD_BIT)) {
                // MEMORY READ
                if (poll_wait()) {
                    az.wait_count++;
                    az.state = ST_MR_TW;
                } else {
                    az.state = ST_MR_T3;
                }
            } else {
                // MEMORY WRITE — capture write data at T2↑
                az.data = sample_data(sample);
                if (poll_wait()) {
                    az.wait_count++;
                    az.state = ST_MW_TW;
                } else {
                    az.state = ST_MW_T3;
                }
            }
        } else if (!(sample & SAMPLE_IORQ_BIT)) {
            // I/O cycle — /IORQ asserted
            if (!(sample & SAMPLE_RD_BIT)) {
                // I/O READ — auto TW* always inserted
                az.wait_count = 1;
                az.state = ST_IR_TW;
            } else {
                // I/O WRITE — capture write data at T2↑, auto TW*
                az.data = sample_data(sample);
                az.wait_count = 1;
                az.state = ST_IW_TW;
            }
        } else {
            // Neither /MREQ nor /IORQ — internal operation.
            // Re-interpret this rising edge as a potential T1↑.
            if (!(sample & SAMPLE_M1_BIT)) {
                begin_cycle(sample);
                az.syncing = false;
                az.state = ST_T2_M1;
            } else {
                begin_cycle(sample);
                az.state = ST_T2_NONM1;
            }
        }
        break;

    // ================================================================
    // M1 OPCODE FETCH — §4.1
    // ================================================================

    case ST_M1_TW:
        // TW↑: poll TW↓ for /WAIT
        if (poll_wait()) {
            az.wait_count++;
            // stay in ST_M1_TW
        } else {
            az.state = ST_M1_T3;
        }
        break;

    case ST_M1_T3: {
        // T3↑: capture opcode from PIO sample (rising edge)
        az.data = sample_data(sample);
        // Poll T3↓: capture refresh address A[6:0]
        uint32_t bus = poll_bus();
        az.refresh = bus & 0x7F;
        az.state = ST_M1_T4;
        break;
    }

    case ST_M1_T4: {
        // T4↑: sample /INT, /NMI, emit record
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

    case ST_MR_TW:
        // TW↑: poll TW↓ for /WAIT
        if (poll_wait()) {
            az.wait_count++;
        } else {
            az.state = ST_MR_T3;
        }
        break;

    case ST_MR_T3: {
        // T3↑: poll T3↓ for read data (data valid through T3↓)
        uint32_t bus = poll_bus();
        az.data = (bus >> SAMPLE_DATA_SHIFT) & 0xFF;
        emit_record(CYCLE_MEM_READ);
        az.state = ST_IDLE;
        break;
    }

    // ================================================================
    // MEMORY WRITE — §4.3
    // ================================================================

    case ST_MW_TW:
        // TW↑: poll TW↓ for /WAIT
        if (poll_wait()) {
            az.wait_count++;
        } else {
            az.state = ST_MW_T3;
        }
        break;

    case ST_MW_T3:
        // T3↑: cycle complete (write data already captured at T2↑)
        emit_record(CYCLE_MEM_WRITE);
        az.state = ST_IDLE;
        break;

    // ================================================================
    // I/O READ — §4.4
    // ================================================================

    case ST_IR_TW:
        // TW*↑ (auto) or additional TW↑: poll TW↓ for /WAIT
        if (poll_wait()) {
            az.wait_count++;
            // stay in ST_IR_TW
        } else {
            az.state = ST_IR_T3;
        }
        break;

    case ST_IR_T3: {
        // T3↑: poll T3↓ for read data
        uint32_t bus = poll_bus();
        az.data = (bus >> SAMPLE_DATA_SHIFT) & 0xFF;
        emit_record(CYCLE_IO_READ);
        az.state = ST_IDLE;
        break;
    }

    // ================================================================
    // I/O WRITE — §4.5
    // ================================================================

    case ST_IW_TW:
        // TW*↑ (auto) or additional TW↑: poll TW↓ for /WAIT
        if (poll_wait()) {
            az.wait_count++;
        } else {
            az.state = ST_IW_T3;
        }
        break;

    case ST_IW_T3:
        // T3↑: cycle complete (write data already captured at T2↑)
        emit_record(CYCLE_IO_WRITE);
        az.state = ST_IDLE;
        break;

    // ================================================================
    // INTERRUPT ACKNOWLEDGE — §4.6
    // ================================================================

    case ST_IA_TW1:
        // TW1*↑: first auto-wait (always present, TW2* always follows)
        az.state = ST_IA_TW2;
        break;

    case ST_IA_TW2:
        // TW2*↑: second auto-wait — poll TW2*↓ for /WAIT
        if (poll_wait()) {
            az.wait_count++;
            az.state = ST_IA_TW;
        } else {
            az.state = ST_IA_T3;
        }
        break;

    case ST_IA_TW:
        // Additional TW↑: poll TW↓ for /WAIT
        if (poll_wait()) {
            az.wait_count++;
        } else {
            az.state = ST_IA_T3;
        }
        break;

    case ST_IA_T3: {
        // T3↑: poll T3↓ for interrupt vector byte
        uint32_t bus = poll_bus();
        az.data = (bus >> SAMPLE_DATA_SHIFT) & 0xFF;
        az.state = ST_IA_T4;
        break;
    }

    case ST_IA_T4: {
        // T4↑: refresh done, sample /INT + /NMI, emit record
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

// ---- /WAIT flow control (open-drain on GPIO 32) ----
// Output latch preset to 0.  Assert = set dir to GPIO_OUT (drives low).
// Release = set dir to GPIO_IN (high-Z, external pull-up restores high).
// Shared with screen controller — never drive high.

#define DMA_WAIT_ASSERT_THRESHOLD   (CAPTURE_BUF_SIZE_WORDS / 2)      // 50%
#define DMA_WAIT_RELEASE_THRESHOLD  (CAPTURE_BUF_SIZE_WORDS / 4)      // 25%

static bool wait_asserted = false;

static inline void wait_assert(void) {
    if (!wait_asserted) {
        gpio_set_dir(PIN_WAIT, GPIO_OUT);  // drive low
        wait_asserted = true;
    }
}

static inline void wait_release(void) {
    if (wait_asserted) {
        gpio_set_dir(PIN_WAIT, GPIO_IN);   // high-Z
        wait_asserted = false;
    }
}

// ---- Diagnostic mode (bitmask, set by CMD_SET_DIAG_MODE) ----
// Each bit disables a component to isolate performance bottlenecks.
#define DIAG_SKIP_ANALYZER   (1u << 0)  // don't call process_sample, just count
#define DIAG_SKIP_PSRAM      (1u << 1)  // don't drain to PSRAM, discard records
#define DIAG_SKIP_BOTH       (DIAG_SKIP_ANALYZER | DIAG_SKIP_PSRAM)

static volatile uint8_t diag_mode = 0;

// ---- Core 1 entry point ----

// Diagnostic counters (read by core 0)
static volatile uint32_t diag_dma_overflows;
static volatile uint32_t diag_max_dma_distance;   // worst DMA backlog seen
static volatile uint32_t diag_max_stage_depth;     // worst staging buffer depth
static volatile uint32_t diag_total_samples;       // total PIO samples processed
static volatile uint32_t diag_wait_asserts;        // /WAIT assertion count

static void __not_in_flash_func(core1_main)(void) {
    // Enable DWT cycle counter for performance measurement
    DEMCR_REG |= DEMCR_TRCENA;
    DWT_CYCCNT_REG = 0;
    DWT_CTRL_REG |= DWT_CYCCNTENA;

    uint32_t tail = 0;
    uint32_t sample_count = 0;

    memset(&az, 0, sizeof(az));
    az.state = ST_IDLE;
    az.syncing = true;
    stage_wr = 0;
    stage_rd = 0;

    while (true) {
        if (needs_reset) {
            tail = dma_write_index();
            memset(&az, 0, sizeof(az));
            az.state = ST_IDLE;
            az.syncing = true;
            stage_wr = 0;
            stage_rd = 0;
            sample_count = 0;
            diag_dma_overflows = 0;
            diag_max_dma_distance = 0;
            diag_max_stage_depth = 0;
            diag_total_samples = 0;
            diag_wait_asserts = 0;
            perf_sample_max = 0;
            perf_sample_min = 0xFFFFFFFF;
            perf_pio_overflows = 0;
            perf_staging_overflows = 0;
            perf_poll_timeouts = 0;
            perf_data_loss_records = 0;
            wait_release();
            needs_reset = false;
        }

        // Check PIO RX FIFO stall (overflow — samples lost at source)
        uint32_t rxstall_mask = 1u << (PIO_FDEBUG_RXSTALL_LSB + sm);
        if (__builtin_expect(pio->fdebug & rxstall_mask, 0)) {
            pio->fdebug = rxstall_mask;  // write 1 to clear
            perf_pio_overflows++;
            emit_data_loss(LOSS_PIO_OVERFLOW, 0);
        }

        uint32_t head = dma_write_index();

        // DMA buffer fullness check + /WAIT flow control
        uint32_t distance = (head - tail) & (CAPTURE_BUF_SIZE_WORDS - 1);
        if (distance > diag_max_dma_distance)
            diag_max_dma_distance = distance;

        if (__builtin_expect(distance > DMA_WAIT_ASSERT_THRESHOLD, 0)
            && distance != 0) {
            // Assert /WAIT to pause Z80 before we overflow
            if (!wait_asserted) diag_wait_asserts++;
            wait_assert();
        }

        // DMA overflow detection (shouldn't happen with /WAIT, but safety net)
        if (__builtin_expect(distance > (CAPTURE_BUF_SIZE_WORDS * 3 / 4), 0)
            && distance != 0) {
            uint32_t lost = (distance - (CAPTURE_BUF_SIZE_WORDS / 2));
            diag_dma_overflows++;
            emit_data_loss(LOSS_DMA_OVERFLOW, lost > 0xFFFF ? 0xFFFF : lost);
            tail = head;
            az.state = ST_IDLE;
            az.syncing = true;
        }

        // Process PIO samples with interleaved PSRAM drain.
        // Each process_sample() call is instrumented with DWT cycle counting.
        uint8_t mode = diag_mode;
        if (mode & DIAG_SKIP_ANALYZER) {
            // Mode 1/3: skip state machine entirely, just count samples
            uint32_t n = (head - tail) & (CAPTURE_BUF_SIZE_WORDS - 1);
            sample_count += n;
            tail = head;
        } else if (mode & DIAG_SKIP_PSRAM) {
            // Mode 2: run state machine but discard records (no PSRAM writes)
            while (tail != head) {
                uint32_t t0 = DWT_CYCCNT_REG;
                process_sample(sample_buf[tail]);
                uint32_t dt = DWT_CYCCNT_REG - t0;
                if (dt > perf_sample_max) perf_sample_max = dt;
                if (dt < perf_sample_min) perf_sample_min = dt;
                tail = (tail + 1) & (CAPTURE_BUF_SIZE_WORDS - 1);
                sample_count++;
            }
            // Discard staged records
            stage_rd = stage_wr;
        } else {
            // Normal mode: state machine + interleaved PSRAM drain
            while (tail != head) {
                uint32_t t0 = DWT_CYCCNT_REG;
                process_sample(sample_buf[tail]);
                uint32_t dt = DWT_CYCCNT_REG - t0;
                if (dt > perf_sample_max) perf_sample_max = dt;
                if (dt < perf_sample_min) perf_sample_min = dt;
                tail = (tail + 1) & (CAPTURE_BUF_SIZE_WORDS - 1);
                sample_count++;
                if ((sample_count & 3) == 0)
                    stage_drain_one();
            }
        }

        // Drain any remaining staged records
        uint32_t stage_depth = stage_wr - stage_rd;
        if (stage_depth > diag_max_stage_depth)
            diag_max_stage_depth = stage_depth;
        if (!(mode & DIAG_SKIP_PSRAM))
            stage_drain_all();

        // Release /WAIT once we've caught up
        if (wait_asserted) {
            uint32_t new_dist = (dma_write_index() - tail) & (CAPTURE_BUF_SIZE_WORDS - 1);
            if (new_dist <= DMA_WAIT_RELEASE_THRESHOLD)
                wait_release();
        }

        diag_total_samples = sample_count;
    }
}

// ---- Capture start / stop ----

static void capture_start(void) {
    // Reset PIO + DMA first (stopped state)
    pio_sm_set_enabled(pio, sm, false);
    dma_channel_abort(dma_chan);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_jmp(pio_offset));
    dma_channel_set_write_addr(dma_chan, sample_buf, false);
    dma_channel_set_trans_count(dma_chan, 0xFFFFFFFF, false);

    // Reset PSRAM write index
    psram_write_idx = 0;

    // Signal core 1 to reset its state while DMA is still stopped.
    needs_reset = true;
    while (needs_reset) tight_loop_contents();

    // Now start capture — core 1's tail is in sync with DMA write pointer.
    dma_channel_start(dma_chan);
    pio_sm_set_enabled(pio, sm, true);
}

static void capture_stop(void) {
    pio_sm_set_enabled(pio, sm, false);
    dma_channel_abort(dma_chan);
    wait_release();
}

// ---- USB helpers ----

// Read exactly n bytes from USB CDC, blocking until available
static void usb_read_bytes(uint8_t *buf, uint32_t n) {
    uint32_t got = 0;
    while (got < n) {
        tud_task();
        uint32_t avail = tud_cdc_available();
        if (avail > 0) {
            uint32_t want = n - got;
            if (want > avail) want = avail;
            tud_cdc_read(buf + got, want);
            got += want;
        }
    }
}

// Write raw bytes to USB CDC, handling backpressure
static void usb_write_bytes(const void *data, uint32_t len) {
    const uint8_t *p = (const uint8_t *)data;
    uint32_t sent = 0;
    while (sent < len) {
        tud_task();
        uint32_t avail = tud_cdc_write_available();
        if (avail > 0) {
            uint32_t chunk = len - sent;
            if (chunk > avail) chunk = avail;
            tud_cdc_write(p + sent, chunk);
            sent += chunk;
            tud_cdc_write_flush();
        }
    }
}

// ---- Trigger + capture state machine (core 0) ----

static capture_state_t cap_state = CAP_IDLE;
static trigger_t triggers[MAX_TRIGGERS];
static uint32_t trigger_count = 0;
static uint32_t trigger_idx;          // write_idx when trigger fired
static uint32_t post_trigger_count;   // how many records to capture after trigger
static uint32_t post_trigger_remain;  // countdown after trigger
static uint32_t trigger_chase_idx;    // core 0's read position chasing core 1

// Check one record against all configured triggers
static bool check_triggers(const trace_record_t *rec) {
    for (uint32_t i = 0; i < trigger_count; i++) {
        const trigger_t *t = &triggers[i];
        bool match = false;

        switch (t->type) {
        case TRIG_PC_MATCH:
            if (rec->cycle_type == CYCLE_M1_FETCH &&
                rec->address >= t->addr_lo && rec->address <= t->addr_hi)
                match = true;
            break;
        case TRIG_MEM_READ:
            if (rec->cycle_type == CYCLE_MEM_READ &&
                rec->address >= t->addr_lo && rec->address <= t->addr_hi)
                match = true;
            break;
        case TRIG_MEM_WRITE:
            if (rec->cycle_type == CYCLE_MEM_WRITE &&
                rec->address >= t->addr_lo && rec->address <= t->addr_hi)
                match = true;
            break;
        case TRIG_IO_READ:
            if (rec->cycle_type == CYCLE_IO_READ &&
                rec->address >= t->addr_lo && rec->address <= t->addr_hi)
                match = true;
            break;
        case TRIG_IO_WRITE:
            if (rec->cycle_type == CYCLE_IO_WRITE &&
                rec->address >= t->addr_lo && rec->address <= t->addr_hi)
                match = true;
            break;
        case TRIG_INT_ACK:
            if (rec->cycle_type == CYCLE_INT_ACK)
                match = true;
            break;
        }

        if (match && t->data_mask) {
            if ((rec->data & t->data_mask) != (t->data_val & t->data_mask))
                match = false;
        }

        if (match) return true;
    }
    return false;
}

// Run trigger evaluation: chase core 1's write_idx, check each record
static void trigger_eval(void) {
    uint32_t widx = psram_write_idx;

    while (trigger_chase_idx != widx) {
        uint32_t idx = trigger_chase_idx & PSRAM_RING_MASK;
        // Read from uncached PSRAM
        const volatile trace_record_t *src = &psram_ring[idx];
        trace_record_t rec;
        rec.address    = src->address;
        rec.data       = src->data;
        rec.cycle_type = src->cycle_type;
        rec.refresh    = src->refresh;
        rec.wait_count = src->wait_count;
        rec.flags      = src->flags;
        rec.seq        = src->seq;

        if (cap_state == CAP_ARMED) {
            if (check_triggers(&rec)) {
                trigger_idx = trigger_chase_idx;
                post_trigger_remain = post_trigger_count;
                cap_state = CAP_TRIGGERED;
            }
        }

        if (cap_state == CAP_TRIGGERED) {
            if (post_trigger_remain == 0) {
                capture_stop();
                cap_state = CAP_DONE;
                return;
            }
            post_trigger_remain--;
        }

        trigger_chase_idx++;
    }
}

// ---- Command handlers ----

static void cmd_start_capture(void) {
    if (cap_state == CAP_IDLE || cap_state == CAP_DONE) {
        capture_start();
        trigger_chase_idx = psram_write_idx;
        cap_state = CAP_RUNNING;
    }
}

static void cmd_stop_capture(void) {
    if (cap_state != CAP_IDLE) {
        capture_stop();
        cap_state = CAP_IDLE;
    }
}

static void cmd_arm_trigger(void) {
    uint8_t params[4];
    usb_read_bytes(params, 4);
    post_trigger_count = params[0] | (params[1] << 8) |
                         (params[2] << 16) | (params[3] << 24);

    if (cap_state == CAP_RUNNING) {
        trigger_chase_idx = psram_write_idx;
        cap_state = CAP_ARMED;
    } else if (cap_state == CAP_IDLE || cap_state == CAP_DONE) {
        // Start capture and arm in one step
        capture_start();
        trigger_chase_idx = psram_write_idx;
        cap_state = CAP_ARMED;
    }
}

static void cmd_set_trigger(void) {
    uint8_t buf[7];
    usb_read_bytes(buf, 7);
    if (trigger_count < MAX_TRIGGERS) {
        trigger_t *t = &triggers[trigger_count];
        t->type    = buf[0];
        t->addr_lo = buf[1] | (buf[2] << 8);
        t->addr_hi = buf[3] | (buf[4] << 8);
        t->data_val  = buf[5];
        t->data_mask = buf[6];
        trigger_count++;
    }
}

static void cmd_clear_triggers(void) {
    trigger_count = 0;
    memset(triggers, 0, sizeof(triggers));
}

static void cmd_get_status(void) {
    status_response_t resp;
    resp.magic = STATUS_MAGIC;
    resp.capture_state = cap_state;
    resp.write_idx = psram_write_idx;
    resp.dma_overflows = diag_dma_overflows;
    resp.max_dma_distance = diag_max_dma_distance;
    resp.max_stage_depth = diag_max_stage_depth;
    resp.total_samples = diag_total_samples;
    resp.cpu_clock_khz = clock_get_hz(clk_sys) / 1000;
    resp.trigger_idx = trigger_idx;
    resp.trigger_count = trigger_count;
    resp.wait_asserts = diag_wait_asserts;
    // Performance measurement
    resp.sample_max_cycles = perf_sample_max;
    resp.sample_min_cycles = perf_sample_min;
    // Data loss counters
    resp.pio_overflows = perf_pio_overflows;
    resp.staging_overflows = perf_staging_overflows;
    resp.poll_timeouts = perf_poll_timeouts;
    resp.data_loss_records = perf_data_loss_records;
    usb_write_bytes(&resp, sizeof(resp));
}

static void cmd_read_buffer(void) {
    uint8_t params[8];
    usb_read_bytes(params, 8);
    uint32_t pre_count  = params[0] | (params[1] << 8) |
                          (params[2] << 16) | (params[3] << 24);
    uint32_t post_count = params[4] | (params[5] << 8) |
                          (params[6] << 16) | (params[7] << 24);

    // Suspend capture while reading
    capture_state_t prev_state = cap_state;
    if (cap_state != CAP_IDLE) {
        capture_stop();
    }

    // Reference point: trigger_idx if we triggered, else current write_idx
    uint32_t ref_idx;
    bool has_trigger = (prev_state == CAP_TRIGGERED || prev_state == CAP_DONE);
    if (has_trigger) {
        ref_idx = trigger_idx;
    } else {
        ref_idx = psram_write_idx;
    }

    uint32_t widx = psram_write_idx;

    // Determine available records before and after reference point
    uint32_t available = widx;
    if (available > PSRAM_RING_RECORDS)
        available = PSRAM_RING_RECORDS;

    // Records before ref point
    uint32_t avail_pre = ref_idx - (widx - available);
    if (avail_pre > available) avail_pre = available;
    if (pre_count > avail_pre) pre_count = avail_pre;

    // Records after ref point (only meaningful if triggered)
    uint32_t avail_post = widx - ref_idx;
    if (avail_post > available) avail_post = 0;
    if (post_count > avail_post) post_count = avail_post;

    uint32_t total = pre_count + post_count;
    uint32_t start_idx = ref_idx - pre_count;

    // Send header
    readout_header_t hdr;
    hdr.magic = READOUT_MAGIC;
    hdr.total_records = total;
    hdr.write_idx = widx;
    hdr.trigger_offset = has_trigger ? pre_count : 0xFFFFFFFF;
    usb_write_bytes(&hdr, sizeof(hdr));

    // Send records from PSRAM
    for (uint32_t i = 0; i < total; i++) {
        uint32_t idx = (start_idx + i) & PSRAM_RING_MASK;
        trace_record_t rec;
        const volatile trace_record_t *src = &psram_ring[idx];
        rec.address    = src->address;
        rec.data       = src->data;
        rec.cycle_type = src->cycle_type;
        rec.refresh    = src->refresh;
        rec.wait_count = src->wait_count;
        rec.flags      = src->flags;
        rec.seq        = src->seq;
        usb_write_bytes(&rec, sizeof(rec));
    }

    // Resume capture if it was running (not if trigger-done)
    if (prev_state == CAP_RUNNING || prev_state == CAP_ARMED) {
        capture_start();
        trigger_chase_idx = psram_write_idx;
        cap_state = prev_state;
    } else {
        cap_state = CAP_IDLE;
    }
}

// ---- Entry point (core 0) ----

int main(void) {
    // Overclock to 300 MHz for comfortable margin on state machine throughput.
    // At 4 MHz Z80 (4M rising-edge samples/sec), this gives ~75 cycles/sample.
    set_sys_clock_khz(300000, true);

    stdio_init_all();

    // Init PSRAM before anything else that uses it
    size_t psram_size = psram_init();
    (void)psram_size;  // 8 MB expected; detection errors are silent for now

    gpio_setup();
    pio_setup();
    dma_setup();

    // Launch analyzer state machine on core 1
    multicore_launch_core1(core1_main);

    uint8_t cmd_state = 0;  // 0 = idle, 1 = got CMD_SYNC

    while (true) {
        tud_task();

        if (!tud_cdc_connected()) {
            if (cap_state != CAP_IDLE) {
                capture_stop();
                cap_state = CAP_IDLE;
            }
            cmd_state = 0;
            continue;
        }

        // Process incoming commands (binary: 0xFF + command byte + payload)
        while (tud_cdc_available()) {
            uint8_t ch;
            tud_cdc_read(&ch, 1);
            if (cmd_state == 0) {
                if (ch == CMD_SYNC) cmd_state = 1;
            } else {
                cmd_state = 0;
                switch (ch) {
                case CMD_START_CAPTURE:
                    cmd_start_capture();
                    break;
                case CMD_STOP_CAPTURE:
                    cmd_stop_capture();
                    break;
                case CMD_READ_BUFFER:
                    cmd_read_buffer();
                    break;
                case CMD_GET_STATUS:
                    cmd_get_status();
                    break;
                case CMD_SET_DIAG_MODE: {
                    uint8_t mode;
                    usb_read_bytes(&mode, 1);
                    diag_mode = mode;
                    break;
                }
                case CMD_SET_TRIGGER:
                    cmd_set_trigger();
                    break;
                case CMD_CLEAR_TRIGGERS:
                    cmd_clear_triggers();
                    break;
                case CMD_ARM_TRIGGER:
                    cmd_arm_trigger();
                    break;
                }
            }
        }

        // Run trigger evaluation when armed or post-trigger
        if (cap_state == CAP_ARMED || cap_state == CAP_TRIGGERED)
            trigger_eval();
    }
}
