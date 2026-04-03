#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include "tusb.h"

#include "z80_trace.h"
#include "psram.h"

// ---- PSRAM trace ring buffer ----
// Core 1 writes records here.  Core 0 reads for trigger eval / readout.
// Using uncached address so writes go straight to PSRAM (no cache coherency issues).

static volatile trace_record_t *psram_ring =
    (volatile trace_record_t *)PSRAM_BASE_UNCACHED;

// Written by core 1 only, read by core 0.
static volatile uint32_t psram_write_idx;

// ---- Shared state between cores ----

static volatile bool needs_reset = false;     // core 0 signals core 1 to reset
static volatile bool capture_running = false; // core 0 starts/stops capture

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

// ========================================================================
// Z80 Bus Analyzer State Machine — runs on core 1
// ========================================================================
//
// GPIO-polled, rising-edge-only. Core 1 polls sio_hw->gpio_in directly
// for CLK edges — no PIO, no DMA, no ring buffer.
//
// At 4 MHz Z80, one rising edge per 250ns = ~75 ARM cycles at 300 MHz.
// Work is split across CLK phases:
//   CLK-high: process_sample() handles rising-edge state logic (~15 cycles)
//   CLK-low:  complete_falling_capture() for states needing T3↓ data,
//             plus PSRAM drain (~20-30 cycles)
//
// This gives ~37 cycles per phase — ample headroom.

// ---- Analyzer states ----
// All states correspond to rising-edge samples only.

typedef enum {
    // Between cycles
    ST_IDLE,
    ST_RESET,

    // M1 opcode fetch (§4.1)
    ST_M1_T2,          // T2↑ — confirm /MREQ low (fetch vs INTACK), check /WAIT
    ST_M1_WAIT,        // TW↑ — wait state, check /WAIT
    ST_M1_T3,          // T3↑ — capture opcode; needs_falling for refresh
    ST_M1_T4,          // T4↑ — sample interrupts, emit

    // Non-M1 cycle type discrimination at T2↑
    ST_IO_T2,

    // Memory read (§4.2)
    ST_MR_WAIT,        // TW↑ — check /WAIT
    ST_MR_T3,          // T3↑ — needs_falling for read data

    // Memory write (§4.3)
    ST_MW_WAIT,        // TW↑ — check /WAIT
    ST_MW_T3,          // T3↑ — emit

    // I/O read (§4.4)
    ST_IR_WAIT,        // TW↑ — auto-wait + additional, check /WAIT
    ST_IR_T3,          // T3↑ — needs_falling for read data

    // I/O write (§4.5)
    ST_IW_WAIT,        // TW↑ — check /WAIT
    ST_IW_T3,          // T3↑ — emit

    // Interrupt acknowledge (§4.6)
    ST_IA_T2,          // T2↑ — nothing, /IORQ goes low at T2↓
    ST_IA_TW1,         // TW1↑ — first auto-wait
    ST_IA_TW2,         // TW2↑ — second auto-wait, check /WAIT
    ST_IA_WAIT,        // TW↑ — additional wait, check /WAIT
    ST_IA_T3,          // T3↑ — needs_falling for vector
    ST_IA_T4,          // T4↑ — sample interrupts, emit
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

    // Set by process_sample when CLK↓ data capture is needed
    bool     needs_falling;
} az;

#define HI_WAIT_BIT   (1u << (PIN_WAIT  - 32))
#define HI_INT_BIT    (1u << (PIN_INT   - 32))
#define HI_NMI_BIT    (1u << (PIN_NMI   - 32))
#define HI_RESET_BIT  (1u << (PIN_RESET - 32))

// ---- SRAM staging buffer (core 1 → PSRAM) ----
#define STAGE_BUF_SIZE  256  // power of 2
#define STAGE_BUF_MASK  (STAGE_BUF_SIZE - 1)

static trace_record_t stage_buf[STAGE_BUF_SIZE];
static uint32_t stage_wr;
static uint32_t stage_rd;

// ---- Emit a completed trace record into SRAM staging buffer ----
// Uses two aligned 32-bit writes instead of individual byte stores.

static void __always_inline emit_record(uint8_t cycle_type) {
    uint8_t flags = 0;
    if (az.halt)        flags |= TRACE_FLAG_HALT;
    if (az.int_pending) flags |= TRACE_FLAG_INT;
    if (az.nmi_pending) flags |= TRACE_FLAG_NMI;

    // Build on stack, then struct-assign (compiler emits two 32-bit stores)
    trace_record_t rec = {
        .address    = az.address,
        .data       = az.data,
        .cycle_type = cycle_type,
        .refresh    = az.refresh,
        .wait_count = az.wait_count,
        .flags      = flags,
        .seq        = az.seq++ & 0x7F,
    };
    stage_buf[stage_wr & STAGE_BUF_MASK] = rec;

    stage_wr++;
    az.nmi_pending = false;
}

// Drain one staged record to PSRAM
static void __not_in_flash_func(stage_drain_one)(void) {
    if (stage_rd != stage_wr) {
        psram_ring[psram_write_idx & PSRAM_RING_MASK] =
            stage_buf[stage_rd & STAGE_BUF_MASK];
        psram_write_idx++;
        stage_rd++;
    }
}

// Drain all remaining staged records
static void __not_in_flash_func(stage_drain_all)(void) {
    while (stage_rd != stage_wr) {
        psram_ring[psram_write_idx & PSRAM_RING_MASK] =
            stage_buf[stage_rd & STAGE_BUF_MASK];
        psram_write_idx++;
        stage_rd++;
    }
}

// ---- Begin a new cycle at T1↑ ----

static void __always_inline begin_cycle(uint32_t sample) {
    az.address    = sample_addr(sample);
    az.data       = 0;
    az.refresh    = 0;
    az.wait_count = 0;
    az.halt       = !(sample & SAMPLE_HALT_BIT);
}

// ---- Process one rising-edge sample ----
// Called during CLK-high phase. Sets az.needs_falling if the state
// requires a falling-edge data capture (handled by the main loop).

static void __always_inline process_sample(uint32_t sample) {

    switch (az.state) {

    // ================================================================
    // IDLE — waiting for T1 rising edge
    // ================================================================
    case ST_IDLE:
        // Check /RESET once per bus cycle
        if (__builtin_expect(!(sio_hw->gpio_hi_in & HI_RESET_BIT), 0)) {
            az.state = ST_RESET;
            return;
        }
        if (!(sample & SAMPLE_M1_BIT)) {
            begin_cycle(sample);
            az.state = ST_M1_T2;
            az.syncing = false;
        } else if (!az.syncing) {
            begin_cycle(sample);
            az.state = ST_IO_T2;
        }
        break;

    // ================================================================
    // RESET
    // ================================================================
    case ST_RESET:
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
    // M1 OPCODE FETCH — T2↑ through T4↑
    // ================================================================
    case ST_M1_T2:
        // T2↑: /MREQ low → opcode fetch, /MREQ high → INTACK
        if (!(sample & SAMPLE_MREQ_BIT)) {
            if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
                az.wait_count++;
                az.state = ST_M1_WAIT;
            } else {
                az.state = ST_M1_T3;
            }
        } else {
            az.wait_count = 2;  // 2 auto-waits for INTACK
            az.state = ST_IA_T2;
        }
        break;

    case ST_M1_WAIT:
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
        } else {
            az.state = ST_M1_T3;
        }
        break;

    case ST_M1_T3:
        // T3↑: capture opcode; request falling edge for refresh address
        az.data = sample_data(sample);
        az.needs_falling = true;  // capture refresh at CLK↓
        // State stays M1_T3 — complete_falling_capture advances to M1_T4
        break;

    case ST_M1_T4: {
        // T4↑: sample interrupts, emit record
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
    // NON-M1 CYCLE TYPE DISCRIMINATION — at T2↑
    // ================================================================
    case ST_IO_T2:
        if (!(sample & SAMPLE_MREQ_BIT)) {
            // Memory cycle: /MREQ low
            if (!(sample & SAMPLE_RD_BIT)) {
                // Memory read — check /WAIT
                if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
                    az.wait_count++;
                    az.state = ST_MR_WAIT;
                } else {
                    az.state = ST_MR_T3;
                }
            } else {
                // Memory write — capture data at T2↑, check /WAIT
                az.data = sample_data(sample);
                if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
                    az.wait_count++;
                    az.state = ST_MW_WAIT;
                } else {
                    az.state = ST_MW_T3;
                }
            }
        } else if (!(sample & SAMPLE_IORQ_BIT)) {
            // I/O cycle: /IORQ low at T2↑
            if (!(sample & SAMPLE_RD_BIT)) {
                az.wait_count = 1;  // 1 auto-wait
                az.state = ST_IR_WAIT;
            } else if (!(sample & SAMPLE_WR_BIT)) {
                az.data = sample_data(sample);
                az.wait_count = 1;  // 1 auto-wait
                az.state = ST_IW_WAIT;
            } else {
                // /IORQ low but neither /RD nor /WR — shouldn't happen
                if (!(sample & SAMPLE_M1_BIT)) {
                    begin_cycle(sample);
                    az.state = ST_M1_T2;
                    az.syncing = false;
                } else {
                    begin_cycle(sample);
                    az.state = ST_IO_T2;
                }
            }
        } else {
            // Neither /MREQ nor /IORQ active — internal operation.
            // Re-interpret this rising edge as a new T1↑.
            if (!(sample & SAMPLE_M1_BIT)) {
                begin_cycle(sample);
                az.state = ST_M1_T2;
                az.syncing = false;
            } else if (!az.syncing) {
                begin_cycle(sample);
                az.state = ST_IO_T2;
            } else {
                az.state = ST_IDLE;
            }
        }
        break;

    // ================================================================
    // MEMORY READ
    // ================================================================
    case ST_MR_WAIT:
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
        } else {
            az.state = ST_MR_T3;
        }
        break;

    case ST_MR_T3:
        // T3↑: capture read data (valid at rising edge due to data hold time)
        az.data = sample_data(sample);
        emit_record(CYCLE_MEM_READ);
        az.state = ST_IDLE;
        break;

    // ================================================================
    // MEMORY WRITE
    // ================================================================
    case ST_MW_WAIT:
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
        } else {
            az.state = ST_MW_T3;
        }
        break;

    case ST_MW_T3:
        emit_record(CYCLE_MEM_WRITE);
        az.state = ST_IDLE;
        break;

    // ================================================================
    // I/O READ
    // ================================================================
    case ST_IR_WAIT:
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
        } else {
            az.state = ST_IR_T3;
        }
        break;

    case ST_IR_T3:
        // T3↑: capture read data (valid at rising edge due to data hold time)
        az.data = sample_data(sample);
        emit_record(CYCLE_IO_READ);
        az.state = ST_IDLE;
        break;

    // ================================================================
    // I/O WRITE
    // ================================================================
    case ST_IW_WAIT:
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
        } else {
            az.state = ST_IW_T3;
        }
        break;

    case ST_IW_T3:
        emit_record(CYCLE_IO_WRITE);
        az.state = ST_IDLE;
        break;

    // ================================================================
    // INTERRUPT ACKNOWLEDGE
    // ================================================================
    case ST_IA_T2:
        az.state = ST_IA_TW1;
        break;

    case ST_IA_TW1:
        az.state = ST_IA_TW2;
        break;

    case ST_IA_TW2:
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
            az.state = ST_IA_WAIT;
        } else {
            az.state = ST_IA_T3;
        }
        break;

    case ST_IA_WAIT:
        if (!(sio_hw->gpio_hi_in & HI_WAIT_BIT)) {
            az.wait_count++;
        } else {
            az.state = ST_IA_T3;
        }
        break;

    case ST_IA_T3:
        // T3↑: capture interrupt vector (valid at rising edge)
        az.data = sample_data(sample);
        az.state = ST_IA_T4;
        break;

    case ST_IA_T4: {
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

// ---- Complete falling-edge data capture ----
// Called during CLK-low phase when az.needs_falling was set.
// Reads the bus via GPIO and finishes the current state's work.

// Only M1_T3 needs falling-edge capture (refresh address).
// All read data is captured at T3↑ (rising edge).
static void __always_inline complete_falling_capture(uint32_t falling) {
    az.refresh = sample_addr(falling) & 0x7F;
    az.state = ST_M1_T4;
    az.needs_falling = false;
}

// ---- Diagnostic mode (bitmask, set by CMD_SET_DIAG_MODE) ----
#define DIAG_SKIP_ANALYZER   (1u << 0)
#define DIAG_SKIP_PSRAM      (1u << 1)
#define DIAG_SKIP_BOTH       (DIAG_SKIP_ANALYZER | DIAG_SKIP_PSRAM)

static volatile uint8_t diag_mode = 0;

// ---- Core 1 entry point ----

// Diagnostic counters (read by core 0)
static volatile uint32_t diag_dma_overflows;    // vestigial, always 0
static volatile uint32_t diag_max_dma_distance;  // vestigial, always 0
static volatile uint32_t diag_max_stage_depth;
static volatile uint32_t diag_total_samples;
static volatile uint32_t diag_wait_asserts;      // vestigial, always 0

static void __not_in_flash_func(core1_main)(void) {
    uint32_t sample_count = 0;

    memset(&az, 0, sizeof(az));
    az.state = ST_IDLE;
    az.syncing = true;
    stage_wr = 0;
    stage_rd = 0;

    while (true) {
        // Handle reset request from core 0
        if (needs_reset) {
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
            needs_reset = false;
        }

        // Wait for capture to be enabled
        if (!capture_running) {
            tight_loop_contents();
            continue;
        }

        // ---- Sync: ensure CLK is low ----
        while (sio_hw->gpio_in & SAMPLE_CLK_BIT) {
            if (!capture_running) goto idle;
        }

        // ---- CLK↑: capture bus + rising-edge processing ----
        while (!(sio_hw->gpio_in & SAMPLE_CLK_BIT)) {
            if (!capture_running) goto idle;
        }
        uint32_t sample = sio_hw->gpio_in;
        sample_count++;

        uint8_t mode = diag_mode;
        if (!(mode & DIAG_SKIP_ANALYZER))
            process_sample(sample);

        // ---- CLK↓: falling-edge capture if needed ----
        bool did_falling = false;
        if (az.needs_falling && !(mode & DIAG_SKIP_ANALYZER)) {
            while (sio_hw->gpio_in & SAMPLE_CLK_BIT) ;
            uint32_t falling = sio_hw->gpio_in;
            complete_falling_capture(falling);
            did_falling = true;
        }

        // ---- PSRAM drain ----
        // Drain one record every other sample. Skip on falling-capture
        // samples (M1_T3 refresh) to stay within cycle budget.
        if (!(mode & DIAG_SKIP_PSRAM)) {
            if (!did_falling && (sample_count & 1) == 0)
                stage_drain_one();
        } else if (!(mode & DIAG_SKIP_ANALYZER)) {
            // Skip-PSRAM mode: discard staged records
            stage_rd = stage_wr;
        }

        // Track staging buffer depth
        uint32_t stage_depth = stage_wr - stage_rd;
        if (stage_depth > diag_max_stage_depth)
            diag_max_stage_depth = stage_depth;

        diag_total_samples = sample_count;
        continue;

    idle:
        // Drain remaining records when capture stops
        stage_drain_all();
        diag_total_samples = sample_count;
    }
}

// ---- Capture start / stop ----

static void capture_start(void) {
    psram_write_idx = 0;

    // Signal core 1 to reset its state
    needs_reset = true;
    while (needs_reset) tight_loop_contents();

    capture_running = true;
}

static void capture_stop(void) {
    capture_running = false;
    // Give core 1 time to drain staged records
    sleep_us(10);
}

// ---- USB helpers ----

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
static uint32_t trigger_idx;
static uint32_t post_trigger_count;
static uint32_t post_trigger_remain;
static uint32_t trigger_chase_idx;

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

static void trigger_eval(void) {
    uint32_t widx = psram_write_idx;

    while (trigger_chase_idx != widx) {
        uint32_t idx = trigger_chase_idx & PSRAM_RING_MASK;
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
    usb_write_bytes(&resp, sizeof(resp));
}

static void cmd_read_buffer(void) {
    uint8_t params[8];
    usb_read_bytes(params, 8);
    uint32_t pre_count  = params[0] | (params[1] << 8) |
                          (params[2] << 16) | (params[3] << 24);
    uint32_t post_count = params[4] | (params[5] << 8) |
                          (params[6] << 16) | (params[7] << 24);

    capture_state_t prev_state = cap_state;
    if (cap_state != CAP_IDLE) {
        capture_stop();
    }

    uint32_t ref_idx;
    bool has_trigger = (prev_state == CAP_TRIGGERED || prev_state == CAP_DONE);
    if (has_trigger) {
        ref_idx = trigger_idx;
    } else {
        ref_idx = psram_write_idx;
    }

    uint32_t widx = psram_write_idx;

    uint32_t available = widx;
    if (available > PSRAM_RING_RECORDS)
        available = PSRAM_RING_RECORDS;

    uint32_t avail_pre = ref_idx - (widx - available);
    if (avail_pre > available) avail_pre = available;
    if (pre_count > avail_pre) pre_count = avail_pre;

    uint32_t avail_post = widx - ref_idx;
    if (avail_post > available) avail_post = 0;
    if (post_count > avail_post) post_count = avail_post;

    uint32_t total = pre_count + post_count;
    uint32_t start_idx = ref_idx - pre_count;

    readout_header_t hdr;
    hdr.magic = READOUT_MAGIC;
    hdr.total_records = total;
    hdr.write_idx = widx;
    hdr.trigger_offset = has_trigger ? pre_count : 0xFFFFFFFF;
    usb_write_bytes(&hdr, sizeof(hdr));

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
    // Overclock to 300 MHz for comfortable margin.
    // At 4 MHz Z80 (4M rising edges/sec), this gives ~75 cycles/sample.
    set_sys_clock_khz(300000, true);

    stdio_init_all();

    size_t psram_size = psram_init();
    (void)psram_size;

    gpio_setup();

    // Launch analyzer on core 1 (no PIO/DMA — core 1 polls GPIO directly)
    multicore_launch_core1(core1_main);

    uint8_t cmd_state = 0;

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

        if (cap_state == CAP_ARMED || cap_state == CAP_TRIGGERED)
            trigger_eval();
    }
}
