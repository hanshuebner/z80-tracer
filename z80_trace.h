#ifndef Z80_TRACE_H
#define Z80_TRACE_H

#include <stdint.h>
#include <stdbool.h>

// --- GPIO Pin Assignment (PGA2350) ---
// GPIO 0-15:  Z80 A0-A15  (address bus)
// GPIO 16-23: Z80 D0-D7   (data bus)
// GPIO 24:    /M1          (machine cycle 1 - opcode fetch)
// GPIO 25:    /MREQ        (memory request)
// GPIO 26:    /IORQ        (IO request)
// GPIO 27:    /RD          (read strobe)
// GPIO 28:    /WR          (write strobe)
// GPIO 29:    /RFSH        (refresh)
// GPIO 30:    CLK          (Z80 clock)
// GPIO 31:    /HALT        (halt status)
// GPIO 32:    /WAIT        (active low - directly driven for flow control, also observed)
// GPIO 33:    /INT         (interrupt request)
// GPIO 34:    /NMI         (non-maskable interrupt)
// GPIO 35:    /RESET       (reset)

#define PIN_ADDR_BASE   0
#define PIN_DATA_BASE   16
#define PIN_M1          24
#define PIN_MREQ        25
#define PIN_IORQ        26
#define PIN_RD          27
#define PIN_WR          28
#define PIN_RFSH        29
#define PIN_CLK         30
#define PIN_HALT        31
#define PIN_WAIT        32  // output (flow control) + input (observe)
#define PIN_INT         33
#define PIN_NMI         34
#define PIN_RESET       35

#define PIN_IN_BASE     0   // PIO in_base: GPIO 0
#define PIN_IN_COUNT    32  // capture GPIO 0-31 (32 pins)

// --- Captured bus sample format (32-bit word from PIO) ---
// Bits 0-15:  address (A0-A15)
// Bits 16-23: data (D0-D7)
// Bit  24:    /M1
// Bit  25:    /MREQ
// Bit  26:    /IORQ
// Bit  27:    /RD
// Bit  28:    /WR
// Bit  29:    /RFSH
// Bit  30:    CLK  (1 = sample taken at rising edge, 0 = falling edge)
// Bit  31:    /HALT

#define SAMPLE_ADDR_MASK    0x0000FFFF
#define SAMPLE_DATA_SHIFT   16
#define SAMPLE_DATA_MASK    0x00FF0000
#define SAMPLE_M1_BIT       (1u << 24)
#define SAMPLE_MREQ_BIT     (1u << 25)
#define SAMPLE_IORQ_BIT     (1u << 26)
#define SAMPLE_RD_BIT       (1u << 27)
#define SAMPLE_WR_BIT       (1u << 28)
#define SAMPLE_RFSH_BIT     (1u << 29)
#define SAMPLE_CLK_BIT      (1u << 30)
#define SAMPLE_HALT_BIT     (1u << 31)

// Cycle type classification (produced by analyzer state machine)
typedef enum {
    CYCLE_M1_FETCH   = 0,   // M1 opcode fetch
    CYCLE_MEM_READ   = 1,   // Memory read
    CYCLE_MEM_WRITE  = 2,   // Memory write
    CYCLE_IO_READ    = 3,   // I/O read
    CYCLE_IO_WRITE   = 4,   // I/O write
    CYCLE_INT_ACK    = 5,   // Interrupt acknowledge
    CYCLE_RESET      = 6,   // Reset (emitted when /RESET releases)
    CYCLE_DATA_LOSS  = 7,   // Data loss marker (addr = lost count, data = loss cause)
} cycle_type_t;

// Data loss cause codes (stored in trace_record_t.data for CYCLE_DATA_LOSS)
#define LOSS_DMA_OVERFLOW    1  // DMA ring buffer overrun — samples lost
#define LOSS_PIO_OVERFLOW    2  // PIO FIFO stall — samples lost
#define LOSS_STAGING_OVERFLOW 3 // Staging buffer full — records lost
#define LOSS_POLL_TIMEOUT    4  // GPIO poll timed out — CLK stuck or too slow

// Trace record produced by analyzer state machine (core 1 → PSRAM ring buffer)
typedef struct {
    uint16_t address;       // A[15:0] captured at T1 rising edge
    uint8_t  data;          // D[7:0] captured at cycle-specific edge
    uint8_t  cycle_type;    // cycle_type_t
    uint8_t  refresh;       // A[6:0] refresh address (M1_FETCH only)
    uint8_t  wait_count;    // number of wait states (includes auto-waits for I/O)
    uint8_t  flags;         // TRACE_FLAG_*
    uint8_t  seq;           // Sequence counter (7-bit, wraps), for gap detection
} trace_record_t;

#define TRACE_FLAG_HALT  (1u << 0)  // CPU is halted
#define TRACE_FLAG_INT   (1u << 1)  // /INT was active at end of cycle
#define TRACE_FLAG_NMI   (1u << 2)  // /NMI falling edge detected

// --- Helpers ---

static inline uint16_t sample_addr(uint32_t sample) {
    return sample & SAMPLE_ADDR_MASK;
}

static inline uint8_t sample_data(uint32_t sample) {
    return (sample & SAMPLE_DATA_MASK) >> SAMPLE_DATA_SHIFT;
}

// --- Ring buffer for DMA capture ---
// Must be power-of-2 sized and aligned for DMA ring mode.
#define CAPTURE_BUF_RING_BITS   15            // log2(32KB) = 15 — max DMA ring size
#define CAPTURE_BUF_SIZE_BYTES  (1u << CAPTURE_BUF_RING_BITS)
#define CAPTURE_BUF_SIZE_WORDS  (CAPTURE_BUF_SIZE_BYTES / sizeof(uint32_t))

// --- PSRAM trace ring buffer ---
// 8 MB PSRAM / 8 bytes per record = 1M records, power of 2
#define PSRAM_RING_RECORDS      (1u << 20)    // 1,048,576 records
#define PSRAM_RING_MASK         (PSRAM_RING_RECORDS - 1)

// --- Binary command protocol (host → firmware) ---
// Byte 0: 0xFF (sync — distinguishes commands from stale data)
// Byte 1: command code
// Byte 2+: command-specific payload (if any)

#define CMD_SYNC            0xFF

#define CMD_START_CAPTURE   0x01  // Start PIO/DMA, enter CAP_RUNNING
#define CMD_STOP_CAPTURE    0x02  // Stop PIO/DMA, enter CAP_IDLE
#define CMD_READ_BUFFER     0x03  // + 4B pre_count (LE) + 4B post_count (LE)
                                  //   Suspends capture, sends records, resumes
#define CMD_GET_STATUS      0x04  // Query capture state + counters
#define CMD_SET_DIAG_MODE   0x05  // + 1 byte: bitmask of disabled components
#define CMD_SET_TRIGGER     0x10  // + 7 bytes trigger_t payload
#define CMD_CLEAR_TRIGGERS  0x11  // Clear all triggers
#define CMD_ARM_TRIGGER     0x12  // + 4B post_trigger_count (LE); enter CAP_ARMED

// --- Trigger types ---

typedef enum {
    TRIG_NONE       = 0,
    TRIG_PC_MATCH   = 1,  // M1 fetch at address/range
    TRIG_MEM_READ   = 2,  // Memory read at address, optional value match
    TRIG_MEM_WRITE  = 3,  // Memory write at address, optional value match
    TRIG_IO_READ    = 4,  // IO read at port, optional value match
    TRIG_IO_WRITE   = 5,  // IO write at port, optional value match
    TRIG_INT_ACK    = 6,  // Interrupt acknowledge
} trigger_type_t;

typedef struct {
    uint8_t  type;       // trigger_type_t
    uint16_t addr_lo;    // address/port match low bound
    uint16_t addr_hi;    // address/port match high bound (= addr_lo for exact)
    uint8_t  data_val;   // data value to match
    uint8_t  data_mask;  // 0x00 = don't check data, 0xFF = exact match
} __attribute__((packed)) trigger_t;

#define MAX_TRIGGERS 8

// --- Capture state machine ---

typedef enum {
    CAP_IDLE      = 0,  // PIO/DMA stopped
    CAP_RUNNING   = 1,  // Capturing, no triggers armed
    CAP_ARMED     = 2,  // Capturing, core 0 checking triggers
    CAP_TRIGGERED = 3,  // Trigger fired, collecting post-trigger records
    CAP_DONE      = 4,  // Post-trigger complete, awaiting readout
} capture_state_t;

// --- Capture buffer readout header (firmware → host) ---
// Sent at the start of a CMD_READ_BUFFER response.
#define READOUT_MAGIC       0x5A383054u  // "Z80T"

typedef struct {
    uint32_t magic;           // READOUT_MAGIC
    uint32_t total_records;   // number of trace_record_t following this header
    uint32_t write_idx;       // psram_write_idx at time of readout
    uint32_t trigger_offset;  // index of trigger record within transfer (0xFFFFFFFF if none)
} readout_header_t;

// --- Status response (firmware → host for CMD_GET_STATUS) ---
#define STATUS_MAGIC        0x53544154u  // "STAT"

typedef struct {
    uint32_t magic;           // STATUS_MAGIC
    uint32_t capture_state;   // capture_state_t
    uint32_t write_idx;       // current psram_write_idx
    uint32_t dma_overflows;   // DMA ring buffer overrun count
    uint32_t max_dma_distance; // worst DMA backlog (samples)
    uint32_t max_stage_depth;  // worst staging buffer depth (records)
    uint32_t total_samples;    // total PIO samples processed
    uint32_t cpu_clock_khz;    // actual CPU clock in kHz
    uint32_t trigger_idx;      // psram_write_idx when trigger fired (valid in TRIGGERED/DONE)
    uint32_t trigger_count;    // number of configured triggers
    uint32_t wait_asserts;     // /WAIT assertion count
    // Performance measurement (ARM cycle counts per process_sample call)
    uint32_t sample_max_cycles; // worst-case cycles for any process_sample
    uint32_t sample_min_cycles; // best-case cycles (0xFFFFFFFF if no samples)
    // Data loss counters
    uint32_t pio_overflows;    // PIO FIFO stall count (samples lost at source)
    uint32_t staging_overflows; // staging buffer overrun count (records dropped)
    uint32_t poll_timeouts;    // GPIO poll timeout count (CLK stuck/too slow)
    uint32_t data_loss_records; // CYCLE_DATA_LOSS records emitted into trace
} status_response_t;

#endif // Z80_TRACE_H
