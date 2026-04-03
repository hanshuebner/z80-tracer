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
    CYCLE_STATUS     = 7,   // Diagnostic status (flow control / overflow)
} cycle_type_t;

// Status event subtypes (carried in byte 0 bits 2:0 for CYCLE_STATUS packets)
#define STATUS_WAIT_ASSERT  0x01  // /WAIT asserted (count = total pauses)
#define STATUS_WAIT_RELEASE 0x02  // /WAIT released
#define STATUS_DMA_OVERFLOW 0x03  // DMA ring buffer overrun (count = samples lost)
#define STATUS_TRACE_START  0x04  // Trace started (ack for start command)
#define STATUS_TRACE_STOP   0x05  // Trace stopped (ack for stop command)
#define STATUS_FLOW_DISCARD 0x06  // Samples discarded during flow control
#define STATUS_FRAMES_SENT  0x07  // Total trace frames sent (28-bit: seq=high14, count=low14)

// Binary command protocol (client → firmware):
// Byte 0: 0xFF  (sync — no trace packet has type >= 16)
// Byte 1: command code
#define CMD_SYNC          0xFF
#define CMD_TRACE_START   0x01
#define CMD_TRACE_STOP    0x02

// Trace record produced by analyzer state machine (core 1 → core 0)
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

// USB CDC output packet format (binary, bit-7 sync framing):
// Only byte 0 has bit 7 set — scan for it to resync mid-stream.
//
// Normal packets (5 or 6 bytes, cycle_type 0-6):
//   Byte 0:  1TTTT_AAA   bit7=1 (sync), bits 6-3 = cycle_type, bits 2-0 = addr[15:13]
//   Byte 1:  0AAA_AAAA   bits 6-0 = addr[12:6]
//   Byte 2:  0AAA_AAAD   bits 6-1 = addr[5:0], bit 0 = data[7]
//   Byte 3:  0DDD_DDDD   bits 6-0 = data[6:0]
//   Byte 4:  0HWW_WWWW   bit 6 = halt, bits 5-0 = wait_count[5:0]
//   Byte 5 (M1 only): 0RRR_RRRR  bits 6-0 = refresh[6:0]
//
// Status packets (5 bytes, cycle_type 7 = CYCLE_STATUS):
//   Byte 0:  1_0111_SSS  bit7=1, type=7, bits 2-0 = subtype[2:0]
//   Byte 1:  0SSS_SSSS   bits 6-0 = seq[13:7]  (record sequence counter)
//   Byte 2:  0SSS_SSSS   bits 6-0 = seq[6:0]
//   Byte 3:  0CCC_CCCC   bits 6-0 = count[13:7] (event-specific counter)
//   Byte 4:  0CCC_CCCC   bits 6-0 = count[6:0]
//
// Subtypes (in byte 0 bits 2-0):
//   1 = WAIT asserted (backpressure), count = queue level
//   2 = WAIT released, count = queue level
//   3 = DMA ring buffer overrun, count = samples lost
//
// The 14-bit sequence counter increments for every record emitted and wraps.
// Client detects gaps by checking for non-consecutive sequence numbers.
#define USB_PACKET_SIZE 5

// Flow control thresholds (trace record queue entries)
#define TRACE_QUEUE_SIZE            256
#define FLOW_CTRL_ASSERT_THRESHOLD  (TRACE_QUEUE_SIZE * 3 / 4)
#define FLOW_CTRL_RELEASE_THRESHOLD (TRACE_QUEUE_SIZE / 4)

#endif // Z80_TRACE_H
