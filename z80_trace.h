#ifndef Z80_TRACE_H
#define Z80_TRACE_H

#include <stdint.h>

// --- GPIO Pin Assignment (PGA2350) ---
// GPIO 0-15:  Z80 A0-A15  (address bus)
// GPIO 16-23: Z80 D0-D7   (data bus)
// GPIO 24:    /M1          (machine cycle 1 - opcode fetch)
// GPIO 25:    /MREQ        (memory request)
// GPIO 26:    /IORQ        (IO request)
// GPIO 27:    /RD          (read strobe)
// GPIO 28:    /WR          (write strobe)
// GPIO 29:    /RFSH        (refresh - used to filter DRAM refresh cycles)
// GPIO 30:    CLK          (Z80 clock)
// GPIO 31:    /HALT        (halt status, directly monitored)
// GPIO 32:    /WAIT        (active low OUTPUT - directly driven to throttle Z80)
// GPIO 33:    /INT         (interrupt request, directly monitored)
// GPIO 34:    /NMI         (non-maskable interrupt, directly monitored)
// GPIO 35:    /RESET       (reset, directly monitored)

#define PIN_ADDR_BASE   0   // A0 = GPIO 0, A15 = GPIO 15
#define PIN_DATA_BASE   16  // D0 = GPIO 16, D7 = GPIO 23
#define PIN_M1          24
#define PIN_MREQ        25
#define PIN_IORQ        26
#define PIN_RD          27
#define PIN_WR          28
#define PIN_RFSH        29
#define PIN_CLK         30
#define PIN_HALT        31
#define PIN_WAIT        32  // output
#define PIN_INT         33
#define PIN_NMI         34
#define PIN_RESET       35

#define PIN_IN_BASE     0   // PIO in_base: GPIO 0
#define PIN_IN_COUNT    31  // capture GPIO 0-30 (31 pins)

// --- Captured bus sample format (32-bit word from PIO) ---
// Bits 0-15:  address (A0-A15)
// Bits 16-23: data (D0-D7)
// Bit  24:    /M1
// Bit  25:    /MREQ
// Bit  26:    /IORQ
// Bit  27:    /RD
// Bit  28:    /WR
// Bit  29:    /RFSH
// Bit  30:    CLK

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

// Cycle type classification (derived from control signals, active-low inverted)
typedef enum {
    CYCLE_OPCODE_FETCH,   // /M1=0, /MREQ=0, /RD=0
    CYCLE_MEM_READ,       // /M1=1, /MREQ=0, /RD=0
    CYCLE_MEM_WRITE,      // /M1=1, /MREQ=0, /WR=0
    CYCLE_IO_READ,        // /IORQ=0, /RD=0
    CYCLE_IO_WRITE,       // /IORQ=0, /WR=0
    CYCLE_UNKNOWN
} cycle_type_t;

static inline cycle_type_t classify_sample(uint32_t sample) {
    // Control signals are active-low: bit=0 means active
    int m1   = !(sample & SAMPLE_M1_BIT);
    int mreq = !(sample & SAMPLE_MREQ_BIT);
    int iorq = !(sample & SAMPLE_IORQ_BIT);
    int rd   = !(sample & SAMPLE_RD_BIT);
    int wr   = !(sample & SAMPLE_WR_BIT);

    if (m1 && mreq && rd)  return CYCLE_OPCODE_FETCH;
    if (mreq && rd)        return CYCLE_MEM_READ;
    if (mreq && wr)        return CYCLE_MEM_WRITE;
    if (iorq && rd)        return CYCLE_IO_READ;
    if (iorq && wr)        return CYCLE_IO_WRITE;
    return CYCLE_UNKNOWN;
}

static inline uint16_t sample_addr(uint32_t sample) {
    return sample & SAMPLE_ADDR_MASK;
}

static inline uint8_t sample_data(uint32_t sample) {
    return (sample & SAMPLE_DATA_MASK) >> SAMPLE_DATA_SHIFT;
}

// --- Ring buffer for DMA capture ---
// Must be power-of-2 sized and aligned for DMA ring mode
#define CAPTURE_BUF_SIZE_WORDS  (32 * 1024)  // 32K entries = 128KB
#define CAPTURE_BUF_SIZE_BYTES  (CAPTURE_BUF_SIZE_WORDS * sizeof(uint32_t))

// USB CDC output packet format (binary):
//   Byte 0:    cycle type (cycle_type_t)
//   Byte 1-2:  address (little-endian)
//   Byte 3:    data
#define USB_PACKET_SIZE 4

// Flow control thresholds (in ring buffer entries)
#define FLOW_CTRL_ASSERT_THRESHOLD   (CAPTURE_BUF_SIZE_WORDS * 3 / 4)
#define FLOW_CTRL_RELEASE_THRESHOLD  (CAPTURE_BUF_SIZE_WORDS / 4)

#endif // Z80_TRACE_H
