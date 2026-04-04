# Z80 Bus Analyzer — State Machine Description

## Reference: Zilog Z80 CPU User Manual UM008011-0816, pp. 7–17

---

## 1. Monitored Signals

| Signal    | Dir | Active | Sampling Notes                                |
|-----------|-----|--------|-----------------------------------------------|
| CLK       | in  | —      | State transitions keyed to CLK rising edges   |
| A15–A0    | out | High   | Active during T1                              |
| D7–D0     | bi  | High   | Sampled at rising edges (T2↑ or T3↑)          |
| /M1       | out | Low    | Identifies opcode fetch or interrupt ack      |
| /MREQ     | out | Low    | Memory access (fetch, read, write, refresh)   |
| /IORQ     | out | Low    | I/O access or interrupt acknowledge           |
| /RD       | out | Low    | CPU wants to read                             |
| /WR       | out | Low    | CPU data bus holds valid write data           |
| /RFSH     | out | Low    | Used to detect M1 T4 (refresh phase)          |
| /HALT     | out | Low    | CPU is in HALT state (executing NOPs)         |
| /WAIT     | in  | Low    | Peripheral requests wait states               |
| /INT      | in  | Low    | Maskable interrupt request                    |
| /NMI      | in  | Low    | Nonmaskable interrupt (edge-triggered)        |
| /RESET    | in  | Low    | System reset                                 |

Signals NOT monitored by the PIO (not in GPIO 0–31 capture window):
- /BUSREQ, /BUSACK: not connected
- /WAIT, /INT, /NMI, /RESET: on GPIO 32–35, read via ARM `sio_hw->gpio_hi_in`

---

## 2. State Machine Overview

The analyzer tracks two levels of state:

- **Cycle type**: Which kind of machine cycle is in progress
- **T-state**: Which clock period within that cycle

### Rising-Edge-Only Architecture

All state transitions are driven by **CLK rising edges only**, sampled by a
PIO state machine. The PIO captures GPIO 0–31 (32 bits) at each rising edge
and autopushes to a FIFO. DMA drains the FIFO into a 32 KB ring buffer in
SRAM. Core 1 processes samples from this buffer.

**No falling-edge polling is performed.** All data capture (opcodes, read
data, write data) uses the PIO rising-edge samples. T-state boundaries and
wait states are detected from control signal transitions between consecutive
rising-edge samples:

- **M1/INT ACK cycles**: T4 is detected by /RFSH LOW in the PIO sample.
  /RFSH asserts ~60 ns after T3↑, so it is reliably visible in the T4↑
  sample (captured 250 ns later). Samples between T2 and T4 with /RFSH
  HIGH are wait states (TW) or T3.

- **Non-M1 cycles (MEM_RD, MEM_WR, IO_RD, IO_WR)**: Cycle end is detected
  when /MREQ (or /IORQ) goes HIGH. These signals deassert at T3↑ but with
  ~85 ns propagation delay, so the T3↑ PIO sample still sees them LOW. At
  the next T1↑ (250 ns later), they are HIGH. The current sample is then
  re-dispatched as T1 of the new cycle.

**Data capture timing**: /RD (and for I/O cycles, /IORQ) is asserted well
before T3↑, giving peripherals a full T-state or more to drive the data
bus. The CMOS Z80 specifies ≥20 ns data setup time before T3↑. The PIO
captures at CLK↑, when /RD is still active and data is valid. This is the
same mechanism already used for M1 opcode fetch (data captured at T3↑ from
the PIO sample). All read cycles use this approach uniformly.

**Refresh address is not tracked.** The R register value can be computed by
the host client from the initial state plus the M1 fetch count.

### Data flow: PSRAM buffering, not USB streaming

Trace records are written to the PGA2350's **PSRAM ring buffer** during
capture, not streamed over USB. USB bandwidth is insufficient for sustained
4 MHz capture. The host client connects after capture to download the trace
buffer for visualization and analysis.

### Triggers are firmware-resident

Trigger conditions (start/stop capture, address match, data match, etc.)
are evaluated **in the tracer firmware**, not by the host client. The client
is used only for post-capture visualization and analysis. This eliminates
USB round-trip latency from the trigger path.

### Cycle type discrimination at T2↑ (not T1↓)

In the original Z80 timing, cycle type identification partially depends on
signals that change at T1↓ (/MREQ for memory cycles). However, these signals
remain asserted through T2↑, so the analyzer defers cycle type discrimination
to T2↑ where all relevant signals (/MREQ, /IORQ, /RD, /WR) have settled.
This eliminates the need to process T1 falling edges.

---

## 3. Cycle Type Identification

At the **start of T1** (rising edge of CLK entering T1), the analyzer captures
the address bus and checks /M1 to determine the cycle class.

At **T2↑**, all control signals have settled, and the analyzer determines the
exact cycle type.

### Phase 1 — T1 rising edge: Check /M1

| /M1 at T1↑  | Meaning                                         |
|-------------|-------------------------------------------------|
| Low         | This is an M1 cycle (opcode fetch or INT ack)   |
| High        | Non-M1 cycle (mem r/w, I/O r/w)                 |

### Phase 2 — T2 rising edge: Determine exact cycle type

For **M1 cycles** (/M1 low at T1↑):

| /MREQ at T2↑ | Cycle Type                    | Rationale                         |
|--------------|-------------------------------|-----------------------------------|
| Low          | **OPCODE FETCH**              | /MREQ went low at T1↓, still low |
| High         | **INTERRUPT ACKNOWLEDGE**     | /MREQ never asserts; /IORQ later |

For **non-M1 cycles** (/M1 high at T1↑):

| /MREQ | /IORQ | /RD  | /WR  | Cycle Type          | Rationale                     |
|--------|-------|------|------|---------------------|-------------------------------|
| Low    | High  | Low  | High | **MEMORY READ**     | /MREQ, /RD went low at T1↓   |
| Low    | High  | High | Low  | **MEMORY WRITE**    | /MREQ at T1↓, /WR at T2↑     |
| High   | Low   | Low  | High | **I/O READ**        | /IORQ, /RD go low at T2↑     |
| High   | Low   | High | Low  | **I/O WRITE**       | /IORQ, /WR go low at T2↑     |

Note: For memory write, /WR goes active early in T2 (before T2↑ per manual
Fig. 6), so it is visible at T2↑. For I/O, /IORQ goes active at T2↑ (Fig. 7).

### Special states (detected at any rising edge):

| Condition                  | State                | Ref       |
|----------------------------|----------------------|-----------|
| /HALT low + M1 cycling     | **HALT** (NOP fetch) | Fig. 11   |
| /RESET low                 | **RESET**            | p. 6      |

---

## 4. Cycle State Machines

Each cycle type below is a sub-state-machine. All state transitions and data
captures occur at **CLK rising edges** using PIO samples from the DMA buffer.
No falling-edge GPIO polling is performed.

### 4.1 OPCODE FETCH (M1 cycle) — Ref: Figure 5

```
Entry: /M1 low at T1↑, /MREQ low at T2↑ (confirms fetch, not INT ACK)

         ┌──────────────────────────────────────────────────┐
         │                  M1 Cycle                        │
         │                                                  │
  T1 ↑   │  /M1=low, Address bus = PC                       │
         │  ► CAPTURE address (this is the PC)              │
         │                                                  │
  T2 ↑   │  /MREQ=low, /RD=low (confirms opcode fetch)     │
         │  → enter M1_ACTIVE state                         │
         │                                                  │
  TW ↑   │  (wait state — may repeat, 0 or more)            │
         │  /RFSH=high in PIO sample → still in TW or T3   │
         │  ► CAPTURE data bus (overwritten each sample;    │
         │    the last capture before T4 is the opcode)     │
         │  wait_count++                                    │
         │                                                  │
  T3 ↑   │  Same as TW from the analyzer's perspective:     │
         │  /RFSH still high (asserts ~60ns AFTER T3↑)     │
         │  ► CAPTURE data bus (this IS the opcode)         │
         │  wait_count++                                    │
         │                                                  │
  T4 ↑   │  /RFSH=low in PIO sample (asserted ~190ns ago)  │
         │  wait_count-- (undo T3 count — T3 is not a wait) │
         │  ► SAMPLE /INT, /NMI via gpio_hi_in              │
         │  ► EMIT record                                   │
         │  → IDLE                                          │
         └──────────────────────────────────────────────────┘

Note: /RFSH goes low ~60ns after T3↑ (Z80 propagation delay). The PIO
captures ~3ns after CLK↑, so /RFSH is NOT yet low in the T3 sample. But
at T4↑ (250ns after T3↑), /RFSH has been low for ~190ns — reliably
captured. This is the T3/T4 discriminator.

Capture record:
  { type: M1_FETCH, addr: A[15:0]@T1↑, data: D[7:0]@T3↑,
    waits: count_of_TW, halt: /HALT level }
```

### 4.2 MEMORY READ — Ref: Figure 6 (left half)

```
Entry: /M1 high at T1↑, /MREQ low + /RD low at T2↑

         ┌──────────────────────────────────────────────────┐
         │              Memory Read Cycle                   │
         │                                                  │
  T1 ↑   │  Address bus = memory address                    │
         │  ► CAPTURE address                               │
         │                                                  │
  T2 ↑   │  /MREQ=low, /RD=low (confirms memory read)      │
         │  → enter MR_ACTIVE state                         │
         │                                                  │
  TW ↑   │  (wait state — may repeat, 0 or more)            │
         │  /MREQ still low in PIO sample                   │
         │  ► CAPTURE data bus (overwritten each sample)    │
         │  wait_count++                                    │
         │                                                  │
  T3 ↑   │  /MREQ still low in PIO sample (deasserts ~85ns │
         │  after T3↑, but PIO captures at ~3ns)            │
         │  ► CAPTURE data bus (this IS the read data)      │
         │  wait_count++                                    │
         │                                                  │
  next   │  /MREQ=high → cycle ended                        │
  T1 ↑   │  wait_count-- (T3 was not a wait state)          │
         │  ► EMIT record                                   │
         │  ► RE-DISPATCH this sample as T1 of next cycle   │
         └──────────────────────────────────────────────────┘

Data capture timing: /RD has been active since T1↓ (memory read). By T3↑,
the data bus has had a full T-state to settle. The CMOS Z80 specifies ≥20ns
data setup before T3↑. Data is captured from the PIO sample at T3↑, the
same mechanism used for M1 opcode fetch.

Cycle end detection: /MREQ deasserts at T3↑ with ~85ns propagation delay.
At the NEXT rising edge (T1↑ of new cycle, 250ns after T3↑), /MREQ is HIGH.
The analyzer detects this and re-dispatches the sample as T1.

Capture record:
  { type: MEM_RD, addr: A[15:0]@T1↑, data: D[7:0]@T3↑,
    waits: count_of_TW }
```

### 4.3 MEMORY WRITE — Ref: Figure 6 (right half)

```
Entry: /M1 high at T1↑, /MREQ low + /WR low at T2↑

         ┌──────────────────────────────────────────────────┐
         │              Memory Write Cycle                  │
         │                                                  │
  T1 ↑   │  Address bus = memory address                    │
         │  ► CAPTURE address                               │
         │                                                  │
  T2 ↑   │  /MREQ=low, /WR=low (confirms memory write)     │
         │  ► CAPTURE data bus (write data — stable by T2↑) │
         │  → enter MW_ACTIVE state                         │
         │                                                  │
  TW ↑   │  (wait state — may repeat, 0 or more)            │
         │  /MREQ still low → wait_count++                  │
         │                                                  │
  T3 ↑   │  /MREQ still low → wait_count++                  │
         │                                                  │
  next   │  /MREQ=high → cycle ended                        │
  T1 ↑   │  wait_count--                                    │
         │  ► EMIT record                                   │
         │  ► RE-DISPATCH as T1 of next cycle               │
         └──────────────────────────────────────────────────┘

Capture record:
  { type: MEM_WR, addr: A[15:0]@T1↑, data: D[7:0]@T2↑,
    waits: count_of_TW }
```

### 4.4 I/O READ — Ref: Figure 7 (upper half)

```
Entry: /M1 high at T1↑, /IORQ low + /RD low at T2↑

         ┌──────────────────────────────────────────────────┐
         │              I/O Read Cycle                      │
         │                                                  │
  T1 ↑   │  Address bus = port address                      │
         │  (A[7:0] = port, A[15:8] = register content)     │
         │  ► CAPTURE address                               │
         │                                                  │
  T2 ↑   │  /IORQ=low, /RD=low (confirms I/O read)         │
         │  → enter IR_ACTIVE state                         │
         │                                                  │
  TW* ↑  │  (automatic wait — always present for I/O)       │
         │  /IORQ still low → CAPTURE data bus, wait_count++│
         │                                                  │
  TW ↑   │  (additional waits if /WAIT held low — 0 or more)│
         │  /IORQ still low → CAPTURE data bus, wait_count++│
         │                                                  │
  T3 ↑   │  /IORQ still low (deasserts ~85ns after T3↑)    │
         │  ► CAPTURE data bus (this IS the read data)      │
         │  wait_count++                                    │
         │                                                  │
  next   │  /IORQ=high → cycle ended                        │
  T1 ↑   │  wait_count--                                    │
         │  ► EMIT record                                   │
         │  ► RE-DISPATCH as T1 of next cycle               │
         └──────────────────────────────────────────────────┘

IMPORTANT: The cycle is T1–T2–TW*–T3 minimum (4 CLK periods). TW* is always
inserted for I/O. The auto-wait and any additional waits are counted naturally
by the ACTIVE state — each sample with /IORQ still LOW increments wait_count,
and T3 is subtracted at the end.

Capture record:
  { type: IO_RD, addr: A[15:0]@T1↑, data: D[7:0]@T3↑,
    waits: 1 + count_of_additional_TW }
```

### 4.5 I/O WRITE — Ref: Figure 7 (lower half)

```
Entry: /M1 high at T1↑, /IORQ low + /WR low at T2↑

         ┌──────────────────────────────────────────────────┐
         │              I/O Write Cycle                     │
         │                                                  │
  T1 ↑   │  Address bus = port address                      │
         │  ► CAPTURE address                               │
         │                                                  │
  T2 ↑   │  /IORQ=low, /WR=low (confirms I/O write)        │
         │  ► CAPTURE data bus (write data — stable)        │
         │  → enter IW_ACTIVE state                         │
         │                                                  │
  TW* ↑  │  (automatic wait — always present)               │
         │  /IORQ still low → wait_count++                  │
         │                                                  │
  TW ↑   │  (additional waits if needed)                    │
         │  /IORQ still low → wait_count++                  │
         │                                                  │
  T3 ↑   │  /IORQ still low → wait_count++                  │
         │                                                  │
  next   │  /IORQ=high → cycle ended                        │
  T1 ↑   │  wait_count--                                    │
         │  ► EMIT record                                   │
         │  ► RE-DISPATCH as T1 of next cycle               │
         └──────────────────────────────────────────────────┘

Capture record:
  { type: IO_WR, addr: A[15:0]@T1↑, data: D[7:0]@T2↑,
    waits: 1 + count_of_additional_TW }
```

### 4.6 INTERRUPT ACKNOWLEDGE — Ref: Figure 9

```
Entry: /M1 low at T1↑, /MREQ HIGH at T2↑ (distinguishes from fetch)

         ┌──────────────────────────────────────────────────┐
         │          Interrupt Acknowledge Cycle             │
         │                                                  │
  T1 ↑   │  /M1 ↓, Address bus = PC (but not used)          │
         │  ► CAPTURE address (PC at time of interrupt)     │
         │                                                  │
  T2 ↑   │  /MREQ still HIGH (confirms INT ACK, not fetch)  │
         │  → enter M1_ACTIVE state (same as opcode fetch)  │
         │                                                  │
  TW1*↑  │  First auto-wait. /RFSH=high → wait_count++      │
         │  ► CAPTURE data bus                              │
         │                                                  │
  TW2*↑  │  Second auto-wait. /RFSH=high → wait_count++     │
         │  ► CAPTURE data bus                              │
         │                                                  │
  TW ↑   │  (additional waits if /WAIT held — 0 or more)    │
         │  /RFSH=high → CAPTURE data bus, wait_count++     │
         │                                                  │
  T3 ↑   │  /RFSH still high → CAPTURE data bus (vector!)   │
         │  wait_count++                                    │
         │                                                  │
  T4 ↑   │  /RFSH=low → T4 detected (same as M1 fetch)     │
         │  wait_count--                                    │
         │  ► SAMPLE /INT, /NMI                             │
         │  ► EMIT record                                   │
         │  → IDLE                                          │
         └──────────────────────────────────────────────────┘

INT ACK uses the same M1_ACTIVE state as opcode fetch — the /RFSH-based
T4 detection works identically. The 2 auto-waits are counted naturally.

Capture record:
  { type: INT_ACK, addr: A[15:0]@T1↑, data: D[7:0]@T3↑,
    waits: 2 + count_of_additional_TW }
```

### 4.7 NONMASKABLE INTERRUPT RESPONSE — Ref: Figure 10

```
The NMI response is NOT a special bus cycle — it is a normal
M1 opcode fetch. The CPU:
  1. Performs a normal M1 fetch (but ignores the fetched byte)
  2. Pushes PC onto the stack (two memory write cycles)
  3. Sets PC = 0066h

From the analyzer's perspective, the NMI response looks like:
  M1 FETCH → MEM_WR (push PCH) → MEM_WR (push PCL)

The analyzer detects /NMI falling edge (sampled at M1/INTACK T4↑ via
gpio_hi_in) and flags the subsequent M1 as:
  { type: M1_FETCH, addr: ..., data: ..., flags: NMI }
```

### 4.8 HALT STATE — Ref: Figures 11, 12

```
During HALT:
  - /HALT output goes low
  - CPU repeatedly executes M1 fetch cycles (reading NOPs or
    whatever is in memory, but forcing NOP internally)
  - /INT and /NMI sampled at each T4↑

From the analyzer's perspective, HALT looks like repeated M1
fetch cycles with /HALT=low. The analyzer flags these:
  { type: M1_FETCH, addr: ..., data: ..., halt: true }

The PC does NOT increment during HALT — the same address
repeats for each M1 fetch.

Exit: /INT (if IFF1 set) or /NMI triggers exit at T4↑.
The next cycle is the interrupt acknowledge or NMI response.
```

### 4.9 RESET — Ref: p. 6

```
When /RESET is low:
  - Address bus, data bus → Hi-Z
  - All control outputs → inactive (high)
  - /RESET must be held low for minimum 3 full CLK cycles

The analyzer detects /RESET=low (via gpio_hi_in) and enters RESET state.
On /RESET returning high, the CPU begins execution:
  - PC = 0000h
  - I = 00h, R = 00h
  - Interrupt mode 0
  - IFF1 = IFF2 = 0 (interrupts disabled)

  { type: RESET }
```

---

## 5. Unified Analyzer State Machine

```
                          ┌────────┐
                /RESET=low│ RESET  │
               ┌─────────►│        │
               │          └───┬────┘
               │   /RESET=high│
               │              ▼
               │          ┌────────┐
               │          │  IDLE  │
               │          │(T1 ↑)  │
               │          └───┬────┘
               │              │
               │     Examine signals at T1↑
               │              │
               │    ┌─────────┼───────────┐
               │    │         │           │
               │    ▼         ▼           ▼
               │  /M1=low   /M1=high    /M1=high
               │    │         │           │
               │    │    ┌────┴────┐      │
               │    │    │   T2↑   │      │  T2↑
               │    │    │         │      │
               │    │  /MREQ=low  /MREQ=high
               │    │    │         │
               │    │  ┌─┴──┐   ┌──┴──┐
               │    │ /RD  /WR /RD   /WR
               │    │  ↓    ↓   ↓     ↓
               │    │ MR   MW  IR    IW
               │    │ ACT  ACT ACT   ACT
               │    │
               │    ├── T2↑: /MREQ low?
               │    │     Yes → M1_ACTIVE (opcode fetch)
               │    │     No  → M1_ACTIVE (int ack)
               │    │
               │    ▼
               │  ┌────────────────────────────┐
               │  │   M1_ACTIVE                │
               │  │ (opcode fetch or int ack)   │
               │  │ loops until /RFSH LOW → T4 │
               │  └────────────────────────────┘
               │
               └──── (can interrupt any state)
```

### State list

| State          | Purpose                                              |
|----------------|------------------------------------------------------|
| ST_IDLE        | Expecting T1↑. Checks /RESET, dispatches by /M1.    |
| ST_RESET       | /RESET active, waiting for release.                  |
| ST_T2_M1       | T2↑ of M1 cycle — discriminate fetch vs INT ACK.     |
| ST_T2_NONM1   | T2↑ of non-M1 — discriminate MR/MW/IR/IW.            |
| ST_M1_ACTIVE   | M1/INTACK: loops TW/T3, detects T4 by /RFSH LOW.    |
| ST_MR_ACTIVE   | Memory read: captures data, detects end by /MREQ HIGH.|
| ST_MW_ACTIVE   | Memory write: detects end by /MREQ HIGH.             |
| ST_IR_ACTIVE   | I/O read: captures data, detects end by /IORQ HIGH.  |
| ST_IW_ACTIVE   | I/O write: detects end by /IORQ HIGH.                |

10 states total.

---

## 6. CLK Rising Edge Actions Summary

| State          | Action                                                     |
|----------------|------------------------------------------------------------|
| IDLE           | Capture A[15:0]. Check /M1. Begin cycle.                   |
| T2_M1          | Check /MREQ → set cycle type (fetch or int ack). → M1_ACTIVE |
| T2_NONM1       | Check /MREQ, /IORQ, /RD, /WR → dispatch to ACTIVE state. Capture write data at T2↑ for MW/IW. |
| M1_ACTIVE      | /RFSH LOW? → T4: emit record, → IDLE. Else: capture D[7:0], wait_count++. |
| MR_ACTIVE      | /MREQ HIGH? → emit, re-dispatch as T1. Else: capture D[7:0], wait_count++. |
| MW_ACTIVE      | /MREQ HIGH? → emit, re-dispatch as T1. Else: wait_count++. |
| IR_ACTIVE      | /IORQ HIGH? → emit, re-dispatch as T1. Else: capture D[7:0], wait_count++. |
| IW_ACTIVE      | /IORQ HIGH? → emit, re-dispatch as T1. Else: wait_count++. |

No falling-edge actions are performed. All data comes from PIO rising-edge
samples.

---

## 7. Capture Record Format

Each completed machine cycle produces one trace record:

```
typedef struct {
    uint16_t address;       // A[15:0] captured at T1↑
    uint8_t  data;          // D[7:0] — all captures at rising edges:
                            //   M1_FETCH: T3↑ (opcode)
                            //   MEM_RD:   T3↑ (read data)
                            //   MEM_WR:   T2↑ (write data)
                            //   IO_RD:    T3↑ (read data)
                            //   IO_WR:    T2↑ (write data)
                            //   INT_ACK:  T3↑ (interrupt vector)
    uint8_t  cycle_type;    // M1_FETCH, MEM_RD, MEM_WR, IO_RD, IO_WR,
                            // INT_ACK, RESET, DATA_LOSS
    uint8_t  refresh;       // unused (always 0)
    uint8_t  wait_count;    // number of TW states (includes auto-waits for I/O)
    uint8_t  flags;         // HALT, INT, NMI flags
    uint8_t  seq;           // 7-bit sequence counter for gap detection
} trace_record_t;           // 8 bytes
```

---

## 8. Cycle Counting on the RP2350

The RP2350 (Cortex-M33) does **not** have the DWT CYCCNT register available
on the Pimoroni PGA2350 (it is an Armv8-M Baseline feature that may be
absent or disabled). Instead, the analyzer uses the **SysTick timer** as a
free-running cycle counter:

```c
// SysTick registers (Cortex-M standard, memory-mapped)
#define SYST_CSR   (*(volatile uint32_t *)0xE000E010)  // control/status
#define SYST_RVR   (*(volatile uint32_t *)0xE000E014)  // reload value
#define SYST_CVR   (*(volatile uint32_t *)0xE000E018)  // current value

// Configure: max reload, no interrupt, processor clock source
SYST_RVR = 0x00FFFFFF;  // 24-bit max
SYST_CVR = 0;           // writing any value clears to 0
SYST_CSR = SYST_CSR_ENABLE | SYST_CSR_CLKSOURCE;  // bits 0 and 2

// Read cycle count (SysTick counts DOWN from reload value)
static inline uint32_t cycle_count(void) {
    return 0x00FFFFFF - SYST_CVR;
}

// Elapsed cycles with 24-bit wrap handling
static inline uint32_t cycle_elapsed(uint32_t t0, uint32_t t1) {
    return (t1 - t0) & 0x00FFFFFF;
}
```

SysTick runs at the CPU clock rate (300 MHz) and provides 1-cycle
resolution. The 24-bit counter wraps every ~56 ms, which is far longer
than any single measurement interval. Each core has its own SysTick, so
core 1's counter does not conflict with core 0.

The cycle counter is used **only in diagnostic mode** (mode 2: analyzer
without PSRAM) to measure per-sample processing cost. In normal capture
mode (mode 0), instrumentation is disabled to avoid wasting cycles on
the hot path.

---

## 9. Performance Budget at 4 MHz Z80

At 4 MHz Z80 clock, each T-state is 250 ns. With a 300 MHz ARM core,
that's **75 ARM clock cycles per Z80 T-state**.

The rising-edge-only PIO/DMA architecture provides a DMA ring buffer (32 KB)
that absorbs per-sample timing variation. Individual samples may exceed 75
cycles as long as the **average throughput** stays under budget. This is
critical — unlike GPIO-polled designs, the DMA buffer decouples sample
arrival from processing.

### Per rising-edge budget breakdown (normal mode)

| Operation                           | Cycles | Frequency     |
|-------------------------------------|--------|---------------|
| `process_sample()` — cheap state    | ~5–10  | Most samples  |
| `process_sample()` — emit + dispatch| ~25–35 | ~1 per 3–4 samples |
| Loop overhead (tail++, count++)     | ~5     | Every sample  |
| `stage_drain_one()` PSRAM write     | ~42    | Every 8th sample |
| Outer loop (DMA head, /WAIT check)  | ~5–10  | Per batch     |
| **Weighted average**                | **~25–35** | |

This is well within the 75-cycle budget, leaving ~40 cycles of headroom
for DMA buffer absorption of occasional spikes.

### Diagnostic mode performance measurement

Mode 2 (`DIAG_SKIP_PSRAM`) instruments each `process_sample()` call with
SysTick cycle counting and tracks min/max. This measures the state machine
in isolation, without PSRAM drain overhead. The results are reported via
the status command and the `capture_test.py` diagnostic script.

---

## 10. Practical Considerations

### 10.1 What the analyzer CANNOT see from bus signals alone

- **Instruction boundaries**: Multi-byte instructions (prefixed with CB, DD,
  ED, FD) have multiple M1 fetches. The analyzer must decode opcodes to
  group them.

- **Interrupt mode**: The CPU's current IM setting is internal state. Must
  track IM instructions to interpret INT_ACK vector bytes.

- **IFF state**: Whether interrupts are enabled. Must track EI/DI/RETN.

- **Stack pointer value**: Must track SP-modifying instructions to annotate
  PUSH/POP/CALL/RET memory accesses.

### 10.2 M1 cycle length ambiguity

Standard M1 is 4 T-states (T1–T2–T3–T4). The manual states the first M
cycle "is four, five, or six T cycles long." The 5/6 T-state M1 cycles
occur with DD/FD/CB/ED prefix handling and certain instructions. The
analyzer handles this because the extra T-states look like "internal
operation" — no /MREQ, /IORQ, /RD, /WR activity. The ST_T2_NONM1 handler
detects this (neither /MREQ nor /IORQ asserted) and re-interprets the
sample as a potential T1↑.

### 10.3 PSRAM ring buffer

- PSRAM capacity: 8 MB, providing ~1M trace records at 8 bytes/record
- Ring buffer with write index on core 1, read on core 0
- Writes via uncached QMI address (no cache coherency issues)
- SRAM staging buffer (256 entries) decouples emit_record() from PSRAM
  write latency; drained interleaved with sample processing

### 10.4 Non-M1 deferred emission

Non-M1 ACTIVE states (MR, MW, IR, IW) cannot detect T3 at the T3 rising
edge — /MREQ and /IORQ are still asserted. They detect the cycle end one
sample late (at the next T1↑). This means:

1. Record emission is deferred by one sample compared to M1 cycles
2. The "cycle end" sample must be re-dispatched as T1 of the new cycle
3. The `dispatch_t1()` helper handles this: it calls `begin_cycle()` for
   the new cycle and sets the appropriate T2 state

This adds no processing cost — the re-dispatch is a simple branch with
the sample already in hand.

---

## 11. Cross-Reference to Manual Figures

| Figure     | Manual Page | Section | Cycle Type            |
|------------|-------------|---------|-----------------------|
| Fig. 4     | p. 8        | §2      | General M/T structure |
| Fig. 5     | p. 9        | §4.1    | Opcode Fetch (M1)     |
| Fig. 6     | p. 10       | §4.2–3  | Memory Read/Write     |
| Fig. 7     | p. 11       | §4.4–5  | I/O Read/Write        |
| Fig. 9     | p. 13       | §4.6    | Interrupt Ack         |
| Fig. 10    | p. 14       | §4.7    | NMI Response          |
| Fig. 11    | p. 15       | §4.8    | HALT Exit             |
