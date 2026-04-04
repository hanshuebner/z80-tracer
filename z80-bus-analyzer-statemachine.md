# Z80 Bus Analyzer — State Machine Description

## Reference: Zilog Z80 CPU User Manual UM008011-0816, pp. 7–17

---

## 1. Monitored Signals

| Signal    | Dir | Active | Sampling Notes                                |
|-----------|-----|--------|-----------------------------------------------|
| CLK       | in  | —      | State transitions keyed to CLK rising edges   |
| A15–A0    | out | High   | Active during T1; refresh addr during M1 T3–T4 |
| D7–D0     | bi  | High   | Sampled at specific points per cycle type     |
| /M1       | out | Low    | Identifies opcode fetch or interrupt ack      |
| /MREQ     | out | Low    | Memory access (fetch, read, write, refresh)   |
| /IORQ     | out | Low    | I/O access or interrupt acknowledge           |
| /RD       | out | Low    | CPU wants to read                             |
| /WR       | out | Low    | CPU data bus holds valid write data           |
| /RFSH     | out | Low    | Refresh address on low 7 bits of addr bus     |
| /HALT     | out | Low    | CPU is in HALT state (executing NOPs)         |
| /WAIT     | in  | Low    | Peripheral requests wait states               |
| /BUSREQ   | in  | Low    | External device requests bus                  |
| /BUSACK   | out | Low    | CPU has released bus                          |
| /INT      | in  | Low    | Maskable interrupt request                    |
| /NMI      | in  | Low    | Nonmaskable interrupt (edge-triggered)        |
| /RESET    | in  | Low    | System reset                                 |

---

## 2. State Machine Overview

The analyzer tracks two levels of state:

- **Cycle type**: Which kind of machine cycle is in progress
- **T-state**: Which clock period within that cycle

### Rising-Edge-Only Architecture

All state transitions are driven by **CLK rising edges only**, sampled by the
PIO state machine. This halves the PIO/DMA throughput compared to dual-edge
sampling, which is critical for meeting the 4 MHz Z80 performance target.

The tracer is a **passive observer** — it does not control /WAIT or any other
Z80 bus signal. It must keep up with the Z80 at full speed (up to 4 MHz)
without ever falling behind. The total budget per rising edge is approximately
**70 ARM cycles** (at 300 MHz ARM, with a 250 ns Z80 T-state).

A small number of operations require data that becomes valid on the **falling
edge** of CLK:
- /WAIT sampling (at T2↓ and TW↓) — **always required**, wait states are used
- Data bus capture for read cycles (MEM_RD, IO_RD, INT_ACK at T3↓)
- Refresh address capture (M1 at T3↓)

These are handled by **GPIO polling** (busy-wait for CLK to go low, then
read the bus via GPIO) or by a **secondary PIO state machine** dedicated to
falling-edge capture. The secondary PIO is triggered on demand by the ARM
core only when a falling-edge sample is actually needed — most T-states do
not need one.

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
| /BUSACK low                | **BUS RELEASED**     | Fig. 8    |
| /HALT low + M1 cycling     | **HALT** (NOP fetch) | Fig. 11   |
| /RESET low                 | **RESET**            | p. 6      |

---

## 4. Cycle State Machines

Each cycle type below is a sub-state-machine. All state transitions occur at
**CLK rising edges**. Where falling-edge data is needed, the analyzer uses
GPIO polling or a secondary PIO (marked with ⚡ below).

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
         │  ⚡ POLL for T2↓: SAMPLE /WAIT                   │
         │  if /WAIT=low → enter TW loop                    │
         │  if /WAIT=high → proceed to T3                   │
         │                                                  │
  TW ↑   │  (wait state — may repeat)                       │
         │  ⚡ POLL for TW↓: SAMPLE /WAIT again             │
         │  if /WAIT=low → another TW                       │
         │  if /WAIT=high → proceed to T3                   │
         │                                                  │
  T3 ↑   │  ► CAPTURE data bus (this is the OPCODE)         │
         │  /MREQ ↑, /RD ↑                                  │
         │  Address bus transitions to refresh address      │
         │  /RFSH ↓                                         │
         │  ⚡ POLL for T3↓: CAPTURE refresh addr A[6:0]    │
         │                                                  │
  T4 ↑   │  /MREQ ↑ (refresh complete)                      │
         │  /RFSH ↑, /M1 ↑                                  │
         │  ► SAMPLE /BUSREQ (for possible bus release)     │
         │  ► SAMPLE /INT, /NMI (at end of last M cycle     │
         │    of instruction only — tracked by decoder)     │
         │  → Next cycle's T1 follows at next CLK ↑         │
         └──────────────────────────────────────────────────┘

Capture record:
  { type: M1_FETCH, addr: A[15:0]@T1↑, data: D[7:0]@T3↑,
    refresh: A[6:0]@T3↓⚡, waits: count_of_TW,
    halt: /HALT level }
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
         │  ⚡ POLL for T2↓: SAMPLE /WAIT                   │
         │  if /WAIT=low → insert TW                        │
         │  if /WAIT=high → proceed to T3                   │
         │                                                  │
  TW ↑   │  (wait state — may repeat)                       │
         │  ⚡ POLL for TW↓: SAMPLE /WAIT                   │
         │                                                  │
  T3 ↑   │  /MREQ ↑, /RD ↑                                  │
         │  (data bus still valid — hold time after /RD↑)   │
         │  ⚡ POLL for T3↓: CAPTURE data bus (read data)   │
         │  → Next cycle's T1 follows at next CLK ↑         │
         └──────────────────────────────────────────────────┘

NOTE: Unlike M1 fetch (which captures data at T3↑), non-M1 memory read
captures data at T3↓ via GPIO poll. The data remains stable through T3↓
even though /RD rises at T3↑.

Capture record:
  { type: MEM_RD, addr: A[15:0]@T1↑, data: D[7:0]@T3↓⚡,
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
         │  ⚡ POLL for T2↓: SAMPLE /WAIT                   │
         │  if /WAIT=low → insert TW                        │
         │  if /WAIT=high → proceed to T3                   │
         │                                                  │
  TW ↑   │  (wait state — may repeat)                       │
         │  ⚡ POLL for TW↓: SAMPLE /WAIT                   │
         │                                                  │
  T3 ↑   │  /WR ↑ (data hold satisfied)                     │
         │  → Next cycle's T1 follows at next CLK ↑         │
         └──────────────────────────────────────────────────┘

Note: /WR goes inactive at T3↑, half a T-state before address
and data change. This satisfies hold time for memory.

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
         │  (auto-inserted wait TW* follows)                │
         │  ⚡ POLL for T2↓: SAMPLE /WAIT (during auto TW)  │
         │  if /WAIT=low → insert additional TW             │
         │  if /WAIT=high → proceed to T3                   │
         │                                                  │
  TW* ↑  │  (automatic wait — always present)               │
         │  ⚡ POLL for TW*↓: SAMPLE /WAIT                  │
         │  if /WAIT=low → additional TW                    │
         │  if /WAIT=high → proceed to T3                   │
         │                                                  │
  TW ↑   │  (additional wait states if /WAIT held low)      │
         │  ⚡ POLL for TW↓: SAMPLE /WAIT                   │
         │                                                  │
  T3 ↑   │  /IORQ ↑, /RD ↑                                  │
         │  (data bus still valid — hold time after /RD↑)   │
         │  ⚡ POLL for T3↓: CAPTURE data bus (read data)   │
         │  → Next cycle's T1 follows at next CLK ↑         │
         └──────────────────────────────────────────────────┘

IMPORTANT: The cycle is T1-T2-TW*-T3 minimum (4 CLK periods).
TW* is always inserted. The manual states the reason: the interval
from /IORQ going active to the WAIT sample point is too short
for I/O devices to decode their address and assert /WAIT.

Capture record:
  { type: IO_RD, addr: A[15:0]@T1↑, data: D[7:0]@T3↓⚡,
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
         │  Data bus = write data (stable)                  │
         │  ► CAPTURE data bus (write data)                 │
         │  (auto-inserted wait TW* follows)                │
         │  ⚡ POLL for TW*↓: SAMPLE /WAIT                  │
         │  (same extension logic as I/O read)              │
         │                                                  │
  TW ↑   │  (additional wait states if needed)              │
         │  ⚡ POLL for TW↓: SAMPLE /WAIT                   │
         │                                                  │
  T3 ↑   │  /WR ↑, /IORQ ↑                                  │
         │  → Next cycle's T1 follows at next CLK ↑         │
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
         │  (/IORQ goes low at T2↓ — detected by poll)      │
         │                                                  │
  TW1*↑  │  (first automatic wait — always present)         │
         │  ⚡ POLL for TW1*↓: SAMPLE /WAIT                 │
         │                                                  │
  TW2*↑  │  (second automatic wait — always present)        │
         │  ⚡ POLL for TW2*↓: SAMPLE /WAIT                 │
         │  if /WAIT=low → additional TW                    │
         │  if /WAIT=high → proceed to T3                   │
         │                                                  │
  TW ↑   │  (additional wait states if needed)              │
         │  ⚡ POLL for TW↓: SAMPLE /WAIT                   │
         │                                                  │
  T3 ↑   │  /IORQ ↑, /M1 ↑                                  │
         │  (data bus still valid — vector byte held)       │
         │  ⚡ POLL for T3↓: CAPTURE data bus (vector byte) │
         │  Refresh cycle follows (same as normal M1)       │
         │                                                  │
  T4 ↑   │  Refresh complete                                │
         │  → Next cycle (PUSH PC, then jump to ISR)        │
         └──────────────────────────────────────────────────┘

IMPORTANT: Two automatic wait states are always inserted to allow
time for a daisy-chain priority resolution to propagate.
The interrupt vector on D[7:0] depends on the interrupt mode:
  Mode 0: any instruction (typically RST)
  Mode 1: ignored (CPU jumps to 0038h)
  Mode 2: low byte of vector table pointer (high byte = I register)

Capture record:
  { type: INT_ACK, addr: A[15:0]@T1↑, data: D[7:0]@T3↓⚡,
    waits: 2 + count_of_additional_TW }
```

### 4.7 BUS REQUEST / ACKNOWLEDGE — Ref: Figure 8

```
This is not a machine cycle — it's a bus ownership transfer.

         ┌──────────────────────────────────────────────────┐
         │          Bus Request / Release                   │
         │                                                  │
  Any    │  /BUSREQ is sampled at rising edge of the LAST   │
  M-cycle│  T-state of any machine cycle.                   │
  last T │                                                  │
    ↑    │  if /BUSREQ=low:                                 │
         │    Next CLK ↑:                                   │
         │      Address bus → Hi-Z                          │
         │      Data bus → Hi-Z                             │
         │      /MREQ, /IORQ, /RD, /WR → Hi-Z               │
         │      /BUSACK ↓                                   │
         │                                                  │
  Tx ↑   │  ► Detect: /BUSACK=low                           │
  (each) │  Bus is floating — external controller active    │
         │  ► SAMPLE /BUSREQ each rising edge               │
         │  if /BUSREQ returns high:                        │
         │    /BUSACK ↑                                     │
         │    CPU resumes with T1 of next M cycle           │
         └──────────────────────────────────────────────────┘

Analyzer behavior:
  While /BUSACK is low, the analyzer cannot capture meaningful
  address/data (bus is Hi-Z or driven by DMA controller).
  The analyzer should record DMA activity as:
  { type: BUS_RELEASED, duration: count_of_Tx_clocks }

  If the external DMA controller drives /MREQ, /RD, /WR, the
  analyzer COULD optionally trace DMA transfers using the same
  signal decoding, but must note they are not CPU-originated.

NOTE: During bus request, /INT and /NMI are NOT serviced.
```

### 4.8 NONMASKABLE INTERRUPT RESPONSE — Ref: Figure 10

```
The NMI response is NOT a special bus cycle — it is a normal
M1 opcode fetch. The CPU:
  1. Performs a normal M1 fetch (but ignores the fetched byte)
  2. Pushes PC onto the stack (two memory write cycles)
  3. Sets PC = 0066h

From the analyzer's perspective, the NMI response looks like:
  M1 FETCH → MEM_WR (push PCH) → MEM_WR (push PCL)

The analyzer can detect this sequence by noting:
  - /NMI went low (edge-triggered, sampled at final T-state
    rising edge of last M cycle of current instruction)
  - The next M1 fetch is a "phantom" — the fetched opcode is
    discarded and the CPU forces an internal restart to 0066h
  - The two memory writes that follow write PC to the stack

The analyzer cannot distinguish the phantom M1 from a real M1
purely from bus signals. To flag it, the analyzer should record
the /NMI edge and annotate the subsequent M1 as:
  { type: M1_FETCH, addr: ..., data: ..., note: "NMI_RESPONSE" }
```

### 4.9 HALT STATE — Ref: Figures 11, 12

```
During HALT:
  - /HALT output goes low
  - CPU repeatedly executes M1 fetch cycles (reading NOPs or
    whatever is in memory, but forcing NOP internally)
  - Refresh continues normally during each M1 T3–T4
  - /INT and /NMI sampled at each T4↑

From the analyzer's perspective, HALT looks like repeated M1
fetch cycles with /HALT=low. The analyzer flags these:
  { type: M1_FETCH, addr: ..., data: ..., halt: true }

Exit: /INT (if IFF1 set) or /NMI triggers exit at T4↑.
The next cycle is the interrupt acknowledge or NMI response.
```

### 4.10 RESET — Ref: p. 6

```
When /RESET is low:
  - Address bus, data bus → Hi-Z
  - All control outputs → inactive (high)
  - /RESET must be held low for minimum 3 full CLK cycles

The analyzer detects /RESET=low and enters RESET state.
On /RESET returning high, the CPU begins execution:
  - PC = 0000h
  - I = 00h, R = 00h
  - Interrupt mode 0
  - IFF1 = IFF2 = 0 (interrupts disabled)

  { type: RESET, duration: count_of_clk_while_reset_low }
```

---

## 5. Unified Analyzer State Machine

Combining all cycle types, the top-level state machine is:

```
                          ┌────────┐
                /RESET=low│ RESET  │
               ┌─────────►│        │
               │          └───┬────┘
               │   /RESET=high│
               │              ▼
               │          ┌────────┐  /BUSREQ ack'd   ┌─────────────┐
               │          │  IDLE  │─────────────────►│ BUS_RELEASED│
               │          │(T1 ↑)  │◄─────────────────│  (DMA)      │
               │          └───┬────┘  /BUSACK=high    └─────────────┘
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
               │    │ MEM  MEM IO    IO
               │    │ READ WR  READ  WRITE
               │    │
               │    ├── T2↑: /MREQ low?
               │    │     Yes → OPCODE_FETCH
               │    │     No  → INT_ACK
               │    │
               │    ▼
               │  ┌───────────┬──────────────┐
               │  │ M1_FETCH  │   INT_ACK    │
               │  └───────────┴──────────────┘
               │
               └──── (can interrupt any state)
```

All decisions are made at rising CLK edges (T1↑ and T2↑).

---

## 6. CLK Edge Actions Summary

### Rising Edge (CLK ↑) — PIO-sampled, drives all state transitions

| State              | T  | Action                                             |
|--------------------|----|----------------------------------------------------|
| Any cycle          | T1 | Capture A[15:0]. Check /M1. Begin cycle ID.        |
| Any cycle          | T2 | Check /MREQ, /IORQ, /RD, /WR → determine type.    |
| M1_FETCH           | T3 | Capture D[7:0] = opcode. /MREQ↑, /RD↑.             |
| M1_FETCH           | T4 | /RFSH↑, /M1↑. Sample /BUSREQ, /INT, /NMI.          |
| MEM_READ           | T3 | /MREQ↑, /RD↑. (data captured at T3↓ via poll)      |
| MEM_WRITE          | T2 | Capture D[7:0] = write data. /WR↓ already active.  |
| MEM_WRITE          | T3 | /WR↑ (data hold satisfied). Cycle complete.         |
| IO_READ            | T3 | /IORQ↑, /RD↑. (data captured at T3↓ via poll)      |
| IO_WRITE           | T2 | /IORQ↓, /WR↓. Capture D[7:0] = write data.         |
| IO_WRITE           | T3 | /WR↑, /IORQ↑. Cycle complete.                      |
| INT_ACK            | T3 | /IORQ↑, /M1↑. (data captured at T3↓ via poll)      |
| BUS_RELEASED       | Tx | Sample /BUSREQ. If high → exit to IDLE.            |

### Falling Edge (CLK ↓) — GPIO poll / secondary PIO, on demand only

| State              | T   | Action                                            |
|--------------------|-----|---------------------------------------------------|
| M1_FETCH           | T2  | ⚡ Sample /WAIT. If low → enter TW.               |
| M1_FETCH           | TW  | ⚡ Sample /WAIT. If low → repeat TW.              |
| M1_FETCH           | T3  | ⚡ Capture refresh addr A[6:0].                   |
| MEM_READ           | T2  | ⚡ Sample /WAIT.                                  |
| MEM_READ           | TW  | ⚡ Sample /WAIT.                                  |
| MEM_READ           | T3  | ⚡ Capture D[7:0] = read data.                    |
| MEM_WRITE          | T2  | ⚡ Sample /WAIT.                                  |
| MEM_WRITE          | TW  | ⚡ Sample /WAIT.                                  |
| IO_READ            | TW* | ⚡ Sample /WAIT (auto-inserted wait).              |
| IO_READ            | TW  | ⚡ Sample /WAIT (additional waits).                |
| IO_READ            | T3  | ⚡ Capture D[7:0] = read data.                    |
| IO_WRITE           | TW* | ⚡ Sample /WAIT.                                  |
| IO_WRITE           | TW  | ⚡ Sample /WAIT.                                  |
| INT_ACK            | TW1*| ⚡ Sample /WAIT (first auto wait).                |
| INT_ACK            | TW2*| ⚡ Sample /WAIT (second auto wait).               |
| INT_ACK            | TW  | ⚡ Sample /WAIT (additional).                     |
| INT_ACK            | T3  | ⚡ Capture D[7:0] = interrupt vector.             |

⚡ = falling-edge action performed via GPIO polling or secondary PIO,
not the main rising-edge PIO state machine.

---

## 7. Falling-Edge Capture Strategy

The ARM core on core 1 handles falling-edge operations using one of two
approaches (to be determined during implementation based on measured timing):

### Option A: GPIO Polling

After processing a rising edge, if the current state requires a falling-edge
sample, the ARM core busy-waits on the CLK GPIO pin:

```
while (gpio_get(CLK_PIN) == 1) { /* spin */ }
// CLK just went low — read the bus
uint32_t bus = gpio_get_all();
```

**Pros**: No additional PIO resources needed. Simple. Deterministic.
**Cons**: Burns CPU cycles spinning. At 4 MHz Z80 clock (125 ns per half
cycle), with a 300 MHz ARM core, that's ~37 ARM cycles of spin time.
Acceptable since core 1 is dedicated to the analyzer.

### Option B: Secondary PIO State Machine

A second PIO SM is configured to wait for CLK falling edge and push GPIO
snapshot to a separate FIFO. The ARM core reads this FIFO only when needed.

```
; Secondary PIO: wait for CLK falling edge, push GPIO snapshot
wait 1 pin CLK_PIN    ; wait for CLK high
wait 0 pin CLK_PIN    ; wait for CLK low
in pins, 32           ; capture all GPIO
push                  ; to FIFO
```

The ARM core drains or ignores this FIFO depending on whether the current
state needs falling-edge data. The secondary PIO runs continuously but the
FIFO read is conditional.

**Pros**: Precise timing, no CPU spin. FIFO buffering.
**Cons**: Uses a second PIO SM and FIFO. Must manage FIFO overflow for
falling edges that aren't consumed.

### When falling-edge capture is needed

Every cycle type requires at least one falling-edge sample (for /WAIT).
Wait states are used in the target system and must always be tracked.

| Cycle Type    | Falling-edge captures needed                         |
|---------------|-------------------------------------------------------|
| M1_FETCH      | T2↓ (/WAIT), TW↓ (/WAIT), T3↓ (refresh addr)        |
| MEM_READ      | T2↓ (/WAIT), TW↓ (/WAIT), T3↓ (data bus)             |
| MEM_WRITE     | T2↓ (/WAIT), TW↓ (/WAIT)                             |
| IO_READ       | TW*↓ (/WAIT), TW↓ (/WAIT), T3↓ (data bus)            |
| IO_WRITE      | TW*↓ (/WAIT), TW↓ (/WAIT)                            |
| INT_ACK       | TW1*↓, TW2*↓, TW↓ (/WAIT), T3↓ (data bus)           |

Note: Since /WAIT must always be sampled, every cycle needs at least one
falling-edge read. The advantage of the rising-edge-only PIO architecture
is that the falling-edge work is done inline (poll or FIFO read) rather
than doubling the PIO sample rate. The ARM core decides *which* falling
edges matter based on state, rather than processing all of them.

---

## 8. Capture Record Format

Each completed machine cycle produces one trace record:

```
struct TraceRecord {
    cycle_type:   enum { M1_FETCH, MEM_RD, MEM_WR,
                         IO_RD, IO_WR, INT_ACK,
                         BUS_RELEASED, RESET },
    address:      u16,          // A[15:0] captured at T1↑
    data:         u8,           // D[7:0] — sample point depends on cycle:
                                 //   M1_FETCH: T3↑ (rising edge, PIO)
                                 //   MEM_RD:   T3↓ (falling edge, ⚡poll)
                                 //   MEM_WR:   T2↑ (rising edge, PIO)
                                 //   IO_RD:    T3↓ (falling edge, ⚡poll)
                                 //   IO_WR:    T2↑ (rising edge, PIO)
                                 //   INT_ACK:  T3↓ (falling edge, ⚡poll)
    refresh_addr: u7,           // only for M1_FETCH: A[6:0] at T3↓ ⚡poll
    wait_count:   u8,           // number of TW states inserted
    halt:         bool,         // /HALT was active during this cycle
    clk_count:    u32,          // absolute CLK count for timestamp
    signals:      SignalSnapshot // optional: all signal levels at capture
}
```

---

## 9. Discrimination Timing — Critical Windows

The analyzer makes all cycle-type decisions at rising CLK edges:

```
CLK ↑ (T1)
  │
  ├─ /RESET = low? ─────────────── YES → RESET state
  ├─ /BUSACK = low? ────────────── YES → BUS_RELEASED
  │
  ├─ Capture A[15:0]
  ├─ /M1 low? ──────────────────── YES → M1 class cycle
  │                                 NO → non-M1 class cycle
  │
  ▼
CLK ↑ (T2)  ← all control signals have settled
  │
  ├─ M1 class:
  │    /MREQ low? ──── YES → OPCODE FETCH
  │                     NO → INT_ACK
  │
  ├─ non-M1 class:
  │    /MREQ low?
  │       YES → Memory cycle
  │              /RD low → MEM_READ
  │              /WR low → MEM_WRITE
  │       NO  → I/O cycle (/IORQ low)
  │              /RD low → IO_READ
  │              /WR low → IO_WRITE
```

No falling-edge decisions are needed for cycle type identification.

---

## 10. Practical Considerations for Analyzer Implementation

### 10.1 What the analyzer CANNOT see from bus signals alone

- **Instruction boundaries**: The analyzer knows M1 fetches start a new
  instruction, but multi-byte instructions (prefixed with CB, DD, ED, FD)
  have multiple M1-like fetches. The analyzer must decode opcodes to know
  how many M cycles an instruction has and what types they are.

- **Interrupt mode**: The CPU's current IM setting (0, 1, or 2) is internal
  state. The analyzer must track IM instructions to know how to interpret
  the INT_ACK vector byte.

- **IFF state**: Whether interrupts are enabled. Must track EI/DI/RETN.

- **Stack pointer value**: Must track SP-modifying instructions to annotate
  PUSH/POP/CALL/RET memory accesses.

### 10.2 M1 cycle length ambiguity

Standard M1 is 4 T-states (T1-T2-T3-T4).
But the manual states the first M cycle "is four, five, or six T cycles long."
The 5/6 T-state M1 cycles occur with DD/FD/CB/ED prefix handling and
certain instructions. The analyzer handles this because T4 is always
followed by either another T1 (with /M1 going low again for another
fetch) or by the continuation of the internal operation. The signals
on the bus during the extra T-states look like "internal operation" —
no /MREQ, /IORQ, /RD, /WR activity. The analyzer should detect these
as idle bus states within the instruction.

### 10.3 Performance budget at 4 MHz Z80

At 4 MHz Z80 clock, each T-state is 250 ns. With a 300 MHz ARM core,
that's 75 ARM clock cycles per Z80 T-state. The target budget is
**~70 ARM cycles per rising edge**, leaving a small margin.

The rising-edge-only PIO architecture provides the full 250 ns budget
for processing each T-state, versus 125 ns with dual-edge sampling.

**Per rising edge budget breakdown** (approximate):
- Read PIO FIFO: ~2 cycles
- Decode state + extract signals: ~10 cycles
- State machine logic + branching: ~15 cycles
- GPIO poll for falling edge (when needed): ~37 cycles spin + ~3 read
- Write to PSRAM ring buffer (when record complete): ~10 cycles
- **Total worst case: ~70 cycles** (T-states with falling-edge work)
- **Total best case: ~30 cycles** (T-states with no falling-edge work, e.g. T1, T4)

**Critical**: These cycle counts must be **measured, not estimated**.
During implementation, every state machine path must be instrumented
with cycle counters to verify the budget is met. See §10.6.

### 10.4 PSRAM ring buffer

Trace records are buffered into the PGA2350's PSRAM, not streamed over USB.
USB CDC bandwidth (~1 MB/s) cannot sustain the trace record rate at 4 MHz
Z80 clock.

- PSRAM capacity: 8 MB (PGA2350), providing storage for millions of
  trace records depending on encoding
- Ring buffer with head/tail pointers; core 1 writes, core 0 reads
  for USB transfer after capture stops
- Core 0 handles USB communication: receives commands (start/stop capture,
  configure triggers, download trace), transmits buffered trace data
- Capture continues until: trigger stop condition met, PSRAM full (with
  configurable wrap/stop policy), or host sends stop command

### 10.5 Trigger system (firmware-resident)

All trigger logic runs in the tracer firmware, not the host client.
The client is used only for configuration, visualization, and analysis.

Trigger conditions evaluated by the state machine on core 1:
- **Address match**: start/stop capture when PC or address bus matches
  a value or range
- **Data match**: capture when specific opcode or data byte seen
- **Cycle type filter**: capture only specific cycle types
- **Combined conditions**: AND/OR of the above

Trigger actions:
- Start capture (fill PSRAM from this point)
- Stop capture (freeze PSRAM contents)
- Snapshot (capture N records around trigger point, pre/post)

The trigger evaluation must fit within the per-rising-edge cycle budget.
Simple comparisons (address == value) are cheap (~3 cycles). More complex
triggers may need careful optimization.

### 10.6 Performance measurement and testing

Since the 70-cycle budget is tight, performance must be continuously
monitored during development:

**Cycle-counting instrumentation**: Use the ARM cycle counter (DWT_CYCCNT
or RP2350 equivalent) to measure the actual cycle count of each state
machine iteration. Track min/max/histogram per state.

**Test programs for the Z80 side**:
- **NOP sled**: Continuous NOPs — exercises M1 fetch path at maximum rate
  (4 T-states per instruction = 1 MHz instruction rate at 4 MHz clock)
- **Memory copy loop**: Exercises MEM_RD + MEM_WR cycles back-to-back
- **I/O burst**: Rapid IN/OUT instructions — exercises I/O paths with
  auto-wait states
- **Wait-heavy workload**: Target system with slow peripherals asserting
  /WAIT — verifies wait state handling under load
- **Interrupt storm**: Frequent interrupts — exercises INT_ACK path
- **Prefix-heavy code**: DD/FD/CB/ED prefixed instructions — exercises
  extended M1 sequences

**Synthetic clock for bench testing**: Before real Z80 hardware is available,
drive the CLK pin and bus signals from a second RP2350 or signal generator
to verify timing at exact 4 MHz. This avoids depending on real hardware
for performance validation.

**Overflow detection**: The state machine must detect if it falls behind
(PIO FIFO overrun). If a FIFO overrun is detected, flag it in the trace
stream so the host client can report the gap.

### 10.7 Suggested implementation approach

```
on every PIO rising-edge sample:
    if /RESET = low:
        state = RESET; return
    if /BUSACK = low:
        state = BUS_RELEASED; return

    switch (state):
    case IDLE / expecting T1:
        capture address from PIO sample
        if /M1 = low:
            cycle_class = M1
        else:
            cycle_class = NON_M1
        evaluate trigger (address match)
        state = T1_DONE

    case T1_DONE (this is T2↑):
        if cycle_class = M1:
            if /MREQ = low:
                cycle_type = OPCODE_FETCH
            else:
                cycle_type = INT_ACK
        else:
            if /MREQ = low:
                if /RD = low: cycle_type = MEM_READ
                else:         cycle_type = MEM_WRITE
                              capture data (write data at T2↑)
            else:
                if /RD = low: cycle_type = IO_READ
                else:         cycle_type = IO_WRITE
                              capture data (write data at T2↑)

        ⚡ poll_falling_edge()  // for /WAIT sampling
        check /WAIT → manage wait states
        state = T2_DONE

    case T2_DONE / TW (this is T3↑ or TW↑):
        if in TW:
            ⚡ poll_falling_edge()  // sample /WAIT again
            if /WAIT still low → stay in TW; return
        // Now at T3↑
        if cycle needs rising-edge data (M1_FETCH, MEM_WR, IO_WR):
            capture data from PIO sample
        if cycle needs falling-edge data (MEM_RD, IO_RD, INT_ACK):
            ⚡ poll_falling_edge()
            capture data from GPIO
        if M1_FETCH:
            ⚡ poll_falling_edge()  // capture refresh addr
            state = T3_DONE (wait for T4↑)
        else:
            evaluate trigger (data/type match)
            write trace record to PSRAM ring buffer
            state = IDLE

    case T3_DONE (M1 T4↑):
        sample /BUSREQ, /INT, /NMI, /HALT
        evaluate trigger (opcode match)
        write trace record to PSRAM ring buffer
        state = IDLE
```

---

## 11. Cross-Reference to Manual Figures

| Figure     | Manual Page | State Machine Section | Cycle Type            |
|------------|-------------|-----------------------|-----------------------|
| Fig. 4     | p. 8        | §2 Overview           | General M/T structure |
| Fig. 5     | p. 9        | §4.1                  | Opcode Fetch (M1)     |
| Fig. 6     | p. 10       | §4.2, §4.3            | Memory Read/Write     |
| Fig. 7     | p. 11       | §4.4, §4.5            | I/O Read/Write        |
| Fig. 8     | p. 12       | §4.7                  | Bus Req/Ack           |
| Fig. 9     | p. 13       | §4.6                  | Interrupt Ack         |
| Fig. 10    | p. 14       | §4.8                  | NMI Response          |
| Fig. 11    | p. 15       | §4.9                  | HALT Exit             |
| Fig. 12    | p. 15       | §4.10 (partial)       | Power-Down Ack        |
| Fig. 13–15 | p. 16       | §4.10                 | Power-Down Release    |

---

## 12. Open Questions for Review

1. **I/O auto-wait counting**: The manual shows TW* between T2 and T3 in
   Fig. 7. Is /WAIT sampled during TW* (i.e., can the I/O device add
   *more* wait states on top of the automatic one), or only after TW*?
   The manual text says "During this wait state period, the WAIT request
   signal is sampled" — confirming /WAIT IS sampled during TW*.
   The analyzer should count TW* as wait_count=1, and additional TWs
   increment from there.

2. **INT ACK auto-waits**: Fig. 9 shows TW*, TW* (two automatic waits).
   Same question — /WAIT is sampled during each, and additional waits can
   be added. This needs verification from the timing spec (whether /WAIT
   is checked after TW1*, after TW2*, or only after TW2*).

3. **DD/FD prefix M1 extension**: When the CPU fetches a DD or FD prefix,
   does the subsequent M1 fetch for the actual opcode look like a normal
   M1 on the bus? (Yes — it's a separate M1 cycle.) The analyzer needs
   opcode decode to group them.

4. **NMI phantom fetch**: The M1 fetch during NMI acknowledge reads a
   byte from (PC) but discards it. The analyzer records this as a normal
   M1_FETCH. Is there any bus-visible difference? (No — only /NMI edge
   history distinguishes it.)

5. **HALT M1 address**: During HALT, does the PC increment for each NOP
   M1 fetch, or does it stay constant? (PC does NOT increment — the
   CPU re-fetches the same address, the HALT instruction itself, but
   forces NOP internally.) The analyzer should see the same address
   repeating.

6. **GPIO poll timing margin**: At 4 MHz, the falling edge arrives 125 ns
   after the rising edge. With 300 MHz ARM, that's ~37 cycles. The GPIO
   poll spin loop needs to be tight (ideally 2-3 instructions per
   iteration). Verify that the GPIO read path doesn't add latency that
   could cause the poll to miss the window. **This must be measured
   empirically early in implementation.**

7. **Secondary PIO vs GPIO poll**: Decision to be made based on measured
   timing. GPIO poll is simpler but ties up the core for ~37 cycles.
   Secondary PIO is cleaner but uses PIO resources and needs FIFO
   management. Since /WAIT is always sampled, every cycle needs at
   least one falling-edge read — this tilts toward the secondary PIO
   approach if GPIO polling proves too tight on the budget.

8. **PSRAM write latency**: The PGA2350 PSRAM is accessed via QSPI.
   Write latency must be measured to ensure it fits within the cycle
   budget. If writes are slow, consider batching (accumulate records
   in SRAM, flush to PSRAM in bulk during idle bus periods or T-states
   that don't need falling-edge work).

9. **PSRAM capacity vs trace record size**: With 8 MB PSRAM and a
   compact record encoding, how many trace records can be stored?
   At 6 bytes/record: ~1.3M records. At 4 MHz with ~2 T-states average
   per record, that's ~650K instructions or ~0.16 seconds of trace.
   Consider whether compression or reduced-field encoding is needed.

10. **Trigger complexity vs cycle budget**: How complex can trigger
    conditions be without exceeding the 70-cycle budget? Simple address
    compare is ~3 cycles. Range check is ~5. AND of two conditions is
    ~8. Need to define a trigger language that fits the budget.

11. **Synthetic test clock generation**: What hardware is needed to
    drive realistic Z80 bus signals at 4 MHz for bench testing? A second
    PGA2350 running a Z80 bus simulator would be ideal. Define the
    test harness architecture early.
