# Z80 Bus Analyzer — State Machine Description

## Reference: Zilog Z80 CPU User Manual UM008011-0816, pp. 7–17

---

## 1. Monitored Signals

| Signal    | Dir | Active | Sampling Notes                                |
|-----------|-----|--------|-----------------------------------------------|
| CLK       | in  | —      | All state transitions keyed to CLK edges      |
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

Every transition is triggered by a **CLK edge** (rising ↑ or falling ↓).
Signal levels are sampled at the indicated edge to determine the next state.

---

## 3. Cycle Type Identification

At the **start of T1** (rising edge of CLK entering T1), the analyzer must determine
what cycle type is beginning. The discrimination depends partly on what happens
*during* the cycle (e.g., /MREQ vs /IORQ going low), so the analyzer uses a
two-phase identification:

### Phase 1 — T1 rising edge: Check /M1

| /M1 at T1↑  | Meaning                                         |
|-------------|-------------------------------------------------|
| Low         | This is an M1 cycle (opcode fetch or INT ack)   |
| High        | Non-M1 cycle (mem r/w, I/O r/w)                 |

### Phase 2 — T1 falling edge (or T2 rising): Refine cycle type

For **M1 cycles** (/M1 low):

| /MREQ at T1↓ | /IORQ    | Cycle Type                    | Ref       |
|--------------|----------|-------------------------------|-----------|
| Low          | High     | **OPCODE FETCH**              | Fig. 5    |
| High         | (later)  | **INTERRUPT ACKNOWLEDGE**     | Fig. 9    |

Note: In INT ACK, /IORQ goes low at T2↓ (not T1↓), and /MREQ stays high.

For **non-M1 cycles** (/M1 high), determined at T1↓ / T2↑:

| /MREQ  | /IORQ | /RD  | /WR  | Cycle Type          | Ref       |
|--------|-------|------|------|---------------------|-----------|
| Low    | High  | Low  | High | **MEMORY READ**     | Fig. 6    |
| Low    | High  | High | Low  | **MEMORY WRITE**    | Fig. 6    |
| High   | Low   | Low  | High | **I/O READ**        | Fig. 7    |
| High   | Low   | High | Low  | **I/O WRITE**       | Fig. 7    |

Note on I/O: /IORQ and /RD or /WR go active at T2↑ (not T1↓ like memory).
The analyzer sees /MREQ staying high through T1↓ to distinguish I/O from memory.

### Special states (detected by dedicated signals):

| Condition                  | State                | Ref       |
|----------------------------|----------------------|-----------|
| /BUSACK low                | **BUS RELEASED**     | Fig. 8    |
| /HALT low + M1 cycling     | **HALT** (NOP fetch) | Fig. 11   |
| /RESET low                 | **RESET**            | p. 6      |

---

## 4. Cycle State Machines

Each cycle type below is a sub-state-machine. Transitions are annotated with
the CLK edge that triggers them. **Bold** items are analyzer capture actions.

### 4.1 OPCODE FETCH (M1 cycle) — Ref: Figure 5

```
Entry: /M1 goes low (observed at T1 rising edge)

         ┌──────────────────────────────────────────────────┐
         │                  M1 Cycle                        │
         │                                                  │
  T1 ↑   │  /M1=low, Address bus = PC                       │
         │  ► CAPTURE address (this is the PC)              │
         │                                                  │
  T1 ↓   │  /MREQ ↓, /RD ↓                                  │
         │  (confirms: opcode fetch, not INT ACK)           │
         │                                                  │
  T2 ↑   │  (memory responding)                             │
         │                                                  │
  T2 ↓   │  ► SAMPLE /WAIT                                  │
         │  if /WAIT=low → insert TW, stay at T2↓ check     │
         │  if /WAIT=high → proceed to T3                   │
         │                                                  │
  TW ↑   │  (wait state — may repeat)                       │
  TW ↓   │  ► SAMPLE /WAIT again                            │
         │  if /WAIT=low → another TW                       │
         │  if /WAIT=high → proceed to T3                   │
         │                                                  │
  T3 ↑   │  ► CAPTURE data bus (this is the OPCODE)         │
         │  /MREQ ↑, /RD ↑                                  │
         │  Address bus transitions to refresh address      │
         │  /RFSH ↓                                         │
         │                                                  │
  T3 ↓   │  /MREQ ↓ (for refresh)                           │
         │  ► CAPTURE refresh address (low 7 bits = R reg)  │
         │                                                  │
  T4 ↑   │  /MREQ ↑ (refresh complete)                      │
         │  /RFSH ↑, /M1 ↑                                  │
         │  ► SAMPLE /BUSREQ (for possible bus release)     │
         │  ► SAMPLE /INT, /NMI (at end of last M cycle     │
         │    of instruction only — tracked by decoder)     │
         │                                                  │
  T4 ↓   │  → Next cycle's T1 follows                       │
         └──────────────────────────────────────────────────┘

Capture record:
  { type: M1_FETCH, addr: A[15:0]@T1↑, data: D[7:0]@T3↑,
    refresh: A[6:0]@T3↓, waits: count_of_TW,
    halt: /HALT level }
```

### 4.2 MEMORY READ — Ref: Figure 6 (left half)

```
Entry: /M1 high, then /MREQ↓ + /RD↓ observed at T1↓

         ┌──────────────────────────────────────────────────┐
         │              Memory Read Cycle                   │
         │                                                  │
  T1 ↑   │  Address bus = memory address                    │
         │  ► CAPTURE address                               │
         │                                                  │
  T1 ↓   │  /MREQ ↓, /RD ↓                                  │
         │                                                  │
  T2 ↑   │  (memory responding)                             │
         │                                                  │
  T2 ↓   │  ► SAMPLE /WAIT                                  │
         │  if /WAIT=low → insert TW                        │
         │  if /WAIT=high → proceed to T3                   │
         │                                                  │
  TW ↓   │  ► SAMPLE /WAIT (repeat as needed)               │
         │                                                  │
  T3 ↑   │  /MREQ ↑, /RD ↑                                  │
         │  (data bus still valid — hold time after /RD↑)   │
         │                                                  │
  T3 ↓   │  ► CAPTURE data bus (read data)                  │
         │  → Next cycle's T1 follows                       │
         └──────────────────────────────────────────────────┘

NOTE: Unlike M1 fetch (which samples at T3↑), non-M1 memory read
samples data at T3↓. The timing diagram confirms data remains
stable through T3↓ even though /RD rises at T3↑.

Capture record:
  { type: MEM_RD, addr: A[15:0]@T1↑, data: D[7:0]@T3↓,
    waits: count_of_TW }
```

### 4.3 MEMORY WRITE — Ref: Figure 6 (right half)

```
Entry: /M1 high, then /MREQ↓ at T1↓, /WR↓ early in T2

         ┌──────────────────────────────────────────────────┐
         │              Memory Write Cycle                  │
         │                                                  │
  T1 ↑   │  Address bus = memory address                    │
         │  ► CAPTURE address                               │
         │                                                  │
  T1 ↓   │  /MREQ ↓                                         │
         │  Data bus begins to carry write data             │
         │                                                  │
  T2 ↑   │  /WR ↓ (data bus now stable)                     │
         │  ► CAPTURE data bus (write data)                 │
         │                                                  │
  T2 ↓   │  ► SAMPLE /WAIT                                  │
         │  if /WAIT=low → insert TW                        │
         │  if /WAIT=high → proceed to T3                   │
         │                                                  │
  TW ↓   │  ► SAMPLE /WAIT (repeat as needed)               │
         │                                                  │
  T3 ↑   │  /WR ↑ (half T-state before bus changes)         │
         │                                                  │
  T3 ↓   │  /MREQ ↑, address+data bus change                │
         │  → Next cycle's T1 follows                       │
         └──────────────────────────────────────────────────┘

Note: /WR goes inactive at T3↑, half a T-state before address
and data change. This satisfies hold time for memory.

Capture record:
  { type: MEM_WR, addr: A[15:0]@T1↑, data: D[7:0]@T2↑,
    waits: count_of_TW }
```

### 4.4 I/O READ — Ref: Figure 7 (upper half)

```
Entry: /M1 high, /MREQ stays high, /IORQ↓ + /RD↓ at T2↑

         ┌──────────────────────────────────────────────────┐
         │              I/O Read Cycle                      │
         │                                                  │
  T1 ↑   │  Address bus = port address                      │
         │  (A[7:0] = port, A[15:8] = register content)     │
         │  ► CAPTURE address                               │
         │                                                  │
  T1 ↓   │  (nothing yet — /MREQ stays high)                │
         │                                                  │
  T2 ↑   │  /IORQ ↓, /RD ↓                                  │
         │                                                  │
  T2 ↓   │  (this is the AUTO-INSERTED wait state TW)       │
         │  ► SAMPLE /WAIT (during auto TW)                 │
         │  if /WAIT=low → insert additional TW             │
         │  if /WAIT=high → proceed to T3                   │
         │                                                  │
  TW* ↑  │  (automatic wait — always present)               │
         │                                                  │
  TW* ↓  │  ► SAMPLE /WAIT                                  │
         │  if /WAIT=low → additional TW                    │
         │  if /WAIT=high → proceed to T3                   │
         │                                                  │
  TW ↓   │  ► SAMPLE /WAIT (repeat as needed)               │
         │                                                  │
  T3 ↑   │  /IORQ ↑, /RD ↑                                  │
         │  (data bus still valid — hold time after /RD↑)   │
         │                                                  │
  T3 ↓   │  ► CAPTURE data bus (read data)                  │
         │  → Next cycle's T1 follows                       │
         └──────────────────────────────────────────────────┘

IMPORTANT: The cycle is T1-T2-TW*-T3 minimum (4 CLK periods).
TW* is always inserted. The manual states the reason: the interval
from /IORQ going active to the WAIT sample point is too short
for I/O devices to decode their address and assert /WAIT.

Capture record:
  { type: IO_RD, addr: A[15:0]@T1↑, data: D[7:0]@T3↓,
    waits: 1 + count_of_additional_TW }
```

### 4.5 I/O WRITE — Ref: Figure 7 (lower half)

```
Entry: /M1 high, /MREQ stays high, /IORQ↓ + /WR↓ at T2↑

         ┌──────────────────────────────────────────────────┐
         │              I/O Write Cycle                     │
         │                                                  │
  T1 ↑   │  Address bus = port address                      │
         │  ► CAPTURE address                               │
         │                                                  │
  T1 ↓   │  (nothing yet)                                   │
         │                                                  │
  T2 ↑   │  /IORQ ↓, /WR ↓                                  │
         │  Data bus = write data (stable)                  │
         │  ► CAPTURE data bus (write data)                 │
         │                                                  │
  TW* ↓  │  (automatic wait — always present)               │
         │  ► SAMPLE /WAIT                                  │
         │  (same extension logic as I/O read)              │
         │                                                  │
  T3 ↑   │  /WR ↑, /IORQ ↑                                  │
         │                                                  │
  T3 ↓   │  → Next cycle's T1 follows                       │
         └──────────────────────────────────────────────────┘

Capture record:
  { type: IO_WR, addr: A[15:0]@T1↑, data: D[7:0]@T2↑,
    waits: 1 + count_of_additional_TW }
```

### 4.6 INTERRUPT ACKNOWLEDGE — Ref: Figure 9

```
Entry: /M1 goes low at T1↑, but /MREQ does NOT go low at T1↓.
       Instead, /IORQ goes low (with /M1 still low) at T2↓.

         ┌──────────────────────────────────────────────────┐
         │          Interrupt Acknowledge Cycle             │
         │                                                  │
  T1 ↑   │  /M1 ↓, Address bus = PC (but not used)          │
         │  ► CAPTURE address (PC at time of interrupt)     │
         │                                                  │
  T1 ↓   │  /MREQ stays HIGH (key discriminator vs fetch)   │
         │                                                  │
  T2 ↑   │  (nothing yet)                                   │
         │                                                  │
  T2 ↓   │  /IORQ ↓ (signals interrupt acknowledge)         │
         │                                                  │
  TW1*↑  │  (first automatic wait — always present)         │
  TW1*↓  │  ► SAMPLE /WAIT                                  │
         │                                                  │
  TW2*↑  │  (second automatic wait — always present)        │
  TW2*↓  │  ► SAMPLE /WAIT                                  │
         │  if /WAIT=low → additional TW                    │
         │  if /WAIT=high → proceed to T3                   │
         │                                                  │
  TW ↓   │  ► SAMPLE /WAIT (repeat as needed)               │
         │                                                  │
  T3 ↑   │  /IORQ ↑, /M1 ↑                                  │
         │  (data bus still valid — vector byte held)       │
         │                                                  │
  T3 ↓   │  ► CAPTURE data bus (interrupt vector byte)      │
         │  Refresh cycle follows (same as normal M1)       │
         │                                                  │
  T4 ↑   │  Refresh complete                                │
         │                                                  │
  T4 ↓   │  → Next cycle (PUSH PC, then jump to ISR)        │
         └──────────────────────────────────────────────────┘

IMPORTANT: Two automatic wait states are always inserted to allow
time for a daisy-chain priority resolution to propagate.
The interrupt vector on D[7:0] depends on the interrupt mode:
  Mode 0: any instruction (typically RST)
  Mode 1: ignored (CPU jumps to 0038h)
  Mode 2: low byte of vector table pointer (high byte = I register)

Capture record:
  { type: INT_ACK, addr: A[15:0]@T1↑, data: D[7:0]@T3↓,
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
               │    │    T1↓:/MREQ↓  T1↓:/MREQ stays high
               │    │         │           │
               │    │    ┌────┴────┐  T2↑:/IORQ↓
               │    │    │         │      │
               │    │  /RD↓      /WR↓  ┌──┴───┐
               │    │    │         │   │      │
               │    │    ▼         ▼  /RD↓   /WR↓
               │    │ MEM_READ MEM_WR  │      │
               │    │                  ▼      ▼
               │    │              IO_READ IO_WRITE
               │    │
               │    ├── T1↓: /MREQ↓ ?
               │    │     Yes → OPCODE_FETCH
               │    │     No  → INT_ACK (wait for /IORQ↓ at T2)
               │    │
               │    ▼
               │  ┌───────────┬──────────────┐
               │  │ M1_FETCH  │   INT_ACK    │
               │  └───────────┴──────────────┘
               │
               └──── (can interrupt any state)
```

---

## 6. T-State Counting and CLK Edge Actions

Summary table of what the analyzer does at each CLK edge:

### Rising Edge (CLK ↑)

| State              | T  | Action                                             |
|--------------------|----|----------------------------------------------------|
| Any cycle          | T1 | Capture A[15:0]. Check /M1. Begin cycle ID.        |
| M1_FETCH           | T3 | Capture D[7:0] = opcode. /MREQ↑, /RD↑.             |
| M1_FETCH           | T4 | /RFSH↑, /M1↑. Sample /BUSREQ, /INT, /NMI.          |
| MEM_READ           | T3 | /MREQ↑, /RD↑. (data still valid, captured at ↓)    |
| MEM_WRITE          | T2 | Capture D[7:0] = write data. /WR↓ goes active.     |
| MEM_WRITE          | T3 | /WR↑ (data hold satisfied).                        |
| IO_READ            | T2 | /IORQ↓, /RD↓.                                      |
| IO_READ            | T3 | /IORQ↑, /RD↑. (data still valid, captured at ↓)    |
| IO_WRITE           | T2 | /IORQ↓, /WR↓. Capture D[7:0] = write data.         |
| IO_WRITE           | T3 | /WR↑, /IORQ↑.                                      |
| INT_ACK            | T3 | /IORQ↑, /M1↑. (data still valid, captured at ↓)    |
| BUS_RELEASED       | Tx | Sample /BUSREQ. If high → exit to IDLE.            |

### Falling Edge (CLK ↓)

| State              | T   | Action                                            |
|--------------------|-----|---------------------------------------------------|
| M1_FETCH           | T1  | /MREQ↓, /RD↓. Confirms fetch (vs INT_ACK).        |
| M1_FETCH           | T2  | Sample /WAIT. If low → enter TW.                  |
| M1_FETCH           | TW  | Sample /WAIT. If low → repeat TW. If high → T3.   |
| M1_FETCH           | T3  | /MREQ↓ (refresh). Capture refresh addr A[6:0].    |
| MEM_READ           | T1  | /MREQ↓, /RD↓.                                     |
| MEM_READ           | T2  | Sample /WAIT.                                     |
| MEM_READ           | TW  | Sample /WAIT.                                     |
| MEM_READ           | T3  | ► Capture D[7:0] = read data.                     |
| MEM_WRITE          | T1  | /MREQ↓.                                           |
| MEM_WRITE          | T2  | Sample /WAIT.                                     |
| MEM_WRITE          | TW  | Sample /WAIT.                                     |
| IO_READ            | TW* | Sample /WAIT (auto-inserted wait).                |
| IO_READ            | TW  | Sample /WAIT (additional waits).                  |
| IO_READ            | T3  | ► Capture D[7:0] = read data.                     |
| IO_WRITE           | TW* | Sample /WAIT.                                     |
| IO_WRITE           | TW  | Sample /WAIT.                                     |
| INT_ACK            | T2  | /IORQ↓.                                           |
| INT_ACK            | TW1*| Sample /WAIT (first auto wait).                   |
| INT_ACK            | TW2*| Sample /WAIT (second auto wait).                  |
| INT_ACK            | TW  | Sample /WAIT (additional).                        |
| INT_ACK            | T3  | ► Capture D[7:0] = interrupt vector.              |

---

## 7. Capture Record Format

Each completed machine cycle produces one trace record:

```
struct TraceRecord {
    cycle_type:   enum { M1_FETCH, MEM_RD, MEM_WR,
                         IO_RD, IO_WR, INT_ACK,
                         BUS_RELEASED, RESET },
    address:      u16,          // A[15:0] captured at T1↑
    data:         u8,           // D[7:0] — sample edge depends on cycle:
                                 //   M1_FETCH: T3↑ (rising edge)
                                 //   MEM_RD:   T3↓ (falling edge)
                                 //   MEM_WR:   T2↑ (rising edge)
                                 //   IO_RD:    T3↓ (falling edge)
                                 //   IO_WR:    T2↑ (rising edge)
                                 //   INT_ACK:  T3↓ (falling edge)
    refresh_addr: u7,           // only for M1_FETCH: A[6:0] at T3↓
    wait_count:   u8,           // number of TW states inserted
    halt:         bool,         // /HALT was active during this cycle
    clk_count:    u32,          // absolute CLK count for timestamp
    signals:      SignalSnapshot // optional: all signal levels at capture
}
```

---

## 8. Discrimination Timing — Critical Windows

The analyzer must make cycle-type decisions within tight windows.
Here are the critical decision points:

```
CLK ↑ (T1)
  │
  ├─ /M1 low? ──────────────────── YES → M1 class cycle
  │                                        │
  │  CLK ↓ (T1)                            │
  │    │                                   │
  │    ├─ /MREQ low? ──── YES → OPCODE FETCH
  │    │                   NO  → INT_ACK (confirm at T2: /IORQ↓)
  │    │
  │    ├─ /MREQ low?
  │    │    YES → Memory cycle
  │    │           /RD low → MEM_READ
  │    │           (else)  → MEM_WRITE (confirmed when /WR↓ at T2↑)
  │    │    NO  → I/O cycle (confirmed at T2↑ when /IORQ↓)
  │    │           /RD↓ → IO_READ
  │    │           /WR↓ → IO_WRITE
  │
  ├─ /BUSACK low? ──────────────── YES → BUS_RELEASED
  ├─ /RESET low? ───────────────── YES → RESET
  └─ else → continue in current multi-T-state cycle
```

---

## 9. Practical Considerations for Analyzer Implementation

### 9.1 What the analyzer CANNOT see from bus signals alone

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

### 9.2 M1 cycle length ambiguity

Standard M1 is 4 T-states (T1-T2-T3-T4).
But the manual states the first M cycle "is four, five, or six T cycles long."
The 5/6 T-state M1 cycles occur with DD/FD/CB/ED prefix handling and
certain instructions. The analyzer handles this because T4 is always
followed by either another T1 (with /M1 going low again for another
fetch) or by the continuation of the internal operation. The signals
on the bus during the extra T-states look like "internal operation" —
no /MREQ, /IORQ, /RD, /WR activity. The analyzer should detect these
as idle bus states within the instruction.

### 9.3 Suggested implementation approach

The analyzer's main loop:

```
on every CLK rising edge:
    if /RESET = low:
        state = RESET; return
    if /BUSACK = low:
        state = BUS_RELEASED; return
    if state expects T1:
        capture address
        if /M1 = low:
            cycle_class = M1
        else:
            cycle_class = NON_M1
        advance to T1_done, wait for falling edge

on every CLK falling edge:
    if in T1 and cycle_class = M1:
        if /MREQ = low:
            cycle_type = OPCODE_FETCH
        else:
            cycle_type = INT_ACK
    if in T1 and cycle_class = NON_M1:
        if /MREQ = low:
            cycle_type = (will check /RD vs /WR next)
        else:
            pending_IO = true  (confirm at T2↑)
    if in T2 or TW:
        check /WAIT → insert or exit wait states
    ...proceed through cycle state machine...
```

---

## 10. Cross-Reference to Manual Figures

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

## 11. Open Questions for Review

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
