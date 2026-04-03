# Performance Optimization Notes

## Problem

DMA overflows when tracing a 4 MHz Z80. The current PIO/DMA architecture
samples at both CLK edges (8M samples/sec). At 300 MHz ARM, that's only
37.5 cycles/sample — too tight for the state machine + PSRAM drain.

## Baseline measurements (origin/main firmware)

Diagnostic results with the existing PIO/DMA both-edge architecture:

| Mode | Description | samples/sec | cycles/sample | dma_overflows |
|------|-------------|-------------|---------------|---------------|
| 3 | Skip all (DMA baseline) | ~8M | ~37.5 | 0 |
| 1 | DMA loop only | ~8M | ~37.5 | 0 |
| 2 | Analyzer only (no PSRAM) | varies | varies | varies |
| 0 | Full pipeline | varies | varies | **overflows** |

The DMA read loop itself keeps up, but adding the state machine and
especially PSRAM drain pushes over budget. /WAIT flow control kicks in
but can't fully prevent overflows.

## Approach 1: Rising-edge PIO + simplified state machine

**Idea:** Change PIO to sample only at CLK rising edges (4M/sec), halving
the sample rate. Rewrite the state machine with ~21 states (down from ~40)
since no paired rising/falling states are needed. For states that need
falling-edge data (read captures, refresh), poll `sio_hw->gpio_in`.

**Problem discovered:** `poll_falling_edge()` is unsafe when there's a DMA
backlog — it reads the CURRENT GPIO state, which may be many CLK cycles
ahead of the sample being processed from the DMA buffer. This would
capture data from the wrong bus cycle.

**Status:** Not tested on hardware. Abandoned in favor of Approach 2.

## Approach 2: GPIO-polled CLK (no PIO/DMA)

**Idea:** Eliminate PIO and DMA entirely. Core 1 polls `sio_hw->gpio_in`
directly for CLK edges in a tight loop. No buffer, no backlog, so GPIO
polls for falling-edge data are always safe (we're always in real time).

Budget: 4M rising edges/sec at 300 MHz = 75 cycles/sample.

### Architecture

- Removed: PIO program, DMA channel, DMA ring buffer, overflow detection,
  /WAIT flow control assertion
- Core 1 loop: sync (wait CLK low) → wait CLK high → GPIO read →
  process_sample → optional CLK↓ wait for falling capture →
  PSRAM drain → repeat
- State machine: ~21 states, all keyed to rising edges
- `needs_falling` flag set by states that need T3↓ data (MR_T3, IR_T3,
  IA_T3, M1_T3). Main loop waits for CLK↓ and calls
  `complete_falling_capture()`.
- Staging buffer + PSRAM drain unchanged (8-byte records)
- Trigger system, command protocol, client code unchanged

### Hardware test results

```
Mode 3 (CLK poll only):        4.0M samples/sec   75.0 cycles/sample  ✓
Mode 1 (skip analyzer):        4.0M samples/sec   75.0 cycles/sample  ✓
Mode 2 (analyzer, no PSRAM):   2.5M samples/sec  119.6 cycles/sample  ✗
Mode 0 (full pipeline):        2.0M samples/sec  149.7 cycles/sample  ✗
```

### Analysis of the 119 cycles/sample in Mode 2

The CLK polling itself is fast (modes 1/3 hit 4M). The state machine
adds too many cycles on certain samples, causing missed CLK edges.

**Root cause: falling-edge wait burns ~37 cycles.** States that set
`needs_falling` (MR_T3, IR_T3, IA_T3, M1_T3 for refresh) cause core 1
to spin-wait for CLK↓. Combined with emit_record (~15-25 cycles) and
loop overhead (~15 cycles), the total exceeds 75 cycles. Missing one CLK
edge causes state machine desynchronization, which wastes subsequent edges
as the analyzer tries to realign — a cascade that inflates the average.

**Falling-edge sample breakdown:**
- M1_T3 (refresh capture, no emit): ~55 cycles total — fits in 75 ✓
- MR_T3 (data capture + emit): ~85 cycles — exceeds 75 ✗
- IR_T3 (data capture + emit): ~85 cycles — exceeds 75 ✗
- IA_T3 (vector capture, no emit): ~55 cycles — fits in 75 ✓

The states that BOTH capture falling-edge data AND emit a record are the
problem. They can't fit the CLK↓ wait + capture + emit in 75 cycles.

### Attempted fix: capture read data at T3↑

Moved MEM_READ, IO_READ, and INT_ACK data capture from T3↓ to T3↑
(rising edge), eliminating `needs_falling` for those states. Only M1_T3
keeps the falling-edge capture (for refresh address), and it doesn't
emit — M1_T4 does.

**Rationale:** M1 opcode fetch already captures at T3↑ in the original
code. The Z80 data hold time after /RD↑ should keep data valid at the
rising edge.

**Status:** Built but not yet tested. Snapshot capture (`capture_test.py`
without `--diag`) showed no records with expected addresses (`grep 0100`
returned nothing). Root cause not yet isolated — could be:
1. T3↑ data capture timing issue (data not yet valid or already gone)
2. State machine desync from earlier missed edges (before the T3↑ fix)
3. emit_record bug (strict aliasing issue was found and fixed, but the
   fix — struct initializer + assignment — may have regressed performance)

## Key constraints identified

1. **75 cycles per rising edge is the hard budget.** Any sample that
   exceeds this causes a missed CLK edge and state machine desync.

2. **Falling-edge wait costs ~37 cycles** (half a 4 MHz CLK period).
   This is unavoidable if we need T3↓ data.

3. **emit_record costs ~15-25 cycles** (struct fill + staging buffer
   write). Inlined into process_sample, the compiler generates many
   individual byte/halfword stores.

4. **PSRAM drain costs ~40 cycles** per record (uncached 8-byte write
   via QMI). Must be amortized — draining every Nth sample.

5. **Loop overhead costs ~15 cycles** (CLK sync, GPIO read, diag_mode
   check, sample_count, stage depth tracking, diag_total_samples store).

6. **Combined cost of any two of {falling wait, emit, drain} exceeds
   75 cycles.** They must not coincide on the same sample.

## Options for next steps

### A. Keep GPIO polling, eliminate all falling-edge waits

Capture ALL data at T3↑. Drop refresh capture entirely (client computes
R register from initial value + M1 count). No `needs_falling`, no
`complete_falling_capture`. Every sample is a "cheap" sample.

Per-sample budget: ~15 (process) + ~15 (overhead) = ~30 cycles base,
plus ~40 (drain) every few samples. Well under 75.

**Risk:** T3↑ data capture may not work for all Z80 variants or bus
speeds. The data hold time after /RD↑ at T3↑ is specified as 0ns minimum
in some Z80 datasheets. PIO-captured samples at T3↑ worked for M1 opcode
fetch in the original code, but GPIO polling adds a few nanoseconds of
delay after CLK↑ detection.

### B. Return to PIO/DMA, but rising-edge only

Use PIO to sample at rising edges only (4M/sec via DMA). The DMA buffer
absorbs burst variations. State machine processes from the buffer — no
real-time constraint per sample, only average throughput.

Capture all data at T3↑ to avoid the `poll_falling_edge` backlog
problem. Refresh captured at T3↑ too (lower 7 bits of address bus at
T3↑ — but the address bus has already transitioned to the refresh
address by T3↑ for M1 cycles, since /RFSH goes active at T3↑).

**Advantage:** DMA buffer absorbs timing jitter from PSRAM writes.
No need to guarantee every sample completes in 75 cycles — just
average throughput.

**Risk:** Average throughput must still keep up. At 4M samples/sec,
need average < 75 cycles. With ~30 cycles for process + 10 amortized
drain = ~40 cycles average, plenty of margin.

### C. Hybrid: GPIO polling + DMA assist for PSRAM

Poll CLK via GPIO (real-time), but use a DMA channel to drain the
staging buffer to PSRAM in the background. Core 1 kicks the DMA and
continues processing — no CPU cycles spent on PSRAM writes.

**Advantage:** Removes the ~40-cycle PSRAM drain from the per-sample
budget entirely.

**Complexity:** Need a second DMA channel, transfer setup, completion
detection.

### D. Reduce record size to 4 bytes

Pack trace records into 4 bytes (one 32-bit PSRAM write). Halves PSRAM
drain cost to ~20 cycles. Drop refresh, wait_count, seq.

Can be combined with any of A/B/C.
