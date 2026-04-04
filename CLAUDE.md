# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Z80 bus tracer using a Pimoroni PGA2350 (RP2350). The firmware captures all Z80 bus activity in real time using a PIO sampler (rising-edge only) and a dedicated ARM core (core 1) running a T-state-level bus analyzer state machine. Trace records are stored in 8 MB PSRAM. A Python host client downloads records over USB CDC and decodes them into a Z80 instruction trace with register state tracking and loop detection. Initial hardware testing in progress.

## Build Commands

### Firmware (C, RP2350/pico-sdk)
```bash
cd build
cmake ..
make
```
Requires pico-sdk at `~/Development/privat/pico-sdk`. Board is `pga2350` with custom header in `board/`. Produces `.uf2` to flash to the PGA2350.

### Host Client (Python)
```bash
pip install pyserial
python -m client /dev/ttyACM0 --start-at 0x0100   # live capture with trigger
python -m client --replay test.bin                 # replay captured trace
python -m client.gen_trace > test.bin              # generate synthetic test data
```

### Tests
```bash
python client/test_trace.py
```
Single test file with smoke tests against synthetic data. Run from repo root.

## Architecture

### Firmware (`main.c`, `z80_trace.h`, `z80_trace.pio`)
- PIO samples GPIO 0-31 at **CLK rising edges only** (one sample per Z80 clock). Autopush at 32 bits.
- DMA ring buffer (32KB) drains PIO FIFO continuously
- Core 1 (dedicated): runs the bus analyzer state machine (21 states). Cycle type discrimination at T2 rising edge where all control signals have settled. Produces 4-byte `trace_record_t` into SRAM staging buffer, drained to PSRAM.
- Core 0: services USB CDC command protocol, evaluates triggers against PSRAM records, handles buffer readout
- Sync mode: after power-on/reset/overrun, tests every rising edge for unambiguous M1 fetch signature (/M1 low + /MREQ low). Phase recovery in IDLE detects /MREQ or /IORQ already active.
- Supported cycle types: M1 opcode fetch, memory read/write, I/O read/write (with auto-wait), interrupt acknowledge (with 2 auto-waits), reset detection

### Trace Record Format (4 bytes)
```
Byte 0-1: address[15:0]  (u16 LE, captured at T2 rising edge)
Byte 2:   data[7:0]      (u8)
Byte 3:   cycle_type:3 | wait_count:3 | halt:1 | int:1
```
Data sample point: T3 rising edge for reads (M1 fetch, mem read, I/O read, INT ack), T2 rising edge for writes (mem write, I/O write).

### PSRAM Ring Buffer
8 MB PSRAM / 4 bytes per record = 2M records (`PSRAM_RING_RECORDS = 1 << 21`). Core 1 writes via SRAM staging buffer (256 entries). Core 0 reads for trigger evaluation and USB readout.

### USB Command Protocol
Binary commands over USB CDC (see `z80_trace.h`): start/stop capture, configure/clear/arm triggers, read buffer with pre/post counts, get status with diagnostics, set diagnostic mode. No streaming -- all trace data goes through PSRAM.

### Trigger System (firmware-resident, core 0)
Up to 8 triggers: PC match, memory read/write, I/O read/write, interrupt acknowledge. Optional data value/mask matching. Evaluated by chasing core 1's write index through PSRAM. Post-trigger countdown stops capture after configured number of records.

### Host Client (`client/`)
- `main.py`: Entry point, `InstructionAssembler` state machine, `_decode_to_lines()` for PSRAM record decoding with trigger-relative trimming. Live path uses `capture.py` for PSRAM protocol, builds `live_fcfg` (without start_at/triggers to avoid double-filtering).
- `capture.py`: PSRAM capture protocol client -- `get_status()`, `start_capture()`, `stop_capture()`, `set_trigger()`, `arm_trigger()`, `read_buffer()`, `format_record()`, `unpack_type_wait_flags()`
- `capture_test.py`: Low-level PSRAM protocol test tool with diagnostic modes
- `z80_decoder.py`: Z80 instruction decoder -- maps prefix + opcode to mnemonics, determines operand counts
- `z80_state.py`: `Z80State` -- tracks register values by observing bus data (reads/writes), provides `snapshot_key()` for loop detection
- `z80_loop.py`: `LoopDetector` -- detects PC-sequence and exact-state loops, suppresses repeated output after threshold
- `gen_trace.py`: Generates synthetic 4-byte trace records for replay testing

### Pin Mapping
GPIO 0-15 = address bus, 16-23 = data bus, 24-30 = control signals (/M1, /MREQ, /IORQ, /RD, /WR, /RFSH, CLK), 31 = /HALT. PIO captures GPIO 0-31 (32 bits). GPIO 32 = /WAIT (output for flow control, also read for wait state observation), 33-35 = /INT, /NMI, /RESET (read via ARM GPIO, not PIO). All active-low except CLK. Defined in `z80_trace.h`.
