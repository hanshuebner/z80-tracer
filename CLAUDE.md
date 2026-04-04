# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Z80 bus tracer using a Pimoroni PGA2350 (RP2350). The firmware captures all Z80 bus activity in real time using a PIO sampler (rising CLK edges only) and a dedicated ARM core (core 1) running a full T-state-level bus analyzer state machine. Falling-edge data (wait states, read data, refresh address) is captured via GPIO polling or secondary PIO on demand. Trace records are buffered into PSRAM during capture, then transferred to host over USB. Trigger conditions (address/data match, start/stop) are evaluated in firmware. A Python host client handles post-capture visualization and analysis. **Not yet tested on real hardware.**

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
python -m client /dev/ttyACM0           # live trace from hardware
python -m client --replay test.bin      # replay captured trace
python -m client.gen_trace > test.bin   # generate synthetic test data
```

### Tests
```bash
python client/test_trace.py
```
Single test file with smoke tests against synthetic data. Run from repo root.

## Architecture

### Firmware (`main.c`, `z80_trace.h`, `z80_trace.pio`)
- Primary PIO state machine samples GPIO 0-31 at every CLK **rising edge** only, using autopush at 32 bits. Falling-edge data captured via GPIO poll or secondary PIO on demand.
- DMA ring buffer (32KB) drains PIO FIFO continuously
- Core 1 (dedicated): runs the full Z80 bus analyzer state machine, tracking T-states and cycle types per `z80-bus-analyzer-statemachine.md`. Writes trace records to PSRAM ring buffer. Budget: ~70 ARM cycles per rising edge.
- Core 0: services USB CDC, handles host commands (start/stop capture, configure triggers, download trace from PSRAM)
- Trigger system: firmware-resident address/data/cycle-type match conditions, evaluated inline by core 1 state machine
- PSRAM ring buffer (8 MB): stores trace records during capture; transferred to host after capture stops
- Mid-stream sync: on startup or after reset, the analyzer enters sync mode and only locks onto unambiguous M1 cycles (/M1 low at T1↑). Internal operation T-states are handled by re-interpreting rising edges.
- Supported cycle types: M1 opcode fetch, memory read/write, I/O read/write (with auto-wait), interrupt acknowledge (with 2 auto-waits), reset detection

### USB Packet Protocol
6-byte binary packets with bit-7 sync framing (only byte 0 has bit 7 set). Encodes cycle type (4 bits), 16-bit address, 8-bit data, 7-bit refresh address, halt flag, and 6-bit wait count. Client scans for bit-7 to resync after connecting mid-stream.

### Host Client (`client/`)
- `main.py`: Entry point, packet parser, `InstructionAssembler` state machine that accumulates bus cycles into complete instructions. Handles all Z80 prefix groups (CB, DD, ED, FD, DDCB, FDCB)
- `z80_decoder.py`: Z80 instruction decoder — maps prefix + opcode to mnemonics, determines operand counts
- `z80_state.py`: `Z80State` — tracks register values by observing bus data (reads/writes), provides `snapshot_key()` for loop detection
- `z80_loop.py`: `LoopDetector` — detects PC-sequence and exact-state loops, suppresses repeated output after threshold
- `gen_trace.py`: Generates synthetic binary test traces for replay testing

### Pin Mapping
GPIO 0-15 = address bus, 16-23 = data bus, 24-30 = control signals (/M1, /MREQ, /IORQ, /RD, /WR, /RFSH, CLK), 31 = /HALT. PIO captures GPIO 0-31 (32 bits). GPIO 32 = /WAIT (output for flow control, also read for wait state observation), 33-35 = /INT, /NMI, /RESET (read via ARM GPIO, not PIO). All active-low except CLK. Defined in `z80_trace.h`.
