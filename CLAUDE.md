# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Z80 bus tracer using a Pimoroni PGA2350 (RP2350). The firmware captures all Z80 bus activity in real time via two PIO state machines (read cycles and write cycles), streams captured data over USB CDC, and a Python host client decodes it into a Z80 instruction trace with register state tracking and loop detection. **Not yet tested on real hardware.**

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
- Two PIO state machines on pio0: `z80_read_capture` (SM0, triggers on /RD falling edge, filters refresh via /RFSH) and `z80_write_capture` (SM1, triggers on /WR falling edge)
- Each PIO SM captures 31 GPIO pins (address + data + control) as a 32-bit word into its RX FIFO
- DMA ring buffers (128KB each) drain PIO FIFOs continuously
- Core 0: idle (services USB in background). Core 1: drains both ring buffers, classifies samples, encodes 4-byte packets, sends over USB CDC
- Flow control: asserts /WAIT to pause the Z80 when ring buffers are 75% full, releases at 25%

### USB Packet Protocol
4-byte binary packets with bit-7 sync framing (only byte 0 has bit 7 set). Encodes cycle type (3 bits), 16-bit address, and 8-bit data. Client scans for bit-7 to resync after connecting mid-stream.

### Host Client (`client/`)
- `main.py`: Entry point, packet parser, `InstructionAssembler` state machine that accumulates bus cycles into complete instructions. Handles all Z80 prefix groups (CB, DD, ED, FD, DDCB, FDCB)
- `z80_decoder.py`: Z80 instruction decoder — maps prefix + opcode to mnemonics, determines operand counts
- `z80_state.py`: `Z80State` — tracks register values by observing bus data (reads/writes), provides `snapshot_key()` for loop detection
- `z80_loop.py`: `LoopDetector` — detects PC-sequence and exact-state loops, suppresses repeated output after threshold
- `gen_trace.py`: Generates synthetic binary test traces for replay testing

### Pin Mapping
GPIO 0-15 = address bus, 16-23 = data bus, 24-30 = control signals (/M1, /MREQ, /IORQ, /RD, /WR, /RFSH, CLK), 31 = /HALT (input), 32 = /WAIT (output), 33-35 = /INT, /NMI, /RESET. All active-low except CLK. Defined in `z80_trace.h`.
