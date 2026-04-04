# Z80 Bus Tracer

A hardware bus tracer for the Z80 CPU using a Pimoroni PGA2350 (RP2350).
Captures all bus activity in real time via PIO, stores trace records in
8 MB PSRAM, and downloads them over USB for decoded Z80 instruction
trace with register state tracking and loop detection.

**Not yet tested on real hardware.** All code was written from
datasheets and verified against synthetic test data only.

## Hardware

Connect the PGA2350 directly to the Z80 bus -- no level shifters
needed. The RP2350 GPIOs tolerate 5V inputs provided the RP2350 is
powered before the Z80. **If the Z80 drives 5V signals into an
unpowered RP2350, the GPIO protection diodes will forward-bias into
the unpowered 3.3V rail, which can damage the chip or cause
latch-up.** Ensure correct power sequencing: RP2350 on first,
Z80 on second; Z80 off first, RP2350 off second.

| PGA2350 GPIO | Z80 Signal | Pin |
|---|---|---|
| 0-15 | A0-A15 | Address bus |
| 16-23 | D0-D7 | Data bus |
| 24 | /M1 | 27 |
| 25 | /MREQ | 19 |
| 26 | /IORQ | 20 |
| 27 | /RD | 21 |
| 28 | /WR | 22 |
| 29 | /RFSH | 28 |
| 30 | CLK | 6 |
| 31 | /HALT | 18 |
| 32 | /WAIT | 24 (active-low output) |
| 33 | /INT | 16 |
| 34 | /NMI | 17 |
| 35 | /RESET | 26 |

See `WIRING.md` for the complete perfboard wiring guide and
`wiring-guide.html` for an interactive step-by-step wiring aid.

## Architecture

### Sampling

The PIO state machine samples GPIO 0-31 at every **CLK rising edge
only** (one sample per Z80 clock period). This halves DMA throughput
compared to dual-edge sampling, giving ~140 ARM cycles of budget per
sample at 300 MHz ARM / 4 MHz Z80.

### Analyzer state machine (core 1)

All cycle type discrimination is deferred to **T2 rising edge** where
control signals (/M1, /MREQ, /IORQ, /RD, /WR) have settled. The
state machine has 21 states covering:

- M1 opcode fetch (with /WAIT support)
- Memory read / write
- I/O read / write (with automatic wait state)
- Interrupt acknowledge (with 2 automatic wait states)
- Reset detection

Synchronization after power-on or buffer overrun uses an unambiguous
M1 fetch signature (/M1 low + /MREQ low at rising edge). Phase
recovery in IDLE state detects /MREQ or /IORQ already active after
odd-count internal operations.

See `z80-bus-analyzer-statemachine.md` for the full state machine
description.

### PSRAM trace buffer

Trace records are written to the PGA2350's 8 MB PSRAM ring buffer
during capture, not streamed over USB. At 4 bytes per record, the
buffer holds **2M records** (2,097,152). The host downloads records
after capture stops.

### Trace record format (4 bytes)

```
Byte 0-1: address[15:0]  (u16 LE, captured at T2 rising edge)
Byte 2:   data[7:0]      (u8, sample point depends on cycle type)
Byte 3:   cycle_type:3 | wait_count:3 | halt:1 | int:1
```

Data capture timing: M1 fetch, memory read, I/O read, and interrupt
acknowledge capture data at T3 rising edge. Memory write and I/O write
capture data at T2 rising edge (write data is stable by then).

### Trigger system

Triggers are evaluated in the firmware on core 0, not the host.
Supported trigger types: PC match (address or range), memory read/write,
I/O read/write, interrupt acknowledge. Optional data value matching.
Up to 8 triggers can be configured simultaneously.

### USB command protocol

Binary command protocol over USB CDC. Commands: start/stop capture,
configure/clear/arm triggers, read buffer, get status, set diagnostic
mode. See `z80_trace.h` for protocol definitions.

## Building the firmware

```bash
mkdir build && cd build
cmake ..
make
```

Flash the resulting `.uf2` to the PGA2350.

## Host client

### Installation

```bash
pip install pyserial
```

### Live capture

```bash
python -m client /dev/ttyACM0 --start-at 0x0100
```

Without triggers, captures until Ctrl+C then downloads and decodes:

```bash
python -m client /dev/ttyACM0
```

### Triggers

```bash
# Trigger on execution at address
python -m client /dev/ttyACM0 --start-at 0x0100

# Trigger on execution in range
python -m client /dev/ttyACM0 --start-at 0x0100-0x01FF

# Trigger on memory read/write
python -m client /dev/ttyACM0 --trigger-mem-read 0xC000
python -m client /dev/ttyACM0 --trigger-mem-write 0xC000=0xFF

# Trigger on I/O
python -m client /dev/ttyACM0 --trigger-io-read 0x01
python -m client /dev/ttyACM0 --trigger-io-write 0xFF

# Trigger on interrupt
python -m client /dev/ttyACM0 --trigger-int
```

### Controlling trace window

```bash
# 50 instructions before and after trigger (symmetric)
python -m client /dev/ttyACM0 --start-at 0x0100 --window 50

# 20 instructions before, 200 after
python -m client /dev/ttyACM0 --start-at 0x0100 --pre 20 --post 200

# Only post-trigger (e.g. trace from boot)
python -m client /dev/ttyACM0 --start-at 0x0100 --post 1000

# Fill PSRAM with post-trigger data
python -m client /dev/ttyACM0 --start-at 0x0100 --post-all

# Fill PSRAM evenly around trigger
python -m client /dev/ttyACM0 --start-at 0x0100 --pre-all --post-all
```

Default without options: 100 instructions before and after trigger.

### Display options

```bash
# Show raw bus cycles alongside decoded instructions
python -m client /dev/ttyACM0 --start-at 0x0100 --raw

# Suppress output for PC in range
python -m client /dev/ttyACM0 --start-at 0x0100 --mask 0xE000-0xFFFF

# Enable loop detection
python -m client /dev/ttyACM0 --start-at 0x0100 --loops

# Stop after N instructions
python -m client /dev/ttyACM0 --start-at 0x0100 --max-instructions 500
```

### Replay mode

Replay captured binary traces (from `gen_trace` or saved captures):

```bash
python -m client --replay test.bin
python -m client --replay test.bin --raw --loops
```

### Synthetic test data

```bash
python -m client.gen_trace > test.bin
python -m client --replay test.bin
```

### PSRAM capture test tool

Low-level test tool for the PSRAM capture protocol:

```bash
python -m client.capture_test /dev/ttyACM0 --trigger-mem-read 0x0100
python -m client.capture_test /dev/ttyACM0 --diag  # performance diagnostics
```

### Checking for data loss

After capture, query firmware status:

```python
from client import capture
status = capture.get_status(ser)
print(status['dma_overflows'])    # should be 0 (data lost if > 0)
print(status['wait_asserts'])     # > 0 means Z80 was paused
print(status['max_dma_distance']) # worst DMA backlog (of 8192)
print(status['max_stage_depth'])  # worst staging buffer depth (of 256)
```

The live client prints overflow/wait warnings automatically after
capture.

## Tests

```bash
python client/test_trace.py
```

Smoke tests against synthetic data covering instruction decoding,
state tracking, loop detection, and trace filtering.

## Status

- Firmware: written, initial hardware testing in progress
- Host client: written, passes smoke tests against synthetic data
- Z80 decoder: covers all prefix groups (CB, DD, ED, FD, DDCB, FDCB)
- Register state tracking: relies on observed bus data, may drift for
  instructions that aren't fully emulated
- Loop detection: works for PC-sequence and exact-state loops in tests
