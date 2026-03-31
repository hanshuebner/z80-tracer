# Z80 Bus Tracer

A hardware bus tracer for the Z80 CPU using a Pimoroni PGA2350 (RP2350).
Captures all bus activity in real time via PIO, streams it over USB, and
decodes it into a Z80 instruction trace with register state tracking and
loop detection on the host.

**This project is entirely untested on real hardware.** All code was written
from datasheets and has only been verified against synthetic test data. Expect
bugs in timing, pin assignments, protocol handling, and the Z80 instruction
decoder. Use at your own risk and plan to debug.

## Hardware

Connect the PGA2350 directly to the Z80 bus (active accent level conversion
not required for CMOS Z80 variants running at 3.3 V):

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

NMOS Z80 parts output 5 V and will need level shifters.

## Building the firmware

```bash
mkdir build && cd build
cmake ..
make
```

Flash the resulting `.uf2` to the PGA2350.

## Running the host client

```bash
pip install pyserial
python -m client /dev/ttyACM0          # live trace
python -m client --replay test.bin     # replay a captured trace
```

Generate a synthetic test trace:

```bash
python -m client.gen_trace > test.bin
```

## Status

- Firmware: written, **not tested on hardware**
- Host client: written, passes smoke tests against synthetic data only
- Z80 decoder: covers all prefix groups (CB, DD, ED, FD, DDCB, FDCB) but
  has **not been validated against a real Z80**
- Register state tracking: relies on observed bus data, may drift for
  instructions that aren't fully emulated
- Loop detection: works for PC-sequence and exact-state loops in tests
