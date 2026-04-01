# Z80 Tracer Wiring Guide

**Perfboard setup**: PGA socket for the PGA2350, 40-pin DIP socket for
the Z80. All wiring is done underneath the perfboard between socket pins.

An **interactive wiring aid** is available in `wiring-guide.html` — open
it in a browser and step through the 37 wires one at a time with arrow
keys. It shows the perfboard from the solder side with both sockets and
highlights each wire as you go.

> This wiring guide is derived from the Pimoroni PGA2350 schematic
> (PIM722, 29/08/2024) and the Z80 CPU datasheet. **It is untested.**
> Double-check every connection against the schematic before powering on.

## Voltage Levels

The Z80 runs at 5V. The RP2350 GPIOs on the PGA2350 are considered
5V tolerant enough for direct connection — no level shifters are
needed. The Z80 is powered from its own 5V supply; do **not** connect
the Z80 +5V pin to the PGA2350.

Connect only GND between the two boards (Z80 pin 29 to PGA2350 GND).

## PGA2350 Grid Reference

The PGA2350 is a 10-column by 10-row pin grid. Only the perimeter pins
and rows I/J are populated (64 pins total, including 6 GND pins). Rows
are labeled A-J (top to bottom), columns 1-10 (left to right).

**Orientation**: With the USB connector at the top of the board, pin A1
is at the top-left corner (marked with a dot on the PCB underside).

**IMPORTANT**: When the board is plugged into a PGA socket and you are
wiring underneath, the grid is **mirror-flipped left-to-right** compared
to the top view. The row/column labels in this document refer to the
**top view** (component side) — when you flip the board over, columns
reverse: top-view column 1 is on the RIGHT when looking at solder side.

### PGA2350 Complete Pin Map (top view)

```
        Col1    Col2    Col3    Col4    Col5    Col6    Col7    Col8    Col9    Col10
       ──────  ──────  ──────  ──────  ──────  ──────  ──────  ──────  ──────  ──────
Row A │ GND    GP1    VBUS   3V3_EN  RUN    USB_DP USB_DM BOOTSL  GP46    GND
      │
Row B │ GP2    GND    GP0    3V3    ADC_VRF SWCLK  SWDIO  GP47    GND    GP45
      │
Row C │ GP4    GP3    ---    ---    ---    ---    ---    ---    GP43   GP44
      │
Row D │ GP6    GP5    ---    ---    ---    ---    ---    ---    GP41   GP42
      │
Row E │ GP8    GP7    ---    ---    ---    ---    ---    ---    GP39   GP40
      │
Row F │ GP10   GP9    ---    ---    ---    ---    ---    ---    GP37   GP38
      │
Row G │ GP12   GP11   ---    ---    ---    ---    ---    ---    GP35   GP36
      │
Row H │ GP14   GP13   ---    ---    ---    ---    ---    ---    GP33   GP34
      │
Row I │ GP16   GP15   GP20   GP21   GP23   GP25   GP27   GP29   GP31   GP32
      │
Row J │ GND    GP17   GP18   GP19   GP22   GP24   GP26   GP28   GP30    GND
```

Note: "---" = no pin (interior is empty, chip sits there). "BOOTSL" = BOOTSEL.
Six pins are GND: **A1, A10, B2, B9, J1, J10** (the schematic labels these as "A1*6").

### GPIO to PGA Grid Quick-Reference

```
GPIO  Grid     GPIO  Grid     GPIO  Grid     GPIO  Grid
────  ────     ────  ────     ────  ────     ────  ────
 0    B3        12   G1        24   J6        36   G10
 1    A2        13   H2        25   I6        37   F9
 2    B1        14   H1        26   J7        38   F10
 3    C2        15   I2        27   I7        39   E9
 4    C1        16   I1        28   J8        40   E10
 5    D2        17   J2        29   I8        41   D9
 6    D1        18   J3        30   J9        42   D10
 7    E2        19   J4        31   I9        43   C9
 8    E1        20   I3        32   I10       44   C10
 9    F2        21   I4        33   H9        45   B10
10    F1        22   J5        34   H10       46   A9
11    G2        23   I5        35   G9        47   B8
```

## Z80 DIP-40 Pinout

```
              ┌────U────┐
    A11   1 ──┤         ├── 40  A10
    A12   2 ──┤         ├── 39  A9
    A13   3 ──┤         ├── 38  A8
    A14   4 ──┤         ├── 37  A7
    A15   5 ──┤         ├── 36  A6
    CLK   6 ──┤         ├── 35  A5
     D4   7 ──┤         ├── 34  A4
     D3   8 ──┤         ├── 33  A3
     D5   9 ──┤         ├── 32  A2
     D6  10 ──┤         ├── 31  A1
    +5V  11 ──┤  Z80    ├── 30  A0
     D2  12 ──┤         ├── 29  GND
     D7  13 ──┤         ├── 28  /RFSH
     D0  14 ──┤         ├── 27  /M1
     D1  15 ──┤         ├── 26  /RESET
    /INT  16 ──┤         ├── 25  /BUSRQ
    /NMI  17 ──┤         ├── 24  /WAIT
   /HALT  18 ──┤         ├── 23  /BUSACK
   /MREQ  19 ──┤         ├── 22  /WR
   /IORQ  20 ──┤         ├── 21  /RD
              └─────────┘
```

## Wiring Table

37 wires total. The recommended soldering order alternates outside-in
on the DIP socket: pins 40, 1, 39, 2, 38, 3, ... — see the interactive
wiring aid (`wiring-guide.html`) which presents the wires in this order.

The table below is sorted by Z80 pin number for reference.

```
 Z80                    PGA    PGA
 pin  Signal    GPIO    grid   Find it (counting from top-view position)
────  ────────  ─────   ─────  ─────────────────────────────────────────
  1   A11       GP11    G2     Row G, 2nd from left edge
  2   A12       GP12    G1     Row G, leftmost
  3   A13       GP13    H2     Row H, 2nd from left edge
  4   A14       GP14    H1     Row H, leftmost
  5   A15       GP15    I2     Row I, 2nd from left edge
  6   CLK       GP30    J9     Row J (bottom), 9th from left
  7   D4        GP20    I3     Row I, 3rd from left
  8   D3        GP19    J4     Row J, 4th from left
  9   D5        GP21    I4     Row I, 4th from left
 10   D6        GP22    J5     Row J, 5th from left
 12   D2        GP18    J3     Row J, 3rd from left
 13   D7        GP23    I5     Row I, 5th from left
 14   D0        GP16    I1     Row I, leftmost
 15   D1        GP17    J2     Row J, 2nd from left
 16   /INT      GP33    H9     Row H, 9th from left
 17   /NMI      GP34    H10    Row H, rightmost
 18   /HALT     GP31    I9     Row I, 9th from left
 19   /MREQ     GP25    I6     Row I, 6th from left
 20   /IORQ     GP26    J7     Row J, 7th from left
 21   /RD       GP27    I7     Row I, 7th from left
 22   /WR       GP28    J8     Row J, 8th from left
 24   /WAIT     GP32    I10    Row I, rightmost
 26   /RESET    GP35    G9     Row G, 9th from left
 27   /M1       GP24    J6     Row J, 6th from left
 28   /RFSH     GP29    I8     Row I, 8th from left
 29   GND       GND     A1     Row A, leftmost (any of the 6 GND pins will do)
 30   A0        GP0     B3     Row B, 3rd from left
 31   A1        GP1     A2     Row A, 2nd from left
 32   A2        GP2     B1     Row B, leftmost
 33   A3        GP3     C2     Row C, 2nd from left
 34   A4        GP4     C1     Row C, leftmost
 35   A5        GP5     D2     Row D, 2nd from left
 36   A6        GP6     D1     Row D, leftmost
 37   A7        GP7     E2     Row E, 2nd from left
 38   A8        GP8     E1     Row E, leftmost
 39   A9        GP9     F2     Row F, 2nd from left
 40   A10       GP10    F1     Row F, leftmost
```

Not connected: Z80 pin 11 (+5V), pin 23 (/BUSACK), pin 25 (/BUSRQ).

## GND Connections

Connect Z80 GND (pin 29) to any PGA2350 GND pin (A1, A10, B2, B9, J1,
or J10 — they are all connected). Use whichever is most convenient for
routing. If your perfboard allows it, run additional GND wires between
the boards to reduce noise.

## Checklist

After wiring, verify with a continuity tester:

- [ ] All 16 address lines (A0-A15): Z80 pins 30-40,1-5
- [ ] All 8 data lines (D0-D7): Z80 pins 14,15,12,8,7,9,10,13
- [ ] /M1 (pin 27), /MREQ (pin 19), /IORQ (pin 20)
- [ ] /RD (pin 21), /WR (pin 22), /RFSH (pin 28)
- [ ] CLK (pin 6), /HALT (pin 18), /WAIT (pin 24)
- [ ] /INT (pin 16), /NMI (pin 17), /RESET (pin 26)
- [ ] GND (pin 29)
- [ ] No shorts between adjacent pins
- [ ] Z80 +5V (pin 11) is NOT connected to PGA2350
