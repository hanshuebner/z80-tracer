# Z80 Tracer Wiring Guide

**Perfboard setup**: PGA socket for the PGA2350, 40-pin DIP socket for the Z80.
All wiring is done underneath the perfboard between socket pins.

> This wiring guide is derived from the Pimoroni PGA2350 schematic
> (PIM722, 29/08/2024) and the Z80 CPU datasheet. **It is untested.**
> Double-check every connection against the schematic before powering on.

## PGA2350 Grid Reference

The PGA2350 is a 10-column by 10-row pin grid, but only the perimeter
pins and row I/J are populated (64 pins total). Rows are labeled A-J
(top to bottom), columns 1-10 (left to right).

**Orientation**: With the USB connector at the top of the board, pin A1
is at the top-left corner (marked with a dot on the PCB underside).

**IMPORTANT**: When the board is plugged into a PGA socket and you are
wiring underneath, the grid is **mirror-flipped left-to-right** compared
to the top view. Column 1 on top becomes column 1 on your right when
looking at the bottom of the socket. The row/column labels in this
document refer to the **top view** (component side) — when you flip the
board over, columns reverse: top-view column 1 is on the RIGHT when
looking at solder side.

### PGA2350 Complete Pin Map (top view)

```
        Col1    Col2    Col3    Col4    Col5    Col6    Col7    Col8    Col9    Col10
       ──────  ──────  ──────  ──────  ──────  ──────  ──────  ──────  ──────  ──────
Row A │ GND    GP1    VBUS   3V3_EN  RUN    USB_DP USB_DM BOOTSL  GP46    ---
      │ (x6)
Row B │ GP2    ---    GP0    3V3    ADC_VRF SWCLK  SWDIO  GP47    ---    GP45
      │
Row C │ GP4    GP3    ---    ---    ---    ---    ---    ---    GP43   GP44
      │
Row D │ GP6    GP5    ---    ---    ---    ---    ---    ---    GP42   GP41
      │
Row E │ GP8    GP7    ---    ---    ---    ---    ---    ---    GP39   GP40
      │
Row F │ GP10   GP9    ---    ---    ---    ---    ---    ---    GP37   GP38
      │
Row G │ GP12   GP11   ---    ---    ---    ---    ---    ---    GP35   GP36
      │
Row H │ GP14   GP13   ---    ---    ---    ---    ---    ---    GP33   GP34
      │
Row I │ GP16   GP15   GP20   GP21   GP22   GP25   GP27   GP28   GP31   GP32
      │
Row J │ ---    GP17   GP18   GP19   GP23   GP24   GP26   GP29   GP30    ---
```

Note: "---" = no pin at that position.  "BOOTSL" = BOOTSEL.
A1 is labeled "GND x6" on the schematic (multiple GND vias).

### GPIO to PGA Grid Quick-Reference

```
GPIO  Grid     GPIO  Grid     GPIO  Grid     GPIO  Grid
────  ────     ────  ────     ────  ────     ────  ────
 0    B3        12   G1        24   J6        36   G10
 1    A2        13   H2        25   I6        37   F9
 2    B1        14   H1        26   J7        38   F10
 3    C2        15   I2        27   I7        39   E9
 4    C1        16   I1        28   I8        40   E10
 5    D2        17   J2        29   J8        41   D10
 6    D1        18   J3        30   J9        42   D9
 7    E2        19   J4        31   I9        43   C9
 8    E1        20   I3        32   I10       44   C10
 9    F2        21   I4        33   H9        45   B10
10    F1        22   I5        34   H10       46   A9
11    G2        23   J5        35   G9        47   B8
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

Each row is one wire. Work through them in order — the list is sorted
by Z80 pin number so you can work your way around the DIP socket.

The "PGA grid" column tells you which pin to find on the PGA socket
underneath the perfboard. Remember: **columns are mirrored** on the
solder side.

The "Find it" hint tells you how to locate the PGA pin by counting
from a known landmark (corner or edge).

```
 Z80                    PGA    PGA
 pin  Signal    GPIO    grid   Find it (counting from top-view position)
────  ────────  ─────   ─────  ─────────────────────────────────────────
  1   A11       GP11    G2     Row G, 2nd from left edge
  2   A12       GP12    G1     Row G, 1st from left edge (leftmost)
  3   A13       GP13    H2     Row H, 2nd from left edge
  4   A14       GP14    H1     Row H, 1st from left edge (leftmost)
  5   A15       GP15    I2     Row I, 2nd from left edge
  6   CLK       GP30    J9     Row J (bottom), 9th from left
  7   D4        GP20    I3     Row I, 3rd from left
  8   D3        GP19    J4     Row J (bottom), 4th from left
  9   D5        GP21    I4     Row I, 4th from left
 10   D6        GP22    I5     Row I, 5th from left
 11   +5V       ---     ---    DO NOT connect to PGA2350! (see note below)
 12   D2        GP18    J3     Row J (bottom), 3rd from left
 13   D7        GP23    J5     Row J (bottom), 5th from left
 14   D0        GP16    I1     Row I, 1st from left (leftmost)
 15   D1        GP17    J2     Row J (bottom), 2nd from left
 16   /INT      ---     ---    not connected (directly route to your interrupt source)
 17   /NMI      ---     ---    not connected
 18   /HALT     GP31    I9     Row I, 9th from left
 19   /MREQ     GP25    I6     Row I, 6th from left
 20   /IORQ     GP26    J7     Row J (bottom), 7th from left
 21   /RD       GP27    I7     Row I, 7th from left
 22   /WR       GP28    I8     Row I, 8th from left
 24   /WAIT     GP32    I10    Row I, 10th from left (rightmost)
 27   /M1       GP24    J6     Row J (bottom), 6th from left
 28   /RFSH     GP29    J8     Row J (bottom), 8th from left
 29   GND       GND     A1     Row A, 1st from left (top-left corner)
 30   A0        GP0     B3     Row B, 3rd from left
 31   A1        GP1     A2     Row A, 2nd from left
 32   A2        GP2     B1     Row B, 1st from left (leftmost)
 33   A3        GP3     C2     Row C, 2nd from left
 34   A4        GP4     C1     Row C, 1st from left (leftmost)
 35   A5        GP5     D2     Row D, 2nd from left
 36   A6        GP6     D1     Row D, 1st from left (leftmost)
 37   A7        GP7     E2     Row E, 2nd from left
 38   A8        GP8     E1     Row E, 1st from left (leftmost)
 39   A9        GP9     F2     Row F, 2nd from left
 40   A10       GP10    F1     Row F, 1st from left (leftmost)
```

## Power

**Do NOT connect the Z80 +5V (pin 11) to the PGA2350.**

If you are running a **CMOS Z80 (Z84C00) at 3.3V**:
- Power the Z80 from the PGA2350's **3V3** pin (grid **B4**)
- Connect Z80 pin 11 (+5V) to PGA2350 grid B4 (3V3 output)
- Connect Z80 pin 29 (GND) to PGA2350 grid A1 (GND)

If you are running the Z80 at 5V you need level shifters on every
signal line. This board layout assumes direct 3.3V connection.

## GND Connections

Connect Z80 GND (pin 29) to PGA2350 GND (grid A1). If your perfboard
allows it, run additional GND connections to reduce noise.

## Wiring Strategy

Recommended order to minimize crossed wires on the perfboard:

**Phase 1 — Address bus low (Z80 pins 30-40, right side of DIP)**
These map to PGA rows B through F, columns 1-2 (left edge of PGA grid).
Work from Z80 pin 30 (A0) up to pin 40 (A10).

**Phase 2 — Address bus high (Z80 pins 1-5, left side of DIP)**
These map to PGA rows G through I, columns 1-2.
Work from Z80 pin 1 (A11) down to pin 5 (A15).

**Phase 3 — Data bus (Z80 pins 7-15, left side of DIP)**
These map to PGA rows I-J, columns 1-5.
Note the Z80 data bus pins are NOT in order (D4,D3,D5,D6,D2,D7,D0,D1).

**Phase 4 — Control signals (Z80 pins 18-28)**
These map to PGA rows I-J, columns 6-10 (right side of PGA grid).

**Phase 5 — Power and clock**
- GND: Z80 pin 29 → PGA A1
- 3V3: Z80 pin 11 → PGA B4
- CLK: Z80 pin 6 → PGA J9

## Checklist

After wiring, verify with a continuity tester:

- [ ] All 16 address lines (A0-A15): Z80 pins 30-40,1-5 ↔ GP0-GP15
- [ ] All 8 data lines (D0-D7): Z80 pins 14,15,12,8,7,9,10,13 ↔ GP16-GP23
- [ ] /M1 (pin 27), /MREQ (pin 19), /IORQ (pin 20)
- [ ] /RD (pin 21), /WR (pin 22), /RFSH (pin 28)
- [ ] CLK (pin 6), /HALT (pin 18), /WAIT (pin 24)
- [ ] GND (pin 29) and power (pin 11)
- [ ] No shorts between adjacent pins
- [ ] No connection between Z80 +5V and PGA2350 VBUS
