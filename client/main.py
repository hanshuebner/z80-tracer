#!/usr/bin/env python3
"""Z80 Bus Tracer - Host Client

Reads binary trace data from the PGA2350 tracer over USB CDC (serial),
assembles bus cycles into Z80 instructions, decodes them, tracks register
state, detects loops, and prints a real-time disassembly.

Usage:
    python -m client /dev/ttyACM0
    python -m client /dev/ttyACM0 --baud 115200
    python -m client --replay capture.bin
"""

import argparse
import struct
import sys
import time
from typing import Optional, BinaryIO

from . import z80_decoder as dec
from .z80_state import Z80State
from .z80_loop import LoopDetector


# -- Wire protocol (matches firmware) --

CYCLE_OPCODE_FETCH = 0
CYCLE_MEM_READ = 1
CYCLE_MEM_WRITE = 2
CYCLE_IO_READ = 3
CYCLE_IO_WRITE = 4

CYCLE_NAMES = ("FETCH", "MREAD", "MWRIT", "IOREAD", "IOWRIT")
PACKET_SIZE = 4


def parse_packet(data):
    """Parse a 4-byte trace packet with bit-7 sync framing.

    Format: 1TTTT_AAA 0AAAAAAA 0AAAAAAD 0DDDDDDD
    Returns (cycle_type, addr, value).
    """
    cycle_type = (data[0] >> 3) & 0x0F
    addr = ((data[0] & 0x07) << 13) | (data[1] << 6) | (data[2] >> 1)
    value = ((data[2] & 0x01) << 7) | data[3]
    return cycle_type, addr, value


# -- Instruction assembler --

class InstructionAssembler:
    """Accumulates bus cycles and emits complete Instruction objects.

    State machine that tracks prefix bytes, operand collection, and
    data phase observations to build fully-populated Instruction instances.
    """

    def __init__(self):
        self._reset()

    def _reset(self):
        self._state = "IDLE"
        self._pc = 0
        self._prefix = None       # None, 'CB', 'ED', 'IX'
        self._idx_reg = None      # 'IX' or 'IY'
        self._opcode = 0
        self._operands = []
        self._ops_remaining = 0
        self._ixcb_disp = None
        self._data_reads = []
        self._data_writes = []
        self._io_reads = []
        self._io_writes = []
        self._raw_bytes = []

    def feed(self, cycle_type, addr, value):
        """Feed a bus cycle.  Returns an Instruction when one is complete."""
        if cycle_type == CYCLE_OPCODE_FETCH:
            return self._on_opcode_fetch(addr, value)
        if cycle_type == CYCLE_MEM_READ:
            return self._on_mem_read(addr, value)
        if cycle_type == CYCLE_MEM_WRITE:
            return self._on_mem_write(addr, value)
        if cycle_type == CYCLE_IO_READ:
            self._io_reads.append((addr, value))
            return None
        if cycle_type == CYCLE_IO_WRITE:
            self._io_writes.append((addr, value))
            return None
        return None

    def _on_opcode_fetch(self, addr, value):
        if self._state == "IDLE":
            return self._start_new(addr, value)

        if self._state == "WAIT_IX_OPCODE":
            # After DD/FD prefix, expecting the main opcode or CB/ED
            if value == 0xCB:
                # DDCB / FDCB prefix
                self._prefix = "IXCB"
                self._raw_bytes.append(value)
                self._state = "WAIT_IXCB_DISP"
                return None
            if value == 0xDD:
                # DD DD - restart IX prefix
                self._idx_reg = "IX"
                self._raw_bytes.append(value)
                return None
            if value == 0xFD:
                # DD FD - switch to IY prefix
                self._idx_reg = "IY"
                self._raw_bytes.append(value)
                return None
            if value == 0xED:
                # DD ED - DD is discarded, ED takes over
                self._prefix = "ED"
                self._idx_reg = None
                self._raw_bytes.append(value)
                self._state = "WAIT_ED_OPCODE"
                return None
            # Normal IX/IY opcode
            self._opcode = value
            self._raw_bytes.append(value)
            self._ops_remaining = dec.operand_count_indexed(value)
            self._state = "COLLECTING_OPS" if self._ops_remaining > 0 else "COLLECTING_DATA"
            return None

        if self._state == "WAIT_CB_OPCODE":
            self._opcode = value
            self._raw_bytes.append(value)
            self._state = "COLLECTING_DATA"
            return None

        if self._state == "WAIT_ED_OPCODE":
            self._opcode = value
            self._raw_bytes.append(value)
            self._ops_remaining = dec.operand_count_ed(value)
            self._state = "COLLECTING_OPS" if self._ops_remaining > 0 else "COLLECTING_DATA"
            return None

        # In COLLECTING_OPS or COLLECTING_DATA, a new opcode fetch means
        # the previous instruction is done.
        result = self._finalize()
        self._reset()
        new = self._start_new(addr, value)
        # Return previous instruction (new one is still being assembled)
        return result if result else new

    def _start_new(self, addr, value):
        self._pc = addr
        self._raw_bytes = [value]

        if value == 0xDD or value == 0xFD:
            self._prefix = "IX"
            self._idx_reg = "IX" if value == 0xDD else "IY"
            self._state = "WAIT_IX_OPCODE"
            return None

        if value == 0xCB:
            self._prefix = "CB"
            self._state = "WAIT_CB_OPCODE"
            return None

        if value == 0xED:
            self._prefix = "ED"
            self._state = "WAIT_ED_OPCODE"
            return None

        # Normal unprefixed opcode
        self._opcode = value
        self._ops_remaining = dec.operand_count_main(value)
        self._state = "COLLECTING_OPS" if self._ops_remaining > 0 else "COLLECTING_DATA"
        return None

    def _on_mem_read(self, addr, value):
        if self._state == "WAIT_IXCB_DISP":
            # This memory read is the displacement byte
            self._ixcb_disp = value
            self._raw_bytes.append(value)
            self._state = "WAIT_IXCB_OPCODE"
            return None

        if self._state == "WAIT_IXCB_OPCODE":
            # This memory read is the actual CB opcode
            self._opcode = value
            self._raw_bytes.append(value)
            self._state = "COLLECTING_DATA"
            return None

        if self._state == "COLLECTING_OPS":
            self._operands.append(value)
            self._raw_bytes.append(value)
            self._ops_remaining -= 1
            if self._ops_remaining <= 0:
                self._state = "COLLECTING_DATA"
            return None

        # Data-phase read
        self._data_reads.append((addr, value))
        return None

    def _on_mem_write(self, addr, value):
        self._data_writes.append((addr, value))
        return None

    def _finalize(self):
        instr = dec.decode(
            pc=self._pc,
            prefix=self._prefix,
            opcode=self._opcode,
            operands=self._operands,
            displacement=self._ixcb_disp,
            idx_reg=self._idx_reg,
        )
        instr.raw_bytes = bytes(self._raw_bytes)
        instr.data_reads = self._data_reads
        instr.data_writes = self._data_writes
        instr.io_reads = self._io_reads
        instr.io_writes = self._io_writes
        self._reset()
        return instr

    def flush(self):
        """Flush any in-progress instruction."""
        if self._state != "IDLE":
            return self._finalize()
        return None


# -- Output formatting --

def format_instruction(instr, reg_changes):
    """Format one instruction line for display."""
    # Address
    addr_str = f"{instr.pc:04X}"

    # Raw bytes (up to 4)
    raw = " ".join(f"{b:02X}" for b in instr.raw_bytes[:4])
    raw = raw.ljust(12)

    # Mnemonic
    mnem = instr.mnemonic.ljust(20)

    # Register changes / observations
    parts = []
    if reg_changes:
        parts.append(reg_changes)
    for a, v in instr.data_reads:
        parts.append(f"[{a:04X}]->{v:02X}")
    for a, v in instr.data_writes:
        parts.append(f"[{a:04X}]<-{v:02X}")
    for p, v in instr.io_reads:
        parts.append(f"IN({p:04X})={v:02X}")
    for p, v in instr.io_writes:
        parts.append(f"OUT({p:04X})={v:02X}")

    comment = "; " + " ".join(parts) if parts else ""

    return f"{addr_str}: {raw} {mnem} {comment}"


# -- Main trace loop --

def run_trace(source: BinaryIO, raw_output=False, is_file=False, detect_loops=True):
    assembler = InstructionAssembler()
    state = Z80State()
    loop_det = LoopDetector()
    instr_count = 0

    buf = bytearray()

    try:
        while True:
            chunk = source.read(4096)
            if not chunk:
                if is_file:
                    break  # EOF on replay file
                time.sleep(0.001)
                continue

            buf.extend(chunk)

            while len(buf) >= PACKET_SIZE:
                # Sync: skip bytes until we find one with bit 7 set (packet start)
                while buf and not (buf[0] & 0x80):
                    buf = buf[1:]
                if len(buf) < PACKET_SIZE:
                    break

                pkt = buf[:PACKET_SIZE]
                buf = buf[PACKET_SIZE:]

                cycle_type, addr, value = parse_packet(pkt)

                if raw_output:
                    ct_name = CYCLE_NAMES[cycle_type] if cycle_type < len(CYCLE_NAMES) else "?"
                    print(f"  {ct_name:6s} {addr:04X} {value:02X}")

                instr = assembler.feed(cycle_type, addr, value)
                if instr is None:
                    continue

                # Take state snapshot before execution
                prev_snap = state.snapshot_key()
                state.execute(instr)
                instr_count += 1

                # Loop detection
                state_key = state.snapshot_key()
                action = loop_det.check(instr.pc, state_key) if detect_loops else "normal"

                if action == "normal":
                    reg_changes = state.format_changed(prev_snap)
                    line = format_instruction(instr, reg_changes)
                    print(line)

                elif action == "loop_exit":
                    summary = loop_det.loop_summary()
                    if summary:
                        print(summary)
                    # Print the current instruction that broke the loop
                    reg_changes = state.format_changed(prev_snap)
                    line = format_instruction(instr, reg_changes)
                    print(line)

                # action == "suppress": do nothing

    except KeyboardInterrupt:
        pass

    # Flush any partial instruction
    instr = assembler.flush()
    if instr:
        prev_snap = state.snapshot_key()
        state.execute(instr)
        instr_count += 1
        state_key = state.snapshot_key()
        action = loop_det.check(instr.pc, state_key) if detect_loops else "normal"
        reg_changes = state.format_changed(prev_snap)
        if action == "loop_exit":
            summary = loop_det.loop_summary()
            if summary:
                print(summary)
        if action != "suppress":
            print(format_instruction(instr, reg_changes))

    if loop_det.in_loop:
        summary = loop_det.loop_summary()
        if summary:
            print(summary)

    print(f"\n--- {instr_count} instructions traced ---")
    print(state.format_registers())


# -- Entry point --

def main():
    parser = argparse.ArgumentParser(
        description="Z80 Bus Tracer - Host Client")
    parser.add_argument(
        "port", nargs="?", default=None,
        help="Serial port (e.g. /dev/ttyACM0, COM3)")
    parser.add_argument(
        "--baud", type=int, default=115200,
        help="Baud rate (default: 115200, ignored for CDC)")
    parser.add_argument(
        "--replay", metavar="FILE",
        help="Replay a captured binary trace file instead of live serial")
    parser.add_argument(
        "--raw", action="store_true",
        help="Also print raw bus cycles")
    parser.add_argument(
        "--loops", action="store_true",
        help="Enable loop detection (suppress repeated sequences)")
    parser.add_argument(
        "--hexdump", action="store_true",
        help="Dump raw USB bytes in hex and exit (diagnostic)")
    args = parser.parse_args()

    if args.replay:
        with open(args.replay, "rb") as f:
            run_trace(f, raw_output=args.raw, is_file=True,
                      detect_loops=args.loops)
        return

    if not args.port:
        parser.error("Serial port required (or use --replay)")

    try:
        import serial
    except ImportError:
        print("pyserial is required: pip install pyserial", file=sys.stderr)
        sys.exit(1)

    ser = serial.Serial(args.port, args.baud, timeout=0.1)

    if args.hexdump:
        print(f"Hex dump from {args.port} -- Ctrl+C to stop")
        offset = 0
        try:
            while True:
                chunk = ser.read(4096)
                if not chunk:
                    continue
                for i in range(0, len(chunk), 16):
                    row = chunk[i:i+16]
                    hex_part = " ".join(f"{b:02X}" for b in row)
                    # Mark packet starts (bit 7 set) with '>' prefix
                    ann = "".join(">" if b & 0x80 else " " for b in row)
                    print(f"{offset+i:06X}: {hex_part:<48s}  {ann}")
                offset += len(chunk)
        except KeyboardInterrupt:
            pass
        finally:
            ser.close()
        return

    print(f"Connected to {args.port} -- Ctrl+C to stop")
    try:
        run_trace(ser, raw_output=args.raw, detect_loops=args.loops)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
