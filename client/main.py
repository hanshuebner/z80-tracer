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
import io
import struct
import sys
import time
from typing import Optional, BinaryIO

from . import z80_decoder as dec
from .z80_state import Z80State
from .z80_loop import LoopDetector
from .trace_filter import TraceFilter, FilterConfig, parse_address_range, parse_address_or_range, parse_trigger_addr
from .memory_trace import MemoryTracer


# -- Wire protocol (matches firmware z80_trace.h) --
# 4-byte trace_record_t: address(u16 LE), data(u8), type_wait_flags(u8)
# type_wait_flags packing: [cycle_type:3 | wait_count:3 | halt:1 | int:1]

CYCLE_OPCODE_FETCH = 0  # M1 opcode fetch (CYCLE_M1_FETCH in firmware)
CYCLE_MEM_READ = 1
CYCLE_MEM_WRITE = 2
CYCLE_IO_READ = 3
CYCLE_IO_WRITE = 4
CYCLE_INT_ACK = 5
CYCLE_RESET = 6

CYCLE_NAMES = ("FETCH", "MREAD", "MWRIT", "IOREAD", "IOWRIT", "INTACK", "RESET")
RECORD_SIZE = 4  # bytes per trace_record_t


def parse_record(data):
    """Parse a 4-byte trace record (trace_record_t).

    Returns (cycle_type, addr, value, wait_count, halt).
    """
    addr = data[0] | (data[1] << 8)
    value = data[2]
    packed = data[3]
    cycle_type = (packed >> 5) & 0x7
    wait_count = (packed >> 2) & 0x7
    halt = bool(packed & 0x2)
    return cycle_type, addr, value, wait_count, halt


# -- Instruction assembler --

class InstructionAssembler:
    """Accumulates bus cycles and emits complete Instruction objects.

    State machine that tracks prefix bytes, operand collection, and
    data phase observations to build fully-populated Instruction instances.
    """

    def __init__(self):
        self._reset()
        # Interrupt state — set by feed(), read and cleared by caller
        self.int_ack = None    # (addr, vector) when INT acknowledge detected
        self.nmi_detected = False  # True when NMI detected
        self._int_pending = False  # suppress duplicate INTACKs until next FETCH

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
        """Feed a bus cycle.  Returns an Instruction when one is complete.

        After calling feed(), check:
          - self.int_ack: set to (addr, vector) if INT acknowledge was seen
          - self.nmi_detected: set to True if NMI was detected
        Caller should clear these after handling.
        """
        if cycle_type == CYCLE_OPCODE_FETCH:
            return self._on_opcode_fetch(addr, value)
        if cycle_type == CYCLE_INT_ACK:
            return self._on_int_ack(addr, value)
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

    def _on_int_ack(self, addr, value):
        """Handle INT acknowledge cycle.  Finalizes current instruction.

        On a Z80, INTACK only occurs after the current instruction completes.
        The 2 internal T-states before INTACK may produce misclassified bus
        cycles (MREAD/MWRITE) from the firmware — strip those for instructions
        known to have no data-phase memory access.  Duplicate INTACKs
        (likely misclassified internal T-states) are suppressed until the
        next opcode fetch.
        """
        if self._int_pending:
            return None  # suppress duplicate INTACK
        self._int_pending = True
        self.int_ack = (addr, value)
        if self._state != "IDLE":
            if self._instruction_has_no_data_phase():
                self._data_reads = []
                self._data_writes = []
            result = self._finalize()
            self._reset()
            return result
        return None

    def _instruction_has_no_data_phase(self):
        """True if the instruction definitely has no data-phase memory ops.

        Conservative: returns False (keep data) for unknown instructions.
        """
        op = self._opcode

        if self._prefix == "IXCB":
            return False  # always accesses (IX+d)/(IY+d)

        if self._prefix == "CB":
            return (op & 0x07) != 6  # register variants only

        if self._prefix == "ED":
            if 0xA0 <= op <= 0xBB:  # block ops
                return False
            if (op & 0xC7) == 0x43:  # LD (nn),rr / LD rr,(nn)
                return False
            if op in (0x45, 0x4D, 0x55, 0x5D, 0x65, 0x6D, 0x75, 0x7D):
                return False  # RETN/RETI (pop PC)
            return True  # NEG, IM, LD I/R/A, IN/OUT (C)

        if self._prefix == "IX":
            if (op & 0xCF) in (0xC1, 0xC5):  # POP/PUSH IX/IY
                return False
            if op == 0xE3:  # EX (SP),IX/IY
                return False
            if op in (0x34, 0x35, 0x36):  # INC/DEC/LD (IX+d)
                return False
            if 0x40 <= op <= 0x7F and op != 0x76:
                if (op & 0x07) == 6 or (op & 0x38) == 0x30:
                    return False  # accesses (IX+d)
            if 0x80 <= op <= 0xBF and (op & 0x07) == 6:
                return False
            return True

        # Main table (no prefix)
        if (op & 0xCF) in (0xC1, 0xC5):  # POP/PUSH
            return False
        if op == 0xC9 or (op & 0xC7) == 0xC0:  # RET / RET cc
            return False
        if op == 0xCD or (op & 0xC7) == 0xC4:  # CALL / CALL cc
            return False
        if (op & 0xC7) == 0xC7:  # RST
            return False
        if op == 0xE3:  # EX (SP),HL
            return False
        if op in (0x02, 0x0A, 0x12, 0x1A, 0x22, 0x2A, 0x32, 0x3A):
            return False  # LD (BC/DE/nn),A / LD A,(BC/DE/nn) / LD (nn),HL / LD HL,(nn)
        if op in (0x34, 0x35, 0x36):  # INC/DEC (HL), LD (HL),n
            return False
        if 0x40 <= op <= 0x7F and op != 0x76:
            if (op & 0x07) == 6 or (op & 0x38) == 0x30:
                return False  # LD r,(HL) or LD (HL),r
        if 0x80 <= op <= 0xBF and (op & 0x07) == 6:
            return False  # ALU A,(HL)

        return True

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

        # NMI detection: if the new fetch is at 0x0066 and the current
        # instruction shouldn't branch there, the last 2 data_writes are
        # the NMI's stack push — strip them from the instruction.
        if addr == 0x0066:
            if len(self._data_writes) >= 2:
                self._data_writes = self._data_writes[:-2]
            self.nmi_detected = True

        result = self._finalize()
        self._reset()
        new = self._start_new(addr, value)
        # Return previous instruction (new one is still being assembled)
        return result if result else new

    def _start_new(self, addr, value):
        # Clear any stray bus cycles from between instructions
        # (e.g., INT dispatch stack writes accumulated while IDLE)
        self._data_reads = []
        self._data_writes = []
        self._io_reads = []
        self._io_writes = []
        self._int_pending = False

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


# -- Decode records into structured output (for live capture with trimming) --

def _decode_to_lines(raw_bytes, raw_output=False, detect_loops=True,
                     filter_config=None, memory_tracer=None,
                     show_interrupts=True, watch_ranges=None):
    """Decode raw trace record bytes into a list of instruction entries.

    Each entry is a dict:
      {'instr_idx': int, 'record_end': int, 'lines': [str, ...]}

    record_end is the record index (0-based) past the last record consumed
    by this instruction, used to map trigger offsets to instruction indices.
    """
    assembler = InstructionAssembler()
    state = Z80State()
    loop_det = LoopDetector()
    trace_filter = (TraceFilter(filter_config, format_instruction)
                    if filter_config and filter_config.has_any_config else None)
    watching = watch_ranges or []
    watch_cur = False
    watch_prev = False
    raw_buf = []
    output = []
    instr_count = 0
    record_idx = 0

    data = raw_bytes
    while record_idx * RECORD_SIZE + RECORD_SIZE <= len(data):
        rec = data[record_idx * RECORD_SIZE:(record_idx + 1) * RECORD_SIZE]
        record_idx += 1
        cycle_type, addr, value, wait_count, halt = parse_record(rec)

        if memory_tracer:
            if cycle_type == CYCLE_OPCODE_FETCH:
                memory_tracer.record_fetch(addr)
            elif cycle_type == CYCLE_MEM_READ:
                memory_tracer.record_read(addr)
            elif cycle_type == CYCLE_MEM_WRITE:
                memory_tracer.record_write(addr)

        if watching and cycle_type in (
                CYCLE_OPCODE_FETCH, CYCLE_MEM_READ, CYCLE_MEM_WRITE):
            if cycle_type == CYCLE_OPCODE_FETCH:
                watch_prev = watch_cur
                watch_cur = False
            for ws, we in watching:
                if ws <= addr <= we:
                    watch_cur = True
                    break

        if raw_output:
            ct_name = CYCLE_NAMES[cycle_type] if cycle_type < len(CYCLE_NAMES) else "?"
            extras = ""
            if wait_count:
                extras += f" W={wait_count}"
            if halt:
                extras += " HALT"
            raw_buf.append(f"  {ct_name:6s} {addr:04X} {value:02X}{extras}")

        filter_active = (not trace_filter or trace_filter._active)

        if cycle_type == CYCLE_RESET:
            raw_buf.clear()
            if show_interrupts and filter_active:
                output.append({
                    'instr_idx': instr_count,
                    'record_end': record_idx,
                    'lines': ["; --- RESET ---"],
                })
            continue

        instr = assembler.feed(cycle_type, addr, value)

        int_lines = []
        if assembler.int_ack:
            iaddr, ivec = assembler.int_ack
            assembler.int_ack = None
            if show_interrupts and filter_active:
                int_lines.append(
                    f"; --- INT acknowledge at ${iaddr:04X}, vector=${ivec:02X} ---")
        if assembler.nmi_detected:
            assembler.nmi_detected = False
            if show_interrupts and filter_active:
                int_lines.append("; --- NMI ---")

        if instr is None:
            if int_lines:
                output.append({
                    'instr_idx': instr_count,
                    'record_end': record_idx,
                    'lines': int_lines,
                })
            continue

        if raw_output:
            if cycle_type == CYCLE_OPCODE_FETCH:
                instr_raw = raw_buf[:-1]
                raw_buf = raw_buf[-1:]
            else:
                instr_raw = raw_buf
                raw_buf = []
        else:
            instr_raw = []

        prev_snap = state.snapshot_key()
        state.execute(instr)
        instr_count += 1

        state_key = state.snapshot_key()
        loop_action = loop_det.check(instr.pc, state_key) if detect_loops else "normal"
        reg_changes = state.format_changed(prev_snap)

        if watching:
            show = watch_prev if cycle_type == CYCLE_OPCODE_FETCH else watch_cur
        else:
            show = True

        lines = list(int_lines)
        if trace_filter:
            result = trace_filter.evaluate(instr, reg_changes, loop_action)
            if show and result.lines:
                lines.extend(instr_raw)
                lines.extend(result.lines)
        elif show:
            if loop_action == "normal":
                lines.extend(instr_raw)
                lines.append(format_instruction(instr, reg_changes))
            elif loop_action == "loop_exit":
                summary = loop_det.loop_summary()
                if summary:
                    lines.append(summary)
                lines.extend(instr_raw)
                lines.append(format_instruction(instr, reg_changes))

        if lines:
            output.append({
                'instr_idx': instr_count - 1,
                'record_end': record_idx,
                'lines': lines,
            })

    # Flush partial instruction
    instr = assembler.flush()
    if instr:
        prev_snap = state.snapshot_key()
        state.execute(instr)
        instr_count += 1
        state_key = state.snapshot_key()
        action = loop_det.check(instr.pc, state_key) if detect_loops else "normal"
        reg_changes = state.format_changed(prev_snap)
        lines = list(raw_buf)
        if action != "suppress":
            if action == "loop_exit":
                summary = loop_det.loop_summary()
                if summary:
                    lines.append(summary)
            lines.append(format_instruction(instr, reg_changes))
        if lines:
            output.append({
                'instr_idx': instr_count - 1,
                'record_end': record_idx,
                'lines': lines,
            })

    if loop_det.in_loop:
        summary = loop_det.loop_summary()
        if summary:
            output.append({
                'instr_idx': instr_count - 1,
                'record_end': record_idx,
                'lines': [summary],
            })

    print(f"\n--- {instr_count} instructions, {record_idx} records decoded ---",
          file=sys.stderr)
    print(state.format_registers(), file=sys.stderr)

    return output


# -- Main trace loop (for replay files and streaming) --

def run_trace(source: BinaryIO, raw_output=False, is_file=False,
              detect_loops=True, filter_config=None,
              max_frames=None, max_instructions=None,
              memory_tracer=None, show_interrupts=True,
              stop_fn=None, watch_ranges=None):
    assembler = InstructionAssembler()
    state = Z80State()
    loop_det = LoopDetector()
    raw_buf = []  # buffered raw cycle lines, flushed with their decoded instruction
    trace_filter = (TraceFilter(filter_config, format_instruction)
                    if filter_config and filter_config.has_any_config else None)
    instr_count = 0
    frame_count = 0
    # Watch: list of (start, end) ranges; if set, only show instructions that touch them
    watching = watch_ranges or []
    watch_cur = False   # accumulates hits for the instruction being assembled
    watch_prev = False  # hits for the instruction about to be emitted

    buf = bytearray()
    stopping = False  # True after stop command sent, waiting for TRACE_STOP

    try:
        while True:
            chunk = source.read(4096)
            if not chunk:
                if is_file:
                    break  # EOF on replay file
                time.sleep(0.001)
                continue

            buf.extend(chunk)

            while len(buf) >= RECORD_SIZE:
                rec = buf[:RECORD_SIZE]
                buf = buf[RECORD_SIZE:]

                cycle_type, addr, value, wait_count, halt = parse_record(rec)

                frame_count += 1
                if max_frames is not None and frame_count > max_frames:
                    if stop_fn:
                        stop_fn()
                    raise StopIteration

                if memory_tracer:
                    if cycle_type == CYCLE_OPCODE_FETCH:
                        memory_tracer.record_fetch(addr)
                    elif cycle_type == CYCLE_MEM_READ:
                        memory_tracer.record_read(addr)
                    elif cycle_type == CYCLE_MEM_WRITE:
                        memory_tracer.record_write(addr)

                # Check watch ranges — accumulate per instruction
                if watching and cycle_type in (
                        CYCLE_OPCODE_FETCH, CYCLE_MEM_READ, CYCLE_MEM_WRITE):
                    if cycle_type == CYCLE_OPCODE_FETCH:
                        # New instruction boundary: save current hits for the
                        # instruction about to be emitted, start fresh
                        watch_prev = watch_cur
                        watch_cur = False
                    for ws, we in watching:
                        if ws <= addr <= we:
                            watch_cur = True
                            break

                if raw_output:
                    ct_name = CYCLE_NAMES[cycle_type] if cycle_type < len(CYCLE_NAMES) else "?"
                    extras = ""
                    if wait_count:
                        extras += f" W={wait_count}"
                    if halt:
                        extras += " HALT"
                    raw_buf.append(f"  {ct_name:6s} {addr:04X} {value:02X}{extras}")

                # Check if filter has already stopped tracing
                filter_active = (not trace_filter or trace_filter._active)

                if cycle_type == CYCLE_RESET:
                    raw_buf.clear()
                    if show_interrupts and filter_active:
                        print(f"; --- RESET ---")
                    continue

                instr = assembler.feed(cycle_type, addr, value)

                # Check for interrupt markers (set by assembler)
                if assembler.int_ack:
                    iaddr, ivec = assembler.int_ack
                    assembler.int_ack = None
                    if show_interrupts and filter_active:
                        print(f"; --- INT acknowledge at ${iaddr:04X}, vector=${ivec:02X} ---")
                if assembler.nmi_detected:
                    assembler.nmi_detected = False
                    if show_interrupts and filter_active:
                        print(f"; --- NMI ---")

                if instr is None:
                    continue

                # Split raw lines: if triggered by an opcode fetch, the last
                # raw line belongs to the *next* instruction — carry it over.
                if raw_output:
                    if cycle_type == CYCLE_OPCODE_FETCH:
                        instr_raw = raw_buf[:-1]
                        raw_buf = raw_buf[-1:]
                    else:
                        instr_raw = raw_buf
                        raw_buf = []
                else:
                    instr_raw = []

                # Take state snapshot before execution
                prev_snap = state.snapshot_key()
                state.execute(instr)
                instr_count += 1

                if max_instructions is not None and instr_count > max_instructions:
                    if stop_fn:
                        stop_fn()
                    raise StopIteration

                # Loop detection
                state_key = state.snapshot_key()
                loop_action = loop_det.check(instr.pc, state_key) if detect_loops else "normal"
                reg_changes = state.format_changed(prev_snap)

                # Watch filter: use watch_prev (set at the OPCODE_FETCH boundary)
                # for instructions emitted by a new fetch; use watch_cur for
                # instructions emitted by other events (INTACK, flush).
                if watching:
                    if cycle_type == CYCLE_OPCODE_FETCH:
                        show = watch_prev
                    else:
                        show = watch_cur
                else:
                    show = True

                if trace_filter:
                    result = trace_filter.evaluate(instr, reg_changes, loop_action)
                    if show and result.lines:
                        for line in instr_raw:
                            print(line)
                        for line in result.lines:
                            print(line)
                    if result.stop:
                        if stop_fn:
                            stop_fn()
                        raise StopIteration
                elif show:
                    if loop_action == "normal":
                        for line in instr_raw:
                            print(line)
                        print(format_instruction(instr, reg_changes))
                    elif loop_action == "loop_exit":
                        summary = loop_det.loop_summary()
                        if summary:
                            print(summary)
                        for line in instr_raw:
                            print(line)
                        print(format_instruction(instr, reg_changes))

    except KeyboardInterrupt:
        if not stopping and stop_fn:
            stop_fn()
            stopping = True
    except StopIteration:
        pass

    # Flush any partial instruction
    instr = assembler.flush()
    if instr:
        show_flush = not watching or watch_cur
        if show_flush and raw_output:
            for line in raw_buf:
                print(line)
        raw_buf = []
        prev_snap = state.snapshot_key()
        state.execute(instr)
        instr_count += 1
        state_key = state.snapshot_key()
        action = loop_det.check(instr.pc, state_key) if detect_loops else "normal"
        reg_changes = state.format_changed(prev_snap)
        if show_flush:
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

    print(f"\n--- {instr_count} instructions, {frame_count} frames traced ---",
          file=sys.stderr)
    print(state.format_registers(), file=sys.stderr)

    return {
        "frames_received": frame_count,
    }


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
        "--no-interrupts", action="store_true",
        help="Suppress INT/NMI/RESET event markers in output")
    parser.add_argument(
        "--hexdump", action="store_true",
        help="Dump raw USB bytes in hex and exit (diagnostic)")

    # Address watching
    parser.add_argument(
        "--watch", action="append", default=[], metavar="ADDR|START-END",
        help="Show instructions that access address or range (repeatable)")

    # Filtering and triggering
    parser.add_argument(
        "--mask", action="append", default=[], metavar="START-END",
        help="Suppress output for PC in range (e.g., 0xF000-0xFFFF)")
    parser.add_argument(
        "--start-at", action="append", default=[], metavar="ADDR|START-END",
        help="Start tracing when PC reaches address or enters range (repeatable)")
    parser.add_argument(
        "--stop-at", type=lambda x: int(x, 0), default=None, metavar="ADDR",
        help="Stop tracing when PC reaches this address")
    parser.add_argument(
        "--start-on", default=None, metavar="PATTERN",
        help="Start tracing on mnemonic match (regex)")
    parser.add_argument(
        "--stop-on", default=None, metavar="PATTERN",
        help="Stop tracing on mnemonic match (regex)")
    parser.add_argument(
        "--limit", type=int, default=None, metavar="N",
        help="Stop after printing N instructions")
    parser.add_argument(
        "--trigger-mem-read", action="append", default=[], metavar="ADDR[=VAL]",
        help="Trigger on memory read (e.g., 0xC000 or 0xC000=0xFF)")
    parser.add_argument(
        "--trigger-mem-write", action="append", default=[], metavar="ADDR[=VAL]",
        help="Trigger on memory write")
    parser.add_argument(
        "--trigger-io-read", action="append", default=[], metavar="PORT[=VAL]",
        help="Trigger on I/O port read")
    parser.add_argument(
        "--trigger-io-write", action="append", default=[], metavar="PORT[=VAL]",
        help="Trigger on I/O port write")
    parser.add_argument(
        "--trigger-int", action="store_true",
        help="Trigger on interrupt (unexpected PC jump to RST vector)")
    # Trigger window control (validated manually for mutual exclusion)
    parser.add_argument(
        "--window", type=int, default=None, metavar="N",
        help="Show N instructions before and after trigger")
    parser.add_argument(
        "--pre", type=int, default=None, metavar="N",
        help="Show N instructions before trigger")
    parser.add_argument(
        "--post", type=int, default=None, metavar="N",
        help="Show N instructions after trigger")
    parser.add_argument(
        "--pre-all", action="store_true",
        help="Use all PSRAM for pre-trigger (half if --post-all also set)")
    parser.add_argument(
        "--post-all", action="store_true",
        help="Use all PSRAM for post-trigger (half if --pre-all also set)")
    parser.add_argument(
        "--max-instructions", type=int, default=None, metavar="N",
        help="Stop after processing N instructions (regardless of filtering)")
    parser.add_argument(
        "--memory-report", action="store_true",
        help="Track memory access and print report on exit")
    parser.add_argument(
        "--memory-report-format", default="text", choices=["text"],
        help="Memory report format (default: text)")

    args = parser.parse_args()

    # Validate mutually exclusive window/pre/post options
    has_window = args.window is not None
    has_pre_post = args.pre is not None or args.post is not None
    has_all = args.pre_all or args.post_all
    if has_window and (has_pre_post or has_all):
        parser.error("--window cannot be combined with --pre/--post/--pre-all/--post-all")
    if has_pre_post and has_all:
        parser.error("--pre/--post cannot be combined with --pre-all/--post-all")
    if args.pre_all and args.pre is not None:
        parser.error("--pre-all cannot be combined with --pre")
    if args.post_all and args.post is not None:
        parser.error("--post-all cannot be combined with --post")

    # Build filter config from args
    fcfg = FilterConfig(
        address_masks=[parse_address_range(m) for m in args.mask],
        start_at=[parse_address_or_range(s) for s in args.start_at],
        stop_at=args.stop_at,
        start_on=args.start_on,
        stop_on=args.stop_on,
        limit=args.limit,
        mem_read_triggers=[parse_trigger_addr(t) for t in args.trigger_mem_read],
        mem_write_triggers=[parse_trigger_addr(t) for t in args.trigger_mem_write],
        io_read_triggers=[parse_trigger_addr(t) for t in args.trigger_io_read],
        io_write_triggers=[parse_trigger_addr(t) for t in args.trigger_io_write],
        trigger_int=args.trigger_int,
        window_size=args.window or 0,
    )

    mem_tracer = MemoryTracer() if args.memory_report else None
    watch = [parse_address_or_range(w) for w in args.watch]

    if args.replay:
        with open(args.replay, "rb") as f:
            run_trace(f, raw_output=args.raw, is_file=True,
                      detect_loops=args.loops, filter_config=fcfg,
                      max_instructions=args.max_instructions,
                      memory_tracer=mem_tracer,
                      show_interrupts=not args.no_interrupts,
                      watch_ranges=watch)
        if mem_tracer:
            mem_tracer.report(fmt=args.memory_report_format)
        return

    if not args.port:
        parser.error("Serial port required (or use --replay)")

    try:
        import serial
    except ImportError:
        print("pyserial is required: pip install pyserial", file=sys.stderr)
        sys.exit(1)

    from . import capture

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    ser.reset_input_buffer()

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
                    print(f"{offset+i:06X}: {hex_part}")
                offset += len(chunk)
        except KeyboardInterrupt:
            pass
        finally:
            ser.close()
        return

    # -- Live capture via PSRAM command protocol --

    status = capture.get_status(ser)
    print(f"Connected to {args.port} "
          f"(CPU: {status['cpu_clock_khz']/1000:.0f} MHz, "
          f"state: {status['capture_state_name']})")

    # Configure triggers from CLI args
    capture.clear_triggers(ser)
    has_trigger = False

    for spec in args.trigger_mem_read:
        addr_val = _parse_trigger_spec(spec)
        capture.set_trigger(ser, capture.TRIG_MEM_READ, **addr_val)
        print(f"Trigger: memory read at 0x{addr_val['addr_lo']:04X}")
        has_trigger = True
    for spec in args.trigger_mem_write:
        addr_val = _parse_trigger_spec(spec)
        capture.set_trigger(ser, capture.TRIG_MEM_WRITE, **addr_val)
        print(f"Trigger: memory write at 0x{addr_val['addr_lo']:04X}")
        has_trigger = True
    for spec in args.trigger_io_read:
        addr_val = _parse_trigger_spec(spec)
        capture.set_trigger(ser, capture.TRIG_IO_READ, **addr_val)
        print(f"Trigger: I/O read at 0x{addr_val['addr_lo']:04X}")
        has_trigger = True
    for spec in args.trigger_io_write:
        addr_val = _parse_trigger_spec(spec)
        capture.set_trigger(ser, capture.TRIG_IO_WRITE, **addr_val)
        print(f"Trigger: I/O write at 0x{addr_val['addr_lo']:04X}")
        has_trigger = True
    if args.trigger_int:
        capture.set_trigger(ser, capture.TRIG_INT_ACK, 0x0000, 0xFFFF)
        print("Trigger: interrupt acknowledge")
        has_trigger = True

    # Start-at addresses become PC triggers
    for start_spec in fcfg.start_at:
        if isinstance(start_spec, tuple):
            lo, hi = start_spec
        else:
            lo = hi = start_spec
        capture.set_trigger(ser, capture.TRIG_PC_MATCH, lo, hi)
        print(f"Trigger: PC at 0x{lo:04X}" + (f"-0x{hi:04X}" if hi != lo else ""))
        has_trigger = True

    # Determine instruction counts and record estimates.
    # Records-per-instruction varies (1 for NOP, up to ~8 for complex IX ops).
    # Use 10x overestimate to ensure we fetch enough records from PSRAM.
    PSRAM_MAX = 2 * 1024 * 1024  # PSRAM_RING_RECORDS
    RECORDS_PER_INSTR = 10       # conservative overestimate

    # Resolve --window / --pre / --post / --*-all into instruction counts.
    # Default (nothing specified): 100 pre + 100 post.
    # If only one side is given, the other defaults to 0.
    if args.window is not None:
        pre_instrs = args.window
        post_instrs = args.window
    elif args.pre_all or args.post_all:
        pre_instrs = None if args.pre_all else 0
        post_instrs = None if args.post_all else 0
    elif args.pre is not None or args.post is not None:
        pre_instrs = args.pre if args.pre is not None else 0
        post_instrs = args.post if args.post is not None else 0
    else:
        pre_instrs = 100
        post_instrs = 100

    # Convert instruction counts to record counts for firmware.
    # --pre-all / --post-all: use full PSRAM if alone, half if both set.
    if pre_instrs is None and post_instrs is None:
        # Both "all" — split evenly
        pre_records = PSRAM_MAX // 2
        post_records = PSRAM_MAX // 2
    elif pre_instrs is None:
        # --pre-all only — post gets what it needs, pre gets the rest
        post_records = min(post_instrs * RECORDS_PER_INSTR, PSRAM_MAX)
        pre_records = PSRAM_MAX
    elif post_instrs is None:
        # --post-all only — pre gets what it needs, post gets the rest
        pre_records = min(pre_instrs * RECORDS_PER_INSTR, PSRAM_MAX)
        post_records = PSRAM_MAX
    else:
        pre_records = min(pre_instrs * RECORDS_PER_INSTR, PSRAM_MAX)
        post_records = min(post_instrs * RECORDS_PER_INSTR, PSRAM_MAX)

    if has_trigger:
        pre_label = 'all' if pre_instrs is None else str(pre_instrs)
        post_label = 'all' if post_instrs is None else str(post_instrs)
        print(f"Arming trigger (pre={pre_label}, post={post_label} instructions)...")
        capture.arm_trigger(ser, post_records)

        print("Waiting for trigger...", end='', flush=True)
        try:
            while True:
                time.sleep(0.1)
                st = capture.get_status(ser)
                if st['capture_state'] == capture.CAP_DONE:
                    print(f" triggered! (write_idx={st['write_idx']})")
                    break
                if st['capture_state'] == capture.CAP_TRIGGERED:
                    print(".", end='', flush=True)
                    continue
        except KeyboardInterrupt:
            print(" interrupted")
            capture.stop_capture(ser)
    else:
        # No triggers — just capture until Ctrl+C, then download
        print("Capturing (Ctrl+C to stop)...")
        capture.start_capture(ser)
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        print()

    # Download records from PSRAM
    records, trig_off = capture.read_buffer(
        ser, pre_count=pre_records, post_count=post_records)
    capture.stop_capture(ser)

    # Print diagnostics
    st = capture.get_status(ser)
    if st['dma_overflows']:
        print(f"WARNING: {st['dma_overflows']} DMA overflows (data lost)",
              file=sys.stderr)
    if st['wait_asserts']:
        print(f"Note: {st['wait_asserts']} /WAIT assertions (Z80 was paused)",
              file=sys.stderr)

    # Build a live-mode filter config: start_at and client-side triggers
    # are handled by firmware triggers, so exclude them to avoid
    # double-filtering.  Keep --mask, --stop-at, --stop-on, --limit.
    live_fcfg = FilterConfig(
        address_masks=[parse_address_range(m) for m in args.mask],
        start_at=[],
        stop_at=args.stop_at,
        start_on=None,
        stop_on=args.stop_on,
        limit=args.limit,
        mem_read_triggers=[],
        mem_write_triggers=[],
        io_read_triggers=[],
        io_write_triggers=[],
        trigger_int=False,
        window_size=0,
    )

    # Decode all records through the analysis pipeline, then trim by
    # instruction count around the trigger point.
    raw_bytes = b''.join(struct.pack('<HBB', *rec) for rec in records)
    all_output = _decode_to_lines(
        raw_bytes, raw_output=args.raw,
        detect_loops=args.loops, filter_config=live_fcfg,
        memory_tracer=mem_tracer,
        show_interrupts=not args.no_interrupts,
        watch_ranges=watch)

    # Find the trigger position in the output list.  trig_off is the
    # record index within the downloaded buffer; find the output entry
    # whose records span that index.
    if trig_off is not None and all_output:
        trig_pos = len(all_output) - 1  # default: last entry
        for i, entry in enumerate(all_output):
            if entry['record_end'] > trig_off:
                trig_pos = i
                break
        # Trim around trigger by output position (not instruction number)
        if pre_instrs is not None:
            start = max(0, trig_pos - pre_instrs)
        else:
            start = 0
        if post_instrs is not None:
            end = min(len(all_output), trig_pos + post_instrs + 1)
        else:
            end = len(all_output)
        trimmed = all_output[start:end]
    else:
        # No trigger or no output — show everything (trimmed by pre/post)
        if pre_instrs is not None and not has_trigger:
            trimmed = all_output[-pre_instrs:] if all_output else []
        else:
            trimmed = all_output

    for entry in trimmed:
        for line in entry['lines']:
            print(line)

    if mem_tracer:
        mem_tracer.report(fmt=args.memory_report_format)
    ser.close()


def _parse_trigger_spec(spec):
    """Parse 'ADDR' or 'ADDR=VAL' trigger spec into capture.set_trigger kwargs."""
    if '=' in spec:
        addr_s, val_s = spec.split('=', 1)
        addr = int(addr_s, 0)
        val = int(val_s, 0)
        return {'addr_lo': addr, 'data_val': val, 'data_mask': 0xFF}
    else:
        addr = int(spec, 0)
        return {'addr_lo': addr}


if __name__ == "__main__":
    main()
