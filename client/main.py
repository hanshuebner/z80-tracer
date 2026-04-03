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
from .trace_filter import TraceFilter, FilterConfig, parse_address_range, parse_address_or_range, parse_trigger_addr
from .memory_trace import MemoryTracer


# -- Wire protocol (matches firmware z80_trace.h) --

CYCLE_OPCODE_FETCH = 0  # M1 opcode fetch (CYCLE_M1_FETCH in firmware)
CYCLE_MEM_READ = 1
CYCLE_MEM_WRITE = 2
CYCLE_IO_READ = 3
CYCLE_IO_WRITE = 4
CYCLE_INT_ACK = 5
CYCLE_RESET = 6
CYCLE_STATUS = 7        # Diagnostic status packet

CYCLE_NAMES = ("FETCH", "MREAD", "MWRIT", "IOREAD", "IOWRIT", "INTACK", "RESET", "STATUS")
MIN_PACKET_SIZE = 5  # without refresh
MAX_PACKET_SIZE = 6  # with refresh

# Status subtypes (in byte 0 bits 2:0 for CYCLE_STATUS packets)
STATUS_WAIT_ASSERT = 1
STATUS_WAIT_RELEASE = 2
STATUS_DMA_OVERFLOW = 3
STATUS_TRACE_START = 4
STATUS_TRACE_STOP = 5
STATUS_FLOW_DISCARD = 6
STATUS_FRAMES_SENT = 7
STATUS_NAMES = {
    1: "WAIT_ASSERT", 2: "WAIT_RELEASE", 3: "DMA_OVERFLOW",
    4: "TRACE_START", 5: "TRACE_STOP", 6: "FLOW_DISCARD",
    7: "FRAMES_SENT",
}

# Binary command protocol (client → firmware)
CMD_SYNC = 0xFF
CMD_TRACE_START = 0x01
CMD_TRACE_STOP = 0x02


def parse_packet(data):
    """Parse a 5- or 6-byte trace packet with bit-7 sync framing.

    Normal packets (5 or 6 bytes, cycle_type 0-6):
      Byte 0: 1TTTT_AAA  sync=1, type[3:0], addr[15:13]
      Byte 1: 0AAAAAAA   addr[12:6]
      Byte 2: 0AAAAAAD   addr[5:0], data[7]
      Byte 3: 0DDDDDDD   data[6:0]
      Byte 4: 0HWWWWWW   halt, wait_count[5:0]
      Byte 5 (M1 only): 0RRRRRRR  refresh[6:0]

    Status packets (5 bytes, cycle_type 7):
      Byte 0: 1_0111_SSS  type=7, subtype[2:0]
      Byte 1-2: seq[13:0] (14-bit, 7 bits each)
      Byte 3-4: count[13:0] (14-bit, 7 bits each)

    Returns (cycle_type, addr, value, refresh, wait_count, halt).
    For status packets: addr=seq, value=subtype, refresh=0, wait_count=count, halt=False.
    """
    cycle_type = (data[0] >> 3) & 0x0F
    if cycle_type == CYCLE_STATUS:
        subtype = data[0] & 0x07
        seq = (data[1] << 7) | data[2]
        count = (data[3] << 7) | data[4]
        return cycle_type, seq, subtype, 0, count, False
    addr = ((data[0] & 0x07) << 13) | (data[1] << 6) | (data[2] >> 1)
    value = ((data[2] & 0x01) << 7) | data[3]
    halt = bool(data[4] & 0x40)
    wait_count = data[4] & 0x3F
    refresh = data[5] & 0x7F if len(data) > 5 else 0
    return cycle_type, addr, value, refresh, wait_count, halt


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


# -- Main trace loop --

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
    # Diagnostic counters
    diag_wait_asserts = 0
    diag_wait_releases = 0
    diag_dma_overflows = 0
    diag_flow_discards = 0
    diag_fw_frames_sent = 0   # total frames firmware reports sending
    diag_trace_stopped = False
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

            while len(buf) >= MIN_PACKET_SIZE:
                # Sync: skip bytes until we find one with bit 7 set (packet start)
                while buf and not (buf[0] & 0x80):
                    buf = buf[1:]
                if len(buf) < MIN_PACKET_SIZE:
                    break

                # Count continuation bytes (bit 7 clear) to determine packet length
                pkt_len = 1
                while pkt_len < len(buf) and pkt_len < MAX_PACKET_SIZE and not (buf[pkt_len] & 0x80):
                    pkt_len += 1
                if pkt_len < MIN_PACKET_SIZE:
                    break  # need more data

                pkt = buf[:pkt_len]
                buf = buf[pkt_len:]

                cycle_type, addr, value, refresh, wait_count, halt = parse_packet(pkt)

                frame_count += 1
                if max_frames is not None and frame_count > max_frames:
                    if stop_fn:
                        stop_fn()
                    raise StopIteration

                # Handle diagnostic status packets
                if cycle_type == CYCLE_STATUS:
                    subtype = value  # parse_packet puts subtype in value
                    seq = addr       # parse_packet puts seq in addr
                    count = wait_count  # parse_packet puts count in wait_count
                    if subtype == STATUS_TRACE_STOP:
                        diag_trace_stopped = True
                        raise StopIteration
                    if subtype == STATUS_TRACE_START:
                        continue  # already handled in handshake
                    name = STATUS_NAMES.get(subtype, f"UNKNOWN({subtype})")
                    if subtype == STATUS_WAIT_ASSERT:
                        diag_wait_asserts += count
                    elif subtype == STATUS_WAIT_RELEASE:
                        diag_wait_releases += 1
                    elif subtype == STATUS_DMA_OVERFLOW:
                        diag_dma_overflows += count
                    elif subtype == STATUS_FLOW_DISCARD:
                        # 28-bit value: seq=high14, count=low14
                        diag_flow_discards += (seq << 14) | count
                    elif subtype == STATUS_FRAMES_SENT:
                        # 28-bit value: seq=high14, count=low14
                        diag_fw_frames_sent = (seq << 14) | count
                    msg = f"; --- STATUS: {name} seq={seq} count={count} ---"
                    if raw_output:
                        raw_buf.append(msg)
                    else:
                        print(msg, file=sys.stderr)
                    continue

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
                    if cycle_type == CYCLE_OPCODE_FETCH:
                        extras = f" R={refresh:02X}"
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
            # Keep draining until TRACE_STOP or timeout.
            # Scan quickly — only care about STATUS packets.
            try:
                deadline = time.time() + 10.0
                while time.time() < deadline:
                    chunk = source.read(16384)
                    if not chunk:
                        continue
                    # Fast scan: look for status packet sync byte (0xBF = type 7)
                    # Status byte 0 = 0x80 | (7 << 3) | subtype = 0xB8..0xBF
                    for i in range(len(chunk)):
                        b = chunk[i]
                        if b & 0xF8 == 0xB8 and i + MIN_PACKET_SIZE <= len(chunk):
                            # Candidate status packet
                            pkt = chunk[i:i+MIN_PACKET_SIZE]
                            # Verify continuation bytes have bit 7 clear
                            if all(pkt[j] & 0x80 == 0 for j in range(1, MIN_PACKET_SIZE)):
                                ct, _, val, _, wc, _ = parse_packet(pkt)
                                if ct == CYCLE_STATUS:
                                    if val == STATUS_TRACE_STOP:
                                        diag_trace_stopped = True
                                        raise StopIteration
                                    elif val == STATUS_WAIT_ASSERT:
                                        diag_wait_asserts += wc
                                    elif val == STATUS_DMA_OVERFLOW:
                                        diag_dma_overflows += wc
                                    elif val == STATUS_FLOW_DISCARD:
                                        # 28-bit: seq=high14, count=low14
                                        _, sq, _, _, ct2, _ = parse_packet(pkt)
                                        diag_flow_discards += (sq << 14) | ct2
                                    elif val == STATUS_FRAMES_SENT:
                                        # 28-bit: seq=high14, count=low14
                                        _, sq, _, _, ct2, _ = parse_packet(pkt)
                                        diag_fw_frames_sent = (sq << 14) | ct2
            except (KeyboardInterrupt, StopIteration):
                pass
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
        "wait_asserts": diag_wait_asserts,
        "wait_releases": diag_wait_releases,
        "dma_overflows": diag_dma_overflows,
        "flow_discards": diag_flow_discards,
        "fw_frames_sent": diag_fw_frames_sent,
        "frames_received": frame_count,
        "trace_stopped": diag_trace_stopped,
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
    parser.add_argument(
        "--window", type=int, default=0, metavar="N",
        help="Show N instructions before trigger event")
    parser.add_argument(
        "--max-frames", type=int, default=None, metavar="N",
        help="Stop after processing N bus cycle frames (regardless of filtering)")
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
        window_size=args.window,
    )

    mem_tracer = MemoryTracer() if args.memory_report else None
    watch = [parse_address_or_range(w) for w in args.watch]

    if args.replay:
        with open(args.replay, "rb") as f:
            diag = run_trace(f, raw_output=args.raw, is_file=True,
                             detect_loops=args.loops, filter_config=fcfg,
                             max_frames=args.max_frames,
                             max_instructions=args.max_instructions,
                             memory_tracer=mem_tracer,
                             show_interrupts=not args.no_interrupts,
                             watch_ranges=watch)
        if mem_tracer:
            mem_tracer.report(fmt=args.memory_report_format)
        wp = diag.get("wait_asserts", 0)
        do = diag.get("dma_overflows", 0)
        fd = diag.get("flow_discards", 0)
        fw_sent = diag.get("fw_frames_sent", 0)
        rx_frames = diag.get("frames_received", 0)
        if wp or do or fd:
            print(f"\nDiagnostics: {wp} flow control pauses, "
                  f"{do} DMA overflows, {fd} flow control discards")
        if fw_sent > 0:
            usb_lost = fw_sent - rx_frames
            if usb_lost > 0:
                print(f"Firmware sent {fw_sent} frames, client received {rx_frames}"
                      f" — {usb_lost} lost in USB/serial ({usb_lost*100//fw_sent}%)")
        if fd:
            print("WARNING: samples discarded during flow control — "
                  "check /WAIT wiring", file=sys.stderr)
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

    print(f"Connected to {args.port}")

    # Drain any stale data
    ser.reset_input_buffer()

    def send_command(cmd):
        ser.write(bytes([CMD_SYNC, cmd]))
        ser.flush()

    # Send binary start command and wait for ack
    send_command(CMD_TRACE_START)
    _wait_for_status(ser, STATUS_TRACE_START)

    print("Tracing -- Ctrl+C to stop")

    def stop_fn():
        print("Stopping trace...", file=sys.stderr)
        send_command(CMD_TRACE_STOP)

    diag = run_trace(ser, raw_output=args.raw, detect_loops=args.loops,
                     filter_config=fcfg,
                     max_frames=args.max_frames,
                     max_instructions=args.max_instructions,
                     memory_tracer=mem_tracer,
                     show_interrupts=not args.no_interrupts,
                     stop_fn=stop_fn,
                     watch_ranges=watch)

    if mem_tracer:
        mem_tracer.report(fmt=args.memory_report_format)

    wait_pauses = diag.get("wait_asserts", 0)
    dma_overflows = diag.get("dma_overflows", 0)
    flow_discards = diag.get("flow_discards", 0)
    fw_sent = diag.get("fw_frames_sent", 0)
    rx_frames = diag.get("frames_received", 0)
    got_stop = diag.get("trace_stopped", False)
    if got_stop:
        print(f"Stopped. {wait_pauses} flow control pauses, "
              f"{dma_overflows} DMA overflows, "
              f"{flow_discards} flow control discards", file=sys.stderr)
        if fw_sent > 0:
            usb_lost = fw_sent - rx_frames
            print(f"Firmware sent {fw_sent} frames, client received {rx_frames}"
                  f" — {usb_lost} lost in USB/serial ({usb_lost*100//fw_sent}%)"
                  if usb_lost > 0 else
                  f"Firmware sent {fw_sent} frames, client received all",
                  file=sys.stderr)
        if flow_discards:
            print("WARNING: samples discarded during flow control — "
                  "check /WAIT wiring", file=sys.stderr)
    else:
        # Check if firmware is still sending data
        try:
            probe = ser.read(4096)
            if probe:
                print(f"Warning: did not receive stop confirmation "
                      f"(firmware still sending, {len(probe)} bytes pending)",
                      file=sys.stderr)
            else:
                print(f"Warning: did not receive stop confirmation "
                      f"(firmware silent — stop ack likely lost in USB)",
                      file=sys.stderr)
        except Exception:
            print("Warning: did not receive stop confirmation from firmware",
                  file=sys.stderr)
    ser.close()


def _wait_for_status(source, expected_subtype, timeout=5.0):
    """Read packets until a STATUS packet with the expected subtype arrives."""
    buf = bytearray()
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = source.read(4096)
        if not chunk:
            continue
        buf.extend(chunk)
        while len(buf) >= MIN_PACKET_SIZE:
            if not (buf[0] & 0x80):
                buf = buf[1:]
                continue
            pkt_len = 1
            while (pkt_len < len(buf)
                   and pkt_len < MAX_PACKET_SIZE
                   and not (buf[pkt_len] & 0x80)):
                pkt_len += 1
            if pkt_len < MIN_PACKET_SIZE:
                break
            pkt = buf[:pkt_len]
            buf = buf[pkt_len:]
            ct, _, val, _, _, _ = parse_packet(pkt)
            if ct == CYCLE_STATUS and val == expected_subtype:
                return True
    return False


if __name__ == "__main__":
    main()
