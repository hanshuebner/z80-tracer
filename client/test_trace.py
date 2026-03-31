#!/usr/bin/env python3
"""Smoke test: feed a known Z80 program through the assembler/decoder/state."""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from client.main import (InstructionAssembler, format_instruction,
                         CYCLE_OPCODE_FETCH, CYCLE_MEM_READ, CYCLE_MEM_WRITE)
from client.z80_state import Z80State
from client.z80_loop import LoopDetector


class TraceHarness:
    """Test harness that mirrors the main trace loop logic."""

    def __init__(self):
        self.asm = InstructionAssembler()
        self.state = Z80State()
        self.loop_det = LoopDetector()
        self.output_lines = []
        self.instr_count = 0

    def feed(self, cycle_type, addr, value):
        instr = self.asm.feed(cycle_type, addr, value)
        if instr:
            self._process(instr)

    def flush(self):
        instr = self.asm.flush()
        if instr:
            self._process(instr)

    def _process(self, instr):
        prev = self.state.snapshot_key()
        self.state.execute(instr)
        self.instr_count += 1
        state_key = self.state.snapshot_key()
        action = self.loop_det.check(instr.pc, state_key)
        reg_changes = self.state.format_changed(prev)

        if action == "normal":
            self.output_lines.append(format_instruction(instr, reg_changes))
        elif action == "loop_exit":
            summary = self.loop_det.loop_summary()
            if summary:
                self.output_lines.append(summary)
            self.output_lines.append(format_instruction(instr, reg_changes))
        # suppress: silent

    def print_output(self):
        for line in self.output_lines:
            print(line)


def test_basic_program():
    """Simulate:
        0000: LD SP, FFFF    ; 31 FF FF
        0003: XOR A          ; AF
        0004: LD HL, 4000    ; 21 00 40
        0007: LD (HL), A     ; 77        -- loop body start
        0008: INC A          ; 3C
        0009: JR NZ, $0007   ; 20 FC     -- loop body end
        (loops with A=0,1,2,3... until A wraps to 0 -- 256 iterations)
        000B: HALT           ; 76
    """
    h = TraceHarness()

    # LD SP, FFFF
    h.feed(CYCLE_OPCODE_FETCH, 0x0000, 0x31)
    h.feed(CYCLE_MEM_READ, 0x0001, 0xFF)
    h.feed(CYCLE_MEM_READ, 0x0002, 0xFF)

    # XOR A
    h.feed(CYCLE_OPCODE_FETCH, 0x0003, 0xAF)

    # LD HL, 4000
    h.feed(CYCLE_OPCODE_FETCH, 0x0004, 0x21)
    h.feed(CYCLE_MEM_READ, 0x0005, 0x00)
    h.feed(CYCLE_MEM_READ, 0x0006, 0x40)

    # Run the loop 5 times (A goes 0..4)
    for i in range(5):
        a_val = i & 0xFF
        # LD (HL), A
        h.feed(CYCLE_OPCODE_FETCH, 0x0007, 0x77)
        h.feed(CYCLE_MEM_WRITE, 0x4000, a_val)
        # INC A
        h.feed(CYCLE_OPCODE_FETCH, 0x0008, 0x3C)
        # JR NZ, $0007
        h.feed(CYCLE_OPCODE_FETCH, 0x0009, 0x20)
        h.feed(CYCLE_MEM_READ, 0x000A, 0xFC)

    # HALT (exits the loop since PC 0x000B is not in the loop body)
    h.feed(CYCLE_OPCODE_FETCH, 0x000B, 0x76)
    h.flush()

    h.print_output()
    print(f"\nState: {h.state.format_registers()}")
    print(f"Instructions: {h.instr_count}")

    # Verify key state
    assert h.state.sp == 0xFFFF, f"SP={h.state.sp}"
    assert h.state.h == 0x40 and h.state.l == 0x00, "HL should be 4000"
    assert h.state.a == 5, f"A should be 5, got {h.state.a}"

    # Verify loop was detected and output was suppressed
    normal_lines = [l for l in h.output_lines if not l.startswith("  [")]
    loop_lines = [l for l in h.output_lines if l.startswith("  [")]
    # First 2 loop iterations shown (SUPPRESS_AFTER=2), then suppressed
    assert len(loop_lines) >= 1, "Should have loop summary"
    print(f"\nLoop summary: {loop_lines}")

    print("\nBasic test passed!")


def test_prefixed():
    """Test CB and ED prefixed instructions."""
    h = TraceHarness()

    # NOP (dummy to have something to finalize)
    h.feed(CYCLE_OPCODE_FETCH, 0x0000, 0x00)
    # SET 7, A (CB FF)
    h.feed(CYCLE_OPCODE_FETCH, 0x0001, 0xCB)
    h.feed(CYCLE_OPCODE_FETCH, 0x0002, 0xFF)
    # LDIR (ED B0)
    h.feed(CYCLE_OPCODE_FETCH, 0x0003, 0xED)
    h.feed(CYCLE_OPCODE_FETCH, 0x0004, 0xB0)
    # LD IX, 1234 (DD 21 34 12)
    h.feed(CYCLE_OPCODE_FETCH, 0x0005, 0xDD)
    h.feed(CYCLE_OPCODE_FETCH, 0x0006, 0x21)
    h.feed(CYCLE_MEM_READ, 0x0007, 0x34)
    h.feed(CYCLE_MEM_READ, 0x0008, 0x12)
    # Another NOP to finalize LD IX
    h.feed(CYCLE_OPCODE_FETCH, 0x0009, 0x00)
    h.flush()

    h.print_output()

    # Verify IX was set
    assert h.state.ix == 0x1234, f"IX should be 1234, got {h.state.ix}"

    # Check mnemonics in output
    output_text = "\n".join(h.output_lines)
    assert "SET 7, A" in output_text, "Should decode CB FF as SET 7, A"
    assert "LDIR" in output_text, "Should decode ED B0 as LDIR"
    assert "LD IX, $1234" in output_text, f"Should decode DD 21: {output_text}"

    print("\nPrefix test passed!")


def test_infinite_loop():
    """Test detection of a truly infinite loop (same state repeats)."""
    h = TraceHarness()

    # JR $0000 (18 FE) -- jumps to itself forever
    for _ in range(10):
        h.feed(CYCLE_OPCODE_FETCH, 0x0000, 0x18)
        h.feed(CYCLE_MEM_READ, 0x0001, 0xFE)

    h.flush()
    h.print_output()

    assert h.loop_det.in_loop, "Should detect infinite loop"
    print("\nInfinite loop test passed!")


if __name__ == "__main__":
    test_basic_program()
    print()
    test_prefixed()
    print()
    test_infinite_loop()
