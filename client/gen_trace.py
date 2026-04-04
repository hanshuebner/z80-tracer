#!/usr/bin/env python3
"""Generate a synthetic binary trace file for testing the Z80 tracer client.

Emulates the bus cycles a real Z80 would produce for a non-trivial program
that exercises: register loads, ALU, stack operations, subroutine calls,
IO, block instructions, IX-indexed addressing, CB bit operations,
conditional jumps, and a tight loop.

Usage:
    python -m client.gen_trace > test.bin
    python -m client --replay test.bin
"""

import struct
import sys

FETCH = 0
MREAD = 1
MWRITE = 2
IOREAD = 3
IOWRITE = 4

# Matches trace_record_t: 4 bytes per record
# Byte 0-1: address (u16 LE)
# Byte 2:   data (u8)
# Byte 3:   cycle_type:3 | wait_count:3 | halt:1 | int:1
RECORD_FMT = '<HBB'


def pack_type_wait_flags(cycle_type, wait_count=0, halt=False, intr=False):
    return ((cycle_type & 0x7) << 5 |
            (wait_count & 0x7) << 2 |
            (0x2 if halt else 0) |
            (0x1 if intr else 0))


class Z80BusGen:
    """Generates 4-byte trace records matching trace_record_t format."""

    def __init__(self):
        self.packets = bytearray()
        self.pc = 0
        self.sp = 0  # track SP for push/pop/call/ret generation

    def _pkt(self, cycle_type, addr, data, wait_count=0, halt=False):
        packed = pack_type_wait_flags(cycle_type, wait_count, halt)
        self.packets.extend(struct.pack(RECORD_FMT, addr & 0xFFFF, data & 0xFF, packed))

    def fetch(self, opcode, addr=None):
        """Opcode fetch (M1 cycle)."""
        if addr is not None:
            self.pc = addr
        self._pkt(FETCH, self.pc, opcode)
        self.pc += 1

    def mread(self, addr, data):
        """Memory read (non-M1)."""
        self._pkt(MREAD, addr, data)

    def mwrite(self, addr, data):
        """Memory write."""
        self._pkt(MWRITE, addr, data)

    def ioread(self, port, data):
        self._pkt(IOREAD, port, data)

    def iowrite(self, port, data):
        self._pkt(IOWRITE, port, data)

    # -- High-level instruction generators --
    # These emit the exact bus cycles a real Z80 would produce.

    def nop(self):
        self.fetch(0x00)

    def halt(self):
        self.fetch(0x76)

    def ld_r_n(self, r, n, addr=None):
        """LD r, n  (e.g. LD A, $42)"""
        opcode = 0x06 | (r << 3)
        self.fetch(opcode, addr)
        self.mread(self.pc, n)
        self.pc += 1

    def ld_r_r(self, dst, src, addr=None):
        """LD r, r'"""
        opcode = 0x40 | (dst << 3) | src
        self.fetch(opcode, addr)

    def ld_rr_nn(self, rr, nn, addr=None):
        """LD rr, nn  (rr: 0=BC,1=DE,2=HL,3=SP)"""
        opcode = 0x01 | (rr << 4)
        self.fetch(opcode, addr)
        self.mread(self.pc, nn & 0xFF)
        self.pc += 1
        self.mread(self.pc, (nn >> 8) & 0xFF)
        self.pc += 1

    def ld_hl_ind(self, hl_addr, data, addr=None):
        """LD (HL), n  -- HL already set, this is LD (HL), imm8"""
        self.fetch(0x36, addr)
        self.mread(self.pc, data)
        self.pc += 1
        self.mwrite(hl_addr, data)

    def ld_a_mem(self, mem_addr, data, addr=None):
        """LD A, (nn)"""
        self.fetch(0x3A, addr)
        self.mread(self.pc, mem_addr & 0xFF)
        self.pc += 1
        self.mread(self.pc, (mem_addr >> 8) & 0xFF)
        self.pc += 1
        self.mread(mem_addr, data)

    def ld_mem_a(self, mem_addr, a_val, addr=None):
        """LD (nn), A"""
        self.fetch(0x32, addr)
        self.mread(self.pc, mem_addr & 0xFF)
        self.pc += 1
        self.mread(self.pc, (mem_addr >> 8) & 0xFF)
        self.pc += 1
        self.mwrite(mem_addr, a_val)

    def ld_hl_from_r(self, hl_addr, r, r_val, addr=None):
        """LD (HL), r"""
        opcode = 0x70 | r
        self.fetch(opcode, addr)
        self.mwrite(hl_addr, r_val)

    def ld_r_from_hl(self, r, hl_addr, data, addr=None):
        """LD r, (HL)"""
        opcode = 0x46 | (r << 3)
        self.fetch(opcode, addr)
        self.mread(hl_addr, data)

    def xor_a(self, addr=None):
        """XOR A  (A=0, sets Z)"""
        self.fetch(0xAF, addr)

    def alu_r(self, op, r, addr=None):
        """ALU A, r  (op: 0=ADD..7=CP)"""
        opcode = 0x80 | (op << 3) | r
        self.fetch(opcode, addr)

    def alu_n(self, op, n, addr=None):
        """ALU A, n"""
        opcode = 0xC6 | (op << 3)
        self.fetch(opcode, addr)
        self.mread(self.pc, n)
        self.pc += 1

    def inc_r(self, r, addr=None):
        """INC r"""
        self.fetch(0x04 | (r << 3), addr)

    def dec_r(self, r, addr=None):
        """DEC r"""
        self.fetch(0x05 | (r << 3), addr)

    def inc_rr(self, rr, addr=None):
        """INC rr"""
        self.fetch(0x03 | (rr << 4), addr)

    def dec_rr(self, rr, addr=None):
        """DEC rr"""
        self.fetch(0x0B | (rr << 4), addr)

    def push(self, rr, hi, lo, addr=None):
        """PUSH rr  (rr: 0=BC,1=DE,2=HL,3=AF in R16AF encoding)"""
        opcode = 0xC5 | (rr << 4)
        self.fetch(opcode, addr)
        self.sp -= 1
        self.mwrite(self.sp, hi)
        self.sp -= 1
        self.mwrite(self.sp, lo)

    def pop(self, rr, lo, hi, addr=None):
        """POP rr -- lo and hi are the values read from the stack"""
        opcode = 0xC1 | (rr << 4)
        self.fetch(opcode, addr)
        self.mread(self.sp, lo)
        self.sp += 1
        self.mread(self.sp, hi)
        self.sp += 1

    def call(self, target, ret_addr=None, addr=None):
        """CALL nn"""
        self.fetch(0xCD, addr)
        self.mread(self.pc, target & 0xFF)
        self.pc += 1
        self.mread(self.pc, (target >> 8) & 0xFF)
        self.pc += 1
        if ret_addr is None:
            ret_addr = self.pc
        # Push return address
        self.sp -= 1
        self.mwrite(self.sp, (ret_addr >> 8) & 0xFF)
        self.sp -= 1
        self.mwrite(self.sp, ret_addr & 0xFF)
        self.pc = target

    def ret(self, ret_lo, ret_hi, addr=None):
        """RET -- provide the return address bytes from stack"""
        self.fetch(0xC9, addr)
        self.mread(self.sp, ret_lo)
        self.sp += 1
        self.mread(self.sp, ret_hi)
        self.sp += 1
        self.pc = (ret_hi << 8) | ret_lo

    def jp(self, target, addr=None):
        """JP nn"""
        self.fetch(0xC3, addr)
        self.mread(self.pc, target & 0xFF)
        self.pc += 1
        self.mread(self.pc, (target >> 8) & 0xFF)
        self.pc += 1
        self.pc = target

    def jr(self, offset, addr=None):
        """JR e  (offset is signed, relative to instruction after JR)"""
        self.fetch(0x18, addr)
        self.mread(self.pc, offset & 0xFF)
        self.pc += 1
        self.pc = (self.pc + _signed(offset & 0xFF)) & 0xFFFF

    def jr_nz(self, offset, addr=None):
        """JR NZ, e"""
        self.fetch(0x20, addr)
        self.mread(self.pc, offset & 0xFF)
        self.pc += 1

    def jr_z(self, offset, addr=None):
        """JR Z, e"""
        self.fetch(0x28, addr)
        self.mread(self.pc, offset & 0xFF)
        self.pc += 1

    def djnz(self, offset, addr=None):
        """DJNZ e"""
        self.fetch(0x10, addr)
        self.mread(self.pc, offset & 0xFF)
        self.pc += 1

    def cp_n(self, n, addr=None):
        """CP n"""
        self.alu_n(7, n, addr)

    def out_n_a(self, port, a_val, addr=None):
        """OUT (n), A"""
        self.fetch(0xD3, addr)
        self.mread(self.pc, port)
        self.pc += 1
        self.iowrite((a_val << 8) | port, a_val)

    def in_a_n(self, port, data, a_hi, addr=None):
        """IN A, (n)"""
        self.fetch(0xDB, addr)
        self.mread(self.pc, port)
        self.pc += 1
        self.ioread((a_hi << 8) | port, data)

    def ex_de_hl(self, addr=None):
        """EX DE, HL"""
        self.fetch(0xEB, addr)

    def di(self, addr=None):
        self.fetch(0xF3, addr)

    def ei(self, addr=None):
        self.fetch(0xFB, addr)

    # -- CB prefix --

    def cb_op(self, opcode, addr=None):
        """CB-prefixed instruction (no (HL) memory access)."""
        self.fetch(0xCB, addr)
        self.fetch(opcode)

    def cb_op_hl(self, opcode, hl_addr, old_val, new_val, addr=None):
        """CB-prefixed instruction on (HL) -- read-modify-write."""
        self.fetch(0xCB, addr)
        self.fetch(opcode)
        self.mread(hl_addr, old_val)
        if new_val is not None:  # BIT doesn't write back
            self.mwrite(hl_addr, new_val)

    # -- ED prefix --

    def ed_op(self, opcode, addr=None):
        """ED-prefixed instruction (no extra memory/IO)."""
        self.fetch(0xED, addr)
        self.fetch(opcode)

    def ldi(self, hl_addr, de_addr, data, addr=None):
        """LDI: (DE)<-(HL), HL++, DE++, BC--"""
        self.fetch(0xED, addr)
        self.fetch(0xA0)
        self.mread(hl_addr, data)
        self.mwrite(de_addr, data)

    def in_r_c(self, r, port, data, addr=None):
        """IN r, (C)"""
        self.fetch(0xED, addr)
        self.fetch(0x40 | (r << 3))
        self.ioread(port, data)

    def out_c_r(self, r, port, data, addr=None):
        """OUT (C), r"""
        self.fetch(0xED, addr)
        self.fetch(0x41 | (r << 3))
        self.iowrite(port, data)

    def ld_rr_mem_ed(self, rr, mem_addr, lo, hi, addr=None):
        """LD rr, (nn)  (ED encoding)"""
        self.fetch(0xED, addr)
        self.fetch(0x4B | (rr << 4))
        self.mread(self.pc, mem_addr & 0xFF)
        self.pc += 1
        self.mread(self.pc, (mem_addr >> 8) & 0xFF)
        self.pc += 1
        self.mread(mem_addr, lo)
        self.mread(mem_addr + 1, hi)

    # -- DD/FD prefix (IX/IY) --

    def ld_ix_nn(self, nn, addr=None):
        """LD IX, nn"""
        self.fetch(0xDD, addr)
        self.fetch(0x21)
        self.mread(self.pc, nn & 0xFF)
        self.pc += 1
        self.mread(self.pc, (nn >> 8) & 0xFF)
        self.pc += 1

    def ld_ix_d_n(self, ix_val, disp, n, addr=None):
        """LD (IX+d), n"""
        ea = (ix_val + _signed(disp & 0xFF)) & 0xFFFF
        self.fetch(0xDD, addr)
        self.fetch(0x36)
        self.mread(self.pc, disp & 0xFF)
        self.pc += 1
        self.mread(self.pc, n)
        self.pc += 1
        self.mwrite(ea, n)

    def ld_r_ix_d(self, r, ix_val, disp, data, addr=None):
        """LD r, (IX+d)"""
        ea = (ix_val + _signed(disp & 0xFF)) & 0xFFFF
        self.fetch(0xDD, addr)
        self.fetch(0x46 | (r << 3))
        self.mread(self.pc, disp & 0xFF)
        self.pc += 1
        self.mread(ea, data)


def _signed(b):
    return b - 256 if b >= 128 else b


# ============================================================
# Build the test program
# ============================================================

def generate():
    B, C, D, E, H, L, HLi, A = 0, 1, 2, 3, 4, 5, 6, 7

    g = Z80BusGen()

    # ---- Initialisation (0x0000) ----
    g.di(addr=0x0000)                          # DI
    g.ld_rr_nn(3, 0xFFFF)                      # LD SP, $FFFF
    g.sp = 0xFFFF
    g.xor_a()                                  # XOR A          -> A=0
    g.ld_rr_nn(2, 0x8000)                      # LD HL, $8000
    g.ld_rr_nn(1, 0x9000)                      # LD DE, $9000
    g.ld_rr_nn(0, 0x0010)                      # LD BC, $0010

    # ---- Block copy: 16 bytes from $8000 to $9000 using LDI ----
    ldi_addr = g.pc
    for i in range(16):
        g.ldi(0x8000 + i, 0x9000 + i, 0xA0 + i)

    # ---- Stack operations ----
    g.ld_rr_nn(2, 0x1234)                      # LD HL, $1234
    g.push(2, 0x12, 0x34)                      # PUSH HL
    g.ld_rr_nn(2, 0x0000)                      # LD HL, $0000 (clobber)
    g.pop(2, 0x34, 0x12)                       # POP HL  -> HL=$1234 again

    # ---- Subroutine call ----
    # CALL $0200, subroutine fills A with $42
    call_ret = g.pc + 3  # return address = after the CALL instruction
    g.call(0x0200)

    # ...return lands here
    after_call_addr = call_ret

    # ---- IO operations ----
    g.out_n_a(0x01, 0x42, addr=after_call_addr)   # OUT ($01), A  (A=$42)
    g.in_a_n(0x02, 0x55, 0x42)                    # IN A, ($02)   -> A=$55

    # ---- IX-indexed operations ----
    g.ld_ix_nn(0xC000)                             # LD IX, $C000
    g.ld_ix_d_n(0xC000, 5, 0xFF)                   # LD (IX+5), $FF
    g.ld_r_ix_d(A, 0xC000, 5, 0xFF)               # LD A, (IX+5) -> A=$FF

    # ---- CB bit operations ----
    g.cb_op(0x47)                                  # BIT 0, A  (A=$FF, bit 0 set)
    g.cb_op(0x87)                                  # RES 0, A  -> A=$FE
    g.cb_op(0xC7)                                  # SET 0, A  -> A=$FF
    g.ld_rr_nn(2, 0xD000)                          # LD HL, $D000
    g.cb_op_hl(0x46, 0xD000, 0xAA, None)           # BIT 0, (HL) read $AA

    # ---- ALU operations ----
    g.ld_r_n(A, 0x80)                              # LD A, $80
    g.alu_n(0, 0x40)                               # ADD A, $40  -> A=$C0
    g.alu_n(2, 0x20)                               # SUB $20     -> A=$A0
    g.alu_n(4, 0x0F)                               # AND $0F     -> A=$00
    g.alu_n(6, 0x33)                               # OR $33      -> A=$33
    g.alu_n(5, 0x33)                               # XOR $33     -> A=$00

    # ---- Counted loop: DJNZ ----
    g.ld_r_n(B, 0x05)                              # LD B, 5
    loop_top = g.pc
    # INC A / DJNZ loop_top -- runs 5 times, A goes 0->5, B goes 5->0
    djnz_addr = loop_top + 1  # DJNZ is 1 byte after INC A
    offset = (loop_top - (djnz_addr + 2)) & 0xFF
    for iteration in range(5):
        g.inc_r(A, addr=loop_top)
        g.djnz(offset)
        if iteration < 4:
            g.pc = loop_top  # branch taken: next fetch at loop_top

    # After loop: A=5, B=0
    fall_through_addr = g.pc

    # ---- Conditional jump ----
    g.cp_n(0x05, addr=fall_through_addr)           # CP 5  (A=5, so Z set)
    g.jr_z(0x02)                                   # JR Z, +2  (skip next)
    skip_target = g.pc + 2
    g.nop()                                        # (skipped by JR Z)
    g.nop()

    # ---- EX DE, HL ----
    g.ld_rr_nn(1, 0xAAAA, addr=skip_target)        # LD DE, $AAAA
    g.ld_rr_nn(2, 0x5555)                          # LD HL, $5555
    g.ex_de_hl()                                   # EX DE, HL

    # ---- Final: write signature to IO then halt ----
    g.ld_r_n(A, 0x5A)                              # LD A, $5A
    g.out_n_a(0xFF, 0x5A)                          # OUT ($FF), A
    g.halt()

    # ============================================================
    # Subroutine at $0200: loads A with $42 and returns
    # ============================================================
    g.ld_r_n(A, 0x42, addr=0x0200)                 # LD A, $42
    g.ret(call_ret & 0xFF, (call_ret >> 8) & 0xFF)  # RET

    return g.packets


def main():
    data = generate()
    sys.stdout.buffer.write(data)
    sys.stderr.write(f"Generated {len(data)} bytes "
                     f"({len(data) // 4} bus cycles)\n")


if __name__ == "__main__":
    main()
