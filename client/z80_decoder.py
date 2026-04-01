"""Z80 instruction decoder.

Decodes Z80 opcodes into mnemonics and instruction metadata using the
standard bit-field extraction scheme (x, y, z, p, q fields).

Handles all prefix groups: unprefixed, CB, ED, DD/FD (IX/IY), DDCB/FDCB.
"""

from dataclasses import dataclass, field
from typing import Optional


# -- Register name tables --

R8 = ("B", "C", "D", "E", "H", "L", "(HL)", "A")
R16 = ("BC", "DE", "HL", "SP")
R16AF = ("BC", "DE", "HL", "AF")
CC = ("NZ", "Z", "NC", "C", "PO", "PE", "P", "M")
ALU_MNEM = ("ADD A,", "ADC A,", "SUB", "SBC A,", "AND", "XOR", "OR", "CP")
ROT_MNEM = ("RLC", "RRC", "RL", "RR", "SLA", "SRA", "SLL", "SRL")


@dataclass
class Instruction:
    """A fully decoded Z80 instruction."""
    pc: int = 0
    length: int = 1
    mnemonic: str = "???"
    raw_bytes: bytes = b""

    # Effect description for the emulator: (type_str, *args)
    #   type_str is one of: nop, halt, ld8, ld8_imm, ld8_mem, st8_mem,
    #   ld16_imm, ld16_mem, st16_mem, ld_sp_hl, push, pop, ex, exx, ex_af,
    #   alu8, alu8_imm, alu8_mem, inc8, dec8, inc16, dec16,
    #   add16, adc16, sbc16,
    #   rot_a, rot, rot_mem, bit, set_, res, set_mem, res_mem,
    #   jp, jp_hl, jr, djnz, call, ret, reti, retn, rst,
    #   in_, out_, in_c, out_c, in_block, out_block,
    #   block_ld, block_cp,
    #   daa, cpl, scf, ccf, neg, im, ld_i_a, ld_a_i, ld_r_a, ld_a_r,
    #   di, ei, unknown
    effect: tuple = ("nop",)

    # Bus observations filled in by the assembler:
    data_reads: list = field(default_factory=list)
    data_writes: list = field(default_factory=list)
    io_reads: list = field(default_factory=list)
    io_writes: list = field(default_factory=list)


# -- Helpers --

def _signed8(b):
    return b - 256 if b >= 128 else b


def _imm16(lo, hi):
    return (hi << 8) | lo


def _rel_target(pc, instr_len, offset_byte):
    return (pc + instr_len + _signed8(offset_byte)) & 0xFFFF


def _rel_str(pc, instr_len, offset_byte):
    target = _rel_target(pc, instr_len, offset_byte)
    return f"${target:04X}"


# -- Main (unprefixed) opcode decoding --

def _decode_main(opcode, ops, pc):
    """Decode an unprefixed opcode.  ops = operand bytes list."""
    x = (opcode >> 6) & 3
    y = (opcode >> 3) & 7
    z = opcode & 7
    p = y >> 1
    q = y & 1
    op1 = ops[0] if len(ops) > 0 else 0
    op2 = ops[1] if len(ops) > 1 else 0

    if x == 0:
        if z == 0:
            if y == 0:
                return "NOP", 1, ("nop",)
            if y == 1:
                return "EX AF, AF'", 1, ("ex_af",)
            if y == 2:
                return f"DJNZ {_rel_str(pc, 2, op1)}", 2, ("djnz", op1)
            if y == 3:
                return f"JR {_rel_str(pc, 2, op1)}", 2, ("jr", None, op1)
            return f"JR {CC[y-4]}, {_rel_str(pc, 2, op1)}", 2, ("jr", y - 4, op1)

        if z == 1:
            if q == 0:
                nn = _imm16(op1, op2)
                return f"LD {R16[p]}, ${nn:04X}", 3, ("ld16_imm", p, nn)
            return f"ADD HL, {R16[p]}", 1, ("add16", 2, p)  # HL=2

        if z == 2:
            if q == 0:
                if p == 0:
                    return "LD (BC), A", 1, ("st8_mem", "A", "BC")
                if p == 1:
                    return "LD (DE), A", 1, ("st8_mem", "A", "DE")
                if p == 2:
                    nn = _imm16(op1, op2)
                    return f"LD (${nn:04X}), HL", 3, ("st16_mem", "HL", nn)
                nn = _imm16(op1, op2)
                return f"LD (${nn:04X}), A", 3, ("st8_mem", "A", nn)
            else:
                if p == 0:
                    return "LD A, (BC)", 1, ("ld8_mem", "A", "BC")
                if p == 1:
                    return "LD A, (DE)", 1, ("ld8_mem", "A", "DE")
                if p == 2:
                    nn = _imm16(op1, op2)
                    return f"LD HL, (${nn:04X})", 3, ("ld16_mem", "HL", nn)
                nn = _imm16(op1, op2)
                return f"LD A, (${nn:04X})", 3, ("ld8_mem", "A", nn)

        if z == 3:
            if q == 0:
                return f"INC {R16[p]}", 1, ("inc16", p)
            return f"DEC {R16[p]}", 1, ("dec16", p)

        if z == 4:
            return f"INC {R8[y]}", 1, ("inc8", y)

        if z == 5:
            return f"DEC {R8[y]}", 1, ("dec8", y)

        if z == 6:
            return f"LD {R8[y]}, ${op1:02X}", 2, ("ld8_imm", y, op1)

        if z == 7:
            mnems = ("RLCA", "RRCA", "RLA", "RRA", "DAA", "CPL", "SCF", "CCF")
            effects = ("rot_a", "rot_a", "rot_a", "rot_a",
                       "daa", "cpl", "scf", "ccf")
            return mnems[y], 1, (effects[y], y)

    if x == 1:
        if y == 6 and z == 6:
            return "HALT", 1, ("halt",)
        return f"LD {R8[y]}, {R8[z]}", 1, ("ld8", y, z)

    if x == 2:
        return f"{ALU_MNEM[y]} {R8[z]}", 1, ("alu8", y, z)

    # x == 3
    if z == 0:
        return f"RET {CC[y]}", 1, ("ret", y)

    if z == 1:
        if q == 0:
            return f"POP {R16AF[p]}", 1, ("pop", p)
        if p == 0:
            return "RET", 1, ("ret", None)
        if p == 1:
            return "EXX", 1, ("exx",)
        if p == 2:
            return "JP (HL)", 1, ("jp_hl",)
        return "LD SP, HL", 1, ("ld_sp_hl",)

    if z == 2:
        nn = _imm16(op1, op2)
        return f"JP {CC[y]}, ${nn:04X}", 3, ("jp", y, nn)

    if z == 3:
        if y == 0:
            nn = _imm16(op1, op2)
            return f"JP ${nn:04X}", 3, ("jp", None, nn)
        if y == 1:
            return None  # CB prefix - handled externally
        if y == 2:
            return f"OUT (${op1:02X}), A", 2, ("out_", op1)
        if y == 3:
            return f"IN A, (${op1:02X})", 2, ("in_", op1)
        if y == 4:
            return "EX (SP), HL", 1, ("ex", "(SP)", "HL")
        if y == 5:
            return "EX DE, HL", 1, ("ex", "DE", "HL")
        if y == 6:
            return "DI", 1, ("di",)
        return "EI", 1, ("ei",)

    if z == 4:
        nn = _imm16(op1, op2)
        return f"CALL {CC[y]}, ${nn:04X}", 3, ("call", y, nn)

    if z == 5:
        if q == 0:
            return f"PUSH {R16AF[p]}", 1, ("push", p)
        if p == 0:
            nn = _imm16(op1, op2)
            return f"CALL ${nn:04X}", 3, ("call", None, nn)
        return None  # DD/ED/FD prefix - handled externally

    if z == 6:
        return f"{ALU_MNEM[y]} ${op1:02X}", 2, ("alu8_imm", y, op1)

    if z == 7:
        vec = y * 8
        return f"RST ${vec:02X}", 1, ("rst", vec)

    return "???", 1, ("unknown",)


# -- CB prefix decoding --

def _decode_cb(opcode):
    x = (opcode >> 6) & 3
    y = (opcode >> 3) & 7
    z = opcode & 7

    if x == 0:
        return f"{ROT_MNEM[y]} {R8[z]}", 2, ("rot", y, z)
    if x == 1:
        return f"BIT {y}, {R8[z]}", 2, ("bit", y, z)
    if x == 2:
        return f"RES {y}, {R8[z]}", 2, ("res", y, z)
    return f"SET {y}, {R8[z]}", 2, ("set_", y, z)


# -- ED prefix decoding --

def _decode_ed(opcode, ops, pc):
    x = (opcode >> 6) & 3
    y = (opcode >> 3) & 7
    z = opcode & 7
    p = y >> 1
    q = y & 1
    op1 = ops[0] if len(ops) > 0 else 0
    op2 = ops[1] if len(ops) > 1 else 0

    if x == 1:
        if z == 0:
            if y == 6:
                return "IN (C)", 2, ("in_c", 6)  # flags only, no store
            return f"IN {R8[y]}, (C)", 2, ("in_c", y)

        if z == 1:
            if y == 6:
                return "OUT (C), 0", 2, ("out_c", 6)
            return f"OUT (C), {R8[y]}", 2, ("out_c", y)

        if z == 2:
            if q == 0:
                return f"SBC HL, {R16[p]}", 2, ("sbc16", p)
            return f"ADC HL, {R16[p]}", 2, ("adc16", p)

        if z == 3:
            nn = _imm16(op1, op2)
            if q == 0:
                return f"LD (${nn:04X}), {R16[p]}", 4, ("st16_mem", R16[p], nn)
            return f"LD {R16[p]}, (${nn:04X})", 4, ("ld16_mem", R16[p], nn)

        if z == 4:
            return "NEG", 2, ("neg",)

        if z == 5:
            if y == 1:
                return "RETI", 2, ("reti",)
            return "RETN", 2, ("retn",)

        if z == 6:
            im_modes = (0, 0, 1, 2, 0, 0, 1, 2)
            return f"IM {im_modes[y]}", 2, ("im", im_modes[y])

        if z == 7:
            mnems = ("LD I, A", "LD R, A", "LD A, I", "LD A, R",
                     "RRD", "RLD", "NOP", "NOP")
            effects = (("ld_i_a",), ("ld_r_a",), ("ld_a_i",), ("ld_a_r",),
                       ("rrd",), ("rld",), ("nop",), ("nop",))
            return mnems[y], 2, effects[y]

    if x == 2 and z <= 3 and y >= 4:
        # Block instructions
        block_mnems = {
            (4, 0): "LDI",  (5, 0): "LDD",  (6, 0): "LDIR", (7, 0): "LDDR",
            (4, 1): "CPI",  (5, 1): "CPD",  (6, 1): "CPIR", (7, 1): "CPDR",
            (4, 2): "INI",  (5, 2): "IND",  (6, 2): "INIR", (7, 2): "INDR",
            (4, 3): "OUTI", (5, 3): "OUTD", (6, 3): "OTIR", (7, 3): "OTDR",
        }
        key = (y, z)
        if key in block_mnems:
            mnem = block_mnems[key]
            if z == 0:
                etype = "block_ld"
            elif z == 1:
                etype = "block_cp"
            elif z == 2:
                etype = "in_block"
            else:
                etype = "out_block"
            repeating = y >= 6
            return mnem, 2, (etype, mnem.lower(), repeating)

    return f"NOP (ED {opcode:02X})", 2, ("nop",)


# -- DD/FD prefix (IX/IY) decoding --

def _ix_r8(idx_reg):
    """Register names with IX/IY substitution for H, L, (HL)."""
    hi = f"{idx_reg}H"
    lo = f"{idx_reg}L"
    return ("B", "C", "D", "E", hi, lo, f"({idx_reg}+d)", "A")


def _decode_indexed(opcode, ops, pc, idx_reg):
    """Decode DD/FD prefixed instruction.  idx_reg = 'IX' or 'IY'."""
    x = (opcode >> 6) & 3
    y = (opcode >> 3) & 7
    z = opcode & 7
    p = y >> 1
    q = y & 1
    r8 = _ix_r8(idx_reg)

    # Many instructions are the same as unprefixed but with HL -> IX/IY
    # and (HL) -> (IX+d) / (IY+d)

    # Does this instruction use (IX+d)?
    uses_ixd = False
    disp = 0

    if x == 1:
        # LD r, r' group
        if y == 6 and z == 6:
            return "HALT", 2, ("halt",)

        uses_src = (z == 6)
        uses_dst = (y == 6)

        if uses_src or uses_dst:
            # Involves (IX+d)
            disp = _signed8(ops[0]) if len(ops) > 0 else 0
            disp_str = f"+{disp}" if disp >= 0 else str(disp)
            src = f"({idx_reg}{disp_str})" if uses_src else R8[z]
            dst = f"({idx_reg}{disp_str})" if uses_dst else R8[y]
            length = 3  # prefix + opcode + displacement
            if uses_src and uses_dst:
                # LD (IX+d), (IX+d) doesn't exist, this is LD (IX+d), (IX+d)
                # which is actually HALT... already handled above
                pass
            if uses_src and not uses_dst:
                return f"LD {dst}, {src}", length, ("ld8_mem", dst, idx_reg)
            if uses_dst and not uses_src:
                return f"LD {dst}, {src}", length, ("st8_mem", src, idx_reg)
        else:
            # No (IX+d), but IXH/IXL substitution for H/L
            src = r8[z]
            dst = r8[y]
            return f"LD {dst}, {src}", 2, ("ld8", y, z, idx_reg)

    if x == 2:
        # ALU A, r'
        if z == 6:
            disp = _signed8(ops[0]) if len(ops) > 0 else 0
            disp_str = f"+{disp}" if disp >= 0 else str(disp)
            return (f"{ALU_MNEM[y]} ({idx_reg}{disp_str})", 3,
                    ("alu8_mem", y, idx_reg))
        return f"{ALU_MNEM[y]} {r8[z]}", 2, ("alu8", y, z, idx_reg)

    if x == 0:
        if z == 1:
            if q == 0:
                if p == 2:
                    nn = _imm16(ops[0], ops[1]) if len(ops) >= 2 else 0
                    return (f"LD {idx_reg}, ${nn:04X}", 4,
                            ("ld16_imm_ix", idx_reg, nn))
                # Other pairs are not affected by DD/FD
                return _add_prefix_len(*_decode_main(opcode, ops, pc))
            if p == 2:
                return f"ADD {idx_reg}, {R16[p]}", 2, ("add16_ix", idx_reg, p)
            return f"ADD {idx_reg}, {R16[p]}", 2, ("add16_ix", idx_reg, p)

        if z == 2:
            if p == 2:
                nn = _imm16(ops[0], ops[1]) if len(ops) >= 2 else 0
                if q == 0:
                    return (f"LD (${nn:04X}), {idx_reg}", 4,
                            ("st16_mem", idx_reg, nn))
                return (f"LD {idx_reg}, (${nn:04X})", 4,
                        ("ld16_mem", idx_reg, nn))
            return _add_prefix_len(*_decode_main(opcode, ops, pc))

        if z == 3:
            if p == 2:
                if q == 0:
                    return f"INC {idx_reg}", 2, ("inc16_ix", idx_reg)
                return f"DEC {idx_reg}", 2, ("dec16_ix", idx_reg)
            return _add_prefix_len(*_decode_main(opcode, ops, pc))

        if z == 4:
            if y == 6:
                disp = _signed8(ops[0]) if len(ops) > 0 else 0
                disp_str = f"+{disp}" if disp >= 0 else str(disp)
                return (f"INC ({idx_reg}{disp_str})", 3,
                        ("inc8_mem", idx_reg))
            if y == 4 or y == 5:
                return f"INC {r8[y]}", 2, ("inc8", y, idx_reg)
            return _add_prefix_len(*_decode_main(opcode, ops, pc))

        if z == 5:
            if y == 6:
                disp = _signed8(ops[0]) if len(ops) > 0 else 0
                disp_str = f"+{disp}" if disp >= 0 else str(disp)
                return (f"DEC ({idx_reg}{disp_str})", 3,
                        ("dec8_mem", idx_reg))
            if y == 4 or y == 5:
                return f"DEC {r8[y]}", 2, ("dec8", y, idx_reg)
            return _add_prefix_len(*_decode_main(opcode, ops, pc))

        if z == 6:
            if y == 6:
                disp = _signed8(ops[0]) if len(ops) > 0 else 0
                disp_str = f"+{disp}" if disp >= 0 else str(disp)
                n = ops[1] if len(ops) > 1 else 0
                return (f"LD ({idx_reg}{disp_str}), ${n:02X}", 4,
                        ("st8_imm_mem", idx_reg, n))
            if y == 4 or y == 5:
                n = ops[0] if len(ops) > 0 else 0
                return f"LD {r8[y]}, ${n:02X}", 3, ("ld8_imm", y, n, idx_reg)
            return _add_prefix_len(*_decode_main(opcode, ops, pc))

        # z == 0 or z == 7: not affected by DD/FD
        return _add_prefix_len(*_decode_main(opcode, ops, pc))

    if x == 3:
        if z == 1:
            if q == 0 and p == 2:
                return f"POP {idx_reg}", 2, ("pop_ix", idx_reg)
            if q == 1:
                if p == 2:
                    return f"JP ({idx_reg})", 2, ("jp_ix", idx_reg)
                if p == 3:
                    return f"LD SP, {idx_reg}", 2, ("ld_sp_ix", idx_reg)
            return _add_prefix_len(*_decode_main(opcode, ops, pc))

        if z == 3 and y == 4:
            return f"EX (SP), {idx_reg}", 2, ("ex", "(SP)", idx_reg)

        if z == 5:
            if q == 0 and p == 2:
                return f"PUSH {idx_reg}", 2, ("push_ix", idx_reg)
            return _add_prefix_len(*_decode_main(opcode, ops, pc))

        # Everything else is unaffected by DD/FD
        return _add_prefix_len(*_decode_main(opcode, ops, pc))

    return _add_prefix_len(*_decode_main(opcode, ops, pc))


def _add_prefix_len(mnem, length, effect):
    """Add 1 to length for the DD/FD prefix byte."""
    return mnem, length + 1, effect


# -- DDCB/FDCB prefix decoding --

def _decode_ixcb(opcode, disp, idx_reg):
    """Decode DD CB d opcode / FD CB d opcode instructions."""
    if disp is None:
        return f"??? (IXCB, missing displacement)", 4, ("unknown",)
    x = (opcode >> 6) & 3
    y = (opcode >> 3) & 7
    z = opcode & 7
    disp_s = _signed8(disp)
    disp_str = f"+{disp_s}" if disp_s >= 0 else str(disp_s)
    loc = f"({idx_reg}{disp_str})"

    if x == 0:
        if z == 6:
            return f"{ROT_MNEM[y]} {loc}", 4, ("rot_mem", y, idx_reg)
        # Undocumented: rotate (IX+d) and copy to register
        return (f"{ROT_MNEM[y]} {loc}, {R8[z]}", 4,
                ("rot_mem", y, idx_reg, z))

    if x == 1:
        return f"BIT {y}, {loc}", 4, ("bit_mem", y, idx_reg)

    if x == 2:
        if z == 6:
            return f"RES {y}, {loc}", 4, ("res_mem", y, idx_reg)
        return f"RES {y}, {loc}, {R8[z]}", 4, ("res_mem", y, idx_reg, z)

    # x == 3
    if z == 6:
        return f"SET {y}, {loc}", 4, ("set_mem", y, idx_reg)
    return f"SET {y}, {loc}, {R8[z]}", 4, ("set_mem", y, idx_reg, z)


# -- Operand byte counts (used by assembler to know how many more bytes to collect) --

def operand_count_main(opcode):
    """Number of operand bytes following an unprefixed opcode."""
    x = (opcode >> 6) & 3
    y = (opcode >> 3) & 7
    z = opcode & 7
    p = y >> 1
    q = y & 1

    if x == 0:
        if z == 0 and y >= 2:
            return 1  # JR/DJNZ
        if z == 1 and q == 0:
            return 2  # LD rr, nn
        if z == 2 and (p == 2 or p == 3):
            return 2  # LD (nn), A/HL and LD A/HL, (nn)
        if z == 6:
            return 1  # LD r, n
        return 0

    if x == 1 or x == 2:
        return 0

    # x == 3
    if z == 2 or z == 4:
        return 2  # JP cc, nn / CALL cc, nn
    if z == 3:
        if y == 0:
            return 2  # JP nn
        if y == 2 or y == 3:
            return 1  # OUT (n),A / IN A,(n)
        return 0
    if z == 5:
        if q == 0:
            return 0  # PUSH
        if p == 0:
            return 2  # CALL nn
        return 0  # prefix bytes (shouldn't reach here)
    if z == 6:
        return 1  # ALU A, n
    return 0


def operand_count_ed(opcode):
    """Number of operand bytes following an ED-prefixed opcode."""
    x = (opcode >> 6) & 3
    y = (opcode >> 3) & 7
    z = opcode & 7
    if x == 1 and z == 3:
        return 2  # LD (nn), rr / LD rr, (nn)
    return 0


def operand_count_indexed(opcode):
    """Number of operand bytes following a DD/FD-prefixed opcode.

    This is trickier because (IX+d) instructions have an extra displacement byte.
    """
    x = (opcode >> 6) & 3
    y = (opcode >> 3) & 7
    z = opcode & 7
    p = y >> 1
    q = y & 1

    if x == 0:
        if z == 1 and q == 0 and p == 2:
            return 2  # LD IX, nn
        if z == 2 and p == 2:
            return 2  # LD (nn), IX / LD IX, (nn)
        if z == 4 and y == 6:
            return 1  # INC (IX+d)
        if z == 5 and y == 6:
            return 1  # DEC (IX+d)
        if z == 6:
            if y == 6:
                return 2  # LD (IX+d), n: displacement + immediate
            if y in (4, 5):
                return 1  # LD IXH/IXL, n
            return operand_count_main(opcode)
        return operand_count_main(opcode)

    if x == 1:
        if y == 6 and z != 6:
            return 1  # LD (IX+d), r
        if z == 6 and y != 6:
            return 1  # LD r, (IX+d)
        return 0

    if x == 2:
        if z == 6:
            return 1  # ALU A, (IX+d)
        return 0

    # x == 3 - mostly unaffected
    return operand_count_main(opcode)


# -- Public API --

def decode(pc, prefix, opcode, operands, displacement=None, idx_reg=None):
    """Decode a Z80 instruction.

    Args:
        pc: program counter of the instruction start
        prefix: None, 'CB', 'ED', 'IX', 'IXCB'
        opcode: main opcode byte
        operands: list of operand bytes (after opcode, not including prefix/displacement)
        displacement: signed displacement for DDCB/FDCB
        idx_reg: 'IX' or 'IY' when prefix is 'IX' or 'IXCB'

    Returns:
        Instruction with mnemonic and effect populated.
    """
    if prefix is None:
        mnem, length, effect = _decode_main(opcode, operands, pc)
    elif prefix == "CB":
        mnem, length, effect = _decode_cb(opcode)
    elif prefix == "ED":
        mnem, length, effect = _decode_ed(opcode, operands, pc)
    elif prefix == "IX":
        mnem, length, effect = _decode_indexed(opcode, operands, pc, idx_reg)
    elif prefix == "IXCB":
        mnem, length, effect = _decode_ixcb(opcode, displacement, idx_reg)
    else:
        mnem, length, effect = "???", 1, ("unknown",)

    instr = Instruction()
    instr.pc = pc
    instr.length = length
    instr.mnemonic = mnem
    instr.effect = effect
    return instr
