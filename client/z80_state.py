"""Z80 CPU register state model.

Tracks all Z80 registers, updating them based on decoded instructions and
observed bus data from the tracer.  Registers whose value cannot be
determined are stored as None.

The key insight: the bus tracer gives us ground truth for every memory and
IO read/write, so we can learn register values from observed data even
when we don't know them from emulation alone.
"""

from typing import Optional

# Flag bit positions
FLAG_C = 0
FLAG_N = 1
FLAG_PV = 2
FLAG_H = 4
FLAG_Z = 6
FLAG_S = 7

# Register-pair index to name mapping (matches R16 in decoder)
R16_NAMES = ("BC", "DE", "HL", "SP")
R16AF_NAMES = ("BC", "DE", "HL", "AF")


class Z80State:
    __slots__ = (
        "a", "f", "b", "c", "d", "e", "h", "l",
        "a_", "f_", "b_", "c_", "d_", "e_", "h_", "l_",
        "ix", "iy", "sp", "pc",
        "i", "r", "iff1", "iff2", "im",
    )

    def __init__(self):
        self.reset()

    def reset(self):
        for attr in self.__slots__:
            setattr(self, attr, None)

    # -- Register pair accessors --

    def get_pair(self, name):
        """Get 16-bit register pair value.  Returns None if either half unknown."""
        if name == "BC":
            return _combine(self.b, self.c)
        if name == "DE":
            return _combine(self.d, self.e)
        if name == "HL":
            return _combine(self.h, self.l)
        if name == "AF":
            return _combine(self.a, self.f)
        if name == "SP":
            return self.sp
        if name == "IX":
            return self.ix
        if name == "IY":
            return self.iy
        return None

    def set_pair(self, name, value):
        if value is None:
            self._set_pair_none(name)
            return
        value &= 0xFFFF
        if name == "BC":
            self.b = (value >> 8) & 0xFF
            self.c = value & 0xFF
        elif name == "DE":
            self.d = (value >> 8) & 0xFF
            self.e = value & 0xFF
        elif name == "HL":
            self.h = (value >> 8) & 0xFF
            self.l = value & 0xFF
        elif name == "AF":
            self.a = (value >> 8) & 0xFF
            self.f = value & 0xFF
        elif name == "SP":
            self.sp = value
        elif name == "IX":
            self.ix = value
        elif name == "IY":
            self.iy = value

    def _set_pair_none(self, name):
        if name == "BC":
            self.b = self.c = None
        elif name == "DE":
            self.d = self.e = None
        elif name == "HL":
            self.h = self.l = None
        elif name == "AF":
            self.a = self.f = None
        elif name == "SP":
            self.sp = None
        elif name == "IX":
            self.ix = None
        elif name == "IY":
            self.iy = None

    def get_r8(self, idx, idx_reg=None):
        """Get 8-bit register by Z80 encoding index (0=B..7=A).
        idx=6 is (HL) which we can't read here -- return None.
        When idx_reg is set, 4=IXH/IYH, 5=IXL/IYL.
        """
        if idx_reg and idx == 4:
            v = self.ix if idx_reg == "IX" else self.iy
            return (v >> 8) & 0xFF if v is not None else None
        if idx_reg and idx == 5:
            v = self.ix if idx_reg == "IX" else self.iy
            return v & 0xFF if v is not None else None
        return (self.b, self.c, self.d, self.e, self.h, self.l, None, self.a)[idx]

    def set_r8(self, idx, value, idx_reg=None):
        if idx_reg and idx == 4:
            reg = "ix" if idx_reg == "IX" else "iy"
            cur = getattr(self, reg)
            if cur is not None and value is not None:
                setattr(self, reg, (value << 8) | (cur & 0xFF))
            else:
                setattr(self, reg, None)
            return
        if idx_reg and idx == 5:
            reg = "ix" if idx_reg == "IX" else "iy"
            cur = getattr(self, reg)
            if cur is not None and value is not None:
                setattr(self, reg, (cur & 0xFF00) | (value & 0xFF))
            else:
                setattr(self, reg, None)
            return
        attrs = ("b", "c", "d", "e", "h", "l", None, "a")
        attr = attrs[idx]
        if attr:
            setattr(self, attr, value if value is None else value & 0xFF)

    def get_r16(self, idx):
        """Get 16-bit register by R16 index (0=BC,1=DE,2=HL,3=SP)."""
        return self.get_pair(R16_NAMES[idx])

    def set_r16(self, idx, value):
        self.set_pair(R16_NAMES[idx], value)

    # -- Flag helpers --

    def get_flag(self, bit):
        if self.f is None:
            return None
        return (self.f >> bit) & 1

    def _set_flags_z_s(self, result):
        """Set Z and S flags from 8-bit result.  Clears N.  Preserves C."""
        if self.f is None:
            self.f = 0
        result &= 0xFF
        z = 1 if result == 0 else 0
        s = (result >> 7) & 1
        # Keep C, clear N, update Z and S
        self.f = (self.f & (1 << FLAG_C)) | (z << FLAG_Z) | (s << FLAG_S)

    def _set_flags_full(self, result, operand, carry_in, is_sub):
        """Compute all flags for 8-bit ADD/ADC/SUB/SBC result."""
        a = (result - ((-1 if is_sub else 1) * operand) -
             ((-1 if is_sub else 1) * carry_in)) & 0xFF
        res8 = result & 0xFF
        z = 1 if res8 == 0 else 0
        s = (res8 >> 7) & 1
        c = 1 if (result & 0x100) != 0 else 0
        n = 1 if is_sub else 0
        # Half carry
        if is_sub:
            h = 1 if (a & 0xF) < (operand & 0xF) + carry_in else 0
        else:
            h = 1 if ((a & 0xF) + (operand & 0xF) + carry_in) > 0xF else 0
        # Overflow: sign of result differs from sign of both operands
        if is_sub:
            pv = 1 if ((a ^ operand) & (a ^ res8) & 0x80) != 0 else 0
        else:
            pv = 1 if ((~(a ^ operand)) & (a ^ res8) & 0x80) != 0 else 0
        self.f = (c << FLAG_C) | (n << FLAG_N) | (pv << FLAG_PV) | \
                 (h << FLAG_H) | (z << FLAG_Z) | (s << FLAG_S)

    # -- Instruction execution --

    def execute(self, instr):
        """Update registers based on decoded instruction and observed bus data."""
        eff = instr.effect
        etype = eff[0]
        self.pc = (instr.pc + instr.length) & 0xFFFF

        handler = _HANDLERS.get(etype)
        if handler:
            handler(self, instr, eff)

    # -- State snapshot for loop detection --

    def snapshot_key(self):
        """Return a hashable tuple of key register state for loop detection."""
        return (self.pc, self.a, self.f, self.b, self.c, self.d, self.e,
                self.h, self.l, self.sp)

    def format_changed(self, prev_snapshot):
        """Format registers that changed since prev_snapshot."""
        names = ("PC", "A", "F", "B", "C", "D", "E", "H", "L", "SP")
        curr = self.snapshot_key()
        parts = []
        for i, (name, old, new) in enumerate(zip(names, prev_snapshot, curr)):
            if old != new and new is not None:
                if name == "PC":
                    continue  # PC always changes, skip
                if name == "F":
                    parts.append(f"F={_flags_str(new)}")
                elif name in ("SP",):
                    parts.append(f"{name}={new:04X}")
                else:
                    parts.append(f"{name}={new:02X}")
        return " ".join(parts)

    def format_registers(self):
        """Full register dump."""
        def r8(v):
            return f"{v:02X}" if v is not None else "??"
        def r16(v):
            return f"{v:04X}" if v is not None else "????"

        return (
            f"AF={r8(self.a)}{r8(self.f)} BC={r8(self.b)}{r8(self.c)} "
            f"DE={r8(self.d)}{r8(self.e)} HL={r8(self.h)}{r8(self.l)} "
            f"SP={r16(self.sp)} IX={r16(self.ix)} IY={r16(self.iy)}"
        )


def _flags_str(f):
    if f is None:
        return "?"
    parts = []
    if f & (1 << FLAG_S):
        parts.append("S")
    if f & (1 << FLAG_Z):
        parts.append("Z")
    if f & (1 << FLAG_H):
        parts.append("H")
    if f & (1 << FLAG_PV):
        parts.append("PV")
    if f & (1 << FLAG_N):
        parts.append("N")
    if f & (1 << FLAG_C):
        parts.append("C")
    return "".join(parts) if parts else "0"


def _combine(hi, lo):
    if hi is None or lo is None:
        return None
    return ((hi & 0xFF) << 8) | (lo & 0xFF)


# ============================================================
# Effect handlers
# ============================================================

def _h_nop(st, instr, eff):
    pass

def _h_halt(st, instr, eff):
    pass

def _h_ld8(st, instr, eff):
    # ld8, dst_idx, src_idx [, idx_reg]
    dst, src = eff[1], eff[2]
    idx_reg = eff[3] if len(eff) > 3 else None
    if src == 6:  # (HL) / (IX+d)
        # Value comes from a data read
        val = instr.data_reads[0][1] if instr.data_reads else None
        st.set_r8(dst, val, idx_reg)
    else:
        val = st.get_r8(src, idx_reg)
        st.set_r8(dst, val, idx_reg)

def _h_ld8_imm(st, instr, eff):
    # ld8_imm, dst_idx, value [, idx_reg]
    dst, val = eff[1], eff[2]
    idx_reg = eff[3] if len(eff) > 3 else None
    st.set_r8(dst, val, idx_reg)

def _h_ld8_mem(st, instr, eff):
    # ld8_mem, dst_name, src_addr_name
    # The actual value comes from the bus data read
    dst = eff[1]
    if instr.data_reads:
        addr, val = instr.data_reads[0]
        if dst == "A":
            st.a = val
        # Learn pointer register value from the address (but NOT IX/IY
        # since the observed address includes a displacement offset)
        src_reg = eff[2]
        if isinstance(src_reg, str) and src_reg in ("BC", "DE", "HL"):
            st.set_pair(src_reg, addr)
    else:
        if dst == "A":
            st.a = None

def _h_st8_mem(st, instr, eff):
    # st8_mem, src_name, dst_addr_name
    # The value written tells us the source register value
    src = eff[1]
    if instr.data_writes:
        addr, val = instr.data_writes[0]
        if src == "A":
            st.a = val
        elif src in ("B", "C", "D", "E", "H", "L"):
            setattr(st, src.lower(), val)
        # Learn pointer register from address
        dst_reg = eff[2]
        if isinstance(dst_reg, str) and dst_reg in ("BC", "DE", "HL"):
            st.set_pair(dst_reg, addr)

def _h_st8_imm_mem(st, instr, eff):
    # st8_imm_mem, idx_reg, immediate_value
    # LD (IX+d), n - we learn IX from the write address
    idx_reg = eff[1]
    if instr.data_writes:
        addr, _val = instr.data_writes[0]
        # addr = IX + d, but we don't easily know d here, so skip

def _h_ld16_imm(st, instr, eff):
    # ld16_imm, pair_idx, value
    st.set_r16(eff[1], eff[2])

def _h_ld16_imm_ix(st, instr, eff):
    # ld16_imm_ix, idx_reg, value
    st.set_pair(eff[1], eff[2])

def _h_ld16_mem(st, instr, eff):
    # ld16_mem, pair_name, address
    pair_name = eff[1]
    if len(instr.data_reads) >= 2:
        lo = instr.data_reads[0][1]
        hi = instr.data_reads[1][1]
        st.set_pair(pair_name, (hi << 8) | lo)
    else:
        st._set_pair_none(pair_name)

def _h_st16_mem(st, instr, eff):
    # st16_mem, pair_name, address
    pair_name = eff[1]
    if len(instr.data_writes) >= 2:
        lo = instr.data_writes[0][1]
        hi = instr.data_writes[1][1]
        st.set_pair(pair_name, (hi << 8) | lo)

def _h_ld_sp_hl(st, instr, eff):
    st.sp = st.get_pair("HL")

def _h_ld_sp_ix(st, instr, eff):
    st.sp = st.get_pair(eff[1])

def _h_push(st, instr, eff):
    # push, pair_idx (R16AF encoding)
    pair = R16AF_NAMES[eff[1]]
    # We learn SP from the write address and register values from data written
    if len(instr.data_writes) >= 2:
        hi_addr, hi_val = instr.data_writes[0]
        lo_addr, lo_val = instr.data_writes[1]
        st.sp = (lo_addr - 1) & 0xFFFF  # SP was decremented by 2 before push
        # We learn the pushed register values
        if pair == "AF":
            st.a = hi_val
            st.f = lo_val
        elif pair == "BC":
            st.b = hi_val
            st.c = lo_val
        elif pair == "DE":
            st.d = hi_val
            st.e = lo_val
        elif pair == "HL":
            st.h = hi_val
            st.l = lo_val
    # SP decremented by 2
    if st.sp is not None:
        st.sp = (st.sp - 2) & 0xFFFF if not instr.data_writes else \
                (instr.data_writes[-1][0] & 0xFFFF) if instr.data_writes else st.sp

def _h_push_ix(st, instr, eff):
    idx_reg = eff[1]
    if len(instr.data_writes) >= 2:
        hi_val = instr.data_writes[0][1]
        lo_val = instr.data_writes[1][1]
        st.set_pair(idx_reg, (hi_val << 8) | lo_val)
        st.sp = instr.data_writes[1][0] & 0xFFFF

def _h_pop(st, instr, eff):
    pair = R16AF_NAMES[eff[1]]
    if len(instr.data_reads) >= 2:
        lo_val = instr.data_reads[0][1]
        hi_val = instr.data_reads[1][1]
        lo_addr = instr.data_reads[0][0]
        st.set_pair(pair, (hi_val << 8) | lo_val)
        st.sp = (lo_addr + 2) & 0xFFFF
    else:
        st._set_pair_none(pair)

def _h_pop_ix(st, instr, eff):
    idx_reg = eff[1]
    if len(instr.data_reads) >= 2:
        lo_val = instr.data_reads[0][1]
        hi_val = instr.data_reads[1][1]
        lo_addr = instr.data_reads[0][0]
        st.set_pair(idx_reg, (hi_val << 8) | lo_val)
        st.sp = (lo_addr + 2) & 0xFFFF

def _h_ex_af(st, instr, eff):
    st.a, st.a_ = st.a_, st.a
    st.f, st.f_ = st.f_, st.f

def _h_exx(st, instr, eff):
    st.b, st.b_ = st.b_, st.b
    st.c, st.c_ = st.c_, st.c
    st.d, st.d_ = st.d_, st.d
    st.e, st.e_ = st.e_, st.e
    st.h, st.h_ = st.h_, st.h
    st.l, st.l_ = st.l_, st.l

def _h_ex(st, instr, eff):
    r1, r2 = eff[1], eff[2]
    if r1 == "DE" and r2 == "HL":
        st.d, st.h = st.h, st.d
        st.e, st.l = st.l, st.e
    elif r1 == "(SP)":
        # EX (SP), HL/IX/IY - we can learn values from bus data
        if len(instr.data_reads) >= 2 and len(instr.data_writes) >= 2:
            # Old value read from stack, new value written
            lo_rd = instr.data_reads[0][1]
            hi_rd = instr.data_reads[1][1]
            lo_wr = instr.data_writes[0][1]
            hi_wr = instr.data_writes[1][1]
            old_reg = (hi_wr << 8) | lo_wr
            new_reg = (hi_rd << 8) | lo_rd
            st.set_pair(r2, new_reg)
            # SP pointed to the read address
            st.sp = instr.data_reads[0][0]

def _h_inc8(st, instr, eff):
    idx = eff[1]
    idx_reg = eff[2] if len(eff) > 2 else None
    if idx == 6:
        # INC (HL) - read-modify-write
        if instr.data_reads and instr.data_writes:
            old = instr.data_reads[0][1]
            new = instr.data_writes[0][1]
            # Learn pointer value from address
            st.set_pair(idx_reg or "HL", instr.data_reads[0][0])
        return
    val = st.get_r8(idx, idx_reg)
    if val is not None:
        result = (val + 1) & 0xFF
        st.set_r8(idx, result, idx_reg)
        st._set_flags_z_s(result)
    else:
        st.set_r8(idx, None, idx_reg)
        st.f = None

def _h_dec8(st, instr, eff):
    idx = eff[1]
    idx_reg = eff[2] if len(eff) > 2 else None
    if idx == 6:
        if instr.data_reads and instr.data_writes:
            st.set_pair(idx_reg or "HL", instr.data_reads[0][0])
        return
    val = st.get_r8(idx, idx_reg)
    if val is not None:
        result = (val - 1) & 0xFF
        st.set_r8(idx, result, idx_reg)
        st._set_flags_z_s(result)
        # Set N flag for subtract
        if st.f is not None:
            st.f |= (1 << FLAG_N)
    else:
        st.set_r8(idx, None, idx_reg)
        st.f = None

def _h_inc8_mem(st, instr, eff):
    _h_inc8(st, instr, (eff[0], 6, eff[1]))

def _h_dec8_mem(st, instr, eff):
    _h_dec8(st, instr, (eff[0], 6, eff[1]))

def _h_inc16(st, instr, eff):
    val = st.get_r16(eff[1])
    if val is not None:
        st.set_r16(eff[1], (val + 1) & 0xFFFF)
    else:
        st.set_r16(eff[1], None)

def _h_dec16(st, instr, eff):
    val = st.get_r16(eff[1])
    if val is not None:
        st.set_r16(eff[1], (val - 1) & 0xFFFF)
    else:
        st.set_r16(eff[1], None)

def _h_inc16_ix(st, instr, eff):
    v = st.get_pair(eff[1])
    st.set_pair(eff[1], (v + 1) & 0xFFFF if v is not None else None)

def _h_dec16_ix(st, instr, eff):
    v = st.get_pair(eff[1])
    st.set_pair(eff[1], (v - 1) & 0xFFFF if v is not None else None)

def _h_alu8(st, instr, eff):
    # alu8, op, src_idx [, idx_reg]
    op, src_idx = eff[1], eff[2]
    idx_reg = eff[3] if len(eff) > 3 else None
    if src_idx == 6:
        val = instr.data_reads[0][1] if instr.data_reads else None
    else:
        val = st.get_r8(src_idx, idx_reg)
    # Special case: ALU A, A where the operand is the accumulator itself
    if src_idx == 7 and idx_reg is None:
        _do_alu_self(st, op)
    else:
        _do_alu(st, op, val)

def _h_alu8_imm(st, instr, eff):
    # alu8_imm, op, value
    _do_alu(st, eff[1], eff[2])

def _h_alu8_mem(st, instr, eff):
    # alu8_mem, op, idx_reg
    val = instr.data_reads[0][1] if instr.data_reads else None
    _do_alu(st, eff[1], val)

def _do_alu_self(st, op):
    """ALU A, A - both operands are the same value (which may be unknown).
    Some operations have deterministic results regardless of A's value."""
    if op == 2:  # SUB A, A -> always 0
        st.a = 0
        st.f = (1 << FLAG_Z) | (1 << FLAG_N)  # Z set, N set, C clear
        return
    if op == 5:  # XOR A, A -> always 0
        st.a = 0
        st.f = (1 << FLAG_Z)  # Z set, all others clear
        return
    if op == 7:  # CP A, A -> A unchanged, Z set
        if st.f is None:
            st.f = 0
        st.f = (st.f & (1 << FLAG_C)) | (1 << FLAG_Z) | (1 << FLAG_N)
        return
    if op == 4:  # AND A, A -> A unchanged
        if st.a is not None:
            st._set_flags_z_s(st.a)
            st.f = (st.f or 0) | (1 << FLAG_H)
            st.f &= ~(1 << FLAG_C)
        return
    if op == 6:  # OR A, A -> A unchanged
        if st.a is not None:
            st._set_flags_z_s(st.a)
            st.f = (st.f or 0) & ~((1 << FLAG_H) | (1 << FLAG_C))
        return
    # ADD/ADC/SBC A,A - need the actual value
    _do_alu(st, op, st.a)


def _do_alu(st, op, val):
    """Execute ALU operation on A. op: 0=ADD,1=ADC,2=SUB,3=SBC,4=AND,5=XOR,6=OR,7=CP"""
    if val is None or st.a is None:
        st.a = None
        st.f = None
        return

    a = st.a
    carry = (st.f >> FLAG_C) & 1 if st.f is not None else None

    if op == 0:  # ADD
        result = a + val
        st.a = result & 0xFF
        st._set_flags_full(result, val, 0, False)
    elif op == 1:  # ADC
        if carry is None:
            st.a = None
            st.f = None
            return
        result = a + val + carry
        st.a = result & 0xFF
        st._set_flags_full(result, val, carry, False)
    elif op == 2:  # SUB
        result = a - val
        st.a = result & 0xFF
        st._set_flags_full(result & 0x1FF, val, 0, True)
    elif op == 3:  # SBC
        if carry is None:
            st.a = None
            st.f = None
            return
        result = a - val - carry
        st.a = result & 0xFF
        st._set_flags_full(result & 0x1FF, val, carry, True)
    elif op == 4:  # AND
        st.a = a & val
        st._set_flags_z_s(st.a)
        st.f = (st.f or 0) | (1 << FLAG_H)  # H always set
        st.f &= ~(1 << FLAG_C)  # C always clear
    elif op == 5:  # XOR
        st.a = a ^ val
        st._set_flags_z_s(st.a)
        st.f = (st.f or 0) & ~((1 << FLAG_H) | (1 << FLAG_C))
    elif op == 6:  # OR
        st.a = a | val
        st._set_flags_z_s(st.a)
        st.f = (st.f or 0) & ~((1 << FLAG_H) | (1 << FLAG_C))
    elif op == 7:  # CP
        result = a - val
        # A is unchanged, only flags affected
        st._set_flags_full(result & 0x1FF, val, 0, True)

def _h_add16(st, instr, eff):
    # add16, dst_idx, src_idx
    dst_val = st.get_r16(eff[1])
    src_val = st.get_r16(eff[2])
    if dst_val is not None and src_val is not None:
        result = dst_val + src_val
        st.set_r16(eff[1], result & 0xFFFF)
        # Only C flag affected (and H, N cleared)
        if st.f is not None:
            c = 1 if result > 0xFFFF else 0
            st.f = (st.f & ~((1 << FLAG_N) | (1 << FLAG_H) | (1 << FLAG_C))) | \
                   (c << FLAG_C)
    else:
        st.set_r16(eff[1], None)

def _h_add16_ix(st, instr, eff):
    idx_reg = eff[1]
    src_idx = eff[2]
    dst_val = st.get_pair(idx_reg)
    src_val = st.get_r16(src_idx) if src_idx != 2 else st.get_pair(idx_reg)
    if dst_val is not None and src_val is not None:
        result = dst_val + src_val
        st.set_pair(idx_reg, result & 0xFFFF)
    else:
        st.set_pair(idx_reg, None)

def _h_adc16(st, instr, eff):
    src_val = st.get_r16(eff[1])
    hl = st.get_pair("HL")
    carry = (st.f >> FLAG_C) & 1 if st.f is not None else None
    if hl is not None and src_val is not None and carry is not None:
        result = hl + src_val + carry
        st.set_pair("HL", result & 0xFFFF)
        z = 1 if (result & 0xFFFF) == 0 else 0
        s = (result >> 15) & 1
        c = 1 if result > 0xFFFF else 0
        st.f = (c << FLAG_C) | (z << FLAG_Z) | (s << FLAG_S)
    else:
        st.set_pair("HL", None)
        st.f = None

def _h_sbc16(st, instr, eff):
    src_val = st.get_r16(eff[1])
    hl = st.get_pair("HL")
    carry = (st.f >> FLAG_C) & 1 if st.f is not None else None
    if hl is not None and src_val is not None and carry is not None:
        result = hl - src_val - carry
        st.set_pair("HL", result & 0xFFFF)
        z = 1 if (result & 0xFFFF) == 0 else 0
        s = (result >> 15) & 1
        c = 1 if result < 0 else 0
        st.f = (c << FLAG_C) | (z << FLAG_Z) | (s << FLAG_S) | (1 << FLAG_N)
    else:
        st.set_pair("HL", None)
        st.f = None

def _h_jp(st, instr, eff):
    # PC already set from opcode fetch of next instruction
    pass

def _h_jp_hl(st, instr, eff):
    pass

def _h_jp_ix(st, instr, eff):
    pass

def _h_jr(st, instr, eff):
    pass

def _h_djnz(st, instr, eff):
    if st.b is not None:
        st.b = (st.b - 1) & 0xFF

def _h_call(st, instr, eff):
    # CALL pushes return address to stack
    if len(instr.data_writes) >= 2:
        st.sp = instr.data_writes[1][0] & 0xFFFF

def _h_ret(st, instr, eff):
    if len(instr.data_reads) >= 2:
        st.sp = (instr.data_reads[0][0] + 2) & 0xFFFF

def _h_reti(st, instr, eff):
    _h_ret(st, instr, eff)

def _h_retn(st, instr, eff):
    _h_ret(st, instr, eff)

def _h_rst(st, instr, eff):
    if len(instr.data_writes) >= 2:
        st.sp = instr.data_writes[1][0] & 0xFFFF

def _h_rot_a(st, instr, eff):
    # RLCA, RRCA, RLA, RRA
    y = eff[1]
    if st.a is None:
        st.f = None
        return
    a = st.a
    c_in = (st.f >> FLAG_C) & 1 if st.f is not None else 0
    if y == 0:  # RLCA
        c = (a >> 7) & 1
        st.a = ((a << 1) | c) & 0xFF
    elif y == 1:  # RRCA
        c = a & 1
        st.a = ((a >> 1) | (c << 7)) & 0xFF
    elif y == 2:  # RLA
        c = (a >> 7) & 1
        st.a = ((a << 1) | c_in) & 0xFF
    elif y == 3:  # RRA
        c = a & 1
        st.a = ((a >> 1) | (c_in << 7)) & 0xFF
    else:
        return
    # Only C, H, N flags affected
    if st.f is not None:
        st.f = (st.f & ~((1 << FLAG_C) | (1 << FLAG_H) | (1 << FLAG_N))) | \
               (c << FLAG_C)

def _h_rot(st, instr, eff):
    # rot, op, reg_idx
    op, idx = eff[1], eff[2]
    if idx == 6:
        # (HL) rotation - read-modify-write visible on bus
        return
    val = st.get_r8(idx)
    if val is None:
        st.set_r8(idx, None)
        st.f = None
        return
    c_in = (st.f >> FLAG_C) & 1 if st.f is not None else 0
    result, c = _do_rot(op, val, c_in)
    st.set_r8(idx, result)
    st._set_flags_z_s(result)
    if st.f is not None:
        st.f = (st.f & ~(1 << FLAG_C)) | (c << FLAG_C)

def _do_rot(op, val, c_in):
    if op == 0:  # RLC
        c = (val >> 7) & 1
        return ((val << 1) | c) & 0xFF, c
    if op == 1:  # RRC
        c = val & 1
        return ((val >> 1) | (c << 7)) & 0xFF, c
    if op == 2:  # RL
        c = (val >> 7) & 1
        return ((val << 1) | c_in) & 0xFF, c
    if op == 3:  # RR
        c = val & 1
        return ((val >> 1) | (c_in << 7)) & 0xFF, c
    if op == 4:  # SLA
        c = (val >> 7) & 1
        return (val << 1) & 0xFF, c
    if op == 5:  # SRA
        c = val & 1
        return ((val >> 1) | (val & 0x80)) & 0xFF, c
    if op == 6:  # SLL (undocumented)
        c = (val >> 7) & 1
        return ((val << 1) | 1) & 0xFF, c
    if op == 7:  # SRL
        c = val & 1
        return (val >> 1) & 0xFF, c
    return val, 0

def _h_bit(st, instr, eff):
    bit = eff[1]
    idx = eff[2]
    if idx == 6:
        val = instr.data_reads[0][1] if instr.data_reads else None
    else:
        val = st.get_r8(idx)
    if val is not None and st.f is not None:
        z = 1 if (val & (1 << bit)) == 0 else 0
        st.f = (st.f & (1 << FLAG_C)) | (z << FLAG_Z) | (1 << FLAG_H)
    else:
        st.f = None

def _h_set(st, instr, eff):
    bit, idx = eff[1], eff[2]
    if idx == 6:
        return
    val = st.get_r8(idx)
    if val is not None:
        st.set_r8(idx, val | (1 << bit))

def _h_res(st, instr, eff):
    bit, idx = eff[1], eff[2]
    if idx == 6:
        return
    val = st.get_r8(idx)
    if val is not None:
        st.set_r8(idx, val & ~(1 << bit))

def _h_scf(st, instr, eff):
    if st.f is not None:
        st.f = (st.f & ~((1 << FLAG_N) | (1 << FLAG_H))) | (1 << FLAG_C)

def _h_ccf(st, instr, eff):
    if st.f is not None:
        c = (st.f >> FLAG_C) & 1
        st.f = (st.f & ~((1 << FLAG_N) | (1 << FLAG_C))) | ((c ^ 1) << FLAG_C)

def _h_cpl(st, instr, eff):
    if st.a is not None:
        st.a = st.a ^ 0xFF
    if st.f is not None:
        st.f |= (1 << FLAG_N) | (1 << FLAG_H)

def _h_neg(st, instr, eff):
    if st.a is not None:
        old = st.a
        st.a = (0 - old) & 0xFF
        st._set_flags_full((0 - old) & 0x1FF, old, 0, True)
    else:
        st.a = None
        st.f = None

def _h_daa(st, instr, eff):
    # DAA is complex; mark A and F as unknown if we can't compute
    st.a = None
    st.f = None

def _h_di(st, instr, eff):
    st.iff1 = 0
    st.iff2 = 0

def _h_ei(st, instr, eff):
    st.iff1 = 1
    st.iff2 = 1

def _h_im(st, instr, eff):
    st.im = eff[1]

def _h_ld_i_a(st, instr, eff):
    st.i = st.a

def _h_ld_r_a(st, instr, eff):
    st.r = st.a

def _h_ld_a_i(st, instr, eff):
    st.a = st.i

def _h_ld_a_r(st, instr, eff):
    st.a = st.r

def _h_in(st, instr, eff):
    # IN A, (n) - value from IO read
    if instr.io_reads:
        st.a = instr.io_reads[0][1]

def _h_out(st, instr, eff):
    # OUT (n), A - value tells us A
    if instr.io_writes:
        st.a = instr.io_writes[0][1]

def _h_in_c(st, instr, eff):
    idx = eff[1]
    if instr.io_reads:
        port = instr.io_reads[0][0]
        val = instr.io_reads[0][1]
        if idx != 6:
            st.set_r8(idx, val)
        # Port address = BC
        st.set_pair("BC", port)

def _h_out_c(st, instr, eff):
    idx = eff[1]
    if instr.io_writes:
        port = instr.io_writes[0][0]
        val = instr.io_writes[0][1]
        if idx != 6:
            st.set_r8(idx, val)
        st.set_pair("BC", port)

def _h_block_ld(st, instr, eff):
    # LDI/LDD/LDIR/LDDR
    # These do: (DE) <- (HL), then HL+/-1, DE+/-1, BC-1
    # We see the read and write on the bus
    if instr.data_reads and instr.data_writes:
        src_addr = instr.data_reads[0][0]
        dst_addr = instr.data_writes[0][0]
        st.set_pair("HL", src_addr)
        st.set_pair("DE", dst_addr)
    # BC decremented
    bc = st.get_pair("BC")
    if bc is not None:
        st.set_pair("BC", (bc - 1) & 0xFFFF)
    # HL and DE adjusted
    is_dec = "d" in eff[1]  # ldd/lddr
    adj = -1 if is_dec else 1
    hl = st.get_pair("HL")
    de = st.get_pair("DE")
    if hl is not None:
        st.set_pair("HL", (hl + adj) & 0xFFFF)
    if de is not None:
        st.set_pair("DE", (de + adj) & 0xFFFF)

def _h_block_cp(st, instr, eff):
    # CPI/CPD/CPIR/CPDR: compare A with (HL), adjust HL, dec BC
    if instr.data_reads:
        src_addr = instr.data_reads[0][0]
        val = instr.data_reads[0][1]
        st.set_pair("HL", src_addr)
    bc = st.get_pair("BC")
    if bc is not None:
        st.set_pair("BC", (bc - 1) & 0xFFFF)
    is_dec = "d" in eff[1]
    adj = -1 if is_dec else 1
    hl = st.get_pair("HL")
    if hl is not None:
        st.set_pair("HL", (hl + adj) & 0xFFFF)

def _h_unknown(st, instr, eff):
    pass

def _h_rot_mem(st, instr, eff):
    pass

def _h_bit_mem(st, instr, eff):
    if st.f is not None and instr.data_reads:
        bit = eff[1]
        val = instr.data_reads[0][1]
        z = 1 if (val & (1 << bit)) == 0 else 0
        st.f = (st.f & (1 << FLAG_C)) | (z << FLAG_Z) | (1 << FLAG_H)

def _h_set_mem(st, instr, eff):
    pass

def _h_res_mem(st, instr, eff):
    pass

def _h_rrd(st, instr, eff):
    st.a = None
    st.f = None

def _h_rld(st, instr, eff):
    st.a = None
    st.f = None

def _h_in_block(st, instr, eff):
    # INI/IND/INIR/INDR: (HL) <- IN(C), B--, HL+/-
    if instr.io_reads:
        st.set_pair("BC", instr.io_reads[0][0])
    if st.b is not None:
        st.b = (st.b - 1) & 0xFF

def _h_out_block(st, instr, eff):
    if instr.io_writes:
        st.set_pair("BC", instr.io_writes[0][0])
    if st.b is not None:
        st.b = (st.b - 1) & 0xFF


_HANDLERS = {
    "nop": _h_nop,
    "halt": _h_halt,
    "ld8": _h_ld8,
    "ld8_imm": _h_ld8_imm,
    "ld8_mem": _h_ld8_mem,
    "st8_mem": _h_st8_mem,
    "st8_imm_mem": _h_st8_imm_mem,
    "ld16_imm": _h_ld16_imm,
    "ld16_imm_ix": _h_ld16_imm_ix,
    "ld16_mem": _h_ld16_mem,
    "st16_mem": _h_st16_mem,
    "ld_sp_hl": _h_ld_sp_hl,
    "ld_sp_ix": _h_ld_sp_ix,
    "push": _h_push,
    "push_ix": _h_push_ix,
    "pop": _h_pop,
    "pop_ix": _h_pop_ix,
    "ex_af": _h_ex_af,
    "exx": _h_exx,
    "ex": _h_ex,
    "inc8": _h_inc8,
    "dec8": _h_dec8,
    "inc8_mem": _h_inc8_mem,
    "dec8_mem": _h_dec8_mem,
    "inc16": _h_inc16,
    "dec16": _h_dec16,
    "inc16_ix": _h_inc16_ix,
    "dec16_ix": _h_dec16_ix,
    "alu8": _h_alu8,
    "alu8_imm": _h_alu8_imm,
    "alu8_mem": _h_alu8_mem,
    "add16": _h_add16,
    "add16_ix": _h_add16_ix,
    "adc16": _h_adc16,
    "sbc16": _h_sbc16,
    "jp": _h_jp,
    "jp_hl": _h_jp_hl,
    "jp_ix": _h_jp_ix,
    "jr": _h_jr,
    "djnz": _h_djnz,
    "call": _h_call,
    "ret": _h_ret,
    "reti": _h_reti,
    "retn": _h_retn,
    "rst": _h_rst,
    "rot_a": _h_rot_a,
    "rot": _h_rot,
    "rot_mem": _h_rot_mem,
    "bit": _h_bit,
    "bit_mem": _h_bit_mem,
    "set_": _h_set,
    "set_mem": _h_set_mem,
    "res": _h_res,
    "res_mem": _h_res_mem,
    "scf": _h_scf,
    "ccf": _h_ccf,
    "cpl": _h_cpl,
    "neg": _h_neg,
    "daa": _h_daa,
    "di": _h_di,
    "ei": _h_ei,
    "im": _h_im,
    "ld_i_a": _h_ld_i_a,
    "ld_r_a": _h_ld_r_a,
    "ld_a_i": _h_ld_a_i,
    "ld_a_r": _h_ld_a_r,
    "in_": _h_in,
    "out_": _h_out,
    "in_c": _h_in_c,
    "out_c": _h_out_c,
    "block_ld": _h_block_ld,
    "block_cp": _h_block_cp,
    "in_block": _h_in_block,
    "out_block": _h_out_block,
    "rrd": _h_rrd,
    "rld": _h_rld,
    "unknown": _h_unknown,
}
