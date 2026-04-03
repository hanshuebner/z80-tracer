"""Trace filtering and triggering for Z80 instruction traces.

All filtering happens client-side — the firmware always streams everything.
The TraceFilter sits after instruction decode and state update, deciding
what to print and when to stop.

Trigger semantics:
- Multiple triggers combine with OR (any trigger fires → tracing starts)
- Triggers are one-shot (once fired, stays active until a stop condition)
- If no triggers are defined, tracing is active from the start
"""

import re
from collections import deque
from dataclasses import dataclass, field


# -- Argument parsing helpers --

def parse_address_range(s):
    """Parse '0xF000-0xFFFF' or 'F000-FFFF' into (start, end)."""
    parts = s.split("-", 1)
    if len(parts) != 2:
        raise ValueError(f"Invalid address range: {s} (expected START-END)")
    return (int(parts[0], 0), int(parts[1], 0))


def parse_address_or_range(s):
    """Parse '0x100' into (0x100, 0x100) or '0x100-0x200' into (0x100, 0x200)."""
    if "-" in s:
        return parse_address_range(s)
    addr = int(s, 0)
    return (addr, addr)


def parse_trigger_addr(s):
    """Parse '0xC000' or '0xC000=0xFF' into (addr, value_or_None)."""
    if "=" in s:
        addr_s, val_s = s.split("=", 1)
        return (int(addr_s, 0), int(val_s, 0))
    return (int(s, 0), None)


# -- Configuration --

@dataclass
class FilterConfig:
    address_masks: list = field(default_factory=list)       # [(start, end), ...]
    start_at: list = field(default_factory=list)   # [(start, end), ...]
    stop_at: int = None
    start_on: str = None          # mnemonic regex pattern
    stop_on: str = None           # mnemonic regex pattern
    limit: int = None             # max printed instructions
    mem_read_triggers: list = field(default_factory=list)    # [(addr, value|None), ...]
    mem_write_triggers: list = field(default_factory=list)
    io_read_triggers: list = field(default_factory=list)
    io_write_triggers: list = field(default_factory=list)
    trigger_int: bool = False
    window_size: int = 0

    @property
    def has_triggers(self):
        """True if any start triggers are defined."""
        return any([
            self.start_at,
            self.start_on,
            self.mem_read_triggers,
            self.mem_write_triggers,
            self.io_read_triggers,
            self.io_write_triggers,
            self.trigger_int,
        ])

    @property
    def has_any_config(self):
        """True if any filtering/triggering is configured."""
        return (self.has_triggers or self.address_masks or
                self.stop_at is not None or self.stop_on or
                self.limit is not None)


@dataclass
class OutputAction:
    lines: list = field(default_factory=list)
    stop: bool = False


# -- Filter --

class TraceFilter:

    def __init__(self, config, format_fn):
        """
        Args:
            config: FilterConfig with all filter settings
            format_fn: callable(instr, reg_changes) -> str
        """
        self._config = config
        self._format = format_fn
        self._active = not config.has_triggers
        self._printed = 0
        self._ring = deque(maxlen=max(config.window_size, 1))
        self._prev_pc = None       # for interrupt detection
        self._prev_length = None
        # Track suppressed instructions per mask range
        self._mask_suppressed = 0
        self._in_mask = False

    def evaluate(self, instr, reg_changes, loop_action):
        """Evaluate one decoded instruction. Returns OutputAction."""
        action = OutputAction()

        # Always store in ring buffer (for pre-trigger window)
        self._ring.append((instr, reg_changes, loop_action))

        if not self._active:
            # Check start triggers
            reason = self._check_triggers(instr)
            if reason:
                self._active = True
                # Flush ring buffer as pre-trigger context
                if self._config.window_size > 0:
                    # All entries except the last one (current) are context
                    entries = list(self._ring)
                    if len(entries) > 1:
                        action.lines.append(
                            f"; --- pre-trigger context ({len(entries)-1} instructions) ---")
                        for entry_instr, entry_rc, entry_la in entries[:-1]:
                            action.lines.append(self._format(entry_instr, entry_rc))
                action.lines.append(f"; === TRIGGER: {reason} ===")
                action.lines.append(self._format(instr, reg_changes))
                self._printed += 1
                self._update_prev(instr)
                self._check_limit(action)
                return action
            else:
                self._update_prev(instr)
                return action  # empty, suppress

        # Active — check stop conditions first
        if self._should_stop(instr):
            self._active = False
            self._flush_mask_summary(action)
            action.lines.append(f"; === STOP ===")
            action.stop = True
            self._update_prev(instr)
            return action

        # Check address masks
        if self._is_masked(instr.pc):
            self._mask_suppressed += 1
            self._in_mask = True
            self._update_prev(instr)
            return action  # suppress

        # Leaving a masked region
        self._flush_mask_summary(action)

        # Check limit
        if self._config.limit is not None and self._printed >= self._config.limit:
            action.lines.append(f"; === LIMIT reached ({self._config.limit} instructions) ===")
            action.stop = True
            self._update_prev(instr)
            return action

        # Normal output — respect loop detector
        if loop_action == "normal":
            action.lines.append(self._format(instr, reg_changes))
            self._printed += 1
        elif loop_action == "loop_exit":
            action.lines.append(self._format(instr, reg_changes))
            self._printed += 1
        # "suppress" from loop detector: don't print

        self._update_prev(instr)
        self._check_limit(action)
        return action

    def _check_triggers(self, instr):
        """Check all start triggers. Returns description string or None."""
        cfg = self._config

        # PC-based (single address or range)
        for start, end in cfg.start_at:
            if start <= instr.pc <= end:
                if start == end:
                    return f"PC = ${instr.pc:04X}"
                return f"PC = ${instr.pc:04X} (in ${start:04X}-${end:04X})"

        # Mnemonic-based
        if cfg.start_on and re.search(cfg.start_on, instr.mnemonic, re.IGNORECASE):
            return f"mnemonic match '{instr.mnemonic}'"

        # Memory read triggers
        for addr, val in cfg.mem_read_triggers:
            for (a, v) in instr.data_reads:
                if a == addr and (val is None or v == val):
                    desc = f"mem read ${addr:04X}"
                    if val is not None:
                        desc += f" = ${v:02X}"
                    return desc

        # Memory write triggers
        for addr, val in cfg.mem_write_triggers:
            for (a, v) in instr.data_writes:
                if a == addr and (val is None or v == val):
                    desc = f"mem write ${addr:04X}"
                    if val is not None:
                        desc += f" = ${v:02X}"
                    return desc

        # IO read triggers
        for port, val in cfg.io_read_triggers:
            for (p, v) in instr.io_reads:
                if p == port and (val is None or v == val):
                    desc = f"IO read ${port:04X}"
                    if val is not None:
                        desc += f" = ${v:02X}"
                    return desc

        # IO write triggers
        for port, val in cfg.io_write_triggers:
            for (p, v) in instr.io_writes:
                if p == port and (val is None or v == val):
                    desc = f"IO write ${port:04X}"
                    if val is not None:
                        desc += f" = ${v:02X}"
                    return desc

        # Interrupt trigger: detect unexpected PC discontinuity to RST vector
        if cfg.trigger_int and self._prev_pc is not None and self._prev_length is not None:
            expected_pc = (self._prev_pc + self._prev_length) & 0xFFFF
            if instr.pc != expected_pc:
                # PC jumped unexpectedly — check if it's an RST vector (0x00,0x08,...,0x38)
                if instr.pc <= 0x38 and (instr.pc & 0x07) == 0:
                    return f"interrupt to ${instr.pc:04X}"

        return None

    def _should_stop(self, instr):
        cfg = self._config
        if cfg.stop_at is not None and instr.pc == cfg.stop_at:
            return True
        if cfg.stop_on and re.search(cfg.stop_on, instr.mnemonic, re.IGNORECASE):
            return True
        return False

    def _is_masked(self, pc):
        for start, end in self._config.address_masks:
            if start <= pc <= end:
                return True
        return False

    def _flush_mask_summary(self, action):
        if self._in_mask and self._mask_suppressed > 0:
            action.lines.append(
                f"; --- masked {self._mask_suppressed} instructions ---")
            self._mask_suppressed = 0
            self._in_mask = False

    def _check_limit(self, action):
        if (self._config.limit is not None
                and self._printed >= self._config.limit):
            action.lines.append(
                f"; === LIMIT reached ({self._config.limit} instructions) ===")
            action.stop = True

    def _update_prev(self, instr):
        self._prev_pc = instr.pc
        self._prev_length = instr.length
