"""Loop detection for Z80 instruction traces.

Two detection strategies:

1. **Exact-state loops** (infinite loops): same (PC, registers) seen twice.
   The machine will repeat forever until an interrupt or IO changes state.

2. **PC-sequence loops** (bounded loops): the same short sequence of PCs
   repeats N times.  Registers change each iteration (e.g. a counter
   decrementing) but the control flow pattern is identical.  Output is
   suppressed after a few iterations and a summary is printed when the
   loop exits.
"""

from collections import deque


# After this many repeated iterations of a PC sequence, start suppressing
SUPPRESS_AFTER = 2


class LoopDetector:

    def __init__(self):
        self._instr_count = 0

        # -- PC-sequence detection --
        # Recent PCs in a sliding window
        self._pc_history = deque(maxlen=512)
        # When a loop is detected:
        self._in_loop = False
        self._loop_seq = ()         # the repeating PC sequence
        self._loop_pos = 0          # position within current iteration
        self._loop_iterations = 0
        self._suppressing = False

        # -- Exact-state detection (for truly infinite loops) --
        self._state_seen = {}       # (pc, state_key) -> instr_count
        self._infinite_loop = False
        self._infinite_loop_pc = None

    def check(self, pc, state_key):
        """Check if we're in a loop.

        Returns:
            'normal'    - print this instruction normally
            'suppress'  - suppress this instruction (in a loop)
            'loop_exit' - loop just ended, caller should print summary
        """
        self._instr_count += 1

        # --- Exact-state detection ---
        full_key = (pc, state_key)
        if not self._infinite_loop:
            if full_key in self._state_seen:
                self._infinite_loop = True
                self._infinite_loop_pc = pc
                self._infinite_loop_key = full_key
                self._state_seen.clear()
                self._state_seen[full_key] = self._instr_count
                return "suppress"
            self._state_seen[full_key] = self._instr_count
            if len(self._state_seen) > 1024:
                cutoff = self._instr_count - 512
                self._state_seen = {
                    k: v for k, v in self._state_seen.items() if v >= cutoff
                }
        else:
            # In an infinite loop, check if state changed (interrupt, IO)
            if pc == self._infinite_loop_pc and full_key not in self._state_seen:
                self._infinite_loop = False
                self._state_seen.clear()
                return "loop_exit"
            self._state_seen[full_key] = self._instr_count
            if len(self._state_seen) > 16:
                self._state_seen.clear()
            return "suppress"

        # --- PC-sequence detection ---
        if self._in_loop:
            return self._check_in_pc_loop(pc)

        self._pc_history.append(pc)
        seq = self._find_repeating_sequence()
        if seq:
            self._in_loop = True
            self._loop_seq = seq
            self._loop_pos = 0
            self._loop_iterations = SUPPRESS_AFTER  # we already saw it repeat
            self._suppressing = True
            return "suppress"

        return "normal"

    def _check_in_pc_loop(self, pc):
        expected_pc = self._loop_seq[self._loop_pos]
        if pc == expected_pc:
            self._loop_pos += 1
            if self._loop_pos >= len(self._loop_seq):
                # Completed another iteration
                self._loop_pos = 0
                self._loop_iterations += 1
            return "suppress"
        else:
            # PC deviated from the loop pattern -- loop exited
            return self._exit_loop()

    def _exit_loop(self):
        self._in_loop = False
        self._suppressing = False
        self._pc_history.clear()
        return "loop_exit"

    def _find_repeating_sequence(self):
        """Look for a repeating PC sequence at the end of the history.

        Tries sequence lengths from 1 to max_len.  A sequence is confirmed
        if it repeats at least SUPPRESS_AFTER times consecutively.
        """
        hist = self._pc_history
        n = len(hist)
        max_len = min(64, n // SUPPRESS_AFTER)

        for seq_len in range(1, max_len + 1):
            needed = seq_len * SUPPRESS_AFTER
            if needed > n:
                break
            # Check if the last `needed` PCs consist of the same sequence repeated
            tail = list(hist)[-needed:]
            seq = tuple(tail[:seq_len])
            match = True
            for rep in range(1, SUPPRESS_AFTER):
                chunk = tuple(tail[rep * seq_len:(rep + 1) * seq_len])
                if chunk != seq:
                    match = False
                    break
            if match:
                return seq

        return None

    @property
    def in_loop(self):
        return self._in_loop or self._infinite_loop

    @property
    def loop_iterations(self):
        return self._loop_iterations

    def loop_summary(self):
        """Format a summary line for the loop that just ended."""
        if self._loop_iterations <= 0:
            return None
        if self._infinite_loop:
            return f"  [...infinite loop at ${self._infinite_loop_pc:04X}...]"
        pcs = sorted(set(self._loop_seq))
        if len(pcs) <= 4:
            pc_range = ", ".join(f"${p:04X}" for p in pcs)
        else:
            pc_range = f"${pcs[0]:04X}-${pcs[-1]:04X}"
        return (f"  [...loop at {pc_range}: "
                f"{self._loop_iterations} iterations...]")
