"""Memory access tracing for Z80 bus traces.

Tracks read/write access counts for every address in the 64K address space.
Produces summary reports in various formats after tracing completes.
"""

import sys


class MemoryTracer:
    """Tracks memory access counts across the full 64K address space."""

    MAX_COUNT = 0xFFFFFFFF  # 32-bit cap

    def __init__(self):
        self.reads = bytearray(65536 * 4)   # 32-bit counters, little-endian
        self.writes = bytearray(65536 * 4)

    def _inc(self, counters, addr):
        off = addr * 4
        val = int.from_bytes(counters[off:off+4], 'little')
        if val < self.MAX_COUNT:
            val += 1
            counters[off:off+4] = val.to_bytes(4, 'little')

    def record_read(self, addr):
        self._inc(self.reads, addr)

    def record_write(self, addr):
        self._inc(self.writes, addr)

    def get_read_count(self, addr):
        off = addr * 4
        return int.from_bytes(self.reads[off:off+4], 'little')

    def get_write_count(self, addr):
        off = addr * 4
        return int.from_bytes(self.writes[off:off+4], 'little')

    def report(self, fmt="text", file=None):
        if file is None:
            file = sys.stderr
        if fmt == "text":
            self._report_text(file)
        else:
            raise ValueError(f"Unknown memory report format: {fmt}")

    def _report_text(self, file):
        """ASCII map: 16 rows x 64 columns.

        Each cell covers 64 bytes (16 rows * 64 cols * 64 = 64K).
        R = any read, W = any write, X = both, - = no access.
        """
        bytes_per_cell = 64
        cols = 64
        rows = 16

        print("\nMemory access map (R=read W=write X=both -=none, "
              "each char=64 bytes):", file=file)
        # Column offset header
        hdr = "      "
        for c in range(0, cols, 16):
            offset = c * bytes_per_cell
            hdr += f"+{offset:03X}" + " " * 12
        print(hdr.rstrip(), file=file)

        for row in range(rows):
            base = row * cols * bytes_per_cell
            chars = []
            for col in range(cols):
                cell_base = base + col * bytes_per_cell
                has_read = False
                has_write = False
                for offset in range(bytes_per_cell):
                    addr = cell_base + offset
                    if self.get_read_count(addr) > 0:
                        has_read = True
                    if self.get_write_count(addr) > 0:
                        has_write = True
                    if has_read and has_write:
                        break
                if has_read and has_write:
                    chars.append("X")
                elif has_read:
                    chars.append("R")
                elif has_write:
                    chars.append("W")
                else:
                    chars.append("-")
            print(f"{base:04X}: {''.join(chars)}", file=file)

        print(file=file)
