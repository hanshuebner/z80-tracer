"""
PSRAM capture protocol client for the Z80 tracer.

Communicates with the new firmware that stores trace records in an 8 MB
PSRAM ring buffer.  No real-time streaming — the client requests a
snapshot of the buffer and decodes it offline.
"""

import struct
import time

import serial

# Command protocol constants (must match z80_trace.h)
CMD_SYNC           = 0xFF
CMD_START_CAPTURE  = 0x01
CMD_STOP_CAPTURE   = 0x02
CMD_READ_BUFFER    = 0x03
CMD_GET_STATUS     = 0x04
CMD_SET_DIAG_MODE  = 0x05

# Diagnostic mode bits
DIAG_SKIP_ANALYZER = 0x01  # skip process_sample, just count
DIAG_SKIP_PSRAM    = 0x02  # run analyzer but discard records (no PSRAM writes)
DIAG_SKIP_BOTH     = 0x03  # skip both

READOUT_MAGIC      = 0x5A383054  # "Z80T"
STATUS_MAGIC       = 0x53544154  # "STAT"

# trace_record_t layout: 8 bytes
#   uint16_t address
#   uint8_t  data
#   uint8_t  cycle_type
#   uint8_t  refresh
#   uint8_t  wait_count
#   uint8_t  flags
#   uint8_t  seq
RECORD_FMT = '<HBBBBBB'
RECORD_SIZE = struct.calcsize(RECORD_FMT)  # 8

# Cycle type names (matches cycle_type_t enum)
CYCLE_NAMES = {
    0: 'FETCH',
    1: 'MREAD',
    2: 'MWRIT',
    3: 'IOREAD',
    4: 'IOWRIT',
    5: 'INTACK',
    6: 'RESET',
}


def send_cmd(ser, cmd, payload=b''):
    """Send a command to the firmware."""
    ser.write(bytes([CMD_SYNC, cmd]) + payload)
    ser.flush()


def read_exact(ser, n, timeout=10.0):
    """Read exactly n bytes from serial, with timeout."""
    data = b''
    deadline = time.monotonic() + timeout
    while len(data) < n:
        remaining = n - len(data)
        chunk = ser.read(remaining)
        if chunk:
            data += chunk
        elif time.monotonic() > deadline:
            raise TimeoutError(
                f"Timeout reading from serial (got {len(data)}/{n} bytes)")
    return data


def get_status(ser):
    """Query firmware status. Returns dict with capture state and diagnostics."""
    send_cmd(ser, CMD_GET_STATUS)
    raw = read_exact(ser, 32)
    (magic, capture_active, write_idx, dma_overflows,
     max_dma_distance, max_stage_depth, total_samples,
     cpu_clock_khz) = struct.unpack('<IIIIIIII', raw)
    if magic != STATUS_MAGIC:
        raise ValueError(f"Bad status magic: 0x{magic:08X} (expected 0x{STATUS_MAGIC:08X})")
    return {
        'capture_active': bool(capture_active),
        'write_idx': write_idx,
        'dma_overflows': dma_overflows,
        'max_dma_distance': max_dma_distance,
        'max_stage_depth': max_stage_depth,
        'total_samples': total_samples,
        'cpu_clock_khz': cpu_clock_khz,
    }


def start_capture(ser):
    """Start PIO/DMA capture."""
    send_cmd(ser, CMD_START_CAPTURE)


def stop_capture(ser):
    """Stop PIO/DMA capture."""
    send_cmd(ser, CMD_STOP_CAPTURE)


def set_diag_mode(ser, mode):
    """Set diagnostic mode bitmask.

    0 = normal (full pipeline)
    1 = DIAG_SKIP_ANALYZER: skip state machine, just count samples (max throughput baseline)
    2 = DIAG_SKIP_PSRAM: run analyzer but discard records (isolate analyzer cost)
    3 = DIAG_SKIP_BOTH: skip everything (pure DMA read baseline)
    """
    send_cmd(ser, CMD_SET_DIAG_MODE, bytes([mode]))


def read_buffer(ser, pre_count, post_count=0, progress=True):
    """
    Read trace records from the PSRAM ring buffer.

    Args:
        ser: serial.Serial instance
        pre_count: number of records before the reference point
        post_count: number of records after (0 for snapshot mode)
        progress: print transfer progress

    Returns:
        list of tuples: (address, data, cycle_type, refresh, wait_count, flags, seq)
    """
    payload = struct.pack('<II', pre_count, post_count)
    send_cmd(ser, CMD_READ_BUFFER, payload)

    # Read header
    hdr_raw = read_exact(ser, 16, timeout=5.0)
    magic, total_records, write_idx, flags = struct.unpack('<IIII', hdr_raw)
    if magic != READOUT_MAGIC:
        raise ValueError(f"Bad readout magic: 0x{magic:08X} (expected 0x{READOUT_MAGIC:08X})")

    if progress:
        total_bytes = total_records * RECORD_SIZE
        print(f"Receiving {total_records:,} records ({total_bytes:,} bytes)...")

    # Read all records
    total_bytes = total_records * RECORD_SIZE
    raw = read_exact(ser, total_bytes, timeout=max(30.0, total_bytes / 50000))

    records = []
    for i in range(total_records):
        offset = i * RECORD_SIZE
        rec = struct.unpack(RECORD_FMT, raw[offset:offset + RECORD_SIZE])
        records.append(rec)

    if progress:
        print(f"Received {len(records):,} records (write_idx={write_idx})")

    return records


def format_record(rec):
    """Format a single trace record for display."""
    address, data, cycle_type, refresh, wait_count, flags, seq = rec
    name = CYCLE_NAMES.get(cycle_type, f'?{cycle_type}')
    parts = [f"  {name:6s} {address:04X} {data:02X}"]
    if cycle_type == 0:  # M1 fetch
        parts.append(f" R={refresh:02X}")
    if wait_count > 0:
        parts.append(f" W={wait_count}")
    flag_strs = []
    if flags & 1: flag_strs.append('HALT')
    if flags & 2: flag_strs.append('INT')
    if flags & 4: flag_strs.append('NMI')
    if flag_strs:
        parts.append(f" [{','.join(flag_strs)}]")
    return ''.join(parts)
