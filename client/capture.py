"""
PSRAM capture protocol client for the Z80 tracer.

Communicates with the firmware that stores trace records in an 8 MB
PSRAM ring buffer with on-device trigger support.
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
CMD_SET_TRIGGER    = 0x10
CMD_CLEAR_TRIGGERS = 0x11
CMD_ARM_TRIGGER    = 0x12

# Trigger types (must match trigger_type_t)
TRIG_NONE      = 0
TRIG_PC_MATCH  = 1
TRIG_MEM_READ  = 2
TRIG_MEM_WRITE = 3
TRIG_IO_READ   = 4
TRIG_IO_WRITE  = 5
TRIG_INT_ACK   = 6

# Capture states (must match capture_state_t)
CAP_IDLE      = 0
CAP_RUNNING   = 1
CAP_ARMED     = 2
CAP_TRIGGERED = 3
CAP_DONE      = 4

CAP_STATE_NAMES = {
    CAP_IDLE: 'IDLE',
    CAP_RUNNING: 'RUNNING',
    CAP_ARMED: 'ARMED',
    CAP_TRIGGERED: 'TRIGGERED',
    CAP_DONE: 'DONE',
}

# Diagnostic mode bits
DIAG_SKIP_ANALYZER = 0x01
DIAG_SKIP_PSRAM    = 0x02
DIAG_SKIP_BOTH     = 0x03

READOUT_MAGIC      = 0x5A383054  # "Z80T"
STATUS_MAGIC       = 0x53544154  # "STAT"

# trace_record_t layout: 7 bytes
# address(u16), data(u8), cycle_type(u8), wait_count(u8), flags(u8), seq(u8)
RECORD_FMT = '<HBBBBB'
RECORD_SIZE = struct.calcsize(RECORD_FMT)  # 7

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
    raw = read_exact(ser, 44)
    (magic, capture_state, write_idx, dma_overflows,
     max_dma_distance, max_stage_depth, total_samples,
     cpu_clock_khz, trigger_idx, trigger_count,
     wait_asserts) = struct.unpack('<IIIIIIIIIII', raw)
    if magic != STATUS_MAGIC:
        raise ValueError(f"Bad status magic: 0x{magic:08X} (expected 0x{STATUS_MAGIC:08X})")
    return {
        'capture_state': capture_state,
        'capture_state_name': CAP_STATE_NAMES.get(capture_state, f'?{capture_state}'),
        'capture_active': capture_state != CAP_IDLE,
        'write_idx': write_idx,
        'dma_overflows': dma_overflows,
        'max_dma_distance': max_dma_distance,
        'max_stage_depth': max_stage_depth,
        'total_samples': total_samples,
        'cpu_clock_khz': cpu_clock_khz,
        'trigger_idx': trigger_idx,
        'trigger_count': trigger_count,
        'wait_asserts': wait_asserts,
    }


def start_capture(ser):
    """Start PIO/DMA capture (CAP_RUNNING)."""
    send_cmd(ser, CMD_START_CAPTURE)


def stop_capture(ser):
    """Stop PIO/DMA capture (CAP_IDLE)."""
    send_cmd(ser, CMD_STOP_CAPTURE)


def set_trigger(ser, trig_type, addr_lo, addr_hi=None, data_val=0, data_mask=0):
    """Add a trigger. addr_hi defaults to addr_lo for exact match."""
    if addr_hi is None:
        addr_hi = addr_lo
    payload = struct.pack('<BHHBB', trig_type, addr_lo, addr_hi, data_val, data_mask)
    send_cmd(ser, CMD_SET_TRIGGER, payload)


def clear_triggers(ser):
    """Clear all configured triggers."""
    send_cmd(ser, CMD_CLEAR_TRIGGERS)


def arm_trigger(ser, post_trigger_count):
    """Arm triggers with given post-trigger record count. Starts capture if idle."""
    payload = struct.pack('<I', post_trigger_count)
    send_cmd(ser, CMD_ARM_TRIGGER, payload)


def set_diag_mode(ser, mode):
    """Set diagnostic mode bitmask."""
    send_cmd(ser, CMD_SET_DIAG_MODE, bytes([mode]))


def read_buffer(ser, pre_count, post_count=0, progress=True):
    """
    Read trace records from the PSRAM ring buffer.

    Returns:
        (records, trigger_offset) where trigger_offset is the index of the
        trigger record within the list, or None if no trigger.
    """
    payload = struct.pack('<II', pre_count, post_count)
    send_cmd(ser, CMD_READ_BUFFER, payload)

    # Read header
    hdr_raw = read_exact(ser, 16, timeout=5.0)
    magic, total_records, write_idx, trigger_offset = struct.unpack('<IIII', hdr_raw)
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

    trig_off = trigger_offset if trigger_offset != 0xFFFFFFFF else None

    if progress:
        trig_str = f", trigger at #{trig_off}" if trig_off is not None else ""
        print(f"Received {len(records):,} records{trig_str}")

    return records, trig_off


def format_record(rec):
    """Format a single trace record for display."""
    address, data, cycle_type, wait_count, flags, seq = rec
    name = CYCLE_NAMES.get(cycle_type, f'?{cycle_type}')
    parts = [f"  {name:6s} {address:04X} {data:02X}"]
    if wait_count > 0:
        parts.append(f" W={wait_count}")
    flag_strs = []
    if flags & 1: flag_strs.append('HALT')
    if flags & 2: flag_strs.append('INT')
    if flags & 4: flag_strs.append('NMI')
    if flag_strs:
        parts.append(f" [{','.join(flag_strs)}]")
    return ''.join(parts)
