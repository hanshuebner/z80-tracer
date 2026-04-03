#!/usr/bin/env python3
"""
Quick test for the PSRAM capture protocol.

Usage:
    python -m client.capture_test /dev/ttyACM0
    python -m client.capture_test /dev/ttyACM0 --records 1000
    python -m client.capture_test /dev/ttyACM0 --diag   # run diagnostic modes

Starts capture, waits briefly, then reads back records and displays them.
"""

import argparse
import time

import serial

from . import capture


def run_diag_mode(ser, mode, name, duration):
    """Run one diagnostic mode and report results."""
    capture.set_diag_mode(ser, mode)
    capture.start_capture(ser)
    time.sleep(duration)

    status = capture.get_status(ser)
    capture.stop_capture(ser)

    samples_sec = status['total_samples'] / duration
    records_sec = status['write_idx'] / duration if status['write_idx'] else 0
    cycles_per_sample = 250e6 / samples_sec if samples_sec > 0 else float('inf')

    print(f"\n  [{name}]")
    print(f"    samples/sec:      {samples_sec:,.0f}  ({cycles_per_sample:.1f} cycles/sample at 250MHz)")
    print(f"    records/sec:      {records_sec:,.0f}")
    print(f"    dma_overflows:    {status['dma_overflows']}")
    print(f"    max_dma_distance: {status['max_dma_distance']}/{8192} "
          f"({status['max_dma_distance']*100/8192:.0f}%)")
    print(f"    max_stage_depth:  {status['max_stage_depth']}")

    # Reset for next test
    time.sleep(0.1)
    return status


def run_diagnostics(ser, duration):
    """Run all diagnostic modes to isolate the bottleneck."""
    print(f"=== Performance Diagnostics ({duration}s each) ===")
    print(f"    Expected: 8M samples/sec at 4MHz Z80 → 31.25 cycles/sample budget")

    # Mode 3: skip everything — pure DMA read loop baseline
    run_diag_mode(ser, capture.DIAG_SKIP_BOTH,
                  "Mode 3: Skip all (DMA read baseline)", duration)

    # Mode 1: skip analyzer, count samples — DMA read loop cost
    run_diag_mode(ser, capture.DIAG_SKIP_ANALYZER,
                  "Mode 1: Skip analyzer (DMA loop only)", duration)

    # Mode 2: run analyzer, no PSRAM — isolate state machine cost
    run_diag_mode(ser, capture.DIAG_SKIP_PSRAM,
                  "Mode 2: Analyzer only (no PSRAM)", duration)

    # Mode 0: full pipeline
    run_diag_mode(ser, 0,
                  "Mode 0: Full pipeline (analyzer + PSRAM)", duration)

    # Restore normal mode
    capture.set_diag_mode(ser, 0)


def main():
    parser = argparse.ArgumentParser(description="PSRAM capture protocol test")
    parser.add_argument("port", help="Serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--records", type=int, default=100,
                        help="Number of records to read (default: 100)")
    parser.add_argument("--capture-time", type=float, default=2.0,
                        help="Seconds to capture before reading (default: 2.0)")
    parser.add_argument("--diag", action="store_true",
                        help="Run diagnostic modes to isolate bottleneck")
    args = parser.parse_args()

    ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.5)
    ser.reset_input_buffer()
    time.sleep(0.1)

    if args.diag:
        run_diagnostics(ser, args.capture_time)
        ser.close()
        return

    print("Querying status...")
    status = capture.get_status(ser)
    print(f"  capture_active={status['capture_active']}, "
          f"write_idx={status['write_idx']}, "
          f"dma_overflows={status['dma_overflows']}")

    print(f"\nStarting capture for {args.capture_time}s...")
    capture.start_capture(ser)
    time.sleep(args.capture_time)

    status = capture.get_status(ser)
    print(f"  write_idx={status['write_idx']} "
          f"({status['write_idx'] / args.capture_time:.0f} records/sec)")
    print(f"  total_samples={status['total_samples']} "
          f"({status['total_samples'] / args.capture_time:.0f} samples/sec)")
    print(f"  dma_overflows={status['dma_overflows']}, "
          f"max_dma_distance={status['max_dma_distance']}/{8192} samples "
          f"({status['max_dma_distance']*100/8192:.0f}%)")
    print(f"  max_stage_depth={status['max_stage_depth']}/256 records")

    print(f"\nReading last {args.records} records...")
    records = capture.read_buffer(ser, pre_count=args.records, post_count=0)

    print(f"\n--- Trace ({len(records)} records) ---")
    for rec in records:
        print(capture.format_record(rec))

    print(f"\nStopping capture...")
    capture.stop_capture(ser)

    status = capture.get_status(ser)
    print(f"  capture_active={status['capture_active']}, "
          f"write_idx={status['write_idx']}")

    ser.close()
    print("Done.")


if __name__ == "__main__":
    main()
