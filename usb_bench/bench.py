#!/usr/bin/env python3
"""
USB CDC bandwidth benchmark — host side.

Usage:
    python bench.py /dev/ttyACM0            # default 10s
    python bench.py /dev/ttyACM0 --duration 30
    python bench.py /dev/ttyACM0 --verify   # check data integrity (slower)

Sends 'S' to start the firmware streaming, reads for --duration seconds,
then sends 'X' to stop.  Reports throughput in KB/s and Mbit/s.
"""

import argparse
import sys
import time

import serial


def run_bench(port: str, duration: float, verify: bool) -> None:
    # Use the largest buffer we can; 1 MB read buffer, no timeout on reads
    ser = serial.Serial(port, baudrate=115200, timeout=0.1)

    # Flush any stale data
    ser.reset_input_buffer()

    print(f"Starting benchmark on {port} for {duration:.0f}s ...")
    if verify:
        print("  (data integrity verification enabled)")

    # Start streaming
    ser.write(b"S")
    ser.flush()

    total_bytes = 0
    errors = 0
    expected = 0  # next expected byte value (0-255)
    t_start = time.monotonic()
    deadline = t_start + duration

    # Report interval
    last_report = t_start
    interval_bytes = 0

    try:
        while True:
            now = time.monotonic()
            if now >= deadline:
                break

            data = ser.read(65536)
            if not data:
                continue

            n = len(data)
            total_bytes += n
            interval_bytes += n

            if verify:
                for b in data:
                    if b != expected:
                        errors += 1
                    expected = (b + 1) & 0xFF

            # Print progress every second
            if now - last_report >= 1.0:
                elapsed = now - last_report
                rate_kbs = (interval_bytes / 1024) / elapsed
                rate_mbits = (interval_bytes * 8 / 1_000_000) / elapsed
                print(f"  {rate_kbs:8.1f} KB/s  ({rate_mbits:.2f} Mbit/s)")
                last_report = now
                interval_bytes = 0

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        # Stop streaming
        ser.write(b"X")
        ser.flush()
        ser.close()

    elapsed = time.monotonic() - t_start
    if elapsed > 0:
        avg_kbs = (total_bytes / 1024) / elapsed
        avg_mbits = (total_bytes * 8 / 1_000_000) / elapsed
        print(f"\n--- Results ---")
        print(f"Duration:   {elapsed:.2f}s")
        print(f"Received:   {total_bytes:,} bytes ({total_bytes / (1024*1024):.2f} MB)")
        print(f"Throughput: {avg_kbs:.1f} KB/s  ({avg_mbits:.2f} Mbit/s)")
        if verify:
            print(f"Errors:     {errors:,} byte mismatches")


def main():
    parser = argparse.ArgumentParser(description="USB CDC bandwidth benchmark")
    parser.add_argument("port", help="Serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("--duration", type=float, default=10,
                        help="Test duration in seconds (default: 10)")
    parser.add_argument("--verify", action="store_true",
                        help="Verify data integrity (may reduce throughput due to Python overhead)")
    args = parser.parse_args()

    run_bench(args.port, args.duration, args.verify)


if __name__ == "__main__":
    main()
