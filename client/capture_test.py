#!/usr/bin/env python3
"""
Test for the PSRAM capture protocol with trigger support.

Usage:
    python -m client.capture_test /dev/ttyACM0
    python -m client.capture_test /dev/ttyACM0 --diag
    python -m client.capture_test /dev/ttyACM0 --trigger-pc 0x0100 --pre 500 --post 500
    python -m client.capture_test /dev/ttyACM0 --trigger-mem-read 0xC000
"""

import argparse
import time

import serial

from . import capture


def run_diag_mode(ser, mode, name, duration, cpu_mhz):
    """Run one diagnostic mode and report results."""
    capture.set_diag_mode(ser, mode)
    capture.start_capture(ser)
    time.sleep(duration)

    status = capture.get_status(ser)
    capture.stop_capture(ser)

    samples_sec = status['total_samples'] / duration
    records_sec = status['write_idx'] / duration if status['write_idx'] else 0
    cycles_per_sample = (cpu_mhz * 1e6) / samples_sec if samples_sec > 0 else float('inf')

    print(f"\n  [{name}]")
    print(f"    samples/sec:      {samples_sec:,.0f}  ({cycles_per_sample:.1f} cycles/sample at {cpu_mhz}MHz)")
    print(f"    records/sec:      {records_sec:,.0f}")
    print(f"    dma_overflows:    {status['dma_overflows']}")
    print(f"    max_dma_distance: {status['max_dma_distance']}/{8192} "
          f"({status['max_dma_distance']*100/8192:.0f}%)")
    print(f"    max_stage_depth:  {status['max_stage_depth']}")
    print(f"    wait_asserts:     {status['wait_asserts']}")

    time.sleep(0.1)
    return status


def run_diagnostics(ser, duration):
    """Run all diagnostic modes to isolate the bottleneck."""
    status = capture.get_status(ser)
    cpu_mhz = status['cpu_clock_khz'] / 1000
    budget = cpu_mhz * 1e6 / 8e6

    print(f"=== Performance Diagnostics ({duration}s each) ===")
    print(f"    CPU clock: {cpu_mhz:.0f} MHz")
    print(f"    Expected: 8M samples/sec at 4MHz Z80 → {budget:.1f} cycles/sample budget")

    run_diag_mode(ser, capture.DIAG_SKIP_BOTH,
                  "Mode 3: Skip all (DMA read baseline)", duration, cpu_mhz)
    run_diag_mode(ser, capture.DIAG_SKIP_ANALYZER,
                  "Mode 1: Skip analyzer (DMA loop only)", duration, cpu_mhz)
    run_diag_mode(ser, capture.DIAG_SKIP_PSRAM,
                  "Mode 2: Analyzer only (no PSRAM)", duration, cpu_mhz)
    run_diag_mode(ser, 0,
                  "Mode 0: Full pipeline (analyzer + PSRAM)", duration, cpu_mhz)

    capture.set_diag_mode(ser, 0)


def run_trigger_test(ser, args):
    """Configure triggers, arm, wait, read buffer."""
    capture.clear_triggers(ser)

    trigger_set = False
    if args.trigger_pc is not None:
        for addr in args.trigger_pc:
            capture.set_trigger(ser, capture.TRIG_PC_MATCH, addr)
            print(f"Trigger: PC match at 0x{addr:04X}")
            trigger_set = True
    if args.trigger_mem_read is not None:
        for addr in args.trigger_mem_read:
            capture.set_trigger(ser, capture.TRIG_MEM_READ, addr)
            print(f"Trigger: memory read at 0x{addr:04X}")
            trigger_set = True
    if args.trigger_mem_write is not None:
        for addr in args.trigger_mem_write:
            capture.set_trigger(ser, capture.TRIG_MEM_WRITE, addr)
            print(f"Trigger: memory write at 0x{addr:04X}")
            trigger_set = True

    if not trigger_set:
        print("No triggers configured, capturing snapshot...")
        capture.start_capture(ser)
        time.sleep(args.capture_time)
        records, trig_off = capture.read_buffer(ser, pre_count=args.pre, post_count=0)
        capture.stop_capture(ser)
    else:
        print(f"Arming trigger (post_count={args.post})...")
        capture.arm_trigger(ser, args.post)

        # Poll for trigger
        print("Waiting for trigger...", end='', flush=True)
        timeout = time.monotonic() + args.timeout
        while True:
            time.sleep(0.1)
            status = capture.get_status(ser)
            if status['capture_state'] == capture.CAP_DONE:
                print(f" triggered! (write_idx={status['write_idx']})")
                break
            if status['capture_state'] == capture.CAP_TRIGGERED:
                print(".", end='', flush=True)
                continue
            if time.monotonic() > timeout:
                print(f" timeout ({args.timeout}s)")
                break

        records, trig_off = capture.read_buffer(
            ser, pre_count=args.pre, post_count=args.post)

    # Display records
    print(f"\n--- Trace ({len(records)} records) ---")
    for i, rec in enumerate(records):
        marker = " <<<TRIGGER" if i == trig_off else ""
        print(f"{capture.format_record(rec)}{marker}")

    capture.stop_capture(ser)


def main():
    parser = argparse.ArgumentParser(description="PSRAM capture test with triggers")
    parser.add_argument("port", help="Serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--capture-time", type=float, default=2.0,
                        help="Capture duration for snapshot/diag (default: 2.0)")
    parser.add_argument("--diag", action="store_true",
                        help="Run diagnostic modes")

    # Trigger options
    parser.add_argument("--trigger-pc", type=lambda x: int(x, 0),
                        action="append", help="Trigger on PC address (hex)")
    parser.add_argument("--trigger-mem-read", type=lambda x: int(x, 0),
                        action="append", help="Trigger on memory read address (hex)")
    parser.add_argument("--trigger-mem-write", type=lambda x: int(x, 0),
                        action="append", help="Trigger on memory write address (hex)")
    parser.add_argument("--pre", type=int, default=100,
                        help="Records before trigger/snapshot (default: 100)")
    parser.add_argument("--post", type=int, default=100,
                        help="Records after trigger (default: 100)")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="Trigger wait timeout in seconds (default: 30)")
    args = parser.parse_args()

    ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.5)
    ser.reset_input_buffer()
    time.sleep(0.1)

    if args.diag:
        run_diagnostics(ser, args.capture_time)
    else:
        status = capture.get_status(ser)
        print(f"State: {status['capture_state_name']}, "
              f"triggers: {status['trigger_count']}, "
              f"CPU: {status['cpu_clock_khz']/1000:.0f} MHz")
        run_trigger_test(ser, args)

    ser.close()
    print("Done.")


if __name__ == "__main__":
    main()
