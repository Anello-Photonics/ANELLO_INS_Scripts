#!/usr/bin/env python3
"""
Raspberry Pi GPIO bundle tester for 4-wire harness:

Order (updated):
  0) Ensure RESET is released immediately at startup (so we don't hold device in reset)
  1) PPS test (edge capture, validate ~1 Hz)
  2) SYNC test (drive low/high; operator confirms via MAVLink shell I0)
  3) BIT/RESET test (confirm boot + reset behavior)

Notes:
- RESET is assumed active-low (touching RESET to GND resets device).
- We "release" RESET by leaving the pin in INPUT mode, optionally floating.
  floating=True means Hi-Z with no pull (preferred if target has its own pull-up).
- BCM numbering is used.

Run with sudo:
  sudo python3 bundle_tester.py --pps 17 --bit 27 --reset 22 --sync 23
"""

import argparse
import csv
import json
import sys
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from statistics import mean, stdev

import RPi.GPIO as GPIO


@dataclass
class TestResult:
    name: str
    passed: bool
    details: dict


def now_epoch_s() -> float:
    return time.time()


def sleep_s(sec: float) -> None:
    time.sleep(sec)


def setup_gpio() -> None:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)


def wait_for_level(pin: int, level: int, timeout_s: float, poll_s: float = 0.01) -> bool:
    """Poll input pin until it equals level or timeout."""
    t0 = time.monotonic()
    while time.monotonic() - t0 <= timeout_s:
        if GPIO.input(pin) == level:
            return True
        sleep_s(poll_s)
    return False


def read_level_stable(pin: int, duration_s: float = 0.1, poll_s: float = 0.005) -> int:
    """Return the majority level observed over duration_s (simple debouncing)."""
    t0 = time.monotonic()
    samples = []
    while time.monotonic() - t0 < duration_s:
        samples.append(GPIO.input(pin))
        sleep_s(poll_s)
    return 1 if sum(samples) >= (len(samples) / 2) else 0


def reset_release(reset_pin: int, floating: bool = True) -> None:
    """
    Release RESET so device can run.
    - floating=True: Hi-Z input, no pulls (preferred if target has its own pull-up)
    - floating=False: Hi-Z input with Pi pull-up (use only if target has weak/no pull-up)
    """
    if floating:
        GPIO.setup(reset_pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    else:
        GPIO.setup(reset_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def reset_pulse_open_drain(reset_pin: int, assert_ms: int, release_floating: bool = True) -> None:
    """Assert reset low briefly, then release (Hi-Z)."""
    reset_release(reset_pin, floating=release_floating)
    time.sleep(0.02)

    GPIO.setup(reset_pin, GPIO.OUT)
    GPIO.output(reset_pin, GPIO.LOW)
    try:
        time.sleep(assert_ms / 1000.0)
    finally:
        reset_release(reset_pin, floating=release_floating)
        time.sleep(0.02)


def test_pps(
    pps_pin: int,
    window_s: float,
    min_pulses: int,
    expected_hz: float,
    max_jitter_ms: float,
) -> TestResult:
    """
    Detect PPS rising edges and compute interval stats.
    Uses monotonic timestamps for interval measurement.
    Includes simple glitch rejection by requiring >=0.80s between accepted edges.
    """
    GPIO.setup(pps_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    edge_times = []
    last_accept = None
    MIN_DT = 0.80  # seconds; reject edges closer than this (tune 0.70–0.95)

    def cb(_channel):
        nonlocal last_accept
        t = time.monotonic()
        if last_accept is not None and (t - last_accept) < MIN_DT:
            return
        edge_times.append(t)
        last_accept = t

    GPIO.add_event_detect(pps_pin, GPIO.RISING, callback=cb)

    t0 = time.monotonic()
    sleep_s(window_s)
    GPIO.remove_event_detect(pps_pin)

    n = len(edge_times)
    details = {"edges_captured": n, "window_s": window_s}

    if n < 2:
        details["reason"] = "Not enough edges to measure intervals."
        return TestResult("PPS", False, details)

    intervals = [edge_times[i] - edge_times[i - 1] for i in range(1, n)]
    mean_interval = mean(intervals)
    hz = 1.0 / mean_interval if mean_interval > 0 else 0.0

    interval_stdev = stdev(intervals) if len(intervals) >= 2 else 0.0
    jitter_ms = interval_stdev * 1000.0

    details.update(
        {
            "intervals_count": len(intervals),
            "mean_interval_s": mean_interval,
            "measured_hz": hz,
            "jitter_ms_stdev": jitter_ms,
            "first_edge_dt_s": edge_times[0] - t0,
            "last_edge_dt_s": edge_times[-1] - t0,
        }
    )

    passed_pulses = n >= min_pulses
    passed_freq = (expected_hz * 0.8) <= hz <= (expected_hz * 1.2)
    passed_jitter = jitter_ms <= max_jitter_ms

    passed = passed_pulses and passed_freq and passed_jitter
    if not passed:
        reasons = []
        if not passed_pulses:
            reasons.append(f"Too few pulses (got {n}, need >= {min_pulses}).")
        if not passed_freq:
            reasons.append(f"Frequency out of range (measured {hz:.3f} Hz).")
        if not passed_jitter:
            reasons.append(f"Jitter too high ({jitter_ms:.2f} ms > {max_jitter_ms:.2f} ms).")
        details["reason"] = " ".join(reasons)

    return TestResult("PPS", passed, details)


def test_sync(sync_pin: int, low_s: float, high_s: float) -> TestResult:
    """Drive SYNC low then high. Operator verifies in MAVLink shell by reading I0."""
    GPIO.setup(sync_pin, GPIO.OUT, initial=GPIO.HIGH)

    details = {"sync_low_s": low_s, "sync_high_s": high_s, "operator_confirmed": None}

    print("\nSYNC TEST")
    print("  This will drive SYNC LOW, then HIGH.")
    print("  In your MAVLink shell, read I0 (port I, pin 0) and confirm it matches.\n")

    print(f"Driving SYNC LOW for {low_s:.2f}s...")
    GPIO.output(sync_pin, GPIO.LOW)
    sleep_s(low_s)

    print(f"Driving SYNC HIGH for {high_s:.2f}s...")
    GPIO.output(sync_pin, GPIO.HIGH)
    sleep_s(high_s)

    try:
        ans = input("Did the MAVLink shell show I0 LOW then HIGH accordingly? [y/N]: ").strip().lower()
        details["operator_confirmed"] = ans in ("y", "yes")
    except KeyboardInterrupt:
        details["operator_confirmed"] = False

    passed = bool(details["operator_confirmed"])
    if not passed:
        details["reason"] = "Operator did not confirm SYNC state change on MAVLink shell."

    return TestResult("SYNC", passed, details)


def test_bit_and_reset(
    bit_pin: int,
    reset_pin: int,
    bit_rise_timeout_s: float,
    assert_ms: int,
    bit_drop_timeout_s: float,
    bit_return_timeout_s: float,
) -> TestResult:
    """
    Test sequence:
      1) Wait for BIT to go HIGH after power-up.
      2) Pulse RESET low briefly.
      3) Verify BIT drops LOW shortly after reset.
      4) Verify BIT returns HIGH within specified timeout.
      5) Leave RESET released at the end.
    """
    GPIO.setup(bit_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    reset_release(reset_pin, floating=True)  # keep device out of reset by default

    details = {}

    got_high = wait_for_level(bit_pin, 1, bit_rise_timeout_s)
    details["bit_high_within_s"] = bit_rise_timeout_s
    details["bit_initial_high"] = bool(got_high)

    if not got_high:
        details["bit_level_after_timeout"] = read_level_stable(bit_pin)
        details["reason"] = "BIT did not go high after power-up."
        reset_release(reset_pin, floating=True)
        return TestResult("BIT_RESET", False, details)

    details["reset_assert_ms"] = assert_ms
    details["reset_time_epoch_s"] = now_epoch_s()
    reset_pulse_open_drain(reset_pin, assert_ms, release_floating=True)

    got_low = wait_for_level(bit_pin, 0, bit_drop_timeout_s)
    details["bit_low_within_s_after_reset"] = bit_drop_timeout_s
    details["bit_dropped_low"] = bool(got_low)

    if not got_low:
        details["bit_level_after_drop_timeout"] = read_level_stable(bit_pin)
        details["reason"] = "BIT did not drop low after reset."
        reset_release(reset_pin, floating=True)
        return TestResult("BIT_RESET", False, details)

    got_high_again = wait_for_level(bit_pin, 1, bit_return_timeout_s)
    details["bit_high_again_within_s"] = bit_return_timeout_s
    details["bit_returned_high"] = bool(got_high_again)

    if not got_high_again:
        details["bit_level_after_return_timeout"] = read_level_stable(bit_pin)
        details["reason"] = "BIT did not return high after reset."
        reset_release(reset_pin, floating=True)
        return TestResult("BIT_RESET", False, details)

    reset_release(reset_pin, floating=True)
    details["reset_left_released"] = True
    return TestResult("BIT_RESET", True, details)


def write_results(results, out_json: Path | None, out_csv: Path | None) -> None:
    payload = {
        "timestamp_epoch_s": now_epoch_s(),
        "results": [asdict(r) for r in results],
        "overall_pass": all(r.passed for r in results),
    }

    if out_json:
        out_json.write_text(json.dumps(payload, indent=2))
    if out_csv:
        with out_csv.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["name", "passed", "details_json"])
            for r in results:
                w.writerow([r.name, r.passed, json.dumps(r.details)])


def main() -> int:
    ap = argparse.ArgumentParser(description="GPIO tester for PPS/BIT/RESET/SYNC bundle (Raspberry Pi 4B).")
    ap.add_argument("--pps", type=int, default=17, help="BCM pin for PPS input")
    ap.add_argument("--bit", type=int, default=27, help="BCM pin for BIT_IN input")
    ap.add_argument("--reset", type=int, default=22, help="BCM pin for RESET output (active-low)")
    ap.add_argument("--sync", type=int, default=23, help="BCM pin for SYNC output")

    ap.add_argument("--pps-window", type=float, default=30.0, help="Seconds to observe PPS edges")
    ap.add_argument("--pps-min-pulses", type=int, default=10, help="Minimum PPS edges required to pass")
    ap.add_argument("--pps-hz", type=float, default=1.0, help="Expected PPS frequency (Hz)")
    ap.add_argument("--pps-max-jitter-ms", type=float, default=5.0, help="Max interval stdev (ms)")

    ap.add_argument("--sync-low-s", type=float, default=20.0, help="Seconds to hold SYNC low")
    ap.add_argument("--sync-high-s", type=float, default=20.0, help="Seconds to hold SYNC high")

    ap.add_argument("--bit-rise-timeout", type=float, default=15.0, help="Seconds to wait for BIT high after power-up")
    ap.add_argument("--reset-assert-ms", type=int, default=50, help="RESET assert duration in ms (active-low)")
    ap.add_argument("--bit-drop-timeout", type=float, default=3.0, help="Seconds to wait for BIT low after reset")
    ap.add_argument("--bit-return-timeout", type=float, default=15.0, help="Seconds to wait for BIT high after reset")

    ap.add_argument("--out-json", type=str, default="", help="Write results JSON to this path")
    ap.add_argument("--out-csv", type=str, default="", help="Write results CSV to this path")

    args = ap.parse_args()

    out_json = Path(args.out_json) if args.out_json else None
    out_csv = Path(args.out_csv) if args.out_csv else None

    setup_gpio()

    # IMPORTANT: release reset immediately at startup
    reset_release(args.reset, floating=True)

    results = []
    try:
        print("\n=== BUNDLE TESTER ===")
        print("Wiring assumptions:")
        print("  - Common ground between Pi and unit")
        print("  - GPIO is 3.3V logic")
        print("Order: PPS -> SYNC -> BIT/RESET\n")
        print("RESET is released at startup (Hi-Z, floating).\n")

        print("Step 0: Ensure antennas have satellite time fix (for PPS).")
        input("Press Enter when ready to start PPS capture...")

        results.append(
            test_pps(
                pps_pin=args.pps,
                window_s=args.pps_window,
                min_pulses=args.pps_min_pulses,
                expected_hz=args.pps_hz,
                max_jitter_ms=args.pps_max_jitter_ms,
            )
        )
        print(f"PPS: {'PASS' if results[-1].passed else 'FAIL'}")
        print(json.dumps(results[-1].details, indent=2))

        input("\nPress Enter to run SYNC test (will toggle SYNC low/high)...")
        results.append(test_sync(args.sync, low_s=args.sync_low_s, high_s=args.sync_high_s))
        print(f"SYNC: {'PASS' if results[-1].passed else 'FAIL'}")
        print(json.dumps(results[-1].details, indent=2))

        print("\nStep 2: Ensure the unit is powered ON (bench supply current sensing optional).")
        input("Press Enter to start BIT/RESET test (will pulse RESET)...")

        results.append(
            test_bit_and_reset(
                bit_pin=args.bit,
                reset_pin=args.reset,
                bit_rise_timeout_s=args.bit_rise_timeout,
                assert_ms=args.reset_assert_ms,
                bit_drop_timeout_s=args.bit_drop_timeout,
                bit_return_timeout_s=args.bit_return_timeout,
            )
        )
        print(f"BIT/RESET: {'PASS' if results[-1].passed else 'FAIL'}")
        print(json.dumps(results[-1].details, indent=2))

        overall = all(r.passed for r in results)
        print("\n=== OVERALL RESULT:", "PASS" if overall else "FAIL", "===\n")

        write_results(results, out_json=out_json, out_csv=out_csv)
        return 0 if overall else 2

    except KeyboardInterrupt:
        print("\nAborted by user.")
        return 130
    finally:
        # Leave reset released after script ends; cleanup everything else.
        try:
            reset_release(args.reset, floating=True)
            GPIO.cleanup([args.pps, args.bit, args.sync])
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
