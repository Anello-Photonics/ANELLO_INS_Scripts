#!/usr/bin/env python3
"""
Raspberry Pi GPIO bundle tester for 4-wire harness (RESET via NPN open-collector).

RESET hardware (verified working with debug script):
- Pi GPIO (RESET_CTRL) -> 1k -> 2N3904 base
- 2N3904 emitter -> GND (Pi GND tied to device GND)
- 2N3904 collector -> 1k series -> device RESET wire
- optional 120k base->emitter pulldown (ok)

RESET logic:
- GPIO LOW  = transistor OFF = RESET released (device runs)
- GPIO HIGH = transistor ON  = RESET asserted (reset pulled low)

ORDER:
  1) BIT/RESET
  2) Leave RESET released (GPIO LOW)
  3) SYNC
  4) PPS

Run:
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


def sleep_s(sec: float) -> None:
    time.sleep(sec)


def now_epoch_s() -> float:
    return time.time()


def setup_gpio() -> None:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)


def wait_for_level(pin: int, level: int, timeout_s: float, poll_s: float = 0.01) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 <= timeout_s:
        if GPIO.input(pin) == level:
            return True
        sleep_s(poll_s)
    return False


def read_level_stable(pin: int, duration_s: float = 0.2, poll_s: float = 0.005) -> int:
    t0 = time.monotonic()
    samples = []
    while time.monotonic() - t0 < duration_s:
        samples.append(GPIO.input(pin))
        sleep_s(poll_s)
    return 1 if sum(samples) >= (len(samples) / 2) else 0


# ---------------- RESET (KNOWN-GOOD STYLE) ----------------

def drive(pin: int, level: bool) -> None:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.HIGH if level else GPIO.LOW)


def reset_release(reset_ctrl_pin: int) -> None:
    # GPIO LOW -> transistor OFF -> reset released
    drive(reset_ctrl_pin, False)


def reset_assert(reset_ctrl_pin: int) -> None:
    # GPIO HIGH -> transistor ON -> reset asserted (line low)
    drive(reset_ctrl_pin, True)


def reset_pulse(reset_ctrl_pin: int, assert_ms: int, debug_pause: bool = False) -> dict:
    """
    Assert reset for assert_ms then release.
    Returns GPIO readback states for debug.
    """
    info = {}

    # Ensure released first
    reset_release(reset_ctrl_pin)
    sleep_s(0.05)
    info["gpio_after_release"] = int(GPIO.input(reset_ctrl_pin))

    # Assert
    reset_assert(reset_ctrl_pin)
    sleep_s(0.05)
    info["gpio_after_assert"] = int(GPIO.input(reset_ctrl_pin))

    if debug_pause:
        input("[RESET-DEBUG] Reset ASSERTED now. Press Enter to continue...")

    sleep_s(assert_ms / 1000.0)

    # Release
    reset_release(reset_ctrl_pin)
    sleep_s(0.05)
    info["gpio_after_final_release"] = int(GPIO.input(reset_ctrl_pin))

    if debug_pause:
        input("[RESET-DEBUG] Reset RELEASED now. Press Enter to continue...")

    return info


# ---------------- TESTS ----------------

def test_bit_and_reset(
    bit_pin: int,
    reset_ctrl_pin: int,
    bit_rise_timeout_s: float,
    assert_ms: int,
    post_release_wait_s: float,
    bit_return_timeout_s: float,
    reset_debug: bool,
) -> TestResult:
    """
    BIT/RESET sequence:
      1) Wait for BIT HIGH (boot good).
      2) Pulse RESET (known-good implementation).
      3) Wait post_release_wait_s for reboot to start.
      4) Require BIT HIGH again within bit_return_timeout_s.

    NOTE: We no longer require BIT to drop low (your BIT line may not drop).
    """
    GPIO.setup(bit_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    details = {
        "bit_rise_timeout_s": bit_rise_timeout_s,
        "reset_assert_ms": assert_ms,
        "post_release_wait_s": post_release_wait_s,
        "bit_return_timeout_s": bit_return_timeout_s,
        "reset_debug": bool(reset_debug),
    }

    # Always start with reset released
    reset_release(reset_ctrl_pin)
    sleep_s(0.05)

    got_high = wait_for_level(bit_pin, 1, bit_rise_timeout_s)
    details["bit_initial_high"] = bool(got_high)
    if not got_high:
        details["bit_level_after_timeout"] = read_level_stable(bit_pin)
        details["reason"] = "BIT did not go high after power-up."
        reset_release(reset_ctrl_pin)
        return TestResult("BIT_RESET", False, details)

    details["reset_epoch_s"] = now_epoch_s()
    print("\n[RESET] Pulsing reset now...")
    details.update(reset_pulse(reset_ctrl_pin, assert_ms, debug_pause=reset_debug))

    # Snapshot BIT soon after reset
    details["bit_after_reset_snapshot"] = int(read_level_stable(bit_pin, duration_s=0.25))

    # Let reboot start
    sleep_s(post_release_wait_s)

    got_high_again = wait_for_level(bit_pin, 1, bit_return_timeout_s)
    details["bit_high_again"] = bool(got_high_again)
    details["bit_end_snapshot"] = int(read_level_stable(bit_pin, duration_s=0.25))

    if not got_high_again:
        details["reason"] = "BIT did not confirm high after reset release."
        reset_release(reset_ctrl_pin)
        return TestResult("BIT_RESET", False, details)

    reset_release(reset_ctrl_pin)
    details["reset_left_released"] = True
    return TestResult("BIT_RESET", True, details)


def test_sync(sync_pin: int, low_s: float, high_s: float) -> TestResult:
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


def test_pps(
    pps_pin: int,
    window_s: float,
    min_pulses: int,
    expected_hz: float,
    max_jitter_ms: float,
    min_dt_s: float,
) -> TestResult:
    GPIO.setup(pps_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    edge_times = []
    last_accept = None

    def cb(_channel):
        nonlocal last_accept
        t = time.monotonic()
        if last_accept is not None and (t - last_accept) < min_dt_s:
            return
        edge_times.append(t)
        last_accept = t

    GPIO.add_event_detect(pps_pin, GPIO.RISING, callback=cb)

    t0 = time.monotonic()
    sleep_s(window_s)
    GPIO.remove_event_detect(pps_pin)

    n = len(edge_times)
    details = {"edges_captured": n, "window_s": window_s, "min_dt_s": min_dt_s}

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


def now_epoch_s() -> float:
    return time.time()


def main() -> int:
    ap = argparse.ArgumentParser(description="GPIO tester for PPS/BIT/RESET/SYNC bundle (Pi 4B).")
    ap.add_argument("--pps", type=int, default=17)
    ap.add_argument("--bit", type=int, default=27)
    ap.add_argument("--reset", type=int, default=22, help="RESET_CTRL GPIO (drives transistor base)")
    ap.add_argument("--sync", type=int, default=23)

    ap.add_argument("--bit-rise-timeout", type=float, default=15.0)
    ap.add_argument("--reset-assert-ms", type=int, default=1000)
    ap.add_argument("--post-release-wait-s", type=float, default=1.0)
    ap.add_argument("--bit-return-timeout", type=float, default=30.0)

    ap.add_argument("--sync-low-s", type=float, default=20.0)
    ap.add_argument("--sync-high-s", type=float, default=20.0)

    ap.add_argument("--pps-window", type=float, default=30.0)
    ap.add_argument("--pps-min-pulses", type=int, default=10)
    ap.add_argument("--pps-hz", type=float, default=1.0)
    ap.add_argument("--pps-max-jitter-ms", type=float, default=5.0)
    ap.add_argument("--pps-min-dt-s", type=float, default=0.80)

    ap.add_argument("--reset-debug", action="store_true",
                    help="Pause while reset is asserted/released (for meter/serial observation)")
    ap.add_argument("--out-json", type=str, default="")
    ap.add_argument("--out-csv", type=str, default="")

    args = ap.parse_args()

    out_json = Path(args.out_json) if args.out_json else None
    out_csv = Path(args.out_csv) if args.out_csv else None

    setup_gpio()

    # Start released and keep released unless pulsing.
    reset_release(args.reset)
    sleep_s(0.1)

    results = []
    try:
        print("\n=== BUNDLE TESTER ===")
        print("Wiring assumptions:")
        print("  - Common ground between Pi and unit")
        print("  - GPIO is 3.3V logic")
        print("RESET: GPIO HIGH asserts, GPIO LOW releases.\n")
        print("Order: BIT/RESET -> (leave RESET released) -> SYNC -> PPS\n")

        print("Step 0: Ensure the unit is powered ON and serial is active.")
        input("Press Enter to start BIT/RESET test (will pulse RESET)...")

        results.append(
            test_bit_and_reset(
                bit_pin=args.bit,
                reset_ctrl_pin=args.reset,
                bit_rise_timeout_s=args.bit_rise_timeout,
                assert_ms=args.reset_assert_ms,
                post_release_wait_s=args.post_release_wait_s,
                bit_return_timeout_s=args.bit_return_timeout,
                reset_debug=args.reset_debug,
            )
        )
        print(f"BIT/RESET: {'PASS' if results[-1].passed else 'FAIL'}")
        print(json.dumps(results[-1].details, indent=2))

        reset_release(args.reset)
        print("\nRESET left released (GPIO LOW) so the unit should remain ON.\n")

        input("Press Enter to run SYNC test (will toggle SYNC low/high)...")
        results.append(test_sync(args.sync, low_s=args.sync_low_s, high_s=args.sync_high_s))
        print(f"SYNC: {'PASS' if results[-1].passed else 'FAIL'}")
        print(json.dumps(results[-1].details, indent=2))

        print("\nStep 2: Ensure antennas have satellite time fix (for PPS).")
        input("Press Enter when ready to start PPS capture...")

        results.append(
            test_pps(
                pps_pin=args.pps,
                window_s=args.pps_window,
                min_pulses=args.pps_min_pulses,
                expected_hz=args.pps_hz,
                max_jitter_ms=args.pps_max_jitter_ms,
                min_dt_s=args.pps_min_dt_s,
            )
        )
        print(f"PPS: {'PASS' if results[-1].passed else 'FAIL'}")
        print(json.dumps(results[-1].details, indent=2))

        overall = all(r.passed for r in results)
        print("\n=== OVERALL RESULT:", "PASS" if overall else "FAIL", "===\n")

        write_results(results, out_json=out_json, out_csv=out_csv)
        return 0 if overall else 2

    except KeyboardInterrupt:
        print("\nAborted by user.")
        return 130

    finally:
        # Keep reset released after exit; do NOT cleanup reset pin.
        try:
            reset_release(args.reset)
            GPIO.cleanup([args.pps, args.bit, args.sync])
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
