#!/usr/bin/env python3
"""
Raspberry Pi GPIO PPS tester.

Run:
  sudo python3 pps_tester.py --pps 17
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


def main() -> int:
    ap = argparse.ArgumentParser(description="GPIO PPS tester for Raspberry Pi.")
    ap.add_argument("--pps", type=int, default=17)

    ap.add_argument("--pps-window", type=float, default=30.0)
    ap.add_argument("--pps-min-pulses", type=int, default=10)
    ap.add_argument("--pps-hz", type=float, default=1.0)
    ap.add_argument("--pps-max-jitter-ms", type=float, default=5.0)
    ap.add_argument("--pps-min-dt-s", type=float, default=0.80)

    ap.add_argument("--out-json", type=str, default="")
    ap.add_argument("--out-csv", type=str, default="")

    args = ap.parse_args()

    out_json = Path(args.out_json) if args.out_json else None
    out_csv = Path(args.out_csv) if args.out_csv else None

    setup_gpio()

    try:
        print("\n=== PPS TESTER ===")
        print("Step 1: Ensure antennas have satellite time fix (for PPS).")
        input("Press Enter when ready to start PPS capture...")

        result = test_pps(
            pps_pin=args.pps,
            window_s=args.pps_window,
            min_pulses=args.pps_min_pulses,
            expected_hz=args.pps_hz,
            max_jitter_ms=args.pps_max_jitter_ms,
            min_dt_s=args.pps_min_dt_s,
        )

        print(f"PPS: {'PASS' if result.passed else 'FAIL'}")
        print(json.dumps(result.details, indent=2))

        write_results([result], out_json=out_json, out_csv=out_csv)
        return 0 if result.passed else 2

    except KeyboardInterrupt:
        print("\nAborted by user.")
        return 130

    finally:
        try:
            GPIO.cleanup([args.pps])
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())