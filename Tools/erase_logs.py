#!/usr/bin/env python3

"""Erase INS logs using MAVLink; download and run this file on its own.

Install dependencies: python -m pip install pymavlink pyserial
Run with the default UDP connection: python erase_logs.py
For connection options: python erase_logs.py --help
"""

import argparse
import math
import sys
import time


def erase_logs(mav, timeout=8.0):
    """
    QGC-style erase: send MAVLink LOG_ERASE (id 121), then optionally verify by requesting the log list.
    """
    # Target sys/comp are learned from heartbeat by mavutil
    target_system = getattr(mav, "target_system", 1)
    target_component = getattr(mav, "target_component", 1)

    print("\n[Action] Erasing logs via MAVLink LOG_ERASE (AMC-style)...")

    # This is what QGC sends for "Erase All"
    mav.mav.log_erase_send(target_system, target_component)

    # Give the FC a moment to erase
    t0 = time.time()
    time.sleep(0.5)

    # Optional: verify by requesting list; many stacks will respond with LOG_ENTRY stream (possibly empty)
    print("[Action] Verifying erase by requesting log list...")
    mav.mav.log_request_list_send(target_system, target_component, 0, 0xFFFF)

    last_seen = time.time()
    any_entry = False
    total_logs = None

    while time.time() - t0 < timeout:
        msg = mav.recv_match(type=["LOG_ENTRY"], blocking=True, timeout=1.0)
        if msg is None:
            # If we haven't seen anything for a bit, stop waiting
            if time.time() - last_seen > 2.0:
                break
            continue

        last_seen = time.time()
        any_entry = True
        total_logs = msg.num_logs  # total number of logs onboard (as reported)
        # If there are zero logs, we're done
        if total_logs == 0:
            print("[OK] Vehicle reports 0 logs.")
            return True

        # Otherwise, keep draining entries until quiet; some firmwares don't stream all entries reliably over lossy links

    if total_logs == 0:
        print("[OK] Vehicle reports 0 logs.")
        return True

    if any_entry:
        print(f"[!] Vehicle still reports {total_logs} logs (or erase status unclear over link).")
    else:
        print("[!] No LOG_ENTRY response to verification request (erase may still have worked).")

    # Many setups still succeed even if we can't verify over MAVLink due to link loss / no implementation
    return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--connection", "--port", default="udp:0.0.0.0:14550",
        help="MAVLink connection, e.g. udp:0.0.0.0:14550 or COM3 (default: %(default)s)")
    parser.add_argument("--baud", "--baudrate", type=int, default=57600,
                        help="serial baud rate (default: %(default)s)")
    parser.add_argument("--heartbeat-timeout", type=float, default=10.0,
                        help="seconds to wait for the INS (default: %(default)s)")
    parser.add_argument("--timeout", type=float, default=8.0,
                        help="erase verification timeout in seconds (default: %(default)s)")
    args = parser.parse_args(argv)
    if args.baud <= 0 or any(not math.isfinite(t) or t <= 0
                            for t in (args.heartbeat_timeout, args.timeout)):
        parser.error("baud rate and timeouts must be positive; timeouts must be finite")

    # Import after parsing so --help works even before dependencies are installed.
    try:
        from pymavlink import mavutil
    except ImportError as error:
        print(f"Failed to import pymavlink: {error}", file=sys.stderr)
        print("Install dependencies: python -m pip install pymavlink pyserial", file=sys.stderr)
        return 1

    mav = None
    try:
        print(f"Connecting to {args.connection}...")
        mav = mavutil.mavlink_connection(args.connection, autoreconnect=True, baud=args.baud)
        mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC,
                               mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        if mav.wait_heartbeat(timeout=args.heartbeat_timeout) is None:
            print("[!] No heartbeat received; no erase command sent. "
                  "Check the INS connection and UDP port.", file=sys.stderr)
            return 1
        time.sleep(0.5)
        return 0 if erase_logs(mav, timeout=args.timeout) else 1
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except (OSError, ValueError) as error:
        print(f"[!] {error}", file=sys.stderr)
        return 1
    finally:
        if mav is not None:
            mav.close()


if __name__ == "__main__":
    raise SystemExit(main())
