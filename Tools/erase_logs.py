#!/usr/bin/env python3

import time

from Final_Configs import MavlinkSerialPort


def erase_logs(mav_serialport, timeout=8.0):
    """
    QGC-style erase: send MAVLink LOG_ERASE (id 121), then optionally verify by requesting the log list.
    """
    mav = mav_serialport.mav

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


if __name__ == "__main__":
    mav_serialport = MavlinkSerialPort(
        "udp:0.0.0.0:14550",
        57600,
        devnum=10,
    )

    time.sleep(0.5)

    try:
        erase_logs(mav_serialport)
    finally:
        mav_serialport.close()
