#!/usr/bin/env python3
# ... (your header stays the same)

from __future__ import print_function

import sys
import time
import re

from sys import platform as _platform

try:
    from pymavlink import mavutil
except ImportError as e:
    print("Failed to import pymavlink: " + str(e))
    print("")
    print("You may need to install it with:")
    print("    pip3 install --user pymavlink")
    print("")
    sys.exit(1)

try:
    import serial
except ImportError as e:
    print("Failed to import serial: " + str(e))
    print("")
    print("You may need to install it using:")
    print("    pip3 install --user pyserial")
    print("")
    sys.exit(1)


class MavlinkSerialPort():
    """an object that looks like a serial port, but
    transmits using mavlink SERIAL_CONTROL packets"""
    def __init__(self, portname, baudrate, devnum=0, debug=0):
        self.baudrate = 0
        self._debug = debug
        self.buf = ''
        self.port = devnum
        self.debug("Connecting with MAVLink to %s ..." % portname)
        self.mav = mavutil.mavlink_connection(portname, autoreconnect=True, baud=baudrate)
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        self.mav.wait_heartbeat()
        self.debug("HEARTBEAT OK\n")
        self.debug("Locked serial device\n")

    def debug(self, s, level=1):
        """write some debug text"""
        if self._debug >= level:
            print(s)

    def write(self, b):
        """write some bytes/str"""
        # Harden: accept str or bytes
        if isinstance(b, bytes):
            b = b.decode("utf-8", errors="replace")

        if not b:
            return

        self.debug("sending '%s' (0x%02x) of len %u\n" % (b, ord(b[0]), len(b)), 2)
        while len(b) > 0:
            n = len(b)
            if n > 70:
                n = 70
            buf = [ord(x) for x in b[:n]]
            buf.extend([0] * (70 - len(buf)))
            self.mav.mav.serial_control_send(
                self.port,
                mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                0,
                0,
                n,
                buf
            )
            b = b[n:]

    def close(self):
        self.mav.mav.serial_control_send(self.port, 0, 0, 0, 0, [0]*70)

    def _recv(self):
        """read some bytes into self.buf"""
        m = self.mav.recv_match(
            condition='SERIAL_CONTROL.count!=0',
            type='SERIAL_CONTROL',
            blocking=True,
            timeout=0.03
        )
        if m is not None:
            if self._debug > 2:
                print(m)
            data = m.data[:m.count]
            self.buf += ''.join(str(chr(x)) for x in data)

    def read(self, n):
        """read some bytes"""
        if len(self.buf) == 0:
            self._recv()
        if len(self.buf) > 0:
            if n > len(self.buf):
                n = len(self.buf)
            ret = self.buf[:n]
            self.buf = self.buf[n:]
            if self._debug >= 2:
                for b in ret:
                    self.debug("read 0x%x" % ord(b), 2)
            return ret
        return ''


# ---------- NEW HELPERS ----------

PROMPT_RE = re.compile(r"(?:^|\n)\s*nsh>\s*", re.IGNORECASE)

def read_until_prompt(mav_serialport, timeout=3.0):
    """
    Read shell output until we see an 'nsh>' prompt or timeout.
    Returns the text collected (may include the prompt).
    """
    output = ""
    start = time.time()
    while time.time() - start < timeout:
        mav_serialport._recv()
        chunk = mav_serialport.read(4096)
        if chunk:
            output += chunk
            if PROMPT_RE.search(output):
                return output
        time.sleep(0.02)
    return output


def run_shell_command(mav_serialport, cmd, timeout=3.0, wake=True):
    """
    Send a command and return output up to the next 'nsh>' prompt (or timeout).
    """
    if wake:
        mav_serialport.write("\n")
        time.sleep(0.05)
        _ = read_until_prompt(mav_serialport, timeout=1.0)

    mav_serialport.write(cmd.strip() + "\n")
    time.sleep(0.05)
    out = read_until_prompt(mav_serialport, timeout=timeout)
    return out


HEADING_GOOD_RE = re.compile(r"heading_good_for_control:\s*(True|False)", re.IGNORECASE)

def wait_for_heading_good_for_control(
    mav_serialport,
    total_timeout=120.0,
    poll_interval=1.0,
    per_call_timeout=2.5
):
    """
    Blocks until listener vehicle_local_position reports heading_good_for_control: True.
    Polls using: listener vehicle_local_position -n 1
    Raises TimeoutError if not achieved within total_timeout.
    """
    print(f"Waiting for heading_good_for_control: True (timeout {total_timeout}s) ...")

    start = time.time()
    last_seen = None

    while True:
        if time.time() - start > total_timeout:
            raise TimeoutError(
                f"Timed out after {total_timeout:.1f}s waiting for heading_good_for_control=True "
                f"(last seen: {last_seen})"
            )

        out = run_shell_command(
            mav_serialport,
            "listener vehicle_local_position -n 1",
            timeout=per_call_timeout,
            wake=True
        )

        m = HEADING_GOOD_RE.search(out)
        if m:
            val = m.group(1).lower() == "true"
            last_seen = val
            print(f"  heading_good_for_control: {val}")
            if val:
                print("✅ Dual Antenna heading verified.")
                return
        else:
            # Sometimes output is incomplete; keep trying.
            print("  (did not find heading_good_for_control in output; retrying)")

        time.sleep(poll_interval)


def set_lever_arms(mav_serialport, timeout=2.5):
    print("Setting Lever Arms for red cart Setup...")

    lever_arm_cmds_cart = [
        "param set GPS_SEP_BASE_X 0.495",
        "param set GPS_SEP_BASE_Y 0.0",
        "param set GPS_SEP_BASE_Z 0.0",
        "param set GPS_SEP_ROVER_X -0.52",
        "param set GPS_SEP_ROVER_Y 0.0",
        "param set GPS_SEP_ROVER_Z 0.0",
        "param set EKF2_IMU_POS_X 0.0 ",
        "param set EKF2_IMU_POS_Y 0.0",
        "param set EKF2_IMU_POS_Z 0.0",
        "param set SENS_BOARD_ROT 0",
        # 
        "reboot"
    ]

    for cmd in lever_arm_cmds_cart:
        print(f"\n[Setting] {cmd}")
        out = run_shell_command(mav_serialport, cmd, timeout=timeout, wake=True)
        out = out.strip()
        if out:
            print(out)
        else:
            print("[!] No response received.")

    print("\n[Done] All lever arms attempted to be set.")


if __name__ == "__main__":
    mav_serialport = MavlinkSerialPort("udp:0.0.0.0:14550", 57600, devnum=10)
    time.sleep(0.5)

    # set params, THEN wait until heading is good
    set_lever_arms(mav_serialport)
    time.sleep(1)
    wait_for_heading_good_for_control(mav_serialport, total_timeout=180.0)

    time.sleep(0.5)
    mav_serialport.close()