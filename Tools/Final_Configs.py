#!/usr/bin/env python3

from __future__ import print_function

import sys
import argparse
import binascii
import socket
import struct
import json
import zlib
import base64
import time
import array
import os
import tkinter as tk
import threading
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

# Example hardcoded listener topics list
COMMON_TOPICS = [
    "sensor_gyro",
    "sensor_accel",
    "sensor_gps_0",
    "sensor_gps_1",
    "estimator_status_flags",
    "vehicle_global_position",
    "sensor_septentrio",
    "sensor_gps_heading_0",
    "sensor_water_speed_generic",
    "nmea_engine",
]

# Edit your ethernet parameters here
# ==========================================================================
eth_cfg = {
    "NET_CFG_PROTO": 1,                   # 0=None, 1=Static, 2=DHCP, 3=Fallback
    "NET_CFG_IPADDR": "192.168.0.3",
    "NET_CFG_NETMASK": "255.255.255.0",
    "NET_CFG_ROUTER": "192.168.0.254",
    "NET_CFG_DNS": "192.168.0.254",
}
# ==========================================================================

class MavlinkSerialPort():
    '''an object that looks like a serial port, but
    transmits using mavlink SERIAL_CONTROL packets'''
    def __init__(self, portname, baudrate, devnum=0, debug=0):
        self.baudrate = 0
        self._debug = debug
        self.buf = ''
        self.port = devnum
        self.debug("Connecting with MAVLink to %s ..." % portname)
        self.mav = mavutil.mavlink_connection(portname, autoreconnect=True, baud=baudrate)
        self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        self.mav.wait_heartbeat()
        self.debug("HEARTBEAT OK\n")
        self.debug("Locked serial device\n")

    def debug(self, s, level=1):
        '''write some debug text'''
        if self._debug >= level:
            print(s)

    def write(self, b):
        '''write some bytes'''
        self.debug("sending '%s' (0x%02x) of len %u\n" % (b, ord(b[0]), len(b)), 2)
        while len(b) > 0:
            n = len(b)
            if n > 70:
                n = 70
            buf = [ord(x) for x in b[:n]]
            buf.extend([0]*(70-len(buf)))
            self.mav.mav.serial_control_send(self.port,
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                                             0,
                                             0,
                                             n,
                                             buf)
            b = b[n:]

    def close(self):
        self.mav.mav.serial_control_send(self.port, 0, 0, 0, 0, [0]*70)

    def _recv(self):
        '''read some bytes into self.buf'''
        m = self.mav.recv_match(condition='SERIAL_CONTROL.count!=0',
                                type='SERIAL_CONTROL', blocking=True,
                                timeout=0.03)
        if m is not None:
            if self._debug > 2:
                print(m)
            data = m.data[:m.count]
            self.buf += ''.join(str(chr(x)) for x in data)

    def read(self, n):
        '''read some bytes'''
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

# Define time to use time.time() by default
def _time():
    return time.time()

# Detect python version
if sys.version_info[0] < 3:
    runningPython3 = False
else:
    runningPython3 = True
    if sys.version_info[1] >=3:
        # redefine to use monotonic time when available
        def _time():
            try:
                return time.monotonic()
            except Exception:
                return time.time()




def run_ver_all_command(mav_serialport, timeout=3.0):
    """
    Send 'ver all' command via MAVLink SERIAL_CONTROL and return the output.
    """
    print("Sending 'ver all' command...")

    # Wake up the shell
    mav_serialport.write('\n')
    time.sleep(0.3)

    # Send the command
    mav_serialport.write('ver all\n')
    time.sleep(0.3)

    # Read loop to capture shell output
    output = ''
    start_time = time.time()

    while time.time() - start_time < timeout:
        mav_serialport._recv()  # Pull in SERIAL_CONTROL messages
        chunk = mav_serialport.read(1024)
        if chunk:
            output += chunk
        time.sleep(0.05)

    output = output.strip()

    if output:
        print("==== ver all Output ====")
        print(output)
    else:
        print("[!] No response received within timeout.")

    return output


def extract_ver_all_summary(ver_all_output):
    """
    Extract selected fields from ver all output.
    """
    fields = ("HW arch", "PX4 git-hash", "PX4 version", "Build datetime")
    summary = []
    for field in fields:
        match = re.search(rf"^\s*{re.escape(field)}\s*:\s*(.+)$", ver_all_output, re.MULTILINE)
        if match:
            summary.append((field, match.group(1).strip()))
        else:
            summary.append((field, "[Not found]"))
    return summary


def print_ver_all_summary(ver_all_output):
    """
    Print selected fields from ver all output.
    """
    print("\n==== ver all Summary ====")
    for field, value in extract_ver_all_summary(ver_all_output):
        print(f"{field}: {value}")


def prompt_serial_number():
    while True:
        serial_number = input("Enter full serial number: ").strip()
        digits = re.sub(r"\D", "", serial_number)
        if len(digits) < 7:
            print("[!] Serial number must contain at least 7 digits.")
            continue
        return digits


def set_final_configs(mav_serialport, serial_number, timeout=1.0):
    """
    Sets all lever arm parameters via MAVLink shell commands
    and prints the responses.

    mav_serialport: An object that supports .write(), .read(), and ._recv()
    timeout: seconds to wait for each response
    """
    print("Setting Final Configurations for Maritime INS...")


    serial_year = serial_number[:4]
    serial_tail = serial_number[-4:]

    final_configs = [
        ("IMU_MB_C_FACTORY", 0),
        ("IMU_MB_C_FTOG", 1),
        ("IMU_MB_C_SN", int(serial_tail)),
        ("IMU_MB_C_YR", int(serial_year)),
        ("SYS_AUTOSTART", 60009),
        ("NM2K_CFG", 1),
        ("J1939_CFG", 0),
        ("NM2K_BITRATE", 250000),
        ("NM2K_127257_RATE", 10),
        ("CAN_TERM", 1),
        ("NM0183_CFG", 2),
        ("MAV_0_CONFIG", 101),
        ("SER_TEL1_BAUD", 57600),
        ("SER_TEL2_BAUD", 57600),
        ("GPS_SEP_BASE_X", 0.0),
        ("GPS_SEP_BASE_Y", 0.0),
        ("GPS_SEP_BASE_Z", 0.0),
        ("GPS_SEP_ROVER_X", 0.0),
        ("GPS_SEP_ROVER_Y", 0.0),
        ("GPS_SEP_ROVER_Z", 0.0),
        ("EKF2_IMU_POS_X", 0.0),
        ("EKF2_IMU_POS_Y", 0.0),
        ("EKF2_IMU_POS_Z", 0.0),
        ("SENS_BOARD_ROT", 0),
        ("NMEA_UDP_EN", 1),
        ("NMEA_UDP_ODR_GGA", 10.000),
        ("NMEA_UDP_ODR_RMC", 10.000),
        ("NM0183_ODR_GGA", 10.000),
        ("NM0183_ODR_RMC", 10.000),
        ("NMEA_UDP_MC_IP0", 0),
        ("NMEA_UDP_MC_IP1", 0),
        ("NMEA_UDP_MC_IP2", 0),
        ("NMEA_UDP_MC_IP3", 0),
        ("MAV_2_BROADCAST", 1),
        ("MAV_2_CONFIG", 1000),
    ]


    for param_name, param_value in final_configs:
        cmd = f"param set {param_name} {param_value}"
        print(f"\n[Setting] {cmd}")

        # Wake shell
        mav_serialport.write("\n")
        time.sleep(0.1)

        # Send the command
        mav_serialport.write(cmd + "\n")
        time.sleep(0.1)

        # Capture response
        output = ""
        start_time = time.time()

        while time.time() - start_time < timeout:
            mav_serialport._recv()  # Pull in SERIAL_CONTROL messages
            chunk = mav_serialport.read(1024)
            if chunk:
                output += chunk
            time.sleep(0.05)

        output = output.strip()
        if output:
            print(output)
        else:
            print("[!] No response received.")

    set_ethernet_params(mav_serialport)
    time.sleep(0.5)
    print("\n[Done] All parameters attempted to be set.")
    return dict(final_configs)

# ==================================================================================
# IPv4 string → signed int32 conversion
# ==================================================================================
def ipv4_to_int32(ip_str):
    """
    Convert readable IPv4 like '192.168.0.3' into signed 32-bit integer

    """
    parts = ip_str.split('.')
    if len(parts) != 4:
        raise ValueError("Invalid IPv4 address format: %s" % ip_str)

    a, b, c, d = [int(p) for p in parts]

    unsigned32 = (a << 24) | (b << 16) | (c << 8) | d

    # Convert to signed 32-bit
    if unsigned32 >= (1 << 31):
        signed32 = unsigned32 - (1 << 32)
    else:
        signed32 = unsigned32

    return signed32


# ==================================================================================
# Ethernet parameter setter
# ==================================================================================
def set_ethernet_params(mav_serialport, timeout=1.0):
    """
    Sets Ethernet configuration parameters via MAVLink shell commands
    and prints the responses.
    """

    print("Setting Ethernet parameters...")


    # Convert readable IPs into int32 values
    converted = {}
    for key, value in eth_cfg.items():
        if key == "NET_CFG_PROTO":
            converted[key] = value
        else:
            converted[key] = ipv4_to_int32(value)

    # Send the commands
    for name, val in converted.items():
        print(f"\n[Setting] {name} = {val}")

        # wake shell
        mav_serialport.write("\n")
        time.sleep(0.1)

        # send parameter
        mav_serialport.write(f"param set {name} {val}\n")
        time.sleep(0.1)

        # receive response
        output = ""
        start = time.time()

        while time.time() - start < timeout:
            mav_serialport._recv()
            chunk = mav_serialport.read(1024)
            if chunk:
                output += chunk
            time.sleep(0.05)

        output = output.strip()
        if output:
            print(output)
        else:
            print("[!] No response received.")

    print("\n[Done] All Ethernet parameters attempted to be set.")




def reboot(mav_serialport):
    print("\n[Action] Rebooting via MAVLink...")
    run_shell_command(mav_serialport, "reboot", timeout=6)
    print("[OK] Reboot wait complete.")
    run_shell_command(mav_serialport, "param touch SER_TEL2_BAUD", timeout=1)
    return True


def parse_param_value(value):
    try:
        if "." in value or "e" in value.lower():
            return float(value)
        return int(value)
    except (ValueError, AttributeError):
        return value


def load_params_from_file(filename):
    params = {}
    try:
        with open(filename, "r") as f:
            for line in f:
                if line.startswith("===="):
                    break
                stripped = line.strip()
                if not stripped:
                    continue
                parts = stripped.split(None, 1)
                if len(parts) != 2:
                    continue
                params[parts[0]] = parse_param_value(parts[1])
    except FileNotFoundError:
        return None
    return params


def verify_expected_params_from_file(expected_params, filename, tolerance=1e-3):
    print("\n[Action] Verifying parameter values against saved file...")
    params = load_params_from_file(filename)
    if not params:
        print("[!] No parameters found for verification.")
        return False

    mismatches = []
    for name, expected_value in expected_params.items():
        actual_value = params.get(name)
        if actual_value is None:
            mismatches.append(f"{name} missing (expected {expected_value})")
            continue
        try:
            if abs(float(actual_value) - float(expected_value)) > tolerance:
                mismatches.append(
                    f"{name} expected {expected_value} got {actual_value}"
                )
        except (TypeError, ValueError):
            if actual_value != expected_value:
                mismatches.append(
                    f"{name} expected {expected_value} got {actual_value}"
                )

    if mismatches:
        print("[!] Parameter verification failed:")
        for mismatch in mismatches:
            print(f" - {mismatch}")
        return False

    print("[OK] All parameters match expected values.")
    return True

def nsh_cmd(mav_serialport, cmd, timeout=3.0):
    """
    Send a single NSH command and wait for the nsh> prompt.
    """
    mav_serialport.write(cmd + "\n")

    output = ""
    start = time.time()

    while time.time() - start < timeout:
        mav_serialport._recv()
        chunk = mav_serialport.read(1024)
        if chunk:
            output += chunk
            if "nsh>" in output:
                break
        time.sleep(0.05)

    return output

def run_shell_command(mav_serialport, cmd, timeout=4.0):
    """
    Send MAVLink shell command and return full output.
    """
    print(f"\nRunning command: {cmd}")

    # Wake shell
    mav_serialport.write("\n")
    time.sleep(0.2)

    # Send command
    mav_serialport.write(cmd + "\n")
    time.sleep(0.2)

    output = ""
    start = time.time()

    while time.time() - start < timeout:
        mav_serialport._recv()
        chunk = mav_serialport.read(4096)
        if chunk:
            output += chunk
        time.sleep(0.05)

    output = output.strip()
    if output:
        print(output)
    else:
        print("[!] No response received")

    return output

def erase_logs(mav_serialport):
    print("\n[Action] Erasing logs (QGC-compatible)...")

    # Stop logger just in case
    print(nsh_cmd(mav_serialport, "logger stop"))

    # List folders
    print(nsh_cmd(mav_serialport, "ls /fs/microsd/log"))

    # Delete each dated directory explicitly
    out = nsh_cmd(mav_serialport, "ls /fs/microsd/log")
    for line in out.splitlines():
        line = line.strip()
        if re.match(r"\d{4}-\d{2}-\d{2}/", line):
            folder = "/fs/microsd/log/" + line.rstrip("/")
            print(f"[Deleting] {folder}")
            print(nsh_cmd(mav_serialport, f'rm -r "{folder}"'))

    # Verify
    print("[Verify]")
    print(nsh_cmd(mav_serialport, "ls /fs/microsd/log"))


def _fetch_params_via_mavlink(mav, timeout=20, max_retries=2):
    print("\n[Action] Requesting all parameters via MAVLink...")
    mav.param_fetch_all()

    params = {}
    expected_count = None
    start = time.time()

    while True:
        msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if not msg:
            break

        params[msg.param_id] = msg
        expected_count = msg.param_count

        if expected_count is not None and len(params) >= expected_count:
            break

        if time.time() - start > timeout:
            print("[!] Timeout waiting for all parameters")
            break

    if expected_count is None or not params:
        return None

    for _ in range(max_retries):
        if len(params) >= expected_count:
            break
        missing_indices = {
            idx for idx in range(expected_count)
            if idx not in {msg.param_index for msg in params.values()}
        }
        if not missing_indices:
            break
        print(f"[Action] Requesting {len(missing_indices)} missing parameters...")
        for idx in sorted(missing_indices):
            mav.param_request_read(param_id=b"", param_index=idx)
        retry_start = time.time()
        while time.time() - retry_start < 5:
            msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if not msg:
                break
            params[msg.param_id] = msg
            if len(params) >= expected_count:
                break

    if len(params) < expected_count:
        print(f"[!] Incomplete parameter set: got {len(params)} of {expected_count}")

    def format_param(msg):
        """Return the parameter value in a human-readable form depending on type"""
        mavlink = mavutil.mavlink
        param_type = msg.param_type

        if param_type in (
            mavlink.MAV_PARAM_TYPE_UINT8,
            mavlink.MAV_PARAM_TYPE_UINT16,
            mavlink.MAV_PARAM_TYPE_UINT32,
        ):
            raw = struct.unpack('I', struct.pack('f', msg.param_value))[0]
            if param_type == mavlink.MAV_PARAM_TYPE_UINT8:
                return raw & 0xFF
            if param_type == mavlink.MAV_PARAM_TYPE_UINT16:
                return raw & 0xFFFF
            return raw

        if param_type in (
            mavlink.MAV_PARAM_TYPE_INT8,
            mavlink.MAV_PARAM_TYPE_INT16,
            mavlink.MAV_PARAM_TYPE_INT32,
        ):
            raw = struct.unpack('i', struct.pack('f', msg.param_value))[0]
            if param_type == mavlink.MAV_PARAM_TYPE_INT8:
                return struct.unpack('b', struct.pack('B', raw & 0xFF))[0]
            if param_type == mavlink.MAV_PARAM_TYPE_INT16:
                return struct.unpack('h', struct.pack('H', raw & 0xFFFF))[0]
            return raw

        if param_type in (
            mavlink.MAV_PARAM_TYPE_REAL32,
            mavlink.MAV_PARAM_TYPE_REAL64,
        ):
            return float(msg.param_value)

        return msg.param_value

    formatted_params = {}
    for name, msg in params.items():
        formatted_params[name] = format_param(msg)
    return formatted_params


def save_params_via_mavlink(mav, serial_number, output_dir=".", ver_all_output=None):
    """
    Fetch all parameters via MAVLink PARAM_REQUEST_LIST and save to a timestamped text file.

    Correctly handles:
    - INT32
    - FLOAT32
    - UINT32 / UINT16
    """
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_dir, f"param_dump_{serial_number}_{timestamp}.txt")

    params = _fetch_params_via_mavlink(mav)

    if not params:
        print("[!] No parameters received")
        return None

    # Write to file sorted by parameter name
    with open(filename, "w") as f:
        for name in sorted(params):
            f.write(f"{name} {params[name]}\n")

        if ver_all_output:
            f.write("\n==== ver all Summary ====\n")
            for field, value in extract_ver_all_summary(ver_all_output):
                f.write(f"{field}: {value}\n")

    print(f"[OK] Saved {len(params)} parameters to {filename}")
    return filename




if __name__ == "__main__":
    mav_serialport = MavlinkSerialPort(
        "udp:0.0.0.0:14550",
        57600,
        devnum=10
    )

    time.sleep(0.5)

    serial_number = prompt_serial_number()
    expected_params = set_final_configs(mav_serialport, serial_number)
    time.sleep(0.5)

    reboot(mav_serialport)
    time.sleep(3.0)

    erase_logs(mav_serialport)
    time.sleep(1.0)

    ver_all_output = run_ver_all_command(mav_serialport)
    print_ver_all_summary(ver_all_output)

    filename = save_params_via_mavlink(
        mav_serialport.mav,
        serial_number,
        ver_all_output=ver_all_output
    )
    if filename:
        verify_expected_params_from_file(expected_params, filename)

    mav_serialport.close()
