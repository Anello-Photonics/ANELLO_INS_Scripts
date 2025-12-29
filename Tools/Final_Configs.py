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




def run_ver_mcu_command(mav_serialport, timeout=3.0):
    """
    Send 'ver mcu' command via MAVLink SERIAL_CONTROL and return the output.
    """
    print("Sending 'ver mcu' command...")

    # Wake up the shell
    mav_serialport.write('\n')
    time.sleep(0.3)

    # Send the command
    mav_serialport.write('ver mcu\n')
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
        print("==== ver mcu Output ====")
        print(output)
    else:
        print("[!] No response received within timeout.")

    return output



def set_final_configs(mav_serialport, timeout=1.0):
    """
    Sets all lever arm parameters via MAVLink shell commands
    and prints the responses.

    mav_serialport: An object that supports .write(), .read(), and ._recv()
    timeout: seconds to wait for each response
    """
    print("Setting Final Configurations for Maritime INS...")


    final_configs = [
        "param set IMU_MB_C_FACTORY 0",
        "param set IMU_MB_C_FTOG 1"
        "param set SYS_AUTOSTART 60009",
        "param set GPS_SEP_BASE_X 0.0",
        "param set GPS_SEP_BASE_Y 0.0",
        "param set GPS_SEP_BASE_Z 0.0",
        "param set GPS_SEP_ROVER_X 0.0",
        "param set GPS_SEP_ROVER_Y 0.0",
        "param set GPS_SEP_ROVER_Z 0.0",
        "param set EKF2_IMU_POS_X 0.0",
        "param set EKF2_IMU_POS_Y 0.0",
        "param set EKF2_IMU_POS_Z 0.0",
        "param set SENS_BOARD_ROT 0"
    ]



    for cmd in final_configs:
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

    print("\n[Done] All lever arms attempted to be set.")

def erase_logs(mav_serialport, timeout=5.0):
    """
    Erase all PX4 logs on older PX4/NuttX builds
    (no logger erase support).
    """
    print("\n[Action] Erasing PX4 logs (legacy method)...")

    def run(cmd, wait=0.3, capture_time=1.0):
        mav_serialport.write("\n")
        time.sleep(0.1)
        mav_serialport.write(cmd + "\n")
        time.sleep(wait)

        output = ""
        start = time.time()
        while time.time() - start < capture_time:
            mav_serialport._recv()
            chunk = mav_serialport.read(1024)
            if chunk:
                output += chunk
            time.sleep(0.05)
        return output.strip()

    # 1) Stop logger
    print("[Step 1] Stopping logger")
    out = run("logger stop", capture_time=1.5)
    if out:
        print(out)

    # 2) Delete files inside date folders
    print("[Step 2] Deleting log files")
    run("rm /fs/microsd/log/*/*", capture_time=1.5)

    # 3) Remove empty date directories
    print("[Step 3] Removing empty log directories")
    run("rmdir /fs/microsd/log/*", capture_time=1.5)

    # 4) Verify
    print("[Verify] Remaining log entries:")
    out = run("ls /fs/microsd/log", capture_time=1.5)
    if out:
        print(out)
        print("[Warning] Some entries remain")
    else:
        print("[Done] Log directory is empty")


def print_df():
    """
    Connect via MAVLink and print dmesg / df output.
    """
    print("\n================ SYSTEM DIAGNOSTICS ================\n")

    try:
        mavport = MavlinkSerialPort("udp:0.0.0.0:14550", 57600, devnum=10)
    except Exception as e:
        print(f"[FAIL] Could not open MAVLink shell for diagnostics: {e}")
        return

    print("\n-------------------- df ---------------------")
    run_shell_command(mavport, "df -h", timeout=3)

    print("\n====================================================\n")

    mavport.close()




if __name__ == "__main__":
    mav_serialport = MavlinkSerialPort(
        "udp:0.0.0.0:14550",
        57600,
        devnum=10
    )

    time.sleep(0.5)

    set_final_configs(mav_serialport)
    time.sleep(0.5)

    erase_logs(mav_serialport)
    time.sleep(0.5)

    mav_serialport.close()

