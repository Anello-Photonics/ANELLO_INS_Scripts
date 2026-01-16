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

# =================================================================================
# Enter Lever arm measurements here
# =================================================================================
lever_arm_cmds = [
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


# ==========================================================================
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

 

def set_lever_arms(mav_serialport, timeout=1.0):
    """
    Sets all lever arm parameters via MAVLink shell commands
    and prints the responses.

    mav_serialport: An object that supports .write(), .read(), and ._recv()
    timeout: seconds to wait for each response
    """
    print("Setting Lever Arms for ______ Setup...")
    

    for cmd in lever_arm_cmds:
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




if __name__ == "__main__":
    mav_serialport = MavlinkSerialPort("udp:0.0.0.0:14550", 57600, devnum=10)
    time.sleep(0.5)
    set_lever_arms(mav_serialport)
    time.sleep(0.5)
    set_ethernet_params(mav_serialport)
    time.sleep(0.5)
    mav_serialport.close()
