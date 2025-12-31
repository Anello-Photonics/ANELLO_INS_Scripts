#!/usr/bin/env python3
import time
import sys
import subprocess
import serial
import serial.tools.list_ports
from pymavlink import mavutil
from PCANBasic import *

# ---------------- User Settings ---------------- #
ETH_IP = "192.168.0.3"
RS232_BAUD = 57600
RS232_TIMEOUT = 3.0

CAN_PGN = 127257

# ============================================================
#  MavlinkSerialPort Class
# ============================================================
class MavlinkSerialPort():
    """An object that looks like a serial port, but transmits using MAVLink SERIAL_CONTROL packets"""
    def __init__(self, portname, baudrate, devnum=0, debug=0):
        self.baudrate = 0
        self._debug = debug
        self.buf = ''
        self.port = devnum
        self.debug(f"Connecting with MAVLink to {portname} ...")
        self.mav = mavutil.mavlink_connection(portname, autoreconnect=True, baud=baudrate)
        self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        self.mav.wait_heartbeat()
        self.debug("HEARTBEAT OK\n")
        self.debug("Locked serial device\n")

    def debug(self, s, level=1):
        if self._debug >= level:
            print(s)

    def write(self, b):
        self.debug(f"sending '{b}' of len {len(b)}", 2)
        while len(b) > 0:
            n = min(len(b), 70)
            buf = [ord(x) for x in b[:n]] + [0] * (70 - n)
            self.mav.mav.serial_control_send(
                self.port,
                mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                0,
                0,
                n,
                buf
            )
            b = b[n:]

    def close(self):
        self.mav.mav.serial_control_send(self.port, 0, 0, 0, 0, [0]*70)

    def _recv(self):
        m = self.mav.recv_match(condition='SERIAL_CONTROL.count!=0',
                                type='SERIAL_CONTROL', blocking=True,
                                timeout=0.03)
        if m is not None:
            if self._debug > 2:
                print(m)
            data = m.data[:m.count]
            self.buf += ''.join(chr(x) for x in data)

    def read(self, n):
        if len(self.buf) == 0:
            self._recv()
        if len(self.buf) > 0:
            n = min(n, len(self.buf))
            ret = self.buf[:n]
            self.buf = self.buf[n:]
            if self._debug >= 2:
                for b in ret:
                    self.debug(f"read 0x{ord(b):x}", 2)
            return ret
        return ''

# ============================================================
#  Ethernet Ping
# ============================================================
def check_ethernet_ping(ip=ETH_IP):
    print("\n=== Checking Ethernet (Ping) ===")
    param = "-n" if sys.platform.startswith("win") else "-c"
    cmd = ["ping", param, "1", ip]
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode == 0:
        print(f"[OK] Ping reply from {ip}")
        return True
    print(f"[FAIL] No ping response from {ip}")
    return False

# ============================================================
#  Enable MAVLink on Serial Ports
# ============================================================
def enable_mavlink():
    """Send mavlink start commands to /dev/ttyS6 and /dev/ttyS5"""
    try:
        print("\n[ETHERNET] Connecting to INS via MAVLink...")
        mav_port = MavlinkSerialPort("udp:0.0.0.0:14550", 57600, devnum=10)

        commands = [
            "mavlink start -d /dev/ttyS6 -b 57600 -m 0 -f 1 -r 0\n",  # RS232-1
            "mavlink start -d /dev/ttyS5 -b 57600 -m 0 -f 1 -r 0\n"   # RS232-2
        ]

        for cmd in commands:
            print(f"[ETHERNET] Sending: {cmd.strip()}")
            mav_port.write(cmd)
            time.sleep(0.5)

        print("[ETHERNET] INS MAVLink enabled on serial ports.")
        return True

    except Exception as e:
        print(f"[FAIL] Could not enable INS serial MAVLink: {e}")
        return False

# ============================================================
#  Enable / Disable Factory Mode
# ============================================================

def factory_mode(toggle):
    """Send mavlink start commands to /dev/ttyS6 and /dev/ttyS5"""
    try:
        print("\n[ETHERNET] Connecting to INS via MAVLink...")
        mav_port = MavlinkSerialPort("udp:0.0.0.0:14550", 57600, devnum=10)

        factory_on = [
            "param set IMU_MB_C_FACTORY 1",
            "param set IMU_MB_C_FTOG 0",
            "reboot"
        ]

        factory_off = [
            "param set IMU_MB_C_FACTORY 0",
            "param set IMU_MB_C_FTOG 1",
            "reboot"
        ]

        if toggle == True:
            for cmd in factory_on:
                print(f"Sending: {cmd.strip()}")
                run_shell_command(mav_port, cmd, timeout=6)
                print("Factory Mode ON")

        else :
            for cmd in factory_off:
                print(f"Sending: {cmd.strip()}")
                run_shell_command(mav_port, cmd, timeout=6)
                print("Factory Mode OFF")
        
        return True

    except Exception as e:
        print(f"[FAIL] Could not toggle factory mode: {e}")
        return False


# ============================================================
#  Detect COM Ports (Windows)
# ============================================================
def detect_com_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    print(f"\nDetected COM ports: {ports}")
    return ports

# ============================================================
#  RS232 Verification
# ============================================================
def check_rs232_verhw(com_ports):
    print("\n=== Checking RS232 Ports ===")
    mapping = {}
    for com in com_ports:
        try:
            print(f"Testing port {com} ...")
            with serial.Serial(com, RS232_BAUD, timeout=RS232_TIMEOUT) as ser:
                ser.write(b"ver hw\n")
                time.sleep(0.2)
                resp = ser.read(ser.in_waiting or 64)
                if resp:
                    decoded = resp.strip().decode(errors='ignore')
                    print(f"[OK] Response on {com}: {decoded}")
                    mapping[com] = True
                else:
                    print(f"[FAIL] No response on {com}")
                    mapping[com] = False
        except Exception as e:
            print(f"[FAIL] Could not open {com}: {e}")
            mapping[com] = False
    return mapping

# ============================================================
#  CAN PGN Check
# ============================================================
def check_can_pgn(channel=PCAN_USBBUS1, timeout=5, retries=3):
    print("\n=== Checking CAN (PCAN) ===")
    pcan = PCANBasic()
    ret = pcan.Initialize(channel, PCAN_BAUD_250K)
    if ret != PCAN_ERROR_OK:
        print("[FAIL] PCAN initialization failed.")
        return False

    print(f"Listening for PGN {CAN_PGN} (0x{CAN_PGN:X})...")
    for attempt in range(1, retries + 1):
        start = time.time()
        while time.time() - start < timeout:
            result = pcan.Read(channel)
            if isinstance(result, tuple):
                ret_val, msg, *_ = result
            else:
                ret_val, msg = result, None

            if ret_val != PCAN_ERROR_OK or msg is None:
                continue

            can_id = msg.ID
            pgn = (can_id >> 8) & 0x3FFFF
            if pgn == CAN_PGN:
                print(f"[OK] Received PGN {CAN_PGN} from SA {can_id & 0xFF}")
                pcan.Uninitialize(channel)
                return True

        print(f"[Attempt {attempt}] No PGN received.")
        time.sleep(0.5)

    pcan.Uninitialize(channel)
    print(f"[FAIL] No CAN PGN {CAN_PGN} detected.")
    return False

# ============================================================
#  Run Generic Shell Command via MAVLink
# ============================================================
def run_shell_command(mav_serialport, cmd, timeout=4.0):
    """
    Send MAVLink shell command (e.g., dmesg, df) and return full output.
    """
    print(f"\nRunning command: {cmd}")

    # Wake shell
    mav_serialport.write("\n")
    time.sleep(0.2)

    # Send command
    mav_serialport.write(cmd + "\n")
    time.sleep(0.2)

    # Collect output
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


# ============================================================
#  Print dmesg + df diagnostics
# ============================================================
def print_dmesg_and_df():
    """
    Connect via MAVLink and print dmesg / df output.
    """
    print("\n================ SYSTEM DIAGNOSTICS ================\n")

    try:
        mavport = MavlinkSerialPort("udp:0.0.0.0:14550", 57600, devnum=10)
    except Exception as e:
        print(f"[FAIL] Could not open MAVLink shell for diagnostics: {e}")
        return

    print("\n------------------- dmesg -------------------")
    run_shell_command(mavport, "dmesg", timeout=6)

    print("\n-------------------- df ---------------------")
    run_shell_command(mavport, "df -h", timeout=3)

    print("\n====================================================\n")

    mavport.close()

# ============================================================
#  Enable CAN PGN output
# ============================================================
def enable_CAN():


    try:
        mavport = MavlinkSerialPort("udp:0.0.0.0:14550", 57600, devnum=10)
    except Exception as e:
        print(f"[FAIL] Could not open MAVLink shell for diagnostics: {e}")
        return


    run_shell_command(mavport, "param set NM2K_CFG 1", timeout=6)
    run_shell_command(mavport, "param set NM2K_BITRATE 250000", timeout=6)
    run_shell_command(mavport, "param set NM2K_127257_RATE 10", timeout=6)
    run_shell_command(mavport, "reboot", timeout=6)
    


    mavport.close()


# ============================================================
#  MAIN
# ============================================================
def main():
    results = {
        "ethernet": False,
        "rs232": {},
        "can": False
    }
    # disable factory mode
    factory_mode(False)

    # Ethernet
    results["ethernet"] = check_ethernet_ping()
    if results["ethernet"]:
        enable_mavlink()
        time.sleep(1.0)
        print_dmesg_and_df()   # ← ADD THIS

    # RS232
    coms = detect_com_ports()
    results["rs232"] = check_rs232_verhw(coms)

    # CAN
    enable_CAN()
    results["can"] = check_can_pgn()

    # Summary
    print("\n==================== SUMMARY ====================")
    print(f"Ethernet: {'[OK]' if results['ethernet'] else '[FAIL]'}")
    print("\nRS232 Ports:")
    for com, status in results["rs232"].items():
        print(f"  {com}: {'[OK]' if status else '[FAIL]'}")
    print(f"\nCAN PGN {CAN_PGN}: {'[OK]' if results['can'] else '[FAIL]'}")
    print("=================================================\n")

    # enable factory mode
    factory_mode(True)

if __name__ == "__main__":
    main()
