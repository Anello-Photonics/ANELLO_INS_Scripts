
import time
import socket
import subprocess  # kept in case you want later use
import serial
import serial.tools.list_ports
from pymavlink import mavutil
from PCANBasic import *
import re

# ---------------- User Settings ---------------- #
ETH_IP = "192.168.0.3"

# MAVLink shell
MAVLINK_SHELL = "udp:0.0.0.0:14550"
MAVLINK_BAUD = 57600
MAV_DEVNUM = 10
SHELL_DEBUG = 0

# NMEA UDP injection (PAP*)
UDP_NMEA_PORT = 19551
PAPPOS = "PAPPOS,,37.335390,-122.026130,12.30,2.0,3.5"
PAPRPH = "PAPRPH,203600.00,0.80,1.50,273.20,0.20,0.20,0.50"

# CAN / NMEA2000
CAN_PGN = 127257
PCAN_CHANNEL = PCAN_USBBUS1
PCAN_BITRATE = PCAN_BAUD_250K

POST_REBOOT_WAIT_S = 10
NMEA0183_OUTPUT_PORT = 19550


# ============================================================
#  MavlinkSerialPort Class (SERIAL_CONTROL shell)
# ============================================================
class MavlinkSerialPort:
    """Serial-like interface implemented via MAVLink SERIAL_CONTROL packets."""

    def __init__(self, portname, baudrate, devnum=0, debug=0):
        self._debug = debug
        self.buf = b""
        self.port = devnum

        self.debug(f"Connecting with MAVLink to {portname} ...", 1)
        self.mav = mavutil.mavlink_connection(portname, autoreconnect=True, baud=baudrate)

        # Kick heartbeat so some systems respond quickly
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
        )
        self.mav.wait_heartbeat()
        self.debug("HEARTBEAT OK\nLocked serial device\n", 1)

    def debug(self, s, level=1):
        if self._debug >= level:
            print(s)

    def write(self, data):
        # Accept str or bytes
        if isinstance(data, str):
            data = data.encode("utf-8", errors="replace")

        while len(data) > 0:
            n = min(len(data), 70)
            chunk = data[:n]
            buf = list(chunk) + [0] * (70 - n)

            self.mav.mav.serial_control_send(
                self.port,
                mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
                | mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                0,
                0,
                n,
                buf,
            )
            data = data[n:]

    def close(self):
        # release exclusive access
        self.mav.mav.serial_control_send(self.port, 0, 0, 0, 0, [0] * 70)

    def _recv_once(self, timeout=0.05):
        m = self.mav.recv_match(
            condition="SERIAL_CONTROL.count!=0",
            type="SERIAL_CONTROL",
            blocking=True,
            timeout=timeout,
        )
        if m is None:
            return
        data = bytes(m.data[:m.count])
        if data:
            self.buf += data

    def read_bytes(self, n):
        if not self.buf:
            self._recv_once()
        if not self.buf:
            return b""
        n = min(n, len(self.buf))
        out = self.buf[:n]
        self.buf = self.buf[n:]
        return out

    def flush(self, seconds=0.2):
        end = time.time() + seconds
        while time.time() < end:
            self._recv_once(timeout=0.02)
            _ = self.read_bytes(4096)
            time.sleep(0.01)


# ============================================================
#  Shell command runner (marker-based, more reliable)
# ============================================================
def run_shell_command(mavport, cmd, timeout=4.0):
    """
    Send PX4 shell command and return output. Uses an end-marker for deterministic capture.
    """
    marker = f"__END_{int(time.time() * 1000)}__"
    full_cmd = f"{cmd}; echo {marker}"

    print(f"\nRunning command: {cmd}")

    mavport.flush(0.25)
    mavport.write("\n")
    time.sleep(0.1)
    mavport.flush(0.1)

    mavport.write(full_cmd + "\n")

    out = b""
    end = time.time() + timeout
    while time.time() < end:
        mavport._recv_once(timeout=0.05)
        chunk = mavport.read_bytes(4096)
        if chunk:
            out += chunk
            if marker.encode("utf-8") in out:
                break
        time.sleep(0.01)

    text = out.decode("utf-8", errors="replace")
    if marker in text:
        text = text.split(marker, 1)[0]
    text = text.strip()

    if text:
        print(text)
    else:
        print("[!] No response received")

    return text


# ============================================================
#  CAN PGN Check (your original logic, slightly hardened)
# ============================================================
def check_can_pgn(channel=PCAN_CHANNEL, timeout=5, retries=3):
    print("\n=== Checking CAN (PCAN) ===")
    pcan = PCANBasic()
    ret = pcan.Initialize(channel, PCAN_BITRATE)
    if ret != PCAN_ERROR_OK:
        print("[FAIL] PCAN initialization failed.")
        return False

    print(f"Listening for PGN {CAN_PGN} (0x{CAN_PGN:X})...")
    try:
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

                # Your original extraction:
                pgn = (can_id >> 8) & 0x3FFFF

                if pgn == CAN_PGN:
                    print(f"[OK] Received PGN {CAN_PGN} from SA {can_id & 0xFF}")
                    return True

            print(f"[Attempt {attempt}] No PGN received.")
            time.sleep(0.5)

        print(f"[FAIL] No CAN PGN {CAN_PGN} detected.")
        return False
    finally:
        pcan.Uninitialize(channel)


# ============================================================
#  Enable CAN PGN output
# ============================================================
def enable_CAN():
    try:
        mavport = MavlinkSerialPort(MAVLINK_SHELL, MAVLINK_BAUD, devnum=MAV_DEVNUM, debug=SHELL_DEBUG)
    except Exception as e:
        print(f"[FAIL] Could not open MAVLink shell for diagnostics: {e}")
        return False

    try:
        run_shell_command(mavport, "param set NM2K_CFG 1", timeout=6)
        run_shell_command(mavport, "param set NM2K_BITRATE 250000", timeout=6)
        run_shell_command(mavport, "param set NM2K_127257_RATE 10", timeout=6)
        run_shell_command(mavport, "reboot", timeout=6)
        return True
    finally:
        try:
            mavport.close()
        except Exception:
            pass


# ============================================================
#  Enable NMEA0183 UDP input 
# ============================================================
def enable_NM0183_NAV():
    try:
        mavport = MavlinkSerialPort(MAVLINK_SHELL, MAVLINK_BAUD, devnum=MAV_DEVNUM, debug=SHELL_DEBUG)
    except Exception as e:
        print(f"[FAIL] Could not open MAVLink shell for diagnostics: {e}")
        return False

    try:
        # You had "param NMEA_UDP_EN" (ambiguous). This enables it explicitly.
        run_shell_command(mavport, "param set NMEA_UDP_EN 1", timeout=6)
        run_shell_command(mavport, "param set NMEA_UDP_ODR_GGA 10", timeout=6)
        run_shell_command(mavport, "param set NMEA_UDP_ODR_RMC 10", timeout=6)
        run_shell_command(mavport, "reboot", timeout=6)
        return True
    finally:
        try:
            mavport.close()
        except Exception:
            pass


# ============================================================
#  NMEA UDP injection (PAPPOS / PAPRPH)
# ============================================================
def nmea_checksum(sentence: str) -> str:
    csum = 0
    for ch in sentence:
        csum ^= ord(ch)
    return f"{csum:02X}"


def send_nmea_udp(ip, port, sentences, inter_msg_delay_s=0.1):
    print("\n=== Sending PAPPOS / PAPRPH over UDP ===")
    addr = (ip, port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for s in sentences:
            msg = f"${s}*{nmea_checksum(s)}\r\n"
            sock.sendto(msg.encode("ascii"), addr)
            print("Sent:", msg.strip())
            time.sleep(inter_msg_delay_s)
    finally:
        sock.close()


# ============================================================
#  aux_global_position check (injected)
# ============================================================
def check_aux_global_position(timeout=6.0):
    """
    Connects to MAVLink shell and runs `listener aux_global_position`.
    PASS if it doesn't say "not found" and prints expected fields (timestamp/lat/lon).
    """
    print("\n=== Checking aux_global_position topic ===")
    try:
        mavport = MavlinkSerialPort(MAVLINK_SHELL, MAVLINK_BAUD, devnum=MAV_DEVNUM, debug=SHELL_DEBUG)
    except Exception as e:
        print(f"[FAIL] Could not open MAVLink shell for aux_global_position check: {e}")
        return False

    try:
        out = run_shell_command(mavport, "listener aux_global_position", timeout=timeout)
        low = out.lower()
        if "not found" in low or "unknown uorb topic" in low:
            print("[FAIL] aux_global_position topic not created")
            return False

        if "timestamp" in low or "lat" in low or "lon" in low:
            print("[OK] aux_global_position appears to be published")
            return True

        print("[FAIL] aux_global_position may exist but did not show expected fields")
        return False
    finally:
        try:
            mavport.close()
        except Exception:
            pass


# ============================================================
#  NMEA0183 UDP output check (GGA/RMC)
# ============================================================
def check_nmea0183_udp(port=NMEA0183_OUTPUT_PORT, timeout=6.0):
    """
    Listen on UDP port for NMEA0183 output.
    PASS if both GGA and RMC sentences are observed within timeout.
    """
    print(f"\n=== Checking NMEA0183 UDP output on port {port} (GGA/RMC) ===")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    sock.bind(("0.0.0.0", port))

    found_gga = False
    found_rmc = False
    end_time = time.time() + timeout

    try:
        while time.time() < end_time and not (found_gga and found_rmc):
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue

            text = data.decode("ascii", errors="ignore")
            matches = re.findall(r"\$..(GGA|RMC),[^\r\n]*", text)
            for sentence_type in matches:
                if sentence_type == "GGA" and not found_gga:
                    found_gga = True
                    print(f"[OK] GGA received from {addr}")
                elif sentence_type == "RMC" and not found_rmc:
                    found_rmc = True
                    print(f"[OK] RMC received from {addr}")

        if found_gga and found_rmc:
            return True

        missing = []
        if not found_gga:
            missing.append("GGA")
        if not found_rmc:
            missing.append("RMC")
        print(f"[FAIL] Missing NMEA0183 sentences: {', '.join(missing)}")
        return False
    finally:
        sock.close()


# ============================================================
#  MAIN
# ============================================================
def main():
    results = {
        "nmea2000_pgn": False,
        "udp_sent": False,
        "aux_global_position": False,
        "nmea0183_output": False,
    }

    # 1) Configure CAN/NMEA2000 output and reboot
    can_cfg_ok = enable_CAN()
    time.sleep(POST_REBOOT_WAIT_S)

    # 2) Check CAN PGN on PCAN
    if can_cfg_ok:
        results["nmea2000_pgn"] = check_can_pgn()
    else:
        results["nmea2000_pgn"] = False

    # 3) Send PAP messages over UDP (PAPPOS then PAPRPH)
    send_nmea_udp(ETH_IP, UDP_NMEA_PORT, [PAPPOS, PAPRPH], inter_msg_delay_s=0.1)
    results["udp_sent"] = True

    # Give driver time to parse/publish
    time.sleep(0.5)

    # 4) Check aux_global_position topic created
    results["aux_global_position"] = check_aux_global_position(timeout=6.0)

    # 5) Configure NMEA0183 output and check UDP port
    nmea_cfg_ok = enable_NM0183_NAV()
    time.sleep(POST_REBOOT_WAIT_S)
    if nmea_cfg_ok:
        results["nmea0183_output"] = check_nmea0183_udp(port=NMEA0183_OUTPUT_PORT, timeout=6.0)

    # Summary
    print("\n==================== SUMMARY ====================")
    print(f"CAN NMEA2000 PGN {CAN_PGN}: {'[OK]' if results['nmea2000_pgn'] else '[FAIL]'}")
    print(f"UDP PAPPOS/PAPRPH sent:     {'[OK]' if results['udp_sent'] else '[FAIL]'}")
    print(f"aux_global_position:        {'[OK]' if results['aux_global_position'] else '[FAIL]'}")
    print(f"NMEA0183 GGA/RMC output:     {'[OK]' if results['nmea0183_output'] else '[FAIL]'}")
    print("=================================================\n")


if __name__ == "__main__":
    main()
