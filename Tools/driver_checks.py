
import time
import socket
import subprocess  # kept in case you want later use
import struct
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
NMEA0183_LAT = 37.335390
NMEA0183_LON = -122.026130
PAPPOS = f"PAPPOS,,{NMEA0183_LAT:.6f},{NMEA0183_LON:.6f},12.30,2.0,3.5"
PAPRPH = "PAPRPH,203600.00,0.80,1.50,273.20,0.20,0.20,0.50"

# CAN / NMEA2000
CAN_PGN = 127257
AUX_POS_PGN = 130816
AUX_ATT_PGN = 130817
AUX_POS_LAT = 37.335490
AUX_POS_LON = -122.026330
PCAN_CHANNEL = PCAN_USBBUS1
PCAN_BITRATE = PCAN_BAUD_250K
N2K_PRIORITY = 3
N2K_SOURCE_ADDRESS = 0x80
N2K_INTERFRAME_DELAY_S = 0.01

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
        return True
    finally:
        try:
            mavport.close()
        except Exception:
            pass


# ============================================================
        mavport = MavlinkSerialPort(
            MAVLINK_SHELL,
            MAVLINK_BAUD,
            devnum=MAV_DEVNUM,
            debug=SHELL_DEBUG,
        )
# ============================================================
def nmea_checksum(sentence: str) -> str:
    csum = 0
        # NOTE: Listener output can be minimal; accept topic presence.
        out = run_shell_command(mavport, "listener aux_global_position 1", timeout=timeout)
        if "timestamp" in low or "lat" in low or "lon" in low or "aux_global_position" in low:
        if out.strip():
            print("[WARN] aux_global_position output did not include expected fields")
            return True

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
#  NMEA2000 fast-packet helpers (aux position/attitude)
# ============================================================
def build_n2k_can_id(pgn, source=N2K_SOURCE_ADDRESS, priority=N2K_PRIORITY):
    return ((priority & 0x7) << 26) | ((pgn & 0x3FFFF) << 8) | (source & 0xFF)


def build_fast_packet_frames(payload, sequence=0):
    total_len = len(payload)
    frames = []
    frame_index = 0
    offset = 0

    first_chunk = payload[:6]
    offset = 6
    header = bytes([(sequence << 5) | frame_index, total_len])
    frames.append((header + first_chunk).ljust(8, b"\xFF"))
    frame_index += 1

    while offset < total_len:
        chunk = payload[offset:offset + 7]
        offset += 7
        header = bytes([(sequence << 5) | frame_index])
        frames.append((header + chunk).ljust(8, b"\xFF"))
        frame_index += 1

    return frames


def send_nmea2000_fast_packet(pcan, channel, pgn, payload, sequence=0):
    can_id = build_n2k_can_id(pgn)
    frames = build_fast_packet_frames(payload, sequence=sequence)
    for frame in frames:
        msg = TPCANMsg()
        msg.ID = can_id
        msg.MSGTYPE = PCAN_MESSAGE_EXTENDED
        msg.LEN = 8
        for i in range(8):
            msg.DATA[i] = frame[i]
        result = pcan.Write(channel, msg)
        if result != PCAN_ERROR_OK:
            return False
        time.sleep(N2K_INTERFRAME_DELAY_S)
    return True


def send_nmea2000_aux_messages(channel=PCAN_CHANNEL):
    print("\n=== Sending NMEA2000 Auxiliary Position/Attitude ===")
    pcan = PCANBasic()
    ret = pcan.Initialize(channel, PCAN_BITRATE)
    if ret != PCAN_ERROR_OK:
        print("[FAIL] PCAN initialization failed for NMEA2000 aux messages.")
        return False

    try:
        lat = int(round(AUX_POS_LAT * 1e7))
        lon = int(round(AUX_POS_LON * 1e7))
        alt = 12
        hacc = 2
        vacc = 3
        pos_payload = struct.pack("<iiiii", lat, lon, alt, hacc, vacc)

        roll = 1
        pitch = 2
        heading = 273
        roll_acc = 1
        pitch_acc = 1
        head_acc = 1
        att_payload = struct.pack("<iiiiii", roll, pitch, heading, roll_acc, pitch_acc, head_acc)

        if not send_nmea2000_fast_packet(pcan, channel, AUX_POS_PGN, pos_payload, sequence=0):
            print("[FAIL] Failed to send AUX POS fast packet.")
            return False

        if not send_nmea2000_fast_packet(pcan, channel, AUX_ATT_PGN, att_payload, sequence=1):
            print("[FAIL] Failed to send AUX ATT fast packet.")
            return False

        print("[OK] NMEA2000 AUX POS/ATT packets sent.")
        return True
    finally:
        pcan.Uninitialize(channel)


# ============================================================
#  aux_global_position check (injected)
# ============================================================
def check_aux_global_position(timeout=6.0, expected_lat=None, expected_lon=None, tolerance=1e-6):
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
        out = run_shell_command(mavport, "listener aux_global_position 1", timeout=timeout)
        low = out.lower()
        if "not found" in low or "unknown uorb topic" in low:
            print("[FAIL] aux_global_position topic not created")
            return False

        if "topic: aux_global_position" in low or "aux_global_position" in low:
            lat_match = re.search(r"lat:\s*([-\d.]+)", low)
            lon_match = re.search(r"lon:\s*([-\d.]+)", low)
            if not lat_match or not lon_match:
                print("[FAIL] aux_global_position published but missing lat/lon values")
                return False

            lat_val = float(lat_match.group(1))
            lon_val = float(lon_match.group(1))

            if expected_lat is not None and expected_lon is not None:
                lat_ok = abs(lat_val - expected_lat) <= tolerance
                lon_ok = abs(lon_val - expected_lon) <= tolerance
                if not (lat_ok and lon_ok):
                    print(
                        "[FAIL] aux_global_position lat/lon did not match expected values "
                        f"(lat {lat_val:.6f}, lon {lon_val:.6f})"
                    )
                    return False

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
def check_nmea0183_udp(port=NMEA0183_OUTPUT_PORT, timeout=10.0):
    """
    Listen on UDP port for NMEA0183 output.
    PASS if both GGA and RMC sentences are observed within timeout.
    """
    print(f"\n=== Checking NMEA0183 UDP output on port {port} (GGA/RMC) ===")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.settimeout(0.5)
    try:
        sock.bind(("0.0.0.0", port))
    except OSError as exc:
        print(f"[FAIL] Could not bind UDP port {port}: {exc}")
        sock.close()
        return False

    found_gga = False
    found_rmc = False
    end_time = time.time() + timeout

    try:
        while time.time() < end_time and not (found_gga and found_rmc):
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue

            for match in re.finditer(rb"\$..(GGA|RMC),", data):
                sentence_type = match.group(1).decode("ascii", errors="ignore")
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
        "nmea2000_aux_position": False,
        "udp_sent": False,
        "aux_global_position_nmea2000": False,
        "aux_global_position_nmea0183": False,
        "nmea0183_output": False,
    }

    # 1) Configure outputs, then reboot once
    can_cfg_ok = enable_CAN()
    nmea_cfg_ok = enable_NM0183_NAV()
    if can_cfg_ok or nmea_cfg_ok:
        try:
            mavport = MavlinkSerialPort(MAVLINK_SHELL, MAVLINK_BAUD, devnum=MAV_DEVNUM, debug=SHELL_DEBUG)
        except Exception as e:
            print(f"[FAIL] Could not open MAVLink shell to reboot: {e}")
        else:
            try:
                run_shell_command(mavport, "reboot", timeout=6)
            finally:
                try:
                    mavport.close()
                except Exception:
                    pass
        time.sleep(POST_REBOOT_WAIT_S)

    # 2) Check CAN PGN on PCAN
    if can_cfg_ok:
        results["nmea2000_pgn"] = check_can_pgn()
    else:
        results["nmea2000_pgn"] = False

    # 3) Send NMEA2000 auxiliary position/attitude over CAN
    if can_cfg_ok:
        results["nmea2000_aux_position"] = send_nmea2000_aux_messages()
        if results["nmea2000_aux_position"]:
            time.sleep(0.5)
            results["aux_global_position_nmea2000"] = check_aux_global_position(
                timeout=6.0,
                expected_lat=AUX_POS_LAT,
                expected_lon=AUX_POS_LON,
                tolerance=1e-5,
            )

    # 4) Send PAP messages over UDP (PAPPOS then PAPRPH)
    send_nmea_udp(ETH_IP, UDP_NMEA_PORT, [PAPPOS, PAPRPH], inter_msg_delay_s=0.1)
    results["udp_sent"] = True

    # Give driver time to parse/publish
    time.sleep(0.5)

    # 5) Check aux_global_position topic created from NMEA0183 input
    results["aux_global_position_nmea0183"] = check_aux_global_position(
        timeout=6.0,
        expected_lat=NMEA0183_LAT,
        expected_lon=NMEA0183_LON,
        tolerance=1e-5,
    )

    # 6) Check NMEA0183 output on UDP port
    if nmea_cfg_ok:
        results["nmea0183_output"] = check_nmea0183_udp(port=NMEA0183_OUTPUT_PORT, timeout=30.0)

    # Summary
    print("\n==================== SUMMARY ====================")
    print(f"CAN NMEA2000 PGN {CAN_PGN}: {'[OK]' if results['nmea2000_pgn'] else '[FAIL]'}")
    print(f"NMEA2000 AUX POS/ATT sent:  {'[OK]' if results['nmea2000_aux_position'] else '[FAIL]'}")
    print(f"UDP PAPPOS/PAPRPH sent:     {'[OK]' if results['udp_sent'] else '[FAIL]'}")
    print(f"aux_global_position (N2K):  {'[OK]' if results['aux_global_position_nmea2000'] else '[FAIL]'}")
    print(f"aux_global_position (0183): {'[OK]' if results['aux_global_position_nmea0183'] else '[FAIL]'}")
    print(f"NMEA0183 GGA/RMC output:     {'[OK]' if results['nmea0183_output'] else '[FAIL]'}")
    print("=================================================\n")


if __name__ == "__main__":
    main()
