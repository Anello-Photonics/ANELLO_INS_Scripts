#!/usr/bin/env python3
import time
import serial
import serial.tools.list_ports
from pymavlink import mavutil
from PCANBasic import *

# ---------- User Settings ----------
ETH_UDP_URI = "udp:0.0.0.0:14550"
ETH_BAUD = 57600
ETH_SYSID = 1
ETH_COMPID = 1

RS232_BAUD = 57600
RS232_TIMEOUT = 3.0  # seconds for ver hw response

CAN_PGN = 127257
SERIAL_PORTS = ["/dev/ttyS6", "/dev/ttyS5"]

COMMANDS = [
    "mavlink stop-all",
    "param set NMEA2000_CFG 1",
    f"param set N2K_{CAN_PGN}_RATE 10"
]

# ---------- Utility ----------
def print_info(msg):
    print(msg)

def send_shell_cmd(mav_udp, cmd):
    """Send a shell command to PX4 over MAVLink UDP"""
    print(f"  -> Sending: {cmd}")
    # PX4 expects a shell wakeup
    mav_udp.mav.command_long_send(
        ETH_SYSID,
        ETH_COMPID,
        511,  # MAV_CMD_DO_SEND_SCRIPT
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    cmd_bytes = [ord(c) for c in cmd + "\n"]
    while cmd_bytes:
        chunk = cmd_bytes[:70]
        chunk += [0] * (70 - len(chunk))
        mav_udp.mav.serial_control_send(
            0,
            mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE,
            0, 0, 70,
            chunk
        )
        cmd_bytes = cmd_bytes[70:]
    time.sleep(0.15)

# ---------- Ethernet MAVLink Check ----------
def check_ethernet():
    print("\n=== Checking Ethernet MAVLink (UDP) ===")
    try:
        mav_udp = mavutil.mavlink_connection(ETH_UDP_URI, autoreconnect=True)
        mav_udp.wait_heartbeat(timeout=5)
        print(f"[OK] Heartbeat received (sys={mav_udp.target_system}, comp={mav_udp.target_component})")
        return mav_udp, True
    except Exception as e:
        print(f"[FAIL] Ethernet heartbeat failed: {e}")
        return None, False

# ---------- Enable PX4 Serial MAVLink ----------
def enable_px4_serial(mav_udp):
    print("\n[ETHERNET] Enabling serial MAVLink and NMEA2000...")
    for cmd in COMMANDS:
        send_shell_cmd(mav_udp, cmd)

    # Start both RS232 ports
    for port in SERIAL_PORTS:
        start_cmd = f"mavlink start -d {port} -b {RS232_BAUD} -m 0 -f 1 -r 0"
        send_shell_cmd(mav_udp, start_cmd)

    print("[INFO] Waiting 3 seconds to initialize serial ports...")
    time.sleep(3.0)

# ---------- Detect COM Ports ----------
def detect_com_ports():
    ports = [port.device for port in serial.tools.list_ports.comports()]
    print(f"\nDetected COM ports: {ports}")
    return ports

# ---------- RS232 ver hw Check ----------
def check_rs232_verhw(com_ports):
    mapping = {}
    available_coms = com_ports.copy()
    for dev in SERIAL_PORTS:
        found = False
        for com in available_coms:
            try:
                with serial.Serial(com, baudrate=RS232_BAUD, timeout=RS232_TIMEOUT) as ser:
                    ser.write(b"ver hw\n")
                    time.sleep(0.1)
                    response = ser.read(ser.in_waiting or 64)
                    if response:
                        print(f"MAVLink responds on {com}: {response.strip().decode(errors='ignore')}")
                        mapping[dev] = com
                        available_coms.remove(com)  # avoid double-detecting same COM
                        found = True
                        break
            except Exception:
                continue
        if not found:
            print(f"MAVLink did not respond on any detected COM ports")
            mapping[dev] = None
    return mapping

# ---------- CAN Check ----------
def check_can_pgn(channel=PCAN_USBBUS1, timeout=5, retries=3):
    pcan = PCANBasic()
    ret = pcan.Initialize(channel, PCAN_BAUD_250K)
    if ret != PCAN_ERROR_OK:
        print("❌ PCAN initialization failed!")
        return False

    print(f"\nListening for PGN {CAN_PGN} (0x{CAN_PGN:X}) on {channel}...")
    for attempt in range(1, retries+1):
        start_time = time.time()
        while time.time() - start_time < timeout:
            result = pcan.Read(channel)
            if isinstance(result, tuple):
                ret_val, msg, *_ = result
            else:
                ret_val, msg = result, None

            if ret_val != PCAN_ERROR_OK or msg is None:
                time.sleep(0.001)
                continue

            can_id = msg.ID
            pgn = (can_id >> 8) & 0x3FFFF
            if pgn == CAN_PGN:
                print(f"✅ Received PGN {CAN_PGN} from source {can_id & 0xFF}")
                pcan.Uninitialize(channel)
                return True

        print(f"[Attempt {attempt}] No PGN {CAN_PGN} received in {timeout}s.")
        if attempt < retries:
            print("Retrying...")
            time.sleep(1)

    pcan.Uninitialize(channel)
    print(f"❌ Failed to detect PGN {CAN_PGN} after {retries} attempts.")
    return False

# ---------- Main ----------
def main():
    results = {"Ethernet": False, "RS232_ports": {}, "CAN": False}

    # Ethernet check
    mav_udp, eth_ok = check_ethernet()
    results["Ethernet"] = eth_ok

    # Enable PX4 serial
    if eth_ok:
        enable_px4_serial(mav_udp)

    # Detect COM ports and check ver hw
    com_ports = detect_com_ports()
    rs232_mapping = check_rs232_verhw(com_ports)
    results["RS232_ports"] = rs232_mapping

    # Check CAN PGN
    can_ok = check_can_pgn()
    results["CAN"] = can_ok

    # ---------- Summary ----------
    print("\n=== Summary of MAVLink Communication Checks ===")
    print(f"Ethernet/UDP: {'[OK]' if results['Ethernet'] else '[FAIL]'}")
    if results["RS232_ports"]:
        print("RS232 Ports:")
        for com in results["RS232_ports"].values():
            status = "[OK]" if com else "[FAIL]"
            print(f"  {com if com else 'None'} {status}")
    else:
        print("RS232 Ports: None detected")
    print(f"CAN: {'[OK]' if results['CAN'] else '[FAIL]'}")
    print("\n=== Check Complete ===")

if __name__ == "__main__":
    main()
