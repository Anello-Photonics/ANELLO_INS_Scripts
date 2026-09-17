#!/usr/bin/env python3
"""Reset PX4 parameters while preserving MB calibration, then configure the INS."""

import argparse
import sys
import time

PARAMETER_OVERRIDES = (
    ("IMU_MB_C_FACTORY", 0),
    ("IMU_MB_C_FTOG", 1),
    ("SYS_AUTOSTART", 60009),
    ("NM2K_CFG", 1),
    ("J1939_CFG", 0),
    ("NM2K_BITRATE", 250000),
    ("NM2K_127257_RATE", 10),
    ("CAN_TERM", 1),
    ("NM0183_CFG", 2),
    ("MAV_0_CONFIG", 101),
    ("SER_TEL1_BAUD", 57600),
    ("SER_TEL2_BAUD", 921600),
    ("GPS_SEP_BASE_X", 0.0),
    ("GPS_SEP_BASE_Y", 0.0),
    ("GPS_SEP_BASE_Z", 0.0),
    ("GPS_SEP_ROVER_X", 0.0),
    ("GPS_SEP_ROVER_Y", 0.0),
    ("GPS_SEP_ROVER_Z", 0.0),
    ("GPS_EXT_X", 0.0),
    ("GPS_EXT_Y", 0.0),
    ("GPS_EXT_Z", 0.0),
    ("EKF2_IMU_POS_X", 0.0),
    ("EKF2_IMU_POS_Y", 0.0),
    ("EKF2_IMU_POS_Z", 0.0),
    ("EKF2_WTSPD_POS_X", 0.0),
    ("EKF2_WTSPD_POS_Y", 0.0),
    ("EKF2_WTSPD_POS_Z", 0.0),
    ("SENS_BOARD_ROT", 0),
    ("NMUDP_EN", 1),
    ("NMUDP_ODR_GGA", 0.0),
    ("NMUDP_ODR_RMC", 0.0),
    ("NMUDP_ODR_APIMU", 10.0),
    ("NMUDP_ODR_APINS", 0.0),
    ("NMUDP_ODR_APACC", 0.0),
    ("NMUDP_ODR_HDT", 0.0),
    ("NMUDP_ODR_XDR", 0.0),
    ("NMUDP_ODR_ZDA", 0.0),
    ("NM0183_ODR_GGA", 0.0),
    ("NM0183_ODR_RMC", 0.0),
    ("NM0183_ODR_APIMU", 10.0),
    ("NM0183_ODR_APINS", 0.0),
    ("NM0183_ODR_APACC", 0.0),
    ("NM0183_ODR_HDT", 0.0),
    ("NM0183_ODR_XDR", 0.0),
    ("NM0183_ODR_ZDA", 0.0),
    ("NMUDP_MC_IP0", 0),
    ("NMUDP_MC_IP1", 0),
    ("NMUDP_MC_IP2", 0),
    ("NMUDP_MC_IP3", 0),
    ("NMUDP_UC_IP0", 0),
    ("NMUDP_UC_IP1", 0),
    ("NMUDP_UC_IP2", 0),
    ("NMUDP_UC_IP3", 0),
    ("MAV_2_BROADCAST", 1),
    ("MAV_2_CONFIG", 1000),
    ("EKF2_WTSPD_SF0", 0.0),
    ("EKF2_WTSPD_SF1", 1.0),
    ("EKF2_WTSPD_SF2", 0.0),
    ("EKF2_WTSPD_DBN", -1.0),
    ("EKF2_ENG_CTRL", 0.0),
    ("EKF2_ENG_STAT_L", 0.0),
    ("EKF2_ENG_STAT_H", 0.0),
    ("SDLOG_DIRS_MAX", 200),
    ("SDLOG_FILES_MAX", 200),
)

ALLOWED_MB_OVERRIDES = {"IMU_MB_C_FACTORY", "IMU_MB_C_FTOG"}


class MavlinkSerialPort:
    """Minimal serial-like wrapper around MAVLink SERIAL_CONTROL messages."""

    def __init__(self, mavutil, connection, baud, devnum, debug=0):
        self._mavutil = mavutil
        self._debug = debug
        self._buffer = ""
        self._devnum = devnum
        print(f"Connecting with MAVLink to {connection} ...")
        self.mav = mavutil.mavlink_connection(
            connection, autoreconnect=True, baud=baud
        )
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        self.mav.wait_heartbeat()
        print("HEARTBEAT OK")

    def write(self, data):
        if self._debug >= 2:
            print(f"sending {data!r}")
        while data:
            chunk, data = data[:70], data[70:]
            payload = [ord(character) for character in chunk]
            payload.extend([0] * (70 - len(payload)))
            self.mav.mav.serial_control_send(
                self._devnum,
                self._mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
                | self._mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                0,
                0,
                len(chunk),
                payload,
            )

    def receive(self):
        message = self.mav.recv_match(
            condition="SERIAL_CONTROL.count!=0",
            type="SERIAL_CONTROL",
            blocking=True,
            timeout=0.03,
        )
        if message is not None:
            self._buffer += "".join(
                chr(value) for value in message.data[: message.count]
            )

    def read(self, size):
        result = self._buffer[:size]
        self._buffer = self._buffer[size:]
        return result

    def close(self):
        self.mav.mav.serial_control_send(
            self._devnum, 0, 0, 0, 0, [0] * 70
        )


def run_command(port, command, timeout):
    """Send one shell command and print any response received."""
    print(f"\n[Setting] {command}")
    port.write("\n")
    time.sleep(0.1)
    port.write(command + "\n")
    time.sleep(0.1)

    output = ""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        port.receive()
        output += port.read(1024)
        time.sleep(0.05)

    if output.strip():
        print(output.strip())
    else:
        print("[!] No response received.")


def commands(reboot=False):
    """Return shell commands, protecting all MB calibration parameters first."""
    unexpected = {
        name
        for name, _ in PARAMETER_OVERRIDES
        if name.startswith("IMU_MB_C_") and name not in ALLOWED_MB_OVERRIDES
    }
    if unexpected:
        raise ValueError(f"unexpected MB calibration overrides: {sorted(unexpected)}")

    result = ["param reset_all IMU_MB_C_*"]
    result.extend(f"param set {name} {value}" for name, value in PARAMETER_OVERRIDES)
    result.append("param save")
    if reboot:
        result.append("reboot")
    return result


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Reset all parameters except IMU_MB_C_*, apply the requested "
            "configuration, and save it."
        )
    )
    parser.add_argument(
        "--connection",
        default="udp:0.0.0.0:14550",
        help="pymavlink connection (default: %(default)s)",
    )
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--devnum", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--debug", type=int, default=0)
    parser.add_argument(
        "--reboot",
        action="store_true",
        help="reboot after saving (required for some settings to take effect)",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="print commands without connecting"
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    command_list = commands(args.reboot)

    if args.dry_run:
        print("\n".join(command_list))
        return 0

    try:
        from pymavlink import mavutil
    except ImportError as error:
        print(f"Failed to import pymavlink: {error}", file=sys.stderr)
        print("Install it with: pip3 install --user pymavlink", file=sys.stderr)
        return 1

    port = MavlinkSerialPort(
        mavutil, args.connection, args.baud, args.devnum, debug=args.debug
    )
    try:
        time.sleep(0.5)
        for command in command_list:
            run_command(port, command, args.timeout)
    finally:
        time.sleep(0.5)
        port.close()

    print("\n[Done] Parameter reset and configuration attempted.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
