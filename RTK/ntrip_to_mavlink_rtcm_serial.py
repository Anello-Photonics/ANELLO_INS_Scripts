#!/usr/bin/env python3
"""Stream an NTRIP feed as MAVLink GPS_RTCM_DATA messages to serial."""

import argparse
import base64
import getpass
import json
from pathlib import Path
import socket
import sys
import time

import serial
from serial.tools import list_ports

try:
    from pymavlink.dialects.v20 import common as mavlink2
except ImportError:
    mavlink2 = None


GPS_RTCM_DATA_MSG_ID = 233
GPS_RTCM_DATA_CRC_EXTRA = 35
GPS_RTCM_DATA_FIELD_LEN = 180
GPS_RTCM_DATA_PAYLOAD_LEN = 182
MAX_FRAGMENTED_RTCM_LEN = GPS_RTCM_DATA_FIELD_LEN * 4
DEFAULT_CONFIG_PATH = Path(__file__).with_name("ntrip_config.json")
POSITION_MESSAGE_INTERVAL_US = 500000
POSITION_MESSAGE_IDS = {
    "GPS_RAW_INT": 24,
    "GLOBAL_POSITION_INT": 33,
    "GPS2_RAW": 124,
}


def prompt(label, default=None):
    """Prompt for a non-empty value, optionally displaying a default."""
    suffix = f" [{default}]" if default is not None else ""
    while True:
        value = input(f"{label}{suffix}: ").strip()
        if value:
            return value
        if default is not None:
            return str(default)
        print(f"{label} is required.")


def choose_serial_port():
    """Display detected serial ports and return the selected device name."""
    ports = list(list_ports.comports())
    if ports:
        print("\nAvailable serial ports:")
        for index, port in enumerate(ports, 1):
            description = port.description or "no description"
            print(f"  {index}. {port.device} ({description})")
        print("  M. Enter a port manually")

        while True:
            selection = input("Select the MAVLink serial port: ").strip()
            if selection.lower() == "m":
                break
            try:
                return ports[int(selection) - 1].device
            except (ValueError, IndexError):
                print("Enter a listed number or M.")
    else:
        print("No serial ports were detected.")

    return prompt("MAVLink serial port (for example COM3 or /dev/ttyUSB0)")


def load_config(path):
    """Load NTRIP settings from a JSON config file."""
    try:
        with path.open("r", encoding="utf-8") as config_file:
            config = json.load(config_file)
    except FileNotFoundError:
        if path == DEFAULT_CONFIG_PATH:
            return {}
        raise ValueError(f"config file not found: {path}") from None
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid JSON in {path}: {error}") from error

    if not isinstance(config, dict):
        raise ValueError(f"{path} must contain a JSON object")

    ntrip_config = config.get("ntrip", config)
    if not isinstance(ntrip_config, dict):
        raise ValueError(f"{path} field 'ntrip' must be a JSON object")
    return ntrip_config


def config_text(config, key, strip=True):
    """Return a non-empty text value from config, or None when it is absent."""
    value = config.get(key)
    if value is None:
        return None
    if not isinstance(value, str):
        value = str(value)
    if strip:
        value = value.strip()
    return value or None


def config_int(config, key, default):
    """Return an integer config value with a default for missing/blank values."""
    value = config.get(key, default)
    if value in (None, ""):
        return default
    try:
        value = int(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"config field '{key}' must be an integer") from error
    return value


def build_request(mountpoint, username, password):
    """Build an NTRIP v1 request using Basic authentication."""
    credentials = base64.b64encode(f"{username}:{password}".encode("utf-8"))
    mountpoint = mountpoint.lstrip("/")
    return (
        f"GET /{mountpoint} HTTP/1.0\r\n"
        "User-Agent: NTRIP ANELLO MAVLink RTCM Client\r\n"
        f"Authorization: Basic {credentials.decode('ascii')}\r\n"
        "\r\n"
    ).encode("ascii")


def build_gga(latitude, longitude, altitude=0.0):
    """Build a current GGA sentence for a fixed decimal-degree position."""
    if not -90 <= latitude <= 90:
        raise ValueError("latitude must be between -90 and 90 degrees")
    if not -180 <= longitude <= 180:
        raise ValueError("longitude must be between -180 and 180 degrees")

    utc = time.strftime("%H%M%S.00", time.gmtime())

    def nmea_coordinate(value, degree_width, positive, negative):
        hemisphere = positive if value >= 0 else negative
        absolute = abs(value)
        degrees = int(absolute)
        minutes = (absolute - degrees) * 60
        return f"{degrees:0{degree_width}d}{minutes:08.5f}", hemisphere

    lat, north_south = nmea_coordinate(latitude, 2, "N", "S")
    lon, east_west = nmea_coordinate(longitude, 3, "E", "W")
    payload = (
        f"GNGGA,{utc},{lat},{north_south},{lon},{east_west},"
        f"1,12,1.00,{altitude:.2f},M,,M,,"
    )
    checksum = 0
    for character in payload:
        checksum ^= ord(character)
    return f"${payload}*{checksum:02X}\r\n".encode("ascii")


def read_response_header(connection):
    """Read the caster response header and preserve correction bytes after it."""
    response = bytearray()
    while b"\r\n" not in response:
        block = connection.recv(4096)
        if not block:
            raise ConnectionError("caster closed the connection before responding")
        response.extend(block)
        if len(response) > 65536:
            raise ConnectionError("caster response header is unexpectedly large")

    status, remainder = bytes(response).split(b"\r\n", 1)
    if b"200 OK" not in status:
        raise ConnectionError(
            f"caster rejected the request: {status.decode('ascii', errors='replace')}"
        )

    # NTRIP v1 casters commonly send only ``ICY 200 OK\r\n`` before RTCM.
    if status.startswith(b"ICY"):
        return remainder

    # HTTP/NTRIP v2 responses have normal headers terminated by a blank line.
    while b"\r\n\r\n" not in bytes(response):
        block = connection.recv(4096)
        if not block:
            raise ConnectionError("caster closed the connection before responding")
        response.extend(block)
        if len(response) > 65536:
            raise ConnectionError("caster response header is unexpectedly large")
    return bytes(response).split(b"\r\n\r\n", 1)[1]


def x25_crc(data):
    """Return the MAVLink X.25 CRC for bytes already in wire order."""
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


class MavlinkRtcmPacker:
    """Build MAVLink GPS_RTCM_DATA frames for RTCM correction bytes."""

    def __init__(
        self,
        system_id=255,
        component_id=190,
        mavlink_version=2,
        initial_sequence=0,
    ):
        if not 0 <= system_id <= 255:
            raise ValueError("source system id must be between 0 and 255")
        if not 0 <= component_id <= 255:
            raise ValueError("source component id must be between 0 and 255")
        if mavlink_version not in (1, 2):
            raise ValueError("MAVLink version must be 1 or 2")

        self.system_id = system_id
        self.component_id = component_id
        self.mavlink_version = mavlink_version
        self.mavlink_sequence = initial_sequence & 0xFF
        self.rtcm_sequence = 0

    def pack_rtcm_blob(self, data):
        """Return MAVLink frames carrying one RTCM blob."""
        if not data:
            return []

        if len(data) > MAX_FRAGMENTED_RTCM_LEN:
            frames = []
            for offset in range(0, len(data), GPS_RTCM_DATA_FIELD_LEN):
                chunk = data[offset : offset + GPS_RTCM_DATA_FIELD_LEN]
                flags = (self.rtcm_sequence & 0x1F) << 3
                frames.append(self._pack_gps_rtcm_data(flags, chunk))
                self.rtcm_sequence = (self.rtcm_sequence + 1) & 0x1F
            return frames

        rtcm_sequence = self.rtcm_sequence
        self.rtcm_sequence = (self.rtcm_sequence + 1) & 0x1F

        if len(data) <= GPS_RTCM_DATA_FIELD_LEN:
            flags = (rtcm_sequence & 0x1F) << 3
            return [self._pack_gps_rtcm_data(flags, data)]

        frames = []
        fragment_id = 0
        for offset in range(0, len(data), GPS_RTCM_DATA_FIELD_LEN):
            chunk = data[offset : offset + GPS_RTCM_DATA_FIELD_LEN]
            flags = 0x01 | (fragment_id << 1) | ((rtcm_sequence & 0x1F) << 3)
            frames.append(self._pack_gps_rtcm_data(flags, chunk))
            fragment_id += 1

        if len(data) % GPS_RTCM_DATA_FIELD_LEN == 0 and fragment_id < 4:
            flags = 0x01 | (fragment_id << 1) | ((rtcm_sequence & 0x1F) << 3)
            frames.append(self._pack_gps_rtcm_data(flags, b""))

        return frames

    def _pack_gps_rtcm_data(self, flags, data):
        if len(data) > GPS_RTCM_DATA_FIELD_LEN:
            raise ValueError("GPS_RTCM_DATA payload cannot exceed 180 RTCM bytes")

        payload = bytes([flags, len(data)]) + data.ljust(GPS_RTCM_DATA_FIELD_LEN, b"\0")
        packet_sequence = self.mavlink_sequence
        self.mavlink_sequence = (self.mavlink_sequence + 1) & 0xFF

        if self.mavlink_version == 1:
            header = bytes(
                [
                    GPS_RTCM_DATA_PAYLOAD_LEN,
                    packet_sequence,
                    self.system_id,
                    self.component_id,
                    GPS_RTCM_DATA_MSG_ID,
                ]
            )
            checksum = x25_crc(header + payload + bytes([GPS_RTCM_DATA_CRC_EXTRA]))
            return b"\xFE" + header + payload + checksum.to_bytes(2, "little")

        header = bytes(
            [
                GPS_RTCM_DATA_PAYLOAD_LEN,
                0,
                0,
                packet_sequence,
                self.system_id,
                self.component_id,
            ]
        ) + GPS_RTCM_DATA_MSG_ID.to_bytes(3, "little")
        checksum = x25_crc(header + payload + bytes([GPS_RTCM_DATA_CRC_EXTRA]))
        return b"\xFD" + header + payload + checksum.to_bytes(2, "little")


def send_rtcm_blob(serial_output, packer, data):
    """Package one correction blob and write its MAVLink frames to serial."""
    frames = packer.pack_rtcm_blob(data)
    for frame in frames:
        serial_output.write(frame)
    print(
        f"RTCM {len(data)} bytes -> {len(frames)} GPS_RTCM_DATA MAVLink frame(s)",
        flush=True,
    )


def require_pymavlink():
    if mavlink2 is not None:
        return
    raise RuntimeError(
        "Missing dependency: pymavlink\n"
        "Install it with: python -m pip install pymavlink"
    )


def request_position_messages(mavlink_output, target_system, target_component):
    """Ask the MAVLink device to stream position messages, when supported."""
    command_id = getattr(mavlink2, "MAV_CMD_SET_MESSAGE_INTERVAL", None)
    if command_id is None:
        return

    for message_id in POSITION_MESSAGE_IDS.values():
        mavlink_output.command_long_send(
            target_system,
            target_component,
            command_id,
            0,
            message_id,
            POSITION_MESSAGE_INTERVAL_US,
            0,
            0,
            0,
            0,
            0,
        )


def position_from_mavlink_message(message):
    """Extract decimal-degree latitude/longitude from a MAVLink position message."""
    message_type = message.get_type()
    if message_type == "GLOBAL_POSITION_INT":
        latitude = message.lat / 10000000.0
        longitude = message.lon / 10000000.0
    elif message_type in ("GPS_RAW_INT", "GPS2_RAW"):
        if getattr(message, "fix_type", 0) < 2:
            return None
        latitude = message.lat / 10000000.0
        longitude = message.lon / 10000000.0
    else:
        return None

    if -90 <= latitude <= 90 and -180 <= longitude <= 180:
        return latitude, longitude
    return None


def read_mavlink_position(
    serial_connection,
    timeout,
    source_system,
    source_component,
):
    """Listen on the serial MAVLink connection until a usable position arrives."""
    require_pymavlink()

    parser = mavlink2.MAVLink(None)
    parser.robust_parsing = True
    mavlink_output = mavlink2.MAVLink(
        serial_connection,
        srcSystem=source_system,
        srcComponent=source_component,
    )
    old_timeout = serial_connection.timeout
    deadline = time.monotonic() + timeout
    requested_stream = False
    last_heartbeat = 0.0

    print(f"Waiting up to {timeout:.1f}s for MAVLink latitude/longitude...")
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_heartbeat >= 1.0:
                mavlink_output.heartbeat_send(
                    mavlink2.MAV_TYPE_GCS,
                    mavlink2.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
                last_heartbeat = now

            serial_connection.timeout = min(0.25, max(0.0, deadline - now))
            data = serial_connection.read(serial_connection.in_waiting or 1)
            if not data:
                continue

            for byte in data:
                try:
                    message = parser.parse_char(bytes([byte]))
                except mavlink2.MAVError:
                    continue
                if message is None or message.get_type() == "BAD_DATA":
                    continue

                if message.get_type() == "HEARTBEAT" and not requested_stream:
                    request_position_messages(
                        mavlink_output,
                        message.get_srcSystem(),
                        message.get_srcComponent(),
                    )
                    requested_stream = True
                    continue

                position = position_from_mavlink_message(message)
                if position is not None:
                    latitude, longitude = position
                    print(
                        f"Using MAVLink {message.get_type()} position: "
                        f"{latitude:.7f}, {longitude:.7f}"
                    )
                    return latitude, longitude, mavlink_output.seq
    finally:
        serial_connection.timeout = old_timeout

    raise TimeoutError("timed out waiting for MAVLink latitude/longitude")


def stream(
    caster,
    caster_port,
    mountpoint,
    username,
    password,
    serial_output,
    serial_port,
    latitude,
    longitude,
    altitude,
    gga_interval,
    source_system,
    source_component,
    mavlink_version,
    initial_sequence=0,
):
    """Connect to both endpoints and copy NTRIP corrections as MAVLink frames."""
    request = build_request(mountpoint, username, password)
    packer = MavlinkRtcmPacker(
        source_system,
        source_component,
        mavlink_version,
        initial_sequence,
    )

    with socket.create_connection((caster, caster_port), timeout=10) as source:
        source.sendall(request)
        initial_data = read_response_header(source)
        source.settimeout(min(gga_interval, 1.0))
        print(
            f"Streaming {caster}:{caster_port}/{mountpoint.lstrip('/')} "
            f"to {serial_port} at {serial_output.baudrate} baud as MAVLink "
            f"{mavlink_version} GPS_RTCM_DATA. Press Ctrl+C to stop."
        )

        if initial_data:
            send_rtcm_blob(serial_output, packer, initial_data)

        source.sendall(build_gga(latitude, longitude, altitude))
        last_gga = time.monotonic()
        while True:
            try:
                data = source.recv(MAX_FRAGMENTED_RTCM_LEN)
                if not data:
                    raise ConnectionError("caster closed the correction stream")
                send_rtcm_blob(serial_output, packer, data)
            except socket.timeout:
                pass

            if time.monotonic() - last_gga >= gga_interval:
                source.sendall(build_gga(latitude, longitude, altitude))
                last_gga = time.monotonic()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help=f"JSON config file for NTRIP settings (default: {DEFAULT_CONFIG_PATH})",
    )
    parser.add_argument("--caster", help="NTRIP caster hostname or IP")
    parser.add_argument("--caster-port", type=int, help="NTRIP caster port (default: 2101)")
    parser.add_argument("--mountpoint", help="NTRIP mountpoint")
    parser.add_argument("--username", help="NTRIP username")
    parser.add_argument("--password", help="NTRIP password; prompted when omitted")
    parser.add_argument("--serial-port", help="serial port receiving MAVLink frames")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--latitude", type=float, help="rover latitude in decimal degrees")
    parser.add_argument("--longitude", type=float, help="rover longitude in decimal degrees")
    parser.add_argument("--altitude", type=float, default=0.0, help="MSL altitude in metres")
    parser.add_argument(
        "--gga-interval",
        type=float,
        default=10.0,
        help="seconds between GGA messages (default: 10)",
    )
    parser.add_argument(
        "--position-timeout",
        type=float,
        default=15.0,
        help="seconds to wait for MAVLink latitude/longitude before prompting (default: 15)",
    )
    parser.add_argument(
        "--source-system",
        type=int,
        default=255,
        help="MAVLink source system id (default: 255)",
    )
    parser.add_argument(
        "--source-component",
        type=int,
        default=190,
        help="MAVLink source component id (default: 190)",
    )
    parser.add_argument(
        "--mavlink-version",
        type=int,
        choices=(1, 2),
        default=2,
        help="MAVLink wire version for outgoing frames (default: 2)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        config = load_config(args.config)
    except ValueError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    caster = args.caster or config_text(config, "caster") or prompt("Caster hostname or IP")
    try:
        caster_port = (
            args.caster_port
            if args.caster_port is not None
            else config_int(config, "caster_port", 2101)
        )
    except ValueError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    mountpoint = args.mountpoint or config_text(config, "mountpoint") or prompt("Mountpoint")
    username = args.username or config_text(config, "username") or prompt("Username")
    password = args.password if args.password is not None else config_text(config, "password", strip=False)
    if password is None:
        password = getpass.getpass("Password: ")
    serial_port = args.serial_port or choose_serial_port()

    try:
        if not 0 < caster_port <= 65535:
            raise ValueError("caster port must be between 1 and 65535")
        if args.gga_interval <= 0:
            raise ValueError("GGA interval must be greater than zero")
        if args.position_timeout <= 0:
            raise ValueError("position timeout must be greater than zero")
        if not 0 <= args.source_system <= 255:
            raise ValueError("source system id must be between 0 and 255")
        if not 0 <= args.source_component <= 255:
            raise ValueError("source component id must be between 0 and 255")

        latitude = args.latitude
        longitude = args.longitude
        mavlink_sequence = 0

        with serial.Serial(serial_port, baudrate=args.baud, timeout=1) as serial_connection:
            if latitude is None or longitude is None:
                try:
                    mav_latitude, mav_longitude, mavlink_sequence = read_mavlink_position(
                        serial_connection,
                        args.position_timeout,
                        args.source_system,
                        args.source_component,
                    )
                    if latitude is None:
                        latitude = mav_latitude
                    if longitude is None:
                        longitude = mav_longitude
                except TimeoutError as error:
                    print(f"{error}; falling back to manual entry.")

            if latitude is None:
                latitude = float(prompt("Rover latitude (decimal degrees)"))
            if longitude is None:
                longitude = float(prompt("Rover longitude (decimal degrees)"))

            stream(
                caster,
                caster_port,
                mountpoint,
                username,
                password,
                serial_connection,
                serial_port,
                latitude,
                longitude,
                args.altitude,
                args.gga_interval,
                args.source_system,
                args.source_component,
                args.mavlink_version,
                mavlink_sequence,
            )
    except KeyboardInterrupt:
        print("\nStopped.")
    except (ConnectionError, OSError, RuntimeError, ValueError, serial.SerialException) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
