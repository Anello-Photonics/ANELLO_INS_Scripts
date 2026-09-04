#!/usr/bin/env python3
"""Stream an NTRIP feed as MAVLink GPS_RTCM_DATA messages."""

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
DEFAULT_NTRIP_PORT = 2101
DEFAULT_OUTPUT_TYPE = "serial"
DEFAULT_BAUD = 921600
DEFAULT_ETHERNET_PORT = 14550
DEFAULT_ETHERNET_LOCAL_IP = "0.0.0.0"
DEFAULT_GGA_INTERVAL = 10.0
DEFAULT_POSITION_TIMEOUT = 15.0
DEFAULT_SOURCE_SYSTEM = 255
DEFAULT_SOURCE_COMPONENT = 190
DEFAULT_MAVLINK_VERSION = 2
DEFAULT_ALTITUDE = 0.0
POSITION_MESSAGE_INTERVAL_US = 500000
PREFERRED_POSITION_MESSAGE = "GPS2_RAW"
POSITION_MESSAGE_IDS = (
    ("GPS2_RAW", 124),
    ("GPS_RAW_INT", 24),
    ("GLOBAL_POSITION_INT", 33),
)


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
    """Load settings from a JSON config file."""
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

    return config


def config_section(config, section):
    """Return a named JSON object section, or an empty dict when omitted."""
    section_config = config.get(section, {})
    if section_config in (None, ""):
        return {}
    if not isinstance(section_config, dict):
        raise ValueError(f"config field '{section}' must be a JSON object")
    return section_config


def ntrip_config_section(config):
    """Return NTRIP config, preserving compatibility with old flat configs."""
    if "ntrip" in config:
        return config_section(config, "ntrip")
    return config


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


def config_optional_int(config, key):
    """Return an optional integer config value."""
    if key not in config or config.get(key) in (None, ""):
        return None
    return config_int(config, key, 0)


def config_float(config, key, default):
    """Return a float config value with a default for missing/blank values."""
    value = config.get(key, default)
    if value in (None, ""):
        return default
    try:
        value = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"config field '{key}' must be a number") from error
    return value


def config_optional_float(config, key):
    """Return an optional float config value."""
    if key not in config or config.get(key) in (None, ""):
        return None
    return config_float(config, key, 0.0)


def validate_udp_port(label, value):
    if not 0 < value <= 65535:
        raise ValueError(f"{label} must be between 1 and 65535")


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


def send_rtcm_blob(mavlink_output, packer, data):
    """Package one correction blob and write its MAVLink frames."""
    frames = packer.pack_rtcm_blob(data)
    for frame in frames:
        mavlink_output.write(frame)
    print(
        f"RTCM {len(data)} bytes -> {len(frames)} GPS_RTCM_DATA MAVLink frame(s)",
        flush=True,
    )


class SerialMavlinkEndpoint:
    """File-like MAVLink endpoint backed by a serial port."""

    def __init__(self, port, baud, timeout=1):
        self.port = port
        self._serial = serial.Serial(port, baudrate=baud, timeout=timeout)
        self.description = f"serial {port} at {baud} baud"

    @property
    def timeout(self):
        return self._serial.timeout

    @timeout.setter
    def timeout(self, value):
        self._serial.timeout = value

    @property
    def in_waiting(self):
        return self._serial.in_waiting

    def read(self, size=1):
        return self._serial.read(size)

    def write(self, data):
        return self._serial.write(data)

    def flush(self):
        return self._serial.flush()

    def close(self):
        self._serial.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()


class UdpMavlinkEndpoint:
    """File-like MAVLink endpoint backed by UDP."""

    def __init__(self, remote_ip, remote_port, local_ip, local_port, timeout=1):
        self.remote_address = (remote_ip, remote_port)
        self._buffer = bytearray()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            bind_port = 0 if local_port is None else local_port
            self._socket.bind((local_ip, bind_port))
            self.timeout = timeout
        except OSError:
            self._socket.close()
            raise

        local_address = self._socket.getsockname()
        self.description = (
            f"ethernet UDP {local_address[0]}:{local_address[1]} -> "
            f"{remote_ip}:{remote_port}"
        )

    @property
    def timeout(self):
        return self._timeout

    @timeout.setter
    def timeout(self, value):
        self._timeout = value
        self._socket.settimeout(value)

    @property
    def in_waiting(self):
        return len(self._buffer)

    def read(self, size=1):
        if size is None or size <= 0:
            size = 1
        if not self._buffer:
            try:
                data, _address = self._socket.recvfrom(4096)
            except socket.timeout:
                return b""
            self._buffer.extend(data)

        data = bytes(self._buffer[:size])
        del self._buffer[:size]
        return data

    def write(self, data):
        return self._socket.sendto(bytes(data), self.remote_address)

    def flush(self):
        return None

    def close(self):
        self._socket.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()


def open_mavlink_endpoint(
    output_type,
    serial_port,
    baud,
    ethernet_ip,
    ethernet_port,
    ethernet_local_ip,
    ethernet_local_port,
):
    if output_type == "serial":
        return SerialMavlinkEndpoint(serial_port, baud, timeout=1)
    if output_type == "ethernet":
        return UdpMavlinkEndpoint(
            ethernet_ip,
            ethernet_port,
            ethernet_local_ip,
            ethernet_local_port,
            timeout=1,
        )
    raise ValueError("output type must be 'serial' or 'ethernet'")


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

    print("Requesting MAVLink GPS2_RAW position stream...")
    for _message_name, message_id in POSITION_MESSAGE_IDS:
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
    if message_type in ("GPS2_RAW", "GPS_RAW_INT"):
        if getattr(message, "fix_type", 0) < 2:
            return None
        latitude = message.lat / 10000000.0
        longitude = message.lon / 10000000.0
    elif message_type == "GLOBAL_POSITION_INT":
        latitude = message.lat / 10000000.0
        longitude = message.lon / 10000000.0
    else:
        return None

    if -90 <= latitude <= 90 and -180 <= longitude <= 180:
        return latitude, longitude
    return None


def read_mavlink_position(
    mavlink_connection,
    timeout,
    source_system,
    source_component,
):
    """Listen on the MAVLink connection until a usable position arrives."""
    require_pymavlink()

    parser = mavlink2.MAVLink(None)
    parser.robust_parsing = True
    mavlink_output = mavlink2.MAVLink(
        mavlink_connection,
        srcSystem=source_system,
        srcComponent=source_component,
    )
    old_timeout = mavlink_connection.timeout
    deadline = time.monotonic() + timeout
    requested_stream = False
    last_heartbeat = 0.0
    fallback_position = None
    fallback_message_type = None

    print(f"Waiting up to {timeout:.1f}s for MAVLink GPS2_RAW latitude/longitude...")
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

            mavlink_connection.timeout = min(0.25, max(0.0, deadline - now))
            data = mavlink_connection.read(mavlink_connection.in_waiting or 1)
            if not data:
                continue

            for byte in data:
                try:
                    message = parser.parse_char(bytes([byte]))
                except Exception:
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
                    message_type = message.get_type()
                    if message_type == PREFERRED_POSITION_MESSAGE:
                        print(
                            f"Using MAVLink GPS2_RAW position: "
                            f"{latitude:.7f}, {longitude:.7f}"
                        )
                        return latitude, longitude, mavlink_output.seq
                    if fallback_position is None:
                        fallback_position = position
                        fallback_message_type = message_type
    finally:
        mavlink_connection.timeout = old_timeout

    if fallback_position is not None:
        latitude, longitude = fallback_position
        print(
            f"GPS2_RAW position not received; using MAVLink {fallback_message_type} "
            f"position: {latitude:.7f}, {longitude:.7f}"
        )
        return latitude, longitude, mavlink_output.seq

    raise TimeoutError("timed out waiting for MAVLink GPS2_RAW latitude/longitude")


def stream(
    caster,
    caster_port,
    mountpoint,
    username,
    password,
    mavlink_output,
    output_description,
    latitude,
    longitude,
    altitude,
    gga_interval,
    source_system,
    source_component,
    mavlink_version,
    initial_sequence=0,
):
    """Connect to the NTRIP caster and copy corrections as MAVLink frames."""
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
            f"to {output_description} as MAVLink {mavlink_version} "
            "GPS_RTCM_DATA. Press Ctrl+C to stop."
        )

        if initial_data:
            send_rtcm_blob(mavlink_output, packer, initial_data)

        source.sendall(build_gga(latitude, longitude, altitude))
        last_gga = time.monotonic()
        while True:
            try:
                data = source.recv(MAX_FRAGMENTED_RTCM_LEN)
                if not data:
                    raise ConnectionError("caster closed the correction stream")
                send_rtcm_blob(mavlink_output, packer, data)
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
        help=f"JSON config file (default: {DEFAULT_CONFIG_PATH})",
    )
    parser.add_argument("--caster", help="NTRIP caster hostname or IP")
    parser.add_argument("--caster-port", type=int, help="NTRIP caster port")
    parser.add_argument("--mountpoint", help="NTRIP mountpoint")
    parser.add_argument("--username", help="NTRIP username")
    parser.add_argument("--password", help="NTRIP password; prompted when omitted")
    parser.add_argument(
        "--output-type",
        choices=("serial", "ethernet"),
        help="MAVLink output type: serial or ethernet",
    )
    parser.add_argument("--serial-port", help="serial port receiving MAVLink frames")
    parser.add_argument("--baud", type=int, help="serial baud rate")
    parser.add_argument("--ethernet-ip", help="remote MAVLink device IP for ethernet output")
    parser.add_argument("--ethernet-port", type=int, help="remote MAVLink UDP port")
    parser.add_argument(
        "--ethernet-local-ip",
        help="local bind IP for MAVLink ethernet receive",
    )
    parser.add_argument(
        "--ethernet-local-port",
        type=int,
        help="local UDP port for MAVLink ethernet receive",
    )
    parser.add_argument("--latitude", type=float, help="rover latitude in decimal degrees")
    parser.add_argument("--longitude", type=float, help="rover longitude in decimal degrees")
    parser.add_argument("--altitude", type=float, help="MSL altitude in metres")
    parser.add_argument("--gga-interval", type=float, help="seconds between GGA messages")
    parser.add_argument(
        "--position-timeout",
        type=float,
        help="seconds to wait for MAVLink latitude/longitude before prompting",
    )
    parser.add_argument("--source-system", type=int, help="MAVLink source system id")
    parser.add_argument("--source-component", type=int, help="MAVLink source component id")
    parser.add_argument(
        "--mavlink-version",
        type=int,
        choices=(1, 2),
        help="MAVLink wire version for outgoing frames",
    )
    return parser.parse_args()

def main():
    args = parse_args()
    try:
        config = load_config(args.config)
        ntrip_config = ntrip_config_section(config)
        output_config = config_section(config, "output")
        mavlink_config = config_section(config, "mavlink")

        caster = args.caster or config_text(ntrip_config, "caster") or prompt("Caster hostname or IP")
        caster_port = (
            args.caster_port
            if args.caster_port is not None
            else config_int(ntrip_config, "caster_port", DEFAULT_NTRIP_PORT)
        )
        mountpoint = args.mountpoint or config_text(ntrip_config, "mountpoint") or prompt("Mountpoint")
        username = args.username or config_text(ntrip_config, "username") or prompt("Username")
        password = (
            args.password
            if args.password is not None
            else config_text(ntrip_config, "password", strip=False)
        )

        output_type = (
            args.output_type or config_text(output_config, "type") or DEFAULT_OUTPUT_TYPE
        ).lower()
        if output_type not in ("serial", "ethernet"):
            raise ValueError("output.type must be 'serial' or 'ethernet'")

        serial_port = args.serial_port or config_text(output_config, "serial_port")
        baud = None
        ethernet_ip = None
        ethernet_port = None
        ethernet_local_ip = DEFAULT_ETHERNET_LOCAL_IP
        ethernet_local_port = None

        if output_type == "serial":
            baud = args.baud if args.baud is not None else config_int(output_config, "baud", DEFAULT_BAUD)
        else:
            ethernet_ip = args.ethernet_ip or config_text(output_config, "ethernet_ip")
            ethernet_port = (
                args.ethernet_port
                if args.ethernet_port is not None
                else config_int(output_config, "ethernet_port", DEFAULT_ETHERNET_PORT)
            )
            ethernet_local_ip = (
                args.ethernet_local_ip
                or config_text(output_config, "ethernet_local_ip")
                or DEFAULT_ETHERNET_LOCAL_IP
            )
            ethernet_local_port = (
                args.ethernet_local_port
                if args.ethernet_local_port is not None
                else config_optional_int(output_config, "ethernet_local_port")
            )

        altitude = (
            args.altitude
            if args.altitude is not None
            else config_float(mavlink_config, "altitude", DEFAULT_ALTITUDE)
        )
        gga_interval = (
            args.gga_interval
            if args.gga_interval is not None
            else config_float(mavlink_config, "gga_interval", DEFAULT_GGA_INTERVAL)
        )
        position_timeout = (
            args.position_timeout
            if args.position_timeout is not None
            else config_float(mavlink_config, "position_timeout", DEFAULT_POSITION_TIMEOUT)
        )
        source_system = (
            args.source_system
            if args.source_system is not None
            else config_int(mavlink_config, "source_system", DEFAULT_SOURCE_SYSTEM)
        )
        source_component = (
            args.source_component
            if args.source_component is not None
            else config_int(mavlink_config, "source_component", DEFAULT_SOURCE_COMPONENT)
        )
        mavlink_version = (
            args.mavlink_version
            if args.mavlink_version is not None
            else config_int(mavlink_config, "version", DEFAULT_MAVLINK_VERSION)
        )
        latitude = (
            args.latitude
            if args.latitude is not None
            else config_optional_float(mavlink_config, "latitude")
        )
        longitude = (
            args.longitude
            if args.longitude is not None
            else config_optional_float(mavlink_config, "longitude")
        )
    except ValueError as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    if password is None:
        password = getpass.getpass("Password: ")
    if output_type == "serial" and serial_port is None:
        serial_port = choose_serial_port()

    try:
        validate_udp_port("caster port", caster_port)
        if gga_interval <= 0:
            raise ValueError("GGA interval must be greater than zero")
        if position_timeout <= 0:
            raise ValueError("position timeout must be greater than zero")
        if not 0 <= source_system <= 255:
            raise ValueError("source system id must be between 0 and 255")
        if not 0 <= source_component <= 255:
            raise ValueError("source component id must be between 0 and 255")
        if mavlink_version not in (1, 2):
            raise ValueError("MAVLink version must be 1 or 2")

        if output_type == "serial":
            if baud <= 0:
                raise ValueError("serial baud rate must be greater than zero")
        else:
            if not ethernet_ip:
                raise ValueError("output.ethernet_ip is required when output.type is ethernet")
            validate_udp_port("ethernet port", ethernet_port)
            if ethernet_local_port is not None:
                validate_udp_port("ethernet local port", ethernet_local_port)

        mavlink_sequence = 0
        with open_mavlink_endpoint(
            output_type,
            serial_port,
            baud,
            ethernet_ip,
            ethernet_port,
            ethernet_local_ip,
            ethernet_local_port,
        ) as mavlink_connection:
            if latitude is None or longitude is None:
                try:
                    mav_latitude, mav_longitude, mavlink_sequence = read_mavlink_position(
                        mavlink_connection,
                        position_timeout,
                        source_system,
                        source_component,
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
                mavlink_connection,
                mavlink_connection.description,
                latitude,
                longitude,
                altitude,
                gga_interval,
                source_system,
                source_component,
                mavlink_version,
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
