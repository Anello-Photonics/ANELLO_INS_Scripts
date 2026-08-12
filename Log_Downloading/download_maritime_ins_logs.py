#!/usr/bin/env python3
"""
Download logs from a Maritime INS using the same MAVLink log protocol path that
AMarinerControl uses for normal downloads.

This intentionally does not use MAVLink FTP. It uses:

    LOG_REQUEST_LIST -> LOG_ENTRY -> LOG_REQUEST_DATA -> LOG_DATA

Examples:

    # Listen for the vehicle on UDP and only list logs
    python download_maritime_ins_logs.py --connect udpin:0.0.0.0:14550 --list

    # Download the newest log using specific INS IP address
    python download_maritime_ins_logs.py --connect udpout:192.168.0.3:14550 --latest 1


    # If the INS stalls mid-download, use smaller request windows
    python download_maritime_ins_logs.py --connect udpin:0.0.0.0:14550 --latest 1 --request-packets 32

    # Use AMarinerControl-sized request windows for maximum throughput
    python download_maritime_ins_logs.py --connect udpin:0.0.0.0:14550 --latest 1 --fast

    # Download every listed log
    python download_maritime_ins_logs.py --connect udpin:0.0.0.0:14550 --all

    # Download selected log to a specific folder
    python download_maritime_ins_logs.py --connect udpin:0.0.0.0:14550 --id 3 --fast --output <file-path>

    # Erase all logs from the INS
    python download_maritime_ins_logs.py --connect udpin:0.0.0.0:14550 --erase-all --confirm-erase-all



Install dependency:

    python -m pip install pymavlink
"""

from __future__ import annotations

import argparse
import datetime as dt
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover - handled when a connection is attempted
    mavutil = None


LOG_DATA_LEN = 90  # MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN
AMC_CHUNK_BINS = 512
DEFAULT_REQUEST_PACKETS = 56
DEFAULT_BURST_IDLE_TIMEOUT = 0.02


@dataclass(frozen=True)
class LogEntry:
    """A MAVLink LOG_ENTRY row using the firmware's raw log id."""

    log_id: int
    time_utc: int
    size: int
    num_logs: int
    last_log_num: int

    @property
    def timestamp(self) -> Optional[dt.datetime]:
        if self.time_utc <= 0:
            return None
        timestamp = dt.datetime.fromtimestamp(self.time_utc, tz=dt.timezone.utc)
        return timestamp if timestamp.year >= 2010 else None

    def filename(self, extension: str) -> str:
        if self.timestamp:
            stamp = (
                f"{self.timestamp.year}-"
                f"{self.timestamp.month}-"
                f"{self.timestamp.day}-"
                f"{self.timestamp.hour:02d}-"
                f"{self.timestamp.minute:02d}-"
                f"{self.timestamp.second:02d}"
            )
        else:
            stamp = "UnknownDate"

        return f"log_{self.log_id}_{stamp}{extension}"


class MavlinkLogDownloader:
    def __init__(
        self,
        connection_string: str,
        baud: int,
        source_system: int,
        source_component: int,
        target_system: Optional[int],
        target_component: Optional[int],
        heartbeat_timeout: float,
        quiet: bool,
    ) -> None:
        self.connection_string = connection_string
        self.baud = baud
        self.source_system = source_system
        self.source_component = source_component
        self.target_system = target_system
        self.target_component = target_component
        self.heartbeat_timeout = heartbeat_timeout
        self.quiet = quiet
        self.autopilot = None
        self._last_heartbeat_sent = 0.0

        require_pymavlink()
        self.master = mavutil.mavlink_connection(
            connection_string,
            baud=baud,
            source_system=source_system,
            source_component=source_component,
            autoreconnect=True,
        )

    def connect(self) -> None:
        if self.target_system is not None and self.target_component is not None:
            self._log(
                f"Using explicit target system/component "
                f"{self.target_system}/{self.target_component}"
            )
            self._send_gcs_heartbeat(force=True)
            return

        self._log(f"Waiting for heartbeat on {self.connection_string}...")
        self._send_gcs_heartbeat(force=True)
        heartbeat = self.master.wait_heartbeat(timeout=self.heartbeat_timeout)
        if heartbeat is None:
            raise TimeoutError(
                f"No vehicle heartbeat received within {self.heartbeat_timeout:.1f}s"
            )

        self.target_system = self.target_system or heartbeat.get_srcSystem()
        self.target_component = self.target_component or heartbeat.get_srcComponent()
        self.autopilot = getattr(heartbeat, "autopilot", None)
        self._log(
            f"Connected to target system/component "
            f"{self.target_system}/{self.target_component}"
        )

    def list_logs(self, timeout: float, max_retries: int) -> list[LogEntry]:
        entries: dict[int, LogEntry] = {}
        expected_count: Optional[int] = None
        last_log_num: Optional[int] = None

        requests: list[tuple[int, int]] = [(0, 0xFFFF)]
        for attempt in range(max_retries + 1):
            for start, end in requests:
                self._request_log_list(start, end)
                deadline = time.monotonic() + timeout

                while time.monotonic() < deadline:
                    msg = self._recv_filtered(["LOG_ENTRY"], timeout=0.25)
                    if msg is None:
                        continue

                    expected_count = int(msg.num_logs)
                    last_log_num = int(msg.last_log_num)

                    if expected_count == 0:
                        return []

                    # AMC ignores zero-sized ArduPilot entries, but keeps PX4 entries.
                    if self._is_ardupilot() and int(msg.size) <= 0:
                        continue

                    entries[int(msg.id)] = LogEntry(
                        log_id=int(msg.id),
                        time_utc=int(msg.time_utc),
                        size=int(msg.size),
                        num_logs=expected_count,
                        last_log_num=last_log_num,
                    )

                    if expected_count and len(entries) >= expected_count:
                        return self._sorted_entries(entries.values())

                if expected_count and len(entries) >= expected_count:
                    return self._sorted_entries(entries.values())

            if expected_count is None:
                self._log(f"No LOG_ENTRY messages yet; retrying list ({attempt + 1}/{max_retries})")
                requests = [(0, 0xFFFF)]
                continue

            if len(entries) >= expected_count:
                return self._sorted_entries(entries.values())

            missing_ranges = self._missing_log_ranges(entries, expected_count, last_log_num)
            if not missing_ranges:
                break

            self._log(
                f"Have {len(entries)}/{expected_count} log entries; "
                f"requesting missing range(s): {format_ranges(missing_ranges)}"
            )
            requests = missing_ranges

        if expected_count is None:
            raise TimeoutError("No LOG_ENTRY messages received")

        if len(entries) < expected_count:
            self._log(
                f"Warning: only received {len(entries)}/{expected_count} log entries; "
                "continuing with the entries we have"
            )

        return self._sorted_entries(entries.values())

    def download_log(
        self,
        entry: LogEntry,
        output_dir: Path,
        extension: str,
        timeout: float,
        max_retries: int,
        request_packets: int,
        idle_timeout: float,
        overwrite: bool,
    ) -> Path:
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = unique_path(output_dir / entry.filename(extension), overwrite=overwrite)

        self._log(f"Downloading log {entry.log_id} ({human_size(entry.size)}) -> {output_path}")
        started = time.monotonic()
        last_print = started
        bytes_completed = 0
        completed = False

        try:
            with output_path.open("w+b") as output:
                output.truncate(entry.size)
                if entry.size <= 0:
                    self._log(f"Log {entry.log_id} is empty; created {output_path}")
                    completed = True
                    return output_path

                request_packets = max(1, min(request_packets, AMC_CHUNK_BINS))
                active_request_packets = request_packets
                total_bins = max(1, math.ceil(entry.size / LOG_DATA_LEN))
                received_bins = bytearray(total_bins)
                received_count = 0
                self._log(
                    f"Using {request_packets} LOG_DATA packets per request "
                    f"({human_size(request_packets * LOG_DATA_LEN)} windows)"
                )

                retries = 0
                search_from_bin = 0
                while received_count < total_bins:
                    start_bin = first_missing_index(received_bins, search_from_bin)
                    if start_bin is None:
                        break

                    start_offset = start_bin * LOG_DATA_LEN
                    request_bin_count = min(active_request_packets, total_bins - start_bin)
                    request_end_bin = start_bin + request_bin_count
                    request_count = request_bin_count * LOG_DATA_LEN
                    self._request_log_data(
                        entry.log_id,
                        start_offset,
                        request_count,
                        retry_count=retries,
                    )

                    made_progress = False
                    deadline = time.monotonic() + timeout
                    while time.monotonic() < deadline:
                        wait_timeout = min(idle_timeout, max(0.0, deadline - time.monotonic()))
                        msg = self._recv_filtered(["LOG_DATA"], timeout=wait_timeout)
                        if msg is None:
                            if made_progress:
                                break
                            continue

                        if int(msg.id) != entry.log_id:
                            continue

                        offset = int(msg.ofs)
                        count = max(0, min(int(msg.count), LOG_DATA_LEN))
                        if count == 0:
                            continue

                        if (offset % LOG_DATA_LEN) != 0:
                            self._log(f"Ignoring misaligned LOG_DATA offset {offset}")
                            continue

                        bin_index = offset // LOG_DATA_LEN
                        if bin_index < 0 or bin_index >= total_bins:
                            continue

                        expected_count = min(LOG_DATA_LEN, max(0, entry.size - offset))
                        write_count = min(count, expected_count)
                        if write_count <= 0:
                            continue

                        data = bytes(msg.data[:write_count])
                        output.seek(offset)
                        output.write(data)

                        if not received_bins[bin_index]:
                            if write_count < expected_count:
                                self._log(
                                    f"Ignoring short LOG_DATA packet at offset {offset}: "
                                    f"{write_count}/{expected_count} bytes"
                                )
                                continue

                            received_bins[bin_index] = 1
                            received_count += 1
                            bytes_completed += expected_count
                            made_progress = True
                            deadline = time.monotonic() + timeout

                        now = time.monotonic()
                        if now - last_print >= 0.25 or bytes_completed >= entry.size:
                            print_progress(bytes_completed, entry.size, started)
                            last_print = now

                        if bin_range_complete(received_bins, start_bin, request_end_bin):
                            break

                    if made_progress:
                        retries = 0
                        search_from_bin = start_bin
                        continue

                    retries += 1
                    if retries > max_retries:
                        raise TimeoutError(
                            f"Timed out waiting for log {entry.log_id} data at "
                            f"{bytes_completed}/{entry.size} bytes "
                            f"(next missing offset {start_offset})"
                        )

                    if active_request_packets > 1 and (retries % 3) == 0:
                        active_request_packets = max(1, active_request_packets // 2)
                        self._log(
                            f"No data at offset {start_offset}; reducing request window "
                            f"to {active_request_packets} packet(s)"
                        )

                if received_count != total_bins:
                    raise IOError(
                        f"Incomplete download for log {entry.log_id}: "
                        f"{received_count}/{total_bins} LOG_DATA packets received"
                    )

                output.flush()
                completed = True

        finally:
            if not completed:
                self._request_log_end()
            sys.stdout.write("\n")
            sys.stdout.flush()
            if not completed and output_path.exists():
                try:
                    output_path.unlink()
                except OSError:
                    pass

        if output_path.stat().st_size != entry.size:
            raise IOError(
                f"Downloaded file size mismatch for {output_path}: "
                f"{output_path.stat().st_size} != {entry.size}"
            )

        self._log(f"Finished log {entry.log_id}: {output_path}")
        return output_path

    def default_extension(self, requested_extension: Optional[str]) -> str:
        if requested_extension:
            return normalize_extension(requested_extension)

        if self.autopilot == getattr(mavutil.mavlink, "MAV_AUTOPILOT_ARDUPILOTMEGA", 3):
            return ".bin"

        # Maritime INS/PX4-style logs are normally ULog. AMC has extra access to
        # SYS_LOGGER for old PX4 .px4log detection; this standalone script keeps
        # the safe modern default, and --extension can override it.
        return ".ulg"

    def erase_all_logs(self) -> None:
        assert self.target_system is not None
        assert self.target_component is not None
        self._send_gcs_heartbeat()
        self.master.mav.log_erase_send(self.target_system, self.target_component)
        self._log(
            "Sent MAVLink LOG_ERASE. Refresh/list after the INS has finished deleting logs."
        )

    def _request_log_list(self, start: int, end: int) -> None:
        assert self.target_system is not None
        assert self.target_component is not None
        self._send_gcs_heartbeat()
        self.master.mav.log_request_list_send(
            self.target_system,
            self.target_component,
            int(start),
            int(end),
        )

    def _request_log_data(self, log_id: int, offset: int, count: int, retry_count: int = 0) -> None:
        assert self.target_system is not None
        assert self.target_component is not None
        self._send_gcs_heartbeat()
        self.master.mav.log_request_data_send(
            self.target_system,
            self.target_component,
            int(log_id),
            int(offset),
            int(count),
        )
        if retry_count and not self.quiet:
            self._log(
                f"Retry {retry_count}: requested log {log_id} offset {offset} count {count}"
            )

    def _request_log_end(self) -> None:
        if self.target_system is None or self.target_component is None:
            return

        self._send_gcs_heartbeat()
        self.master.mav.log_request_end_send(self.target_system, self.target_component)

    def _recv_filtered(self, message_types: list[str], timeout: float):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._send_gcs_heartbeat()
            remaining = max(0.0, deadline - time.monotonic())
            msg = self.master.recv_match(
                type=message_types + ["HEARTBEAT"],
                blocking=True,
                timeout=min(0.25, remaining),
            )
            if msg is None:
                continue

            if msg.get_type() == "HEARTBEAT":
                if self.target_system is None:
                    self.target_system = msg.get_srcSystem()
                if self.target_component is None:
                    self.target_component = msg.get_srcComponent()
                self.autopilot = getattr(msg, "autopilot", self.autopilot)
                continue

            if self.target_system is not None and msg.get_srcSystem() != self.target_system:
                continue

            return msg

        return None

    def _send_gcs_heartbeat(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self._last_heartbeat_sent) < 1.0:
            return

        mav = mavutil.mavlink
        self.master.mav.heartbeat_send(
            getattr(mav, "MAV_TYPE_GCS", 6),
            getattr(mav, "MAV_AUTOPILOT_INVALID", 8),
            0,
            0,
            getattr(mav, "MAV_STATE_ACTIVE", 4),
        )
        self._last_heartbeat_sent = now

    def _is_ardupilot(self) -> bool:
        return self.autopilot == getattr(mavutil.mavlink, "MAV_AUTOPILOT_ARDUPILOTMEGA", 3)

    def _missing_log_ranges(
        self,
        entries: dict[int, LogEntry],
        expected_count: int,
        last_log_num: Optional[int],
    ) -> list[tuple[int, int]]:
        if expected_count <= 0:
            return []

        if last_log_num is not None and last_log_num >= 0:
            first_id = max(0, last_log_num - expected_count + 1)
            expected_ids = list(range(first_id, last_log_num + 1))
        else:
            first_id = 1 if self._is_ardupilot() else 0
            expected_ids = list(range(first_id, first_id + expected_count))

        missing_ids = [log_id for log_id in expected_ids if log_id not in entries]
        return contiguous_ranges(missing_ids)

    def _log(self, text: str) -> None:
        if not self.quiet:
            print(text)

    @staticmethod
    def _sorted_entries(entries: Iterable[LogEntry]) -> list[LogEntry]:
        return sorted(entries, key=lambda item: item.log_id)


def parse_id_spec(specs: list[str]) -> set[int]:
    selected: set[int] = set()
    for spec in specs:
        for part in spec.split(","):
            part = part.strip()
            if not part:
                continue
            if "-" in part:
                left, right = part.split("-", 1)
                start = int(left.strip())
                end = int(right.strip())
                if end < start:
                    raise ValueError(f"Invalid id range: {part}")
                selected.update(range(start, end + 1))
            else:
                selected.add(int(part))
    return selected


def require_pymavlink() -> None:
    if mavutil is not None:
        return

    raise SystemExit(
        "Missing dependency: pymavlink\n"
        "Install it with: python -m pip install pymavlink"
    )


def contiguous_ranges(values: Iterable[int]) -> list[tuple[int, int]]:
    sorted_values = sorted(set(values))
    if not sorted_values:
        return []

    ranges: list[tuple[int, int]] = []
    start = prev = sorted_values[0]
    for value in sorted_values[1:]:
        if value == prev + 1:
            prev = value
            continue
        ranges.append((start, prev))
        start = prev = value

    ranges.append((start, prev))
    return ranges


def format_ranges(ranges: Iterable[tuple[int, int]]) -> str:
    parts = []
    for start, end in ranges:
        parts.append(str(start) if start == end else f"{start}-{end}")
    return ", ".join(parts)


def first_missing_index(received_bins: bytearray, start_index: int = 0) -> Optional[int]:
    for index in range(start_index, len(received_bins)):
        received = received_bins[index]
        if not received:
            return index

    return None


def bin_range_complete(received_bins: bytearray, start_index: int, end_index: int) -> bool:
    return all(received_bins[index] for index in range(start_index, end_index))


def normalize_extension(extension: str) -> str:
    return extension if extension.startswith(".") else f".{extension}"


def unique_path(path: Path, overwrite: bool) -> Path:
    if overwrite or not path.exists():
        return path

    stem = path.stem
    suffix = path.suffix
    for index in range(1, 10000):
        candidate = path.with_name(f"{stem}_{index}{suffix}")
        if not candidate.exists():
            return candidate

    raise FileExistsError(f"Could not find a unique filename for {path}")


def human_size(size: int) -> str:
    value = float(size)
    for unit in ["B", "KiB", "MiB", "GiB"]:
        if value < 1024.0 or unit == "GiB":
            return f"{value:.1f} {unit}" if unit != "B" else f"{int(value)} B"
        value /= 1024.0
    return f"{size} B"


def print_log_table(logs: list[LogEntry]) -> None:
    if not logs:
        print("No logs reported by vehicle.")
        return

    print("Available logs:")
    print("  id        size  timestamp_utc")
    print("----  ----------  --------------------")
    for entry in logs:
        stamp = entry.timestamp.isoformat().replace("+00:00", "Z") if entry.timestamp else "UnknownDate"
        print(f"{entry.log_id:4d}  {human_size(entry.size):>10}  {stamp}")


def print_progress(done: int, total: int, started: float) -> None:
    elapsed = max(0.001, time.monotonic() - started)
    rate = done / elapsed
    percent = (100.0 * done / total) if total else 100.0
    sys.stdout.write(
        f"\r  {percent:6.2f}%  {human_size(done)}/{human_size(total)}  {human_size(int(rate))}/s"
    )
    sys.stdout.flush()


def select_logs(args: argparse.Namespace, logs: list[LogEntry]) -> list[LogEntry]:
    by_id = {entry.log_id: entry for entry in logs}

    if args.all:
        return logs

    selected_ids: set[int] = set()
    if args.id:
        selected_ids.update(parse_id_spec(args.id))

    if args.latest:
        selected_ids.update(entry.log_id for entry in sorted(logs, key=lambda item: item.log_id)[-args.latest :])

    missing = sorted(log_id for log_id in selected_ids if log_id not in by_id)
    if missing:
        raise ValueError(f"Requested log id(s) not found: {', '.join(map(str, missing))}")

    return [by_id[log_id] for log_id in sorted(selected_ids)]


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Download Maritime INS logs using the MAVLink LOG_REQUEST_DATA method "
            "used by AMarinerControl."
        )
    )
    parser.add_argument(
        "--connect",
        required=True,
        help=(
            "pymavlink connection string, e.g. COM7, udpin:0.0.0.0:14550, "
            "udp:127.0.0.1:14550, tcp:192.168.2.10:5760"
        ),
    )
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate; ignored for UDP/TCP")
    parser.add_argument(
        "--output",
        "--directory",
        "--download-dir",
        dest="output",
        type=Path,
        default=Path.cwd(),
        help="Directory for downloaded logs; defaults to the current working directory",
    )
    parser.add_argument(
        "--list",
        "--list-only",
        dest="list_only",
        action="store_true",
        help="Print available logs and sizes, then exit without downloading",
    )
    parser.add_argument("--all", action="store_true", help="Download all listed logs")
    parser.add_argument(
        "--erase-all",
        action="store_true",
        help="Send MAVLink LOG_ERASE to erase all logs from the vehicle, then exit",
    )
    parser.add_argument(
        "--confirm-erase-all",
        action="store_true",
        help="Required with --erase-all to acknowledge permanent log deletion",
    )
    parser.add_argument("--latest", type=int, default=0, help="Download the newest N logs by log id")
    parser.add_argument(
        "--id",
        action="append",
        default=[],
        help="Raw log id(s) to download, e.g. --id 3 or --id 3,4,7-9. Can be repeated.",
    )
    parser.add_argument("--extension", help="Output extension override, e.g. ulg, px4log, or bin")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite existing output files")
    parser.add_argument("--target-system", type=int, help="MAVLink target system id; defaults to heartbeat source")
    parser.add_argument("--target-component", type=int, help="MAVLink target component id; defaults to heartbeat source")
    parser.add_argument("--source-system", type=int, default=255, help="MAVLink source system id for this script")
    parser.add_argument("--source-component", type=int, default=190, help="MAVLink source component id for this script")
    parser.add_argument("--heartbeat-timeout", type=float, default=15.0, help="Seconds to wait for vehicle heartbeat")
    parser.add_argument("--list-timeout", type=float, default=5.0, help="Quiet seconds to wait for LOG_ENTRY replies")
    parser.add_argument("--data-timeout", type=float, default=0.5, help="Seconds to wait for any LOG_DATA before retrying")
    parser.add_argument("--max-retries", type=int, default=25, help="Retries per listing/data gap before giving up")
    parser.add_argument(
        "--fast",
        action="store_true",
        help=(
            "Use AMC-style 512-packet request windows for higher throughput. "
            "If the INS stalls, remove this or lower --request-packets."
        ),
    )
    parser.add_argument(
        "--request-packets",
        type=int,
        default=None,
        help=(
            "LOG_DATA packets to request at a time. Overrides --fast. "
            "Default is 56; AMC uses 512."
        ),
    )
    parser.add_argument(
        "--burst-idle-timeout",
        type=float,
        default=DEFAULT_BURST_IDLE_TIMEOUT,
        help=(
            "Seconds of silence after receiving LOG_DATA before sending the next "
            "request window. Lower is faster; raise it if packets arrive in uneven bursts."
        ),
    )
    parser.add_argument(
        "--inter-log-delay",
        type=float,
        default=0.0,
        help="Seconds to wait between sequential log downloads; default is 0",
    )
    parser.add_argument("--quiet", action="store_true", help="Reduce status output")
    return parser


def main(argv: list[str]) -> int:
    args = build_arg_parser().parse_args(argv)

    if args.latest < 0:
        raise SystemExit("--latest must be >= 0")
    if args.confirm_erase_all and not args.erase_all:
        raise SystemExit("--confirm-erase-all only applies with --erase-all")
    if args.erase_all and not args.confirm_erase_all:
        raise SystemExit("--erase-all permanently deletes logs; add --confirm-erase-all to continue")
    if args.erase_all and (args.all or args.latest or args.id or args.list_only):
        raise SystemExit("--erase-all cannot be combined with --all, --latest, --id, or --list")

    request_packets = args.request_packets
    if request_packets is None:
        request_packets = AMC_CHUNK_BINS if args.fast else DEFAULT_REQUEST_PACKETS

    if request_packets < 1 or request_packets > AMC_CHUNK_BINS:
        raise SystemExit(f"--request-packets must be between 1 and {AMC_CHUNK_BINS}")
    if args.data_timeout <= 0:
        raise SystemExit("--data-timeout must be > 0")
    if args.burst_idle_timeout <= 0:
        raise SystemExit("--burst-idle-timeout must be > 0")
    if args.inter_log_delay < 0:
        raise SystemExit("--inter-log-delay must be >= 0")

    client = MavlinkLogDownloader(
        connection_string=args.connect,
        baud=args.baud,
        source_system=args.source_system,
        source_component=args.source_component,
        target_system=args.target_system,
        target_component=args.target_component,
        heartbeat_timeout=args.heartbeat_timeout,
        quiet=args.quiet,
    )

    client.connect()
    if args.erase_all:
        client.erase_all_logs()
        return 0

    logs = client.list_logs(timeout=args.list_timeout, max_retries=args.max_retries)
    print_log_table(logs)
    if args.list_only:
        return 0

    logs_to_download = select_logs(args, logs)
    if not logs_to_download:
        print("\nNo logs selected for download. Add --latest N, --id ID, or --all.")
        return 0

    extension = client.default_extension(args.extension)
    downloaded: list[Path] = []
    for index, entry in enumerate(logs_to_download):
        if index > 0 and args.inter_log_delay > 0:
            time.sleep(args.inter_log_delay)

        downloaded.append(
            client.download_log(
                entry=entry,
                output_dir=args.output,
                extension=extension,
                timeout=args.data_timeout,
                max_retries=args.max_retries,
                request_packets=request_packets,
                idle_timeout=args.burst_idle_timeout,
                overwrite=args.overwrite,
            )
        )

    print("\nDownloaded:")
    for path in downloaded:
        print(f"  {path}")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except KeyboardInterrupt:
        print("\nCanceled by user", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"\nError: {exc}", file=sys.stderr)
        raise SystemExit(1)
