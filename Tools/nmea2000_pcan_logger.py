#!/usr/bin/env python3
"""Log NMEA2000 messages from a PCAN channel to a text file."""

import argparse
import datetime as dt
import sys
import time

from PCANBasic import (
    PCANBasic,
    PCAN_BAUD_250K,
    PCAN_ERROR_OK,
    PCAN_ERROR_QRCVEMPTY,
    PCAN_USBBUS1,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Log NMEA2000 traffic from a PCAN channel to a text file."
    )
    parser.add_argument(
        "--channel",
        type=lambda value: int(value, 0),
        default=PCAN_USBBUS1.value,
        help="PCAN channel handle (default: PCAN_USBBUS1).",
    )
    parser.add_argument(
        "--bitrate",
        type=lambda value: int(value, 0),
        default=PCAN_BAUD_250K.value,
        help="PCAN bitrate constant (default: PCAN_BAUD_250K).",
    )
    parser.add_argument(
        "--outfile",
        default="nmea2000_log.txt",
        help="Path to output log file (default: nmea2000_log.txt).",
    )
    parser.add_argument(
        "--poll-delay",
        type=float,
        default=0.01,
        help="Delay between empty-queue polls in seconds (default: 0.01).",
    )
    return parser.parse_args()


def extract_pgn(can_id: int) -> int:
    return (can_id >> 8) & 0x3FFFF


def format_message(can_id: int, data: bytes) -> str:
    timestamp = dt.datetime.utcnow().isoformat(timespec="milliseconds") + "Z"
    pgn = extract_pgn(can_id)
    sa = can_id & 0xFF
    data_hex = " ".join(f"{byte:02X}" for byte in data)
    return (
        f"{timestamp} CAN_ID=0x{can_id:08X} PGN={pgn} (0x{pgn:05X}) "
        f"SA=0x{sa:02X} DATA=[{data_hex}]"
    )


def main() -> int:
    args = parse_args()
    channel = args.channel
    bitrate = args.bitrate

    pcan = PCANBasic()
    ret = pcan.Initialize(channel, bitrate)
    if ret != PCAN_ERROR_OK:
        print("[FAIL] PCAN initialization failed.")
        return 1

    print(
        f"Logging NMEA2000 messages on channel 0x{channel:X} to {args.outfile}."
    )
    try:
        with open(args.outfile, "a", buffering=1) as log_file:
            while True:
                result = pcan.Read(channel)
                if isinstance(result, tuple):
                    ret_val, msg, *_ = result
                else:
                    ret_val, msg = result, None

                if ret_val == PCAN_ERROR_QRCVEMPTY:
                    time.sleep(args.poll_delay)
                    continue
                if ret_val != PCAN_ERROR_OK or msg is None:
                    continue

                data = bytes(msg.DATA[: msg.LEN])
                line = format_message(msg.ID, data)
                log_file.write(line + "\n")
                print(line)
    except KeyboardInterrupt:
        print("\nStopping logger.")
    finally:
        pcan.Uninitialize(channel)

    return 0


if __name__ == "__main__":
    sys.exit(main())
