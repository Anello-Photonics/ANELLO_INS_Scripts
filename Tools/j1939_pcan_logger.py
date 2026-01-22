#!/usr/bin/env python3
"""Log raw J1939 messages from a PCAN channel to a text file."""

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
        description="Log raw J1939 traffic from a PCAN channel to a text file."
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
        default="j1939_log.txt",
        help="Path to output log file (default: j1939_log.txt).",
    )
    parser.add_argument(
        "--poll-delay",
        type=float,
        default=0.01,
        help="Delay between empty-queue polls in seconds (default: 0.01).",
    )
    return parser.parse_args()


def format_message(can_id: int, data: bytes, length: int) -> str:
    timestamp = dt.datetime.utcnow().isoformat(timespec="milliseconds") + "Z"
    data_hex = " ".join(f"{byte:02X}" for byte in data)
    return (
        f"{timestamp} CAN_ID=0x{can_id:08X} DLC={length} DATA=[{data_hex}]"
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
        f"Logging J1939 messages on channel 0x{channel:X} to {args.outfile}."
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
                line = format_message(msg.ID, data, msg.LEN)
                log_file.write(line + "\n")
                print(line)
    except KeyboardInterrupt:
        print("\nStopping logger.")
    finally:
        pcan.Uninitialize(channel)

    return 0


if __name__ == "__main__":
    sys.exit(main())
