#!/usr/bin/env python3
from __future__ import print_function

import time
import threading
import re
import argparse
import tkinter as tk
from pymavlink import mavutil


class MavlinkSerialPort:
    """Transmit shell commands using MAVLink SERIAL_CONTROL packets."""
    def __init__(self, portname, baudrate, devnum=10):
        self.buf = ""
        self.port = devnum
        self.mav = mavutil.mavlink_connection(portname, autoreconnect=True, baud=baudrate)

        # ensure link is alive
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GENERIC,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        self.mav.wait_heartbeat()

    def write(self, s: str):
        if not s:
            return
        b = s.encode("utf-8", errors="ignore")
        while b:
            chunk = b[:70]
            b = b[70:]
            buf = list(chunk) + [0] * (70 - len(chunk))
            self.mav.mav.serial_control_send(
                self.port,
                mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                0, 0,
                len(chunk),
                buf
            )

    def close(self):
        self.mav.mav.serial_control_send(self.port, 0, 0, 0, 0, [0] * 70)

    def _recv(self):
        m = self.mav.recv_match(
            condition="SERIAL_CONTROL.count!=0",
            type="SERIAL_CONTROL",
            blocking=True,
            timeout=0.05
        )
        if m is not None:
            data = m.data[:m.count]
            self.buf += "".join(chr(x) for x in data)

    def read(self, n: int) -> str:
        if not self.buf:
            self._recv()
        if not self.buf:
            return ""
        out = self.buf[:n]
        self.buf = self.buf[n:]
        return out


def drain(mav: MavlinkSerialPort, duration: float = 0.15):
    """Pull pending output so old command text doesn't pollute the next read."""
    t0 = time.time()
    while time.time() - t0 < duration:
        mav._recv()
        _ = mav.read(8192)
        time.sleep(0.01)


def run_cmd_marked(mav: MavlinkSerialPort, cmd: str, marker: str, max_timeout: float = 3.0) -> str:
    """
    Run a command, then send `echo <marker>` as a separate command.
    Read until marker is seen (or timeout). Returns output before marker.
    """
    drain(mav, 0.15)

    mav.write("\n")
    time.sleep(0.02)

    mav.write(cmd.strip() + "\n")
    time.sleep(0.02)

    mav.write("echo " + marker + "\n")
    time.sleep(0.02)

    out = ""
    t0 = time.time()
    while time.time() - t0 < max_timeout:
        mav._recv()
        chunk = mav.read(8192)
        if chunk:
            out += chunk
            if marker in out:
                break
        time.sleep(0.01)

    out = out.replace("\r", "")
    return out.split(marker, 1)[0] if marker in out else out


def set_dual_heading_params(mav: MavlinkSerialPort):
    """Run param set commands before any other reads/polling."""
    commands = [
        "param set GPS_SEP_BASE_X 0.495",
        "param set GPS_SEP_BASE_Y 0.0",
        "param set GPS_SEP_BASE_Z 0.0",
        "param set GPS_SEP_ROVER_X -0.52",
        "param set GPS_SEP_ROVER_Y 0.0",
        "param set GPS_SEP_ROVER_Z 0.0",
        "param set EKF2_IMU_POS_X 0.0",
        "param set EKF2_IMU_POS_Y 0.0",
        "param set EKF2_IMU_POS_Z 0.0",
        "param set SENS_BOARD_ROT 0",
    ]

    for idx, cmd in enumerate(commands, start=1):
        run_cmd_marked(mav, cmd, f"__DONE_PARAM_{idx}__", max_timeout=2.0)


def parse_gpio_i0(text: str) -> str:
    t = text.replace("\r", "")
    m = re.search(r"\b([01])\s+OK\b", t)
    return m.group(1) if m else "N/A"


def _extract_instance_block(listener_text: str, instance: int) -> str:
    t = listener_text.replace("\r", "")
    m = re.search(
        rf"Instance\s+{instance}:\s*\n(.*?)(?=\n\s*Instance\s+\d+:|\n\s*nsh>|\Z)",
        t,
        flags=re.S,
    )
    return m.group(1) if m else ""


def _parse_field_from_block(block: str, field: str) -> str:
    m = re.search(rf"^\s*{re.escape(field)}:\s*([-\w\.]+)\s*$", block, flags=re.M)
    return m.group(1) if m else "N/A"


def parse_gps_fields(listener_text: str, instance: int) -> dict:
    block = _extract_instance_block(listener_text, instance)
    return {
        "satellites_used": _parse_field_from_block(block, "satellites_used"),
        "fix_type": _parse_field_from_block(block, "fix_type"),
    }


def parse_heading_good(listener_text: str) -> str:
    m = re.search(r"heading_good_for_control:\s*(True|False)", listener_text, flags=re.I)
    return m.group(1).capitalize() if m else "Unknown"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read GPIO/GPS and dual-heading status over MAVLink shell."
    )
    parser.add_argument(
        "port",
        help='MAVLink connection string, e.g. "COM4" or "udp:0.0.0.0:14550"',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    mav = MavlinkSerialPort(args.port, 57600, devnum=10)

    root = tk.Tk()
    root.title("GPIO + GPS + Dual Heading")
    root.geometry("620x280")

    label = tk.Label(root, font=("Courier", 16), justify="left", anchor="w")
    label.pack(fill="both", expand=True, padx=12, pady=12)

    running = True

    gpio_period = 0.25
    gps_period = 1.50
    heading_period = 1.00

    last_gpio_t = 0.0
    last_gps_t = 0.0
    last_heading_t = 0.0

    gpio_val = "N/A"
    gps0_sats = "N/A"
    gps0_fix = "N/A"
    gps1_sats = "N/A"
    gps1_fix = "N/A"
    heading_state = "Pending"
    dual_heading_achieved = False
    status_line = "Setting dual-heading params..."

    def loop():
        nonlocal last_gpio_t, last_gps_t, last_heading_t
        nonlocal gpio_val, gps0_sats, gps0_fix, gps1_sats, gps1_fix
        nonlocal heading_state, dual_heading_achieved, status_line

        # Requirement: run param set commands before anything else.
        set_dual_heading_params(mav)
        status_line = "Params set. Monitoring..."

        while running:
            now = time.time()

            if now - last_gpio_t >= gpio_period:
                gpio_out = run_cmd_marked(mav, "gpio read I0", "__DONE_GPIO__", max_timeout=2.0)
                gpio_val = parse_gpio_i0(gpio_out)
                last_gpio_t = now

            if now - last_gps_t >= gps_period:
                gps_out = run_cmd_marked(mav, "listener sensor_gps", "__DONE_GPS__", max_timeout=8.0)
                g0 = parse_gps_fields(gps_out, instance=0)
                g1 = parse_gps_fields(gps_out, instance=1)
                gps0_sats = g0["satellites_used"]
                gps0_fix = g0["fix_type"]
                gps1_sats = g1["satellites_used"]
                gps1_fix = g1["fix_type"]
                last_gps_t = now

            if now - last_heading_t >= heading_period:
                heading_out = run_cmd_marked(
                    mav,
                    "listener vehicle_local_position -n 1",
                    "__DONE_HEADING__",
                    max_timeout=3.0,
                )
                heading_state = parse_heading_good(heading_out)
                dual_heading_achieved = heading_state == "True"
                status_line = "✅ Dual heading achieved" if dual_heading_achieved else "⏳ Waiting for dual heading"
                last_heading_t = now

            text = (
                f"GPIO I0:                     {gpio_val}\n"
                f"GPS0 satellites_used:         {gps0_sats}\n"
                f"GPS0 fix_type:                {gps0_fix}\n"
                f"GPS1 satellites_used:         {gps1_sats}\n"
                f"GPS1 fix_type:                {gps1_fix}\n"
                f"heading_good_for_control:     {heading_state}\n"
                f"Dual Heading Status:          {status_line}\n"
                f"Updated: {time.strftime('%H:%M:%S')}"
            )
            root.after(0, lambda t=text: label.config(text=t))
            time.sleep(0.05)

    th = threading.Thread(target=loop, daemon=True)
    th.start()

    def on_close():
        nonlocal running
        running = False
        time.sleep(0.1)
        try:
            mav.close()
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
