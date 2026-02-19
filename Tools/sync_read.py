#!/usr/bin/env python3
from __future__ import print_function

import time
import threading
import re
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
    """Pull any pending output so old command text doesn't pollute the next read."""
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

    # Send marker as a separate line (more reliable than '; echo marker')
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


def parse_gpio_i0(text: str) -> str:
    # Matches "0 OK" or "1 OK"
    t = text.replace("\r", "")
    m = re.search(r"\b([01])\s+OK\b", t)
    return m.group(1) if m else "N/A"


def _extract_instance_block(listener_text: str, instance: int) -> str:
    """
    Extract the block for 'Instance <instance>:' from `listener sensor_gps` output.
    """
    t = listener_text.replace("\r", "")
    m = re.search(
        rf"Instance\s+{instance}:\s*\n(.*?)(?=\n\s*Instance\s+\d+:|\n\s*nsh>|\Z)",
        t,
        flags=re.S
    )
    return m.group(1) if m else ""


def _parse_field_from_block(block: str, field: str) -> str:
    """
    Parse a scalar field like:
      fix_type: 3
      satellites_used: 10
    """
    m = re.search(rf"^\s*{re.escape(field)}:\s*([-\w\.]+)\s*$", block, flags=re.M)
    return m.group(1) if m else "N/A"


def parse_gps_fields(listener_text: str, instance: int) -> dict:
    """
    Return satellites_used + fix_type for a given GPS instance.
    """
    block = _extract_instance_block(listener_text, instance)
    return {
        "satellites_used": _parse_field_from_block(block, "satellites_used"),
        "fix_type": _parse_field_from_block(block, "fix_type"),
    }


def main():
    mav = MavlinkSerialPort("COM4", 57600, devnum=10)

    root = tk.Tk()
    root.title("GPIO I0 + GPS (Instances 0/1)")
    root.geometry("520x230")

    label = tk.Label(root, font=("Courier", 16), justify="left", anchor="w")
    label.pack(fill="both", expand=True, padx=12, pady=12)

    running = True

    # Stagger updates
    gpio_period = 0.25
    gps_period = 1.50

    last_gpio_t = 0.0
    last_gps_t = 0.0

    gpio_val = "N/A"

    gps0_sats = "N/A"
    gps0_fix = "N/A"
    gps1_sats = "N/A"
    gps1_fix = "N/A"

    def loop():
        nonlocal last_gpio_t, last_gps_t
        nonlocal gpio_val, gps0_sats, gps0_fix, gps1_sats, gps1_fix

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

                # Debug capture if needed:
                # if gps0_fix == "N/A" or gps1_fix == "N/A":
                #     print("---- GPS RAW (first 1200 chars) ----")
                #     print(gps_out[:1200])

            text = (
                f"GPIO I0:              {gpio_val}\n"
                f"GPS0 satellites_used:  {gps0_sats}\n"
                f"GPS0 fix_type:         {gps0_fix}\n"
                f"GPS1 satellites_used:  {gps1_sats}\n"
                f"GPS1 fix_type:         {gps1_fix}\n"
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
