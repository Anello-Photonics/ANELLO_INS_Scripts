#!/usr/bin/env python3
"""UI tool to send maritime INS Section 2.2 NMEA2000 input messages via PCAN."""

import math
import tkinter as tk
from ctypes import c_ubyte
from dataclasses import dataclass
from tkinter import messagebox, ttk

from PCANBasic import (
    PCANBasic,
    PCAN_BAUD_250K,
    PCAN_ERROR_OK,
    PCAN_MESSAGE_EXTENDED,
    PCAN_USBBUS1,
    TPCANMsg,
)


@dataclass
class FieldSpec:
    name: str
    label: str
    default: str


@dataclass
class MessagePreset:
    title: str
    pgn: int
    description: str
    fields: list[FieldSpec]


def _u16_le(value: int) -> tuple[int, int]:
    value &= 0xFFFF
    return value & 0xFF, (value >> 8) & 0xFF


def _s16_le(value: int) -> tuple[int, int]:
    return _u16_le(value & 0xFFFF)


def _u32_le(value: int) -> tuple[int, int, int, int]:
    value &= 0xFFFFFFFF
    return value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF, (value >> 24) & 0xFF


def _angle_deg_to_rad_1e4_u16(angle_deg: float) -> tuple[int, int]:
    raw = int(round(math.radians(angle_deg) / 1e-4))
    raw = max(0, min(raw, 0xFFFF))
    return _u16_le(raw)


def _angle_deg_to_rad_1e4_s16(angle_deg: float) -> tuple[int, int]:
    raw = int(round(math.radians(angle_deg) / 1e-4))
    raw = max(-32768, min(raw, 32767))
    return _s16_le(raw)


def _speed_mps_to_1e2_u16(speed_mps: float) -> tuple[int, int]:
    raw = int(round(speed_mps / 0.01))
    raw = max(0, min(raw, 0xFFFF))
    return _u16_le(raw)


def encode_127250(values: dict[str, str]) -> list[int]:
    sid = int(values["sid"], 0) & 0xFF
    hdg_lo, hdg_hi = _angle_deg_to_rad_1e4_u16(float(values["heading_deg"]))
    dev_lo, dev_hi = _angle_deg_to_rad_1e4_s16(float(values["deviation_deg"]))
    var_lo, var_hi = _angle_deg_to_rad_1e4_s16(float(values["variation_deg"]))
    reference = int(values["reference"], 0) & 0x03
    return [sid, hdg_lo, hdg_hi, dev_lo, dev_hi, var_lo, var_hi, reference]


def encode_127251(values: dict[str, str]) -> list[int]:
    sid = int(values["sid"], 0) & 0xFF
    rot_rad_s = math.radians(float(values["rot_deg_s"]))
    raw = int(round(rot_rad_s / 3.125e-8))
    raw = max(-2147483648, min(raw, 2147483647))
    b0, b1, b2, b3 = _u32_le(raw)
    return [sid, b0, b1, b2, b3, 0xFF, 0xFF, 0xFF]


def encode_128259(values: dict[str, str]) -> list[int]:
    sid = int(values["sid"], 0) & 0xFF
    spd_lo, spd_hi = _speed_mps_to_1e2_u16(float(values["water_speed_mps"]))
    water_ref_type = int(values["water_ref_type"], 0) & 0xFF
    return [sid, spd_lo, spd_hi, 0xFF, 0xFF, water_ref_type, 0x00, 0xFF]


def encode_128267(values: dict[str, str]) -> list[int]:
    sid = int(values["sid"], 0) & 0xFF
    depth_raw = int(round(float(values["depth_m"]) / 0.01))
    offset_raw = int(round(float(values["offset_m"]) / 0.001))
    range_raw = int(round(float(values["max_range_m"]) / 10.0))
    depth_raw = max(0, min(depth_raw, 0xFFFFFFFF))
    offset_raw = max(-32768, min(offset_raw, 32767))
    range_raw = max(0, min(range_raw, 0xFF))
    d0, d1, d2, d3 = _u32_le(depth_raw)
    o0, o1 = _s16_le(offset_raw)
    return [sid, d0, d1, d2, d3, o0, o1, range_raw]


def encode_129025(values: dict[str, str]) -> list[int]:
    lat_raw = int(round(float(values["latitude_deg"]) * 1e7))
    lon_raw = int(round(float(values["longitude_deg"]) * 1e7))
    a0, a1, a2, a3 = _u32_le(lat_raw)
    b0, b1, b2, b3 = _u32_le(lon_raw)
    return [a0, a1, a2, a3, b0, b1, b2, b3]


def encode_129026(values: dict[str, str]) -> list[int]:
    sid = int(values["sid"], 0) & 0xFF
    reference = int(values["reference"], 0) & 0x03
    cog_lo, cog_hi = _angle_deg_to_rad_1e4_u16(float(values["cog_deg"]))
    sog_lo, sog_hi = _speed_mps_to_1e2_u16(float(values["sog_mps"]))
    return [sid, reference, cog_lo, cog_hi, sog_lo, sog_hi, 0xFF, 0xFF]


def encode_130306(values: dict[str, str]) -> list[int]:
    sid = int(values["sid"], 0) & 0xFF
    spd_lo, spd_hi = _speed_mps_to_1e2_u16(float(values["wind_speed_mps"]))
    ang_lo, ang_hi = _angle_deg_to_rad_1e4_u16(float(values["wind_angle_deg"]))
    reference = int(values["reference"], 0) & 0x07
    return [sid, spd_lo, spd_hi, ang_lo, ang_hi, reference, 0xFF, 0xFF]


SECTION_2_2_PRESETS: list[MessagePreset] = [
    MessagePreset(
        "127250 - Vessel Heading",
        127250,
        "Heading input with optional deviation/variation.",
        [
            FieldSpec("sid", "SID", "1"),
            FieldSpec("heading_deg", "Heading (deg)", "90.0"),
            FieldSpec("deviation_deg", "Deviation (deg)", "0.0"),
            FieldSpec("variation_deg", "Variation (deg)", "0.0"),
            FieldSpec("reference", "Reference (0=True,1=Mag)", "0"),
        ],
    ),
    MessagePreset(
        "127251 - Rate of Turn",
        127251,
        "Rate of turn input in deg/s.",
        [
            FieldSpec("sid", "SID", "1"),
            FieldSpec("rot_deg_s", "Rate of Turn (deg/s)", "0.25"),
        ],
    ),
    MessagePreset(
        "128259 - Speed (Water Referenced)",
        128259,
        "Water speed input.",
        [
            FieldSpec("sid", "SID", "1"),
            FieldSpec("water_speed_mps", "Water Speed (m/s)", "2.00"),
            FieldSpec("water_ref_type", "Water Ref Type", "0"),
        ],
    ),
    MessagePreset(
        "128267 - Water Depth",
        128267,
        "Depth below transducer input.",
        [
            FieldSpec("sid", "SID", "1"),
            FieldSpec("depth_m", "Depth (m)", "12.30"),
            FieldSpec("offset_m", "Offset (m)", "0.00"),
            FieldSpec("max_range_m", "Max Range (m)", "100.0"),
        ],
    ),
    MessagePreset(
        "129025 - Position, Rapid Update",
        129025,
        "Latitude/longitude input.",
        [
            FieldSpec("latitude_deg", "Latitude (deg)", "37.7749"),
            FieldSpec("longitude_deg", "Longitude (deg)", "-122.4194"),
        ],
    ),
    MessagePreset(
        "129026 - COG & SOG, Rapid Update",
        129026,
        "Course/speed over ground input.",
        [
            FieldSpec("sid", "SID", "1"),
            FieldSpec("reference", "Reference (0=True,1=Mag)", "0"),
            FieldSpec("cog_deg", "COG (deg)", "100.0"),
            FieldSpec("sog_mps", "SOG (m/s)", "3.20"),
        ],
    ),
    MessagePreset(
        "130306 - Wind Data",
        130306,
        "Wind speed/angle input.",
        [
            FieldSpec("sid", "SID", "1"),
            FieldSpec("wind_speed_mps", "Wind Speed (m/s)", "4.50"),
            FieldSpec("wind_angle_deg", "Wind Angle (deg)", "30.0"),
            FieldSpec("reference", "Reference (0=True,2=Apparent)", "2"),
        ],
    ),
]

PRESETS = {preset.title: preset for preset in SECTION_2_2_PRESETS}
ENCODERS = {
    127250: encode_127250,
    127251: encode_127251,
    128259: encode_128259,
    128267: encode_128267,
    129025: encode_129025,
    129026: encode_129026,
    130306: encode_130306,
}


def compose_can_id(pgn: int, source_address: int, priority: int) -> int:
    dp = (pgn >> 16) & 0x01
    pf = (pgn >> 8) & 0xFF
    ps = pgn & 0xFF
    if pf < 240:
        ps = 0xFF
    return ((priority & 0x7) << 26) | (dp << 24) | (pf << 16) | (ps << 8) | (source_address & 0xFF)


class NMEA2000InputUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Section 2.2 NMEA2000 Input Sender (PCAN)")

        self.pcan = PCANBasic()
        self.initialized = False
        self.periodic_job = None

        self.channel_var = tk.StringVar(value=hex(PCAN_USBBUS1.value))
        self.bitrate_var = tk.StringVar(value=hex(PCAN_BAUD_250K.value))
        self.source_var = tk.StringVar(value="0x23")
        self.priority_var = tk.StringVar(value="3")
        self.period_ms_var = tk.StringVar(value="200")
        self.preset_var = tk.StringVar(value=SECTION_2_2_PRESETS[0].title)

        self.field_vars: dict[str, tk.StringVar] = {}

        self._build_ui()
        self._load_fields_for_selected_message()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        frame = ttk.Frame(self.root, padding=10)
        frame.grid(row=0, column=0, sticky="nsew")

        ttk.Label(frame, text="Channel").grid(row=0, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.channel_var, width=12).grid(row=0, column=1, sticky="w", padx=6)
        ttk.Label(frame, text="Bitrate").grid(row=0, column=2, sticky="w")
        ttk.Entry(frame, textvariable=self.bitrate_var, width=12).grid(row=0, column=3, sticky="w", padx=6)

        ttk.Label(frame, text="Source Address").grid(row=1, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.source_var, width=12).grid(row=1, column=1, sticky="w", padx=6)
        ttk.Label(frame, text="Priority").grid(row=1, column=2, sticky="w")
        ttk.Entry(frame, textvariable=self.priority_var, width=12).grid(row=1, column=3, sticky="w", padx=6)

        ttk.Button(frame, text="Initialize PCAN", command=self._initialize).grid(row=2, column=0, sticky="w", pady=8)
        ttk.Button(frame, text="Uninitialize", command=self._uninitialize).grid(row=2, column=1, sticky="w", pady=8)

        ttk.Separator(frame).grid(row=3, column=0, columnspan=4, sticky="ew", pady=4)

        ttk.Label(frame, text="Section 2.2 Message").grid(row=4, column=0, sticky="w")
        combo = ttk.Combobox(frame, textvariable=self.preset_var, values=list(PRESETS), state="readonly", width=40)
        combo.grid(row=4, column=1, columnspan=3, sticky="w", padx=6)
        combo.bind("<<ComboboxSelected>>", lambda _e: self._load_fields_for_selected_message())

        self.desc_label = ttk.Label(frame, text="", wraplength=640)
        self.desc_label.grid(row=5, column=0, columnspan=4, sticky="w", pady=(2, 8))

        self.fields_frame = ttk.Frame(frame)
        self.fields_frame.grid(row=6, column=0, columnspan=4, sticky="nsew")

        ttk.Button(frame, text="Send Once", command=self._send_once).grid(row=7, column=0, sticky="w", pady=10)
        ttk.Label(frame, text="Periodic (ms)").grid(row=7, column=1, sticky="e")
        ttk.Entry(frame, textvariable=self.period_ms_var, width=10).grid(row=7, column=2, sticky="w")
        ttk.Button(frame, text="Start Periodic", command=self._start_periodic).grid(row=7, column=3, sticky="w")
        ttk.Button(frame, text="Stop Periodic", command=self._stop_periodic).grid(row=8, column=3, sticky="w")

        self.log = tk.Text(frame, width=92, height=10, state="disabled")
        self.log.grid(row=9, column=0, columnspan=4, sticky="nsew", pady=(10, 0))

    def _append_log(self, text: str):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _load_fields_for_selected_message(self):
        for widget in self.fields_frame.winfo_children():
            widget.destroy()
        self.field_vars.clear()

        preset = PRESETS[self.preset_var.get()]
        self.desc_label.configure(text=f"PGN {preset.pgn}: {preset.description}")

        for idx, field in enumerate(preset.fields):
            ttk.Label(self.fields_frame, text=field.label).grid(row=idx, column=0, sticky="w", pady=2)
            var = tk.StringVar(value=field.default)
            self.field_vars[field.name] = var
            ttk.Entry(self.fields_frame, textvariable=var, width=22).grid(row=idx, column=1, sticky="w", padx=6)

    def _initialize(self):
        if self.initialized:
            self._append_log("PCAN already initialized.")
            return
        channel = int(self.channel_var.get(), 0)
        bitrate = int(self.bitrate_var.get(), 0)
        ret = self.pcan.Initialize(channel, bitrate)
        if ret != PCAN_ERROR_OK:
            self._append_log(f"[FAIL] Initialize failed code={ret}")
            return
        self.initialized = True
        self._append_log(f"[OK] Initialized channel=0x{channel:X}, bitrate=0x{bitrate:X}")

    def _uninitialize(self):
        if not self.initialized:
            return
        self._stop_periodic()
        channel = int(self.channel_var.get(), 0)
        self.pcan.Uninitialize(channel)
        self.initialized = False
        self._append_log("[OK] Uninitialized PCAN")

    def _build_message(self) -> tuple[int, list[int]]:
        preset = PRESETS[self.preset_var.get()]
        payload = ENCODERS[preset.pgn]({k: v.get() for k, v in self.field_vars.items()})
        can_id = compose_can_id(preset.pgn, int(self.source_var.get(), 0), int(self.priority_var.get(), 0))
        return can_id, payload

    def _send_once(self):
        if not self.initialized:
            messagebox.showwarning("PCAN not initialized", "Initialize PCAN before sending messages.")
            return
        try:
            can_id, payload = self._build_message()
            msg = TPCANMsg()
            msg.ID = can_id
            msg.LEN = len(payload)
            msg.MSGTYPE = PCAN_MESSAGE_EXTENDED
            msg.DATA = (c_ubyte * 8)(*payload)

            channel = int(self.channel_var.get(), 0)
            ret = self.pcan.Write(channel, msg)
            if ret == PCAN_ERROR_OK:
                hex_payload = " ".join(f"{b:02X}" for b in payload)
                self._append_log(f"[OK] ID=0x{can_id:08X} DATA=[{hex_payload}]")
            else:
                self._append_log(f"[FAIL] Write failed code={ret}, ID=0x{can_id:08X}")
        except Exception as exc:
            self._append_log(f"[FAIL] {exc}")

    def _tick(self):
        self._send_once()
        period = int(self.period_ms_var.get(), 0)
        self.periodic_job = self.root.after(period, self._tick)

    def _start_periodic(self):
        self._stop_periodic()
        try:
            period = int(self.period_ms_var.get(), 0)
            if period < 20:
                raise ValueError("period must be >= 20 ms")
            self.periodic_job = self.root.after(period, self._tick)
            self._append_log(f"Started periodic send at {period} ms")
        except Exception as exc:
            self._append_log(f"[FAIL] Could not start periodic mode: {exc}")

    def _stop_periodic(self):
        if self.periodic_job is not None:
            self.root.after_cancel(self.periodic_job)
            self.periodic_job = None
            self._append_log("Stopped periodic send")

    def _on_close(self):
        self._uninitialize()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    NMEA2000InputUI().run()
