# ANELLO_INS_Scripts

A collection of Python utilities for ANELLO INS devices. Each script below notes its purpose and typical usage.

## anello_fw_uploader.py
* **Purpose:** Uploads firmware to ANELLO Maritime INS units over MAVLink.
* **Usage:**
  1. Install dependencies: `pip install pymavlink pyserial`.
  2. Place the firmware JSON bundle in the working directory.
  3. Run `python anello_fw_uploader.py --port <connection>` (for example `udp:0.0.0.0:14550` or a serial port) and follow the prompts documented at [ANELLO firmware upgrade guide](https://docs-a1.readthedocs.io/en/maritime_ins/fw_upgrade.html).

## Maritime_INS_CFG.py
* **Purpose:** Applies standard lever-arm offsets and Ethernet settings to a Maritime INS over MAVLink.
* **Usage:**
  1. Edit the `lever_arm_cmds` list and `eth_cfg` dictionary near the top of the file to match your installation.
  2. Ensure MAVLink access to the INS (default `udp:0.0.0.0:14550`).
  3. Run `python Tools/Maritime_INS_CFG.py` to push the configured parameters, then allow the unit to reboot if prompted.

## Tools/Listener_Topics.py
* **Purpose:** Desktop UI for discovering PX4 listener topics and viewing live topic data streamed from the INS.
* **Usage:**
  1. Install GUI and MAVLink dependencies: `pip install pymavlink pyserial tk` (Tkinter is bundled with many Python installations).
  2. Connect to the INS over MAVLink (default `udp:0.0.0.0:14550`).
  3. Run `python Tools/Listener_Topics.py` and use the UI to select topics and start/stop streaming.

## Tools/comm_checks.py
* **Purpose:** Runs quick communication checks (Ethernet ping, RS232 MAVLink startup, and CAN PGN verification) against an INS.
* **Usage:**
  1. Adjust the user settings at the top of the file (Ethernet IP, serial baud rate/timeouts, and CAN PGN).
  2. Install dependencies: `pip install pymavlink pyserial python-can` and ensure PEAK-System PCAN drivers/`PCANBasic.dll` are available when using CAN.
  3. Run `python Tools/comm_checks.py` to execute the checks and review the summary output.

## Tools/PCANBasic.py and Tools/PCANBasic.dll
Support modules for CAN communication used by `comm_checks.py`; they are not intended to be executed directly.
