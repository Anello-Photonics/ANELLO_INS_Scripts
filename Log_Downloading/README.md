# Log downloading

## Download stored Maritime INS logs

`download_maritime_ins_logs.py` lists and downloads logs stored on a Maritime INS
using the same MAVLink log protocol as AMarinerControl
(`LOG_REQUEST_LIST` / `LOG_ENTRY` / `LOG_REQUEST_DATA` / `LOG_DATA`).
It retrieves existing log files rather than streaming live ULog data, and does
not use MAVLink FTP.

Install the dependency and run these examples from `Log_Downloading`:

```powershell
python -m pip install pymavlink

# List available logs without downloading
python .\download_maritime_ins_logs.py --connect udpin:0.0.0.0:14550 --list

# Download the newest log from a specific INS to a local folder
python .\download_maritime_ins_logs.py --connect udpout:192.168.0.3:14550 --latest 1 --output "C:\Users\kryan\Downloads"

# Download selected log IDs, including a range
python .\download_maritime_ins_logs.py --connect udpin:0.0.0.0:14550 --id 3,4,7-9

# Download every listed log
python .\download_maritime_ins_logs.py --connect udpin:0.0.0.0:14550 --all
```

`--connect` is required: use `udpin:` to listen for the INS or `udpout:` to
contact its IP address, adjusting the address and port for your configuration.
Serial and TCP pymavlink connection strings are also supported. `--latest N`
selects the N highest log IDs. Without `--latest`, `--id`, or `--all`, the script
only displays the log list.

Downloads default to the current working directory; `--output` selects a folder.
Files include the log ID and UTC timestamp (or `UnknownDate`) in their names.
Maritime INS/PX4 logs default to `.ulg`; use `--extension` to override this.
Existing files are preserved by adding a numeric suffix unless `--overwrite`
is supplied.

Transfers show progress and retry missing data. The default request window is
56 packets; `--fast` uses AMarinerControl-style 512-packet windows for higher
throughput. If downloads stall, try `--request-packets 32`, which overrides
`--fast`. Use `--help` for timeout, retry, and other connection options.

The script also supports permanent deletion of all logs on the INS with
`--erase-all --confirm-erase-all`. This cannot be combined with listing or
download selection options; it sends the erase request without verifying that
deletion has completed.

## MAVLink ULog streaming

Install the dependency with `python -m pip install pymavlink`.

By default, the script streams directly without configuring any MAVLink instances.
If PORT is omitted, it connects to `udpout:192.168.0.3:16550`.
For example: `python .\mavlink_ulog_streaming.py --output "C:\Users\kryan\Downloads"`. For an already configured streaming link on port 14560:

```powershell
python .\mavlink_ulog_streaming.py 0.0.0.0:14560 --output "C:\Users\kryan\Downloads"
```

To opt into automatic configuration, add `--auto-setup`, use the existing INS UDP
control link (normally 14550), and supply your PC's IPv4 address:

```powershell
python .\mavlink_ulog_streaming.py 0.0.0.0:14550 --auto-setup --local-ip 192.168.1.100 --output "C:\Users\kryan\Downloads"
```

Replace the example IP with your PC's address. Leave the MAVLink Console view in
Mariner Control before running: automatic setup uses the INS shell.

With `--auto-setup`, the script runs the following on the INS, receives the stream on UDP 14560, and
runs the stop command on exit (including Ctrl+C and ordinary errors):

```text
mavlink start -u 14560 -o 14560 -t PC_IP -m minimal -r 200000
mavlink stop -u 14560
```

Use `--stream-port` or `--stream-rate` to override these defaults.
The logger must already be running with its MAVLink backend enabled and MAVLink 2
must be available. Existing control connections and logger configuration are
preserved. The setup is temporary and must be repeated each run.

With `--auto-setup`, if an instance already exists on the selected INS stream port (default 14560),
the script stops it, verifies it stopped, and restarts it with the requested
PC address and rate. No manual stop is needed. To use an existing instance
without restarting it or cleaning it up on exit, use direct streaming:

```powershell
python .\mavlink_ulog_streaming.py 0.0.0.0:14560 --no-auto-setup --output "C:\Users\kryan\Downloads"
```

The optional `--auto-setup` mode requires UDP listener endpoints (`IP:PORT`, `udp:IP:PORT`,
`udpin:IP:PORT`). Serial, TCP, and other endpoints retain direct streaming.
Heartbeat and shell waits default to 10 seconds; override with `--connect-timeout`.
If the process is forcibly terminated or the control link is lost, cleanup may
not complete; remove the temporary instance with the stop command above.
Windows firewall must permit Python UDP traffic on the chosen port.

Offline regression checks (from the repository root):

```powershell
python -m unittest discover -s Log_Downloading -p "test_mavlink_ulog_streaming.py" -v
```


When using `--auto-setup`, after the dedicated streaming heartbeat arrives, the script closes its setup
connection (normally PC UDP port 14550). AMarinerControl can then use that port
while logging continues on 14560. The script does not reopen 14550 for cleanup:
it sends the stop command through the streaming link itself. Since stopping that
instance also removes the reply path, remote shutdown may remain unconfirmed;
the next run checks for and restarts any remaining instance. Setup failures use
the original control link for cleanup before closing it. Direct streaming with
`--no-auto-setup` keeps its selected port open until logging exits.

For a direct outgoing connection to an INS, use the `udpout:` prefix:

```powershell
python .\mavlink_ulog_streaming.py udpout:192.168.0.3:16550 --output "C:\Users\kryan\Downloads"
```

These connections now bind PC UDP port 14560 before sending. Override it with
`--udp-local-port PORT`, keeping the same port on subsequent runs. ANELLO PX4 can
retain the first sender's IP and port; an arbitrary new source port on each run
can leave replies going to the old socket. If the INS already learned an old
port, restart its MAVLink instance or reboot once before the first fixed-port
run. Changing the PC IP or source port later may require resetting that instance
again. This does not enable automatic setup or bind PC port 14550.

## Configure the persistent streaming link

Download or run the standalone `configure_log_stream_link.py`:

```powershell
python -m pip install pymavlink
python .\configure_log_stream_link.py
```

It uses the same `MavlinkSerialPort` connection, shell wake-up delays, and timed
response-reading loop as `Maritime_INS_CFG.py`, with its helper code included
so this file runs independently. It connects to `udp:0.0.0.0:14550` and sends:

```text
param set MAV_1_CONFIG 1000
param set MAV_1_RATE 200000
param set MAV_1_REMOTE_PRT 16550
param set MAV_1_UDP_PRT 16550
param save
reboot
```

Like `Maritime_INS_CFG.py`, it prints responses and continues if no response
arrives; it does not require an echo marker or verify parameter readback.
`--timeout` controls response collection per command (default: 1 second).
It releases the shell and closes the UDP connection on exit. Allow the INS to
restart before connecting.
