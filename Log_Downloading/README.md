# MAVLink ULog streaming

Install the dependency with `python -m pip install pymavlink`.

From this directory, use the existing INS UDP link (normally 14550) and the PC's
IPv4 address on the adapter connected to the INS:

```powershell
python .\mavlink_ulog_streaming.py 0.0.0.0:14550 --local-ip 192.168.1.100 --output "C:\Users\kryan\Downloads"
```

Replace the example IP with your PC's address. Leave the MAVLink Console view in
Mariner Control before running: automatic setup uses the INS shell.

The script runs the following on the INS, receives the stream on UDP 14560, and
runs the stop command on exit (including Ctrl+C and ordinary errors):

```text
mavlink start -u 14560 -o 14560 -t PC_IP -m minimal -r 200000
mavlink stop -u 14560
```

Use `--stream-port` or `--stream-rate` to override these defaults.
The logger must already be running with its MAVLink backend enabled and MAVLink 2
must be available. Existing control connections and logger configuration are
preserved. The setup is temporary and must be repeated each run.

If an instance already exists on the selected INS stream port (default 14560),
the script stops it, verifies it stopped, and restarts it with the requested
PC address and rate. No manual stop is needed. To use an existing instance
without restarting it or cleaning it up on exit, use direct streaming:

```powershell
python .\mavlink_ulog_streaming.py 0.0.0.0:14560 --no-auto-setup --output "C:\Users\kryan\Downloads"
```

Automatic setup applies to UDP listener endpoints (`IP:PORT`, `udp:IP:PORT`,
`udpin:IP:PORT`). Serial, TCP, and other endpoints retain direct streaming.
Heartbeat and shell waits default to 10 seconds; override with `--connect-timeout`.
If the process is forcibly terminated or the control link is lost, cleanup may
not complete; remove the temporary instance with the stop command above.
Windows firewall must permit Python UDP traffic on the chosen port.

Offline regression checks (from the repository root):

```powershell
python -m unittest discover -s Log_Downloading -p "test_mavlink_ulog_streaming.py" -v
```

