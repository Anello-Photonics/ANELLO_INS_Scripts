#!/usr/bin/env python3
"""Configure the log streaming link using the Maritime_INS_CFG.py shell workflow.

Standalone dependency: python -m pip install pymavlink
Run: python configure_log_stream_link.py
Sends the four parameter settings, param save, then reboot via UDP 14550.
"""

import argparse
import math
import sys
import time

COMMANDS = (
    'param set MAV_1_CONFIG 1000',
    'param set MAV_1_RATE 200000',
    'param set MAV_1_REMOTE_PRT 16550',
    'param set MAV_1_UDP_PRT 16550',
    'param save',
    'reboot',
)


class MavlinkSerialPort():
    '''an object that looks like a serial port, but
    transmits using mavlink SERIAL_CONTROL packets'''
    def __init__(self, portname, baudrate, devnum=0, debug=0):
        self.baudrate = 0
        self._debug = debug
        self.buf = ''
        self.port = devnum
        self.debug("Connecting with MAVLink to %s ..." % portname)
        self.mav = mavutil.mavlink_connection(portname, autoreconnect=True, baud=baudrate)
        self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        self.mav.wait_heartbeat()
        self.debug("HEARTBEAT OK\n")
        self.debug("Locked serial device\n")

    def debug(self, s, level=1):
        '''write some debug text'''
        if self._debug >= level:
            print(s)

    def write(self, b):
        '''write some bytes'''
        self.debug("sending '%s' (0x%02x) of len %u\n" % (b, ord(b[0]), len(b)), 2)
        while len(b) > 0:
            n = len(b)
            if n > 70:
                n = 70
            buf = [ord(x) for x in b[:n]]
            buf.extend([0]*(70-len(buf)))
            self.mav.mav.serial_control_send(self.port,
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                                             0,
                                             0,
                                             n,
                                             buf)
            b = b[n:]

    def close(self):
        try:
            self.mav.mav.serial_control_send(self.port, 0, 0, 0, 0, [0]*70)
        finally:
            self.mav.close()

    def _recv(self):
        '''read some bytes into self.buf'''
        m = self.mav.recv_match(condition='SERIAL_CONTROL.count!=0',
                                type='SERIAL_CONTROL', blocking=True,
                                timeout=0.03)
        if m is not None:
            if self._debug > 2:
                print(m)
            data = m.data[:m.count]
            self.buf += ''.join(str(chr(x)) for x in data)

    def read(self, n):
        '''read some bytes'''
        if len(self.buf) == 0:
            self._recv()
        if len(self.buf) > 0:
            if n > len(self.buf):
                n = len(self.buf)
            ret = self.buf[:n]
            self.buf = self.buf[n:]
            if self._debug >= 2:
                for b in ret:
                    self.debug("read 0x%x" % ord(b), 2)
            return ret
        return ''


def configure(mav_serialport, timeout=1.0):
    """Use the same wake, command, delay, and response loop as Maritime_INS_CFG."""
    for command in COMMANDS:
        print('\n[Sending] ' + command)
        mav_serialport.write('\n')
        time.sleep(0.1)
        mav_serialport.write(command + '\n')
        time.sleep(0.1)
        output = ''
        start = time.time()
        while time.time() - start < timeout:
            mav_serialport._recv()
            chunk = mav_serialport.read(1024)
            if chunk:
                output += chunk
            time.sleep(0.05)
        if output.strip():
            print(output.strip())
        elif command == 'reboot':
            print('[Info] Reboot sent; the link may disconnect without a response.')
        else:
            print('[!] No response received.')
    print('\n[Done] Configuration, save, and reboot commands sent. Allow the INS to restart.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=1.0,
                        help='seconds to collect each command response (default: 1)')
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    global mavutil
    try:
        from pymavlink import mavutil
    except ImportError as error:
        print('Install dependency: python -m pip install pymavlink\n{}'.format(error), file=sys.stderr)
        return 1
    mav_serialport = None
    try:
        print('Connecting on UDP 0.0.0.0:14550...')
        mav_serialport = MavlinkSerialPort('udp:0.0.0.0:14550', 57600, devnum=10)
        time.sleep(0.5)
        configure(mav_serialport, args.timeout)
        time.sleep(0.5)
        return 0
    except KeyboardInterrupt:
        print('\nInterrupted; configuration may be incomplete.', file=sys.stderr)
        return 130
    except (OSError, ValueError) as error:
        print('Error: {}'.format(error), file=sys.stderr)
        return 1
    finally:
        if mav_serialport is not None:
            mav_serialport.close()


if __name__ == '__main__':
    raise SystemExit(main())
