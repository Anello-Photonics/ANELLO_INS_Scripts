#!/usr/bin/env python3

"""
Stream ULog data over MAVLink. For UDP, PORT is the existing control link;
--local-ip supplies the PC address for automatic dedicated-link setup.
Use --no-auto-setup for a manually configured streaming link.

@author: Beat Kueng (beat-kueng@gmx.net)
"""


from __future__ import print_function
import sys, os
import ipaddress
import re
import uuid
from contextlib import contextmanager
import datetime
from timeit import default_timer as timer
from time import sleep
os.environ['MAVLINK20'] = '1' # The commands require mavlink 2
from argparse import ArgumentParser
import signal

try:
    from pymavlink import mavutil
except ImportError as e:
    print("Failed to import pymavlink: " + str(e))
    print("")
    print("You may need to install it with:")
    print("    pip3 install --user pymavlink")
    print("")
    sys.exit(1)

class LoggingCompleted(Exception):
    pass


def heartbeat(mav):
    mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                          mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def wait_for_vehicle(mav, timeout, system=None):
    deadline = timer() + timeout
    while timer() < deadline:
        heartbeat(mav)
        msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if (msg is not None and msg.type != mavutil.mavlink.MAV_TYPE_GCS
                and (system is None or msg.get_srcSystem() == system)):
            return msg
    raise RuntimeError('No vehicle heartbeat received. Check the UDP address/port and firewall.')


class MavlinkShell:
    """Run bounded NSH commands through PX4's MAVLink serial shell."""
    def __init__(self, mav, timeout):
        self.mav = mav
        self.timeout = timeout

    def write(self, text):
        data = text.encode('ascii')
        for offset in range(0, len(data), 70):
            chunk = list(data[offset:offset + 70])
            self.mav.mav.serial_control_send(
                mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
                mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                0, 0, len(chunk), chunk + [0] * (70 - len(chunk)))

    def run(self, command, timeout=None):
        marker = 'ULOG_' + uuid.uuid4().hex[:12]
        self.write('\n' + command + '\necho ' + marker + '\n')
        output = ''
        deadline = timer() + (self.timeout if timeout is None else timeout)
        next_heartbeat = 0
        while timer() < deadline:
            if timer() >= next_heartbeat:
                heartbeat(self.mav)
                next_heartbeat = timer() + 1
            msg = self.mav.recv_match(type='SERIAL_CONTROL', blocking=True, timeout=0.1)
            if (msg is None or msg.device != mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL
                    or msg.get_srcSystem() != self.mav.target_system):
                continue
            output += bytes(msg.data[:msg.count]).decode('utf-8', errors='replace')
            # Match the echo result, not the echoed command line.
            if re.search(r'(?m)^' + marker + r'\r?$', output):
                return output
        raise RuntimeError('INS shell timed out running {!r}. Close the MAVLink '
                           'Console in Mariner Control and retry. Output: {}'.format(
                               command, output.strip()))

    def release(self):
        self.mav.mav.serial_control_send(
            mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL, 0, 0, 0, 0, [0] * 70)


def udp_listener_port(endpoint):
    match = re.fullmatch(r'(?:(?:udpin|udp):)?[^:]+:(\d+)', endpoint)
    return int(match.group(1)) if match else None


def has_udp_instance(status, port):
    return re.search(r'\bUDP\s*\(\s*' + str(port) + r'\s*[,)]', status) is not None


def wait_for_udp_instance(shell, port, present, timeout):
    """NSH returning does not mean the MAVLink worker has finished starting/stopping."""
    deadline = timer() + timeout
    status = '(no status received)'
    while timer() < deadline:
        status = shell.run('mavlink status', timeout=max(0.001, deadline - timer()))
        # The control instance should always be present. An empty or malformed
        # response cannot establish that the streaming instance stopped.
        if ('transport protocol:' in status
                and has_udp_instance(status, port) == present):
            return status
        sleep(min(0.25, max(0, deadline - timer())))
    action = 'appear' if present else 'stop'
    raise RuntimeError('Timed out waiting for INS UDP port {} to {}. '
                       'Last mavlink status:\n{}'.format(port, action, status.strip()))


@contextmanager
def streaming_connection(args):
    """Own only the temporary link created by this invocation."""
    control = stream = shell = None
    start_attempted = False
    try:
        control = mavutil.mavlink_connection(args.port, autoreconnect=True, baud=args.baudrate)
        vehicle = wait_for_vehicle(control, args.connect_timeout)
        if args.no_auto_setup or udp_listener_port(args.port) is None:
            yield control
            return

        peers = list(control.clients)
        if len(peers) != 1:
            raise RuntimeError('Automatic setup requires exactly one UDP peer; '
                               'use a dedicated control link or --no-auto-setup.')
        # Pin shell traffic to this peer instead of every sender on the UDP port.
        control.port.connect(peers[0])
        shell = MavlinkShell(control, args.connect_timeout)
        status = shell.run('mavlink status')
        if not re.search(r'transport protocol:', status):
            raise RuntimeError('Could not read MAVLink instances from INS shell: ' + status.strip())
        if has_udp_instance(status, args.stream_port):
            command = 'mavlink stop -u {}'.format(args.stream_port)
            print('INS: ' + command + ' (restarting existing instance)')
            shell.run(command)
            wait_for_udp_instance(shell, args.stream_port, False, args.connect_timeout)
        # Bind first, before the INS sends any streaming packets.
        stream = mavutil.mavlink_connection('udpin:{}:{}'.format(args.local_ip, args.stream_port))
        command = 'mavlink start -u {0} -o {0} -t {1} -m minimal -r {2}'.format(
            args.stream_port, args.local_ip, args.stream_rate)
        print('INS: ' + command)
        start_attempted = True
        response = shell.run(command)
        try:
            wait_for_udp_instance(shell, args.stream_port, True, args.connect_timeout)
        except RuntimeError as exc:
            raise RuntimeError('{}\nStart command output:\n{}'.format(exc, response.strip())) from exc
        shell.release()
        wait_for_vehicle(stream, args.connect_timeout, vehicle.get_srcSystem())
        print('Streaming connection ready on {}:{}'.format(args.local_ip, args.stream_port))
        yield stream
    finally:
        if shell is not None:
            try:
                if start_attempted:
                    command = 'mavlink stop -u {}'.format(args.stream_port)
                    print('\nINS: ' + command)
                    shell.run(command)
                    wait_for_udp_instance(shell, args.stream_port, False, args.connect_timeout)
            except Exception as exc:
                print('Cleanup warning: {}. Run "mavlink stop -u {}" in the INS console.'.format(
                    exc, args.stream_port), file=sys.stderr)
            finally:
                try:
                    shell.release()
                except Exception as exc:
                    print('Shell release warning: {}'.format(exc), file=sys.stderr)
        if stream is not None:
            stream.close()
        if control is not None:
            control.close()


class MavlinkLogStreaming():
    '''Streams log data via MAVLink.
       Assumptions:
       - the sender only sends one acked message at a time
       - the data is in the ULog format '''
    def __init__(self, mav, output_filename, debug=0):
        self.baudrate = 0
        self._debug = debug
        self.buf = ''
        self.mav = mav

        self.got_ulog_header = False
        self.got_header_section = False
        self.ulog_message = []
        self.file = open(output_filename,'wb')
        self.start_time = timer()
        self.last_sequence = -1
        self.logging_started = False
        self.num_dropouts = 0
        self.target_component = 1
        self.got_sig_int = False

    def debug(self, s, level=1):
        '''write some debug text'''
        if self._debug >= level:
            print(s)

    def start_log(self):
        self.mav.mav.command_long_send(self.mav.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_LOGGING_START, 0,
                0, 0, 0, 0, 0, 0, 0)

    def stop_log(self):
        self.mav.mav.command_long_send(self.mav.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_LOGGING_STOP, 0,
                0, 0, 0, 0, 0, 0, 0)

    def _int_handler(self, sig, frame):
        self.got_sig_int = True

    def read_messages(self):
        ''' main loop reading messages '''
        measure_time_start = timer()
        measured_data = 0

        next_heartbeat_time = timer()
        stop_deadline = None
        old_handler = signal.signal(signal.SIGINT, self._int_handler)

        while True:
            if self.got_sig_int:
                signal.signal(signal.SIGINT, old_handler)
                self.got_sig_int = False
                print('\nStopping log...')
                self.stop_log()
                stop_deadline = timer() + 3
                # Continue reading until we get an ACK

            if stop_deadline is not None and timer() > stop_deadline:
                print('Stop acknowledgement timed out; closing the connection.')
                return

            # handle heartbeat sending
            heartbeat_time = timer()
            if heartbeat_time > next_heartbeat_time:
                self.debug('sending heartbeat')
                self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
                next_heartbeat_time = heartbeat_time + 1

            m, first_msg_start, num_drops = self.read_message()
            if m is not None:
                self.process_streamed_ulog_data(m, first_msg_start, num_drops)

                # status output
                if self.logging_started:
                    measured_data += len(m)
                    measure_time_cur = timer()
                    dt = measure_time_cur - measure_time_start
                    if dt > 1:
                        sys.stdout.write('\rData Rate: {:0.1f} KB/s  Drops: {:} \033[K'.format(
                            measured_data / dt / 1024, self.num_dropouts))
                        sys.stdout.flush()
                        measure_time_start = measure_time_cur
                        measured_data = 0

            if not self.logging_started and timer()-self.start_time > 4:
                raise RuntimeError('Start timed out. Check INS logger status and MAVLink command routing.')


    def read_message(self):
        ''' read a single mavlink message, handle ACK & return a tuple of (data, first
        message start, num dropouts) '''
        m = self.mav.recv_match(type=['LOGGING_DATA_ACKED',
                            'LOGGING_DATA', 'COMMAND_ACK'], blocking=True,
                            timeout=0.05)
        if m is not None:
            self.debug(m, 3)

            if m.get_type() == 'COMMAND_ACK':
                if m.command == mavutil.mavlink.MAV_CMD_LOGGING_START and \
                        not self.got_header_section:
                    if m.result == 0:
                        self.logging_started = True
                        print('Logging started. Waiting for Header...')
                    else:
                        raise RuntimeError('Logging start failed: MAV_RESULT {}'.format(m.result))
                elif m.command == mavutil.mavlink.MAV_CMD_LOGGING_STOP and \
                        m.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    raise LoggingCompleted()
                return None, 0, 0

            # m is either 'LOGGING_DATA_ACKED' or 'LOGGING_DATA':
            is_newer, num_drops = self.check_sequence(m.sequence)

            # return an ack, even we already sent it for the same sequence,
            # because the ack could have been dropped
            if m.get_type() == 'LOGGING_DATA_ACKED':
                self.mav.mav.logging_ack_send(self.mav.target_system,
                        self.target_component, m.sequence)

            if is_newer:
                if num_drops > 0:
                    self.num_dropouts += num_drops

                if m.get_type() == 'LOGGING_DATA':
                    if not self.got_header_section:
                        print('Header received in {:0.2f}s (size: {:.1f} KB)'.format(
                              timer()-self.start_time, self.file.tell()/1024))
                        self.logging_started = True
                        self.got_header_section = True
                self.last_sequence = m.sequence
                return m.data[:m.length], m.first_message_offset, num_drops

            else:
                self.debug('dup/reordered message '+str(m.sequence))

        return None, 0, 0


    def check_sequence(self, seq):
        ''' check if a sequence is newer than the previously received one & if
        there were dropped messages between the last and this '''
        if self.last_sequence == -1:
            return True, 0
        if seq == self.last_sequence: # duplicate
            return False, 0
        if seq > self.last_sequence:
            # account for wrap-arounds, sequence is 2 bytes
            if seq - self.last_sequence > (1<<15): # assume reordered
                return False, 0
            return True, seq - self.last_sequence - 1
        else:
            if self.last_sequence - seq > (1<<15):
                return True, (1<<16) - self.last_sequence - 1 + seq
            return False, 0


    def process_streamed_ulog_data(self, data, first_msg_start, num_drops):
        ''' write streamed data to a file '''
        if not self.got_ulog_header: # the first 16 bytes need special treatment
            if len(data) < 16: # that's never the case anyway
                raise Exception('first received message too short')
            self.file.write(bytearray(data[0:16]))
            data = data[16:]
            self.got_ulog_header = True

        if self.got_header_section and num_drops > 0:
            if num_drops > 25: num_drops = 25
            # write a dropout message. We don't really know the actual duration,
            # so just use the number of drops * 10 ms
            self.file.write(bytearray([ 2, 0, 79, num_drops*10, 0 ]))

        if num_drops > 0:
            self.write_ulog_messages(self.ulog_message)
            self.ulog_message = []
            if first_msg_start == 255:
                return # no useful information in this message: drop it
            data = data[first_msg_start:]
            first_msg_start = 0

        if first_msg_start == 255 and len(self.ulog_message) > 0:
            self.ulog_message.extend(data)
            return

        if len(self.ulog_message) > 0:
            self.file.write(bytearray(self.ulog_message + data[:first_msg_start]))
            self.ulog_message = []

        data = self.write_ulog_messages(data[first_msg_start:])
        self.ulog_message = data # store the rest for the next message


    def write_ulog_messages(self, data):
        ''' write ulog data w/o integrity checking, assuming data starts with a
        valid ulog message. returns the remaining data at the end. '''
        while len(data) > 2:
            message_length = data[0] + data[1] * 256 + 3 # 3=ULog msg header
            if message_length > len(data):
                break
            self.file.write(bytearray(data[:message_length]))
            data = data[message_length:]
        return data



def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('port', metavar='PORT', nargs='?', default = None,
            help='Mavlink port name: serial: DEVICE[,BAUD], udp: IP:PORT, tcp: tcp:IP:PORT. Eg: \
/dev/ttyUSB0 or 0.0.0.0:14550. Auto-detect serial if not given.')
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int,
                      help="Mavlink port baud rate (default=115200)", default=115200)
    parser.add_argument("--output", "-o", dest="output", default = '.',
                      help="output file or directory (default=CWD)")
    parser.add_argument('--no-auto-setup', action='store_true',
                        help='Stream directly on PORT without creating an INS UDP instance')
    parser.add_argument('--local-ip', help='PC IPv4 address; required for automatic UDP setup')
    parser.add_argument('--stream-port', type=int, default=14560,
                        help='Dedicated UDP port on INS and PC (default: 14560)')
    parser.add_argument('--stream-rate', type=int, default=200000,
                        help='Dedicated MAVLink maximum rate in B/s (default: 200000)')
    parser.add_argument('--connect-timeout', type=float, default=10,
                        help='Heartbeat and shell timeout in seconds (default: 10)')
    args = parser.parse_args()
    if not 1 <= args.stream_port <= 65535 or args.stream_rate <= 0 or not 0 < args.connect_timeout < float('inf'):
        parser.error('stream-port must be 1..65535; stream-rate and connect-timeout must be positive')
    if args.port and udp_listener_port(args.port) is not None and not args.no_auto_setup:
        if not args.local_ip:
            parser.error('--local-ip PC_IPV4 is required for automatic UDP setup; use --no-auto-setup for an existing link')
        try:
            address = ipaddress.IPv4Address(args.local_ip)
            if address.is_unspecified or address.is_multicast or str(address) == '255.255.255.255':
                raise ValueError()
        except ValueError:
            parser.error('--local-ip must be a unicast PC IPv4 address')
        if udp_listener_port(args.port) == args.stream_port:
            parser.error('PORT is the existing control link; --stream-port must be different')

    if os.path.isdir(args.output):
        filename = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S.ulg")
        filename = os.path.join(args.output, filename)
    else:
        filename = args.output
    print('Output file name: {:}'.format(filename))

    if args.port == None:
        serial_list = mavutil.auto_detect_serial(preferred_list=['*FTDI*',
            "*Arduino_Mega_2560*", "*3D_Robotics*", "*USB_to_UART*", '*PX4*', '*FMU*'])

        if len(serial_list) == 0:
            print("Error: no serial connection found")
            return

        if len(serial_list) > 1:
            print('Auto-detected serial ports are:')
            for port in serial_list:
                print(" {:}".format(port))
        print('Using port {:}'.format(serial_list[0]))
        args.port = serial_list[0].device


    print("Connecting to MAVLINK...")
    try:
        with streaming_connection(args) as mav:
            mav_log_streaming = MavlinkLogStreaming(mav, filename)
            old_handler = signal.getsignal(signal.SIGINT)
            try:
                print('Starting log...')
                mav_log_streaming.start_log()
                mav_log_streaming.read_messages()
            finally:
                signal.signal(signal.SIGINT, old_handler)
                try:
                    mav_log_streaming.stop_log()
                finally:
                    mav_log_streaming.file.close()
    except KeyboardInterrupt:
        print('Aborting')
    except LoggingCompleted:
        print('Done')
    except (RuntimeError, OSError, ValueError) as exc:
        print('Error: {}'.format(exc), file=sys.stderr)
        return 1
    return 0

if __name__ == '__main__':
    sys.exit(main())
