"""Offline checks for automatic MAVLink setup; no INS connection required."""
import contextlib
import io
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import mavlink_ulog_streaming as ulog


class AutoSetupTests(unittest.TestCase):
    def setUp(self):
        self.args = SimpleNamespace(
            port='0.0.0.0:14550', baudrate=115200, no_auto_setup=False,
            local_ip='192.0.2.10', stream_port=14560, stream_rate=200000,
            connect_timeout=1)
        self.control = Mock()
        self.control.clients = {('192.0.2.20', 14550)}
        self.stream = Mock()
        self.vehicle = Mock()
        self.vehicle.get_srcSystem.return_value = 1
        self.commands = []
        self.active = False
        self.shell = Mock()
        self.shell.run.side_effect = self.command
        self.patches = [
            patch.object(ulog.mavutil, 'mavlink_connection',
                         side_effect=[self.control, self.stream]),
            patch.object(ulog, 'wait_for_vehicle', return_value=self.vehicle),
            patch.object(ulog, 'MavlinkShell', return_value=self.shell),
        ]
        for p in self.patches:
            p.start()
            self.addCleanup(p.stop)
        self.output = contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__, None, None, None)

    def command(self, command, **kwargs):
        self.commands.append(command)
        if command.startswith('mavlink start'):
            self.active = True
        elif command.startswith('mavlink stop'):
            self.active = False
        return ('transport protocol: UDP (14550, remote port: 14550)\n'
                + ('transport protocol: UDP (14560, remote port: 14560)' if self.active else ''))

    def test_creates_requested_link_and_removes_it(self):
        with ulog.streaming_connection(self.args) as conn:
            self.assertIs(conn, self.stream)
            self.assertTrue(self.active)
            self.control.port.connect.assert_called_once_with(('192.0.2.20', 14550))
        self.assertIn('mavlink start -u 14560 -o 14560 -t 192.0.2.10 -m minimal -r 200000', self.commands)
        self.assertIn('mavlink stop -u 14560', self.commands)
        self.assertFalse(self.active)
        self.control.close.assert_called_once()
        self.stream.close.assert_called_once()

    def test_restarts_preexisting_instance(self):
        self.active = True
        with ulog.streaming_connection(self.args) as conn:
            self.assertIs(conn, self.stream)
            self.assertTrue(self.active)
            self.assertEqual(self.commands[:4], [
                'mavlink status', 'mavlink stop -u 14560', 'mavlink status',
                'mavlink start -u 14560 -o 14560 -t 192.0.2.10 -m minimal -r 200000'])
        self.assertFalse(self.active)
        self.assertEqual(self.commands.count('mavlink stop -u 14560'), 2)

    def test_failed_stop_does_not_start_replacement(self):
        self.active = True
        def refuse_stop(command, **kwargs):
            response = self.command(command)
            if command.startswith('mavlink stop'):
                self.active = True
            return response
        self.shell.run.side_effect = refuse_stop
        self.args.connect_timeout = 0.02
        with self.assertRaisesRegex(RuntimeError, 'Timed out.*stop'):
            with ulog.streaming_connection(self.args):
                self.fail('must not start replacement')
        self.assertEqual(self.commands, [
            'mavlink status', 'mavlink stop -u 14560', 'mavlink status'])

    def test_unreadable_status_after_stop_does_not_start(self):
        self.args.connect_timeout = 0.02
        outputs = iter(['transport protocol: UDP (14560, remote port: 14560)', ''])
        self.shell.run.side_effect = lambda *a, **kw: next(outputs, '')
        with self.assertRaisesRegex(RuntimeError, 'Last mavlink status'):
            with ulog.streaming_connection(self.args):
                self.fail('must not start replacement')
        self.assertFalse(any(call.args[0].startswith('mavlink start')
                             for call in self.shell.run.call_args_list))

    def test_delayed_start_is_not_treated_as_failure(self):
        pending = 0
        def delayed_start(command, **kwargs):
            nonlocal pending
            response = self.command(command)
            if command.startswith('mavlink start'):
                self.active = False
                pending = 2
            elif command == 'mavlink status' and pending:
                pending -= 1
                if pending == 0:
                    self.active = True
            return response
        self.shell.run.side_effect = delayed_start
        with ulog.streaming_connection(self.args):
            self.assertTrue(self.active)
            self.assertNotIn('mavlink stop -u 14560', self.commands)
        self.assertFalse(self.active)

    def test_failed_start_reports_status_and_cleans_up(self):
        self.args.connect_timeout = 0.02
        def failed_start(command, **kwargs):
            response = self.command(command)
            if command.startswith('mavlink start'):
                self.active = False
                return 'nsh> mavlink start\n'
            return response
        self.shell.run.side_effect = failed_start
        with self.assertRaisesRegex(RuntimeError, 'Last mavlink status') as error:
            with ulog.streaming_connection(self.args):
                self.fail('must not stream')
        self.assertIn('transport protocol: UDP (14550', str(error.exception))
        self.assertIn('Start command output:', str(error.exception))
        self.assertIn('mavlink stop -u 14560', self.commands)

    def test_cleanup_after_streaming_error(self):
        with self.assertRaisesRegex(RuntimeError, 'disk failure'):
            with ulog.streaming_connection(self.args):
                raise RuntimeError('disk failure')
        self.assertFalse(self.active)
        self.stream.close.assert_called_once()

    def test_cleanup_after_missing_stream_heartbeat(self):
        ulog.wait_for_vehicle.side_effect = [self.vehicle, RuntimeError('no heartbeat')]
        with self.assertRaisesRegex(RuntimeError, 'no heartbeat'):
            with ulog.streaming_connection(self.args):
                self.fail('must not stream')
        self.assertFalse(self.active)
        self.assertIn('mavlink stop -u 14560', self.commands)

    def test_manual_mode_sends_no_shell_commands(self):
        self.args.no_auto_setup = True
        with ulog.streaming_connection(self.args) as conn:
            self.assertIs(conn, self.control)
        self.assertEqual(self.commands, [])
        self.control.close.assert_called_once()

    def test_ambiguous_peer_refused_before_shell(self):
        self.control.clients.add(('192.0.2.21', 14550))
        with self.assertRaisesRegex(RuntimeError, 'exactly one'):
            with ulog.streaming_connection(self.args):
                self.fail('must refuse ambiguous peer')
        self.assertEqual(self.commands, [])

    def test_invalid_status_does_not_start_or_stop(self):
        self.shell.run.return_value = ''
        self.shell.run.side_effect = None
        with self.assertRaisesRegex(RuntimeError, 'Could not read'):
            with ulog.streaming_connection(self.args):
                self.fail('must refuse invalid status')
        self.shell.run.assert_called_once_with('mavlink status')


class ShellTests(unittest.TestCase):
    def test_chunking_and_echo_completion(self):
        mav = Mock()
        mav.target_system = 1
        marker = 'ULOG_123456789abc'
        def message(data):
            msg = Mock(device=10, count=len(data), data=data)
            msg.get_srcSystem.return_value = 1
            return msg
        mav.recv_match.side_effect = [
            message(('nsh> echo ' + marker + '\r\n').encode()),
            message((marker[:8]).encode()),
            message((marker[8:] + '\r\n').encode()),
        ]
        with patch.object(ulog.uuid, 'uuid4', return_value=SimpleNamespace(hex='123456789abc')):
            output = ulog.MavlinkShell(mav, 1).run('x' * 100)
        self.assertEqual(mav.recv_match.call_count, 3)
        chunks = mav.mav.serial_control_send.call_args_list
        reconstructed = b''.join(bytes(call.args[5][:call.args[4]]) for call in chunks)
        self.assertEqual(reconstructed, ('\n' + 'x' * 100 + '\necho ' + marker + '\n').encode())
        self.assertTrue(all(len(call.args[5]) == 70 for call in chunks))
        self.assertIn(marker, output)

    def test_missing_shell_response_times_out(self):
        mav = Mock()
        mav.recv_match.return_value = None
        with self.assertRaisesRegex(RuntimeError, 'shell timed out'):
            ulog.MavlinkShell(mav, 0.01).run('mavlink status')

    def test_udp_status_matches_local_port_only(self):
        status = 'transport protocol: UDP (14550, remote port: 14560)'
        self.assertTrue(ulog.has_udp_instance(status, 14550))
        self.assertFalse(ulog.has_udp_instance(status, 14560))
        self.assertIsNone(ulog.udp_listener_port('tcp:127.0.0.1:5760'))
        self.assertEqual(ulog.udp_listener_port('udpin:0.0.0.0:14550'), 14550)


if __name__ == '__main__':
    unittest.main()

