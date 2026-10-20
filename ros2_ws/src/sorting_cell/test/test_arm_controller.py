"""Unit tests for the sorting_cell arm controller subsystem.

Tests command parsing, validation against configurable limits, malformed command handling,
serial wire protocol formatting, connection lifecycle, mock serial responses, and error handling.
"""

import sys
from pathlib import Path
from unittest.mock import MagicMock, patch
import pytest

# Ensure sorting_cell is on PYTHONPATH
pkg_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(pkg_root))

from sorting_cell.arm_controller import ArmCommandParser, ArmControllerCore, JointRange
from sorting_cell.serial_interface import SerialInterface


class TestArmCommandParserValid:
    """Test valid command parsing and conversion to the serial protocol."""

    def setup_method(self):
        self.parser = ArmCommandParser()

    @pytest.mark.parametrize("command,expected_serial,expected_action", [
        ("HOME", "HOME", "HOME"),
        ("home", "HOME", "HOME"),
        ("BASE 90", "BASE:90", "BASE"),
        ("base 0", "BASE:0", "BASE"),
        ("BASE 180", "BASE:180", "BASE"),
        ("SHOULDER 60", "SHOULDER:60", "SHOULDER"),
        ("ELBOW 120", "ELBOW:120", "ELBOW"),
        ("WRIST 90", "WRIST:90", "WRIST"),
        ("wrist 180", "WRIST:180", "WRIST"),
        ("WRIST_PITCH 90", "WRIST_PITCH:90", "WRIST_PITCH"),
        ("wrist_pitch 45", "WRIST_PITCH:45", "WRIST_PITCH"),
        ("  BASE   90  ", "BASE:90", "BASE"),
        ("BASE 90.0", "BASE:90", "BASE"),
        ("ELBOW 120.4", "ELBOW:120", "ELBOW"),
        ("GRIPPER OPEN", "GRIPPER:OPEN", "GRIPPER"),
        ("gripper open", "GRIPPER:OPEN", "GRIPPER"),
        ("GRIPPER CLOSE", "GRIPPER:CLOSE", "GRIPPER"),
        ("gripper close", "GRIPPER:CLOSE", "GRIPPER"),
    ])
    def test_valid_commands(self, command, expected_serial, expected_action):
        is_valid, serial_cmd, action_name, error = self.parser.parse(command)
        assert is_valid is True
        assert serial_cmd == expected_serial
        assert action_name == expected_action
        assert error is None


class TestArmCommandParserInvalid:
    """Test malformed and out-of-limits commands are rejected without crashing."""

    def setup_method(self):
        self.parser = ArmCommandParser()

    @pytest.mark.parametrize("command,expected_error_substr", [
        ("BASE", "MISSING_OR_EXTRA_ANGLE_ARGUMENT"),
        ("SHOULDER", "MISSING_OR_EXTRA_ANGLE_ARGUMENT"),
        ("ELBOW", "MISSING_OR_EXTRA_ANGLE_ARGUMENT"),
        ("WRIST", "MISSING_OR_EXTRA_ANGLE_ARGUMENT"),
        ("WRIST_PITCH", "MISSING_OR_EXTRA_ANGLE_ARGUMENT"),
        ("WRIST_PITCH 999", "ANGLE_OUT_OF_LIMITS"),
        ("WRIST_PITCH hello", "NON_NUMERIC_ANGLE"),
        ("BASE 999", "ANGLE_OUT_OF_LIMITS"),
        ("BASE -50", "ANGLE_OUT_OF_LIMITS"),
        ("BASE hello", "NON_NUMERIC_ANGLE"),
        ("ELBOW abc", "NON_NUMERIC_ANGLE"),
        ("SHOULDER 10", "ANGLE_OUT_OF_LIMITS"),  # default min placeholder is 15.0
        ("SHOULDER 170", "ANGLE_OUT_OF_LIMITS"), # default max placeholder is 165.0
        ("UNKNOWN", "UNKNOWN_COMMAND"),
        ("FOO BAR", "UNKNOWN_COMMAND"),
        ("GRIPPER 123", "INVALID_GRIPPER_STATE"),
        ("GRIPPER", "INVALID_GRIPPER_FORMAT"),
        ("GRIPPER OPEN CLOSE", "INVALID_GRIPPER_FORMAT"),
        ("HOME extra", "UNEXPECTED_ARGUMENTS"),
        ("", "EMPTY_COMMAND"),
        ("   ", "EMPTY_COMMAND"),
    ])
    def test_invalid_commands(self, command, expected_error_substr):
        is_valid, serial_cmd, action_name, error = self.parser.parse(command)
        assert is_valid is False
        assert serial_cmd is None
        assert action_name is None
        assert error is not None
        assert expected_error_substr in error

    def test_non_string_command(self):
        is_valid, _, _, error = self.parser.parse(None)
        assert is_valid is False
        assert error == "EMPTY_COMMAND"


class TestConfigurableLimits:
    """Verify that servo limits remain centralized and configurable."""

    def test_custom_limits_enforced(self):
        custom_limits = {
            "BASE": JointRange(min_angle=45.0, max_angle=135.0),
            "SHOULDER": JointRange(min_angle=30.0, max_angle=90.0),
            "ELBOW": JointRange(min_angle=45.0, max_angle=120.0),
            "WRIST": JointRange(min_angle=10.0, max_angle=80.0),
        }
        parser = ArmCommandParser(joint_limits=custom_limits)

        # Inside custom limits
        assert parser.parse("BASE 90")[0] is True
        assert parser.parse("BASE 45")[0] is True
        assert parser.parse("BASE 135")[0] is True

        # Outside custom limits
        assert parser.parse("BASE 30")[0] is False
        assert parser.parse("BASE 140")[0] is False
        assert parser.parse("BASE 0")[0] is False
        assert parser.parse("BASE 180")[0] is False

    def test_runtime_limit_update(self):
        parser = ArmCommandParser()
        assert parser.parse("BASE 10")[0] is True

        # Restrict base to min 30
        parser.update_joint_limit("BASE", min_angle=30.0, max_angle=150.0)
        assert parser.parse("BASE 10")[0] is False
        assert parser.parse("BASE 90")[0] is True


class TestMockSerialInterface:
    """Verify mock serial communication behaves according to specification."""

    def setup_method(self):
        self.serial = SerialInterface(port='/dev/ttyACM0', mock_mode=True)
        connected, status = self.serial.connect()
        assert connected is True
        assert status == "CONNECTED_MOCK"

    def teardown_method(self):
        self.serial.disconnect()

    @pytest.mark.parametrize("cmd,expected_response", [
        ("HOME", "OK:HOME"),
        ("BASE:90", "OK:BASE"),
        ("SHOULDER:60", "OK:SHOULDER"),
        ("ELBOW:120", "OK:ELBOW"),
        ("WRIST:90", "OK:WRIST"),
        ("GRIPPER:OPEN", "OK:GRIPPER"),
        ("GRIPPER:CLOSE", "OK:GRIPPER"),
    ])
    def test_mock_valid_responses(self, cmd, expected_response):
        success, response = self.serial.send_and_receive(cmd)
        assert success is True
        assert response == expected_response

    @pytest.mark.parametrize("bad_cmd", [
        ("UNKNOWN:123"),
        ("INVALID"),
        ("GRIPPER:MAYBE"),
        ("BASE:NOT_A_NUMBER"),
    ])
    def test_mock_invalid_responses(self, bad_cmd):
        success, response = self.serial.send_and_receive(bad_cmd)
        assert success is False
        assert response == "ERROR:INVALID_COMMAND"


class TestSerialProtocolWireFormat:
    """Verify that all commands transmitted over the wire are properly newline-terminated."""

    def setup_method(self):
        self.serial = SerialInterface(port='/dev/ttyACM0', mock_mode=True)
        self.serial.connect()

    def teardown_method(self):
        self.serial.disconnect()

    @pytest.mark.parametrize("command,expected_wire", [
        ("HOME", "HOME\n"),
        ("BASE:90", "BASE:90\n"),
        ("SHOULDER:60", "SHOULDER:60\n"),
        ("ELBOW:120", "ELBOW:120\n"),
        ("WRIST:90", "WRIST:90\n"),
        ("GRIPPER:OPEN", "GRIPPER:OPEN\n"),
        ("GRIPPER:CLOSE", "GRIPPER:CLOSE\n"),
        ("  BASE:90  ", "BASE:90\n"),
    ])
    def test_newline_termination(self, command, expected_wire):
        self.serial.send_and_receive(command)
        assert self.serial.last_sent_command is not None
        assert self.serial.last_sent_command.endswith('\n')
        assert self.serial.last_sent_command == expected_wire

    @pytest.mark.parametrize("empty_cmd", ["", "   ", "\t", "\n"])
    def test_empty_command_rejection(self, empty_cmd):
        success, response = self.serial.send_and_receive(empty_cmd)
        assert success is False
        assert response == "ERROR:INVALID_COMMAND"


class TestMockSerialLifecycle:
    """Verify connection lifecycle in mock mode."""

    def test_lifecycle_connect_and_disconnect(self):
        ser = SerialInterface(port='/dev/ttyACM0', mock_mode=True)
        assert ser.is_connected is False

        connected, status = ser.connect()
        assert connected is True
        assert status == "CONNECTED_MOCK"
        assert ser.is_connected is True

        ser.disconnect()
        assert ser.is_connected is False

        # Safe repeated disconnect
        ser.disconnect()
        assert ser.is_connected is False

    def test_send_when_disconnected(self):
        ser = SerialInterface(port='/dev/ttyACM0', mock_mode=True)
        assert ser.is_connected is False

        success, response = ser.send_and_receive("HOME")
        assert success is False
        assert response == "ERROR:ARDUINO_DISCONNECTED"

    def test_reconnect_after_disconnect(self):
        ser = SerialInterface(port='/dev/ttyACM0', mock_mode=True)
        ser.connect()
        ser.disconnect()
        assert ser.is_connected is False

        # Reconnect
        connected, _ = ser.connect()
        assert connected is True
        success, response = ser.send_and_receive("HOME")
        assert success is True
        assert response == "OK:HOME"
        ser.disconnect()

    def test_mock_error_simulations(self):
        ser = SerialInterface(port='/dev/ttyACM0', mock_mode=True)
        ser.connect()

        # Simulate timeout
        ser.mock_simulate_timeout = True
        success, response = ser.send_and_receive("BASE:90")
        assert success is False
        assert response == "ERROR:SERIAL_TIMEOUT"
        ser.mock_simulate_timeout = False

        # Simulate disconnect
        ser.mock_simulate_disconnect = True
        success, response = ser.send_and_receive("BASE:90")
        assert success is False
        assert response == "ERROR:ARDUINO_DISCONNECTED"
        assert ser.is_connected is False


class TestRealSerialMockedPaths:
    """Verify real hardware serial logic (pyserial interactions, timeouts, exceptions)."""

    def test_real_serial_unavailable_without_pyserial(self):
        with patch('sorting_cell.serial_interface.SERIAL_AVAILABLE', False):
            ser = SerialInterface(port='/dev/ttyACM0', mock_mode=False, boot_delay=0)
            connected, status = ser.connect()
            assert connected is False
            assert status == "ERROR:SERIAL_UNAVAILABLE"
            assert ser.is_connected is False

    def test_real_serial_connection_failure(self):
        mock_serial_mod = MagicMock()
        mock_serial_mod.Serial.side_effect = Exception("Port /dev/ttyACM0 not found")

        with patch('sorting_cell.serial_interface.SERIAL_AVAILABLE', True), \
             patch('sorting_cell.serial_interface.serial', mock_serial_mod):
            ser = SerialInterface(port='/dev/ttyACM0', mock_mode=False, boot_delay=0)
            connected, status = ser.connect()
            assert connected is False
            assert status == "ERROR:SERIAL_CONNECTION"
            assert ser.is_connected is False

    def test_real_serial_successful_exchange(self):
        mock_serial_mod = MagicMock()
        mock_instance = MagicMock()
        mock_instance.is_open = True
        mock_instance.read.return_value = b"OK:BASE\n"
        mock_serial_mod.Serial.return_value = mock_instance

        with patch('sorting_cell.serial_interface.SERIAL_AVAILABLE', True), \
             patch('sorting_cell.serial_interface.serial', mock_serial_mod):
            ser = SerialInterface(port='/dev/ttyACM0', mock_mode=False, boot_delay=0)
            connected, _ = ser.connect()
            assert connected is True

            success, response = ser.send_and_receive("BASE:90")
            assert success is True
            assert response == "OK:BASE"
            mock_instance.write.assert_called_with(b"BASE:90\n")

    def test_real_serial_timeout_on_read(self):
        mock_serial_mod = MagicMock()
        mock_instance = MagicMock()
        mock_instance.is_open = True
        mock_instance.read.return_value = b""  # pyserial returns empty bytes on timeout
        mock_serial_mod.Serial.return_value = mock_instance

        with patch('sorting_cell.serial_interface.SERIAL_AVAILABLE', True), \
             patch('sorting_cell.serial_interface.serial', mock_serial_mod):
            ser = SerialInterface(port='/dev/ttyACM0', mock_mode=False, boot_delay=0)
            ser.connect()

            success, response = ser.send_and_receive("BASE:90")
            assert success is False
            assert response == "ERROR:SERIAL_TIMEOUT"

    def test_real_serial_write_failure(self):
        mock_serial_mod = MagicMock()
        mock_instance = MagicMock()
        mock_instance.is_open = True
        mock_instance.write.side_effect = Exception("Hardware unplugged during write")
        mock_serial_mod.Serial.return_value = mock_instance

        with patch('sorting_cell.serial_interface.SERIAL_AVAILABLE', True), \
             patch('sorting_cell.serial_interface.serial', mock_serial_mod):
            ser = SerialInterface(port='/dev/ttyACM0', mock_mode=False, boot_delay=0)
            ser.connect()

            success, response = ser.send_and_receive("BASE:90")
            assert success is False
            assert response == "ERROR:ARDUINO_DISCONNECTED"
            assert ser.is_connected is False

    def test_real_serial_read_failure(self):
        mock_serial_mod = MagicMock()
        mock_instance = MagicMock()
        mock_instance.is_open = True
        mock_instance.read.side_effect = Exception("Hardware unplugged during read")
        mock_serial_mod.Serial.return_value = mock_instance

        with patch('sorting_cell.serial_interface.SERIAL_AVAILABLE', True), \
             patch('sorting_cell.serial_interface.serial', mock_serial_mod):
            ser = SerialInterface(port='/dev/ttyACM0', mock_mode=False, boot_delay=0)
            ser.connect()

            success, response = ser.send_and_receive("BASE:90")
            assert success is False
            assert response == "ERROR:ARDUINO_DISCONNECTED"
            assert ser.is_connected is False

    def test_real_serial_malformed_response(self):
        mock_serial_mod = MagicMock()
        mock_instance = MagicMock()
        mock_instance.is_open = True
        mock_instance.read.return_value = b"UNKNOWN_GARBAGE\n"
        mock_serial_mod.Serial.return_value = mock_instance

        with patch('sorting_cell.serial_interface.SERIAL_AVAILABLE', True), \
             patch('sorting_cell.serial_interface.serial', mock_serial_mod):
            ser = SerialInterface(port='/dev/ttyACM0', mock_mode=False, boot_delay=0)
            ser.connect()

            success, response = ser.send_and_receive("BASE:90")
            assert success is False
            assert response == "ERROR:MALFORMED_RESPONSE:UNKNOWN_GARBAGE"

    def test_real_serial_arduino_error_response(self):
        mock_serial_mod = MagicMock()
        mock_instance = MagicMock()
        mock_instance.is_open = True
        mock_instance.read.return_value = b"ERROR:INVALID_COMMAND\n"
        mock_serial_mod.Serial.return_value = mock_instance

        with patch('sorting_cell.serial_interface.SERIAL_AVAILABLE', True), \
             patch('sorting_cell.serial_interface.serial', mock_serial_mod):
            ser = SerialInterface(port='/dev/ttyACM0', mock_mode=False, boot_delay=0)
            ser.connect()

            success, response = ser.send_and_receive("BASE:90")
            assert success is False
            assert response == "ERROR:INVALID_COMMAND"

    def test_real_serial_safe_disconnect(self):
        mock_serial_mod = MagicMock()
        mock_instance = MagicMock()
        mock_instance.is_open = True
        mock_serial_mod.Serial.return_value = mock_instance

        with patch('sorting_cell.serial_interface.SERIAL_AVAILABLE', True), \
             patch('sorting_cell.serial_interface.serial', mock_serial_mod):
            ser = SerialInterface(port='/dev/ttyACM0', mock_mode=False, boot_delay=0)
            ser.connect()
            assert ser.is_connected is True

            ser.disconnect()
            assert ser.is_connected is False
            mock_instance.close.assert_called_once()

            # Calling again should be safe and not re-call close
            ser.disconnect()
            assert ser.is_connected is False

    def test_real_serial_splits_concatenated_responses(self):
        """Regression test for the real-hardware fragmentation bug.

        On real hardware, two full 'OK:<JOINT>\\r\\n' responses can land in
        the same underlying serial read chunk when commands are issued back
        to back (e.g. a multi-joint move). The interface must return only
        the first complete response for the command that triggered it and
        keep the rest buffered for the next command -- it must not require
        another physical read, and it must not corrupt or drop either line.
        """
        mock_serial_mod = MagicMock()
        mock_instance = MagicMock()
        mock_instance.is_open = True
        mock_instance.read.return_value = b"OK:BASE\r\nOK:SHOULDER\r\n"
        mock_serial_mod.Serial.return_value = mock_instance

        with patch('sorting_cell.serial_interface.SERIAL_AVAILABLE', True), \
             patch('sorting_cell.serial_interface.serial', mock_serial_mod):
            ser = SerialInterface(port='/dev/ttyACM0', mock_mode=False, boot_delay=0)
            ser.connect()

            success, response = ser.send_and_receive("BASE:0")
            assert success is True
            assert response == "OK:BASE"

            # Nothing new arrives on the wire for the second command -- the
            # correct response must already be sitting in the buffer from
            # the first read.
            mock_instance.read.return_value = b""
            success, response = ser.send_and_receive("SHOULDER:120")
            assert success is True
            assert response == "OK:SHOULDER"

    def test_real_serial_boot_delay_before_first_command(self):
        """Opening the port resets an Arduino Uno via DTR; the interface must
        wait for it to finish booting, and clear any boot-noise bytes it
        emitted, before the first command can be sent."""
        mock_serial_mod = MagicMock()
        mock_instance = MagicMock()
        mock_instance.is_open = True
        mock_serial_mod.Serial.return_value = mock_instance

        with patch('sorting_cell.serial_interface.SERIAL_AVAILABLE', True), \
             patch('sorting_cell.serial_interface.serial', mock_serial_mod), \
             patch('sorting_cell.serial_interface.time.sleep') as mock_sleep:
            ser = SerialInterface(port='/dev/ttyACM0', mock_mode=False, boot_delay=2.5)
            ser.connect()

            mock_sleep.assert_called_once_with(2.5)
            mock_instance.reset_input_buffer.assert_called_once()


class TestArmControllerCoreIntegration:
    """Integration-style tests verifying the complete status feedback and error handling flow.

    Flow:
      /arm/command -> ArmCommandParser -> SerialInterface -> /arm/status
    """

    def setup_method(self):
        self.parser = ArmCommandParser()
        self.serial = SerialInterface(port='/dev/ttyACM0', mock_mode=True)
        self.serial.connect()
        self.controller = ArmControllerCore(
            parser=self.parser,
            serial_interface=self.serial,
        )

    def teardown_method(self):
        self.serial.disconnect()

    @pytest.mark.parametrize("command,expected_statuses", [
        ("BASE 90", ["EXECUTING:BASE", "OK:BASE"]),
        ("SHOULDER 60", ["EXECUTING:SHOULDER", "OK:SHOULDER"]),
        ("ELBOW 120", ["EXECUTING:ELBOW", "OK:ELBOW"]),
        ("WRIST 90", ["EXECUTING:WRIST", "OK:WRIST"]),
        ("WRIST_PITCH 45", ["EXECUTING:WRIST_PITCH", "OK:WRIST_PITCH"]),
        ("GRIPPER OPEN", ["EXECUTING:GRIPPER", "OK:GRIPPER"]),
        ("GRIPPER CLOSE", ["EXECUTING:GRIPPER", "OK:GRIPPER"]),
        ("HOME", ["EXECUTING:HOME", "OK:HOME"]),
    ])
    def test_successful_command_status_flow(self, command, expected_statuses):
        final_status = self.controller.process_command(command)
        assert self.controller.status_history == expected_statuses
        assert final_status == expected_statuses[-1]

    @pytest.mark.parametrize("bad_cmd", [
        "BASE 999",
        "BASE -50",
        "BASE hello",
        "BASE",
        "SHOULDER",
        "UNKNOWN",
        "GRIPPER 123",
        "HOME extra",
        "",
        "   ",
    ])
    def test_invalid_command_error_status_flow(self, bad_cmd):
        final_status = self.controller.process_command(bad_cmd)
        assert self.controller.status_history == ["ERROR:INVALID_COMMAND"]
        assert final_status == "ERROR:INVALID_COMMAND"

    def test_serial_timeout_status_flow(self):
        self.serial.mock_simulate_timeout = True
        final_status = self.controller.process_command("BASE 90")
        assert self.controller.status_history == ["EXECUTING:BASE", "ERROR:SERIAL_TIMEOUT"]
        assert final_status == "ERROR:SERIAL_TIMEOUT"

    def test_arduino_disconnected_status_flow(self):
        self.serial.mock_simulate_disconnect = True
        final_status = self.controller.process_command("BASE 90")
        assert self.controller.status_history == ["EXECUTING:BASE", "ERROR:ARDUINO_DISCONNECTED"]
        assert final_status == "ERROR:ARDUINO_DISCONNECTED"

    def test_serial_write_failure_status_flow(self):
        mock_serial_mod = MagicMock()
        mock_instance = MagicMock()
        mock_instance.is_open = True
        mock_instance.write.side_effect = Exception("USB cable yanked during write")
        mock_serial_mod.Serial.return_value = mock_instance

        with patch('sorting_cell.serial_interface.SERIAL_AVAILABLE', True), \
             patch('sorting_cell.serial_interface.serial', mock_serial_mod):
            real_ser = SerialInterface(port='/dev/ttyACM0', mock_mode=False, boot_delay=0)
            real_ser.connect()
            ctrl = ArmControllerCore(parser=self.parser, serial_interface=real_ser)
            final_status = ctrl.process_command("BASE 90")
            assert ctrl.status_history == ["EXECUTING:BASE", "ERROR:ARDUINO_DISCONNECTED"]
            assert final_status == "ERROR:ARDUINO_DISCONNECTED"

    def test_serial_read_failure_status_flow(self):
        mock_serial_mod = MagicMock()
        mock_instance = MagicMock()
        mock_instance.is_open = True
        mock_instance.read.side_effect = Exception("USB read error")
        mock_serial_mod.Serial.return_value = mock_instance

        with patch('sorting_cell.serial_interface.SERIAL_AVAILABLE', True), \
             patch('sorting_cell.serial_interface.serial', mock_serial_mod):
            real_ser = SerialInterface(port='/dev/ttyACM0', mock_mode=False, boot_delay=0)
            real_ser.connect()
            ctrl = ArmControllerCore(parser=self.parser, serial_interface=real_ser)
            final_status = ctrl.process_command("BASE 90")
            assert ctrl.status_history == ["EXECUTING:BASE", "ERROR:ARDUINO_DISCONNECTED"]
            assert final_status == "ERROR:ARDUINO_DISCONNECTED"

    def test_malformed_arduino_response_status_flow(self):
        mock_serial_mod = MagicMock()
        mock_instance = MagicMock()
        mock_instance.is_open = True
        mock_instance.read.return_value = b"CORRUPTED_SERIAL_STREAM\n"
        mock_serial_mod.Serial.return_value = mock_instance

        with patch('sorting_cell.serial_interface.SERIAL_AVAILABLE', True), \
             patch('sorting_cell.serial_interface.serial', mock_serial_mod):
            real_ser = SerialInterface(port='/dev/ttyACM0', mock_mode=False, boot_delay=0)
            real_ser.connect()
            ctrl = ArmControllerCore(parser=self.parser, serial_interface=real_ser)
            final_status = ctrl.process_command("BASE 90")
            assert ctrl.status_history == ["EXECUTING:BASE", "ERROR:MALFORMED_RESPONSE:CORRUPTED_SERIAL_STREAM"]
            assert final_status == "ERROR:MALFORMED_RESPONSE:CORRUPTED_SERIAL_STREAM"

    def test_arduino_error_response_status_flow(self):
        mock_serial_mod = MagicMock()
        mock_instance = MagicMock()
        mock_instance.is_open = True
        mock_instance.read.return_value = b"ERROR:INVALID_COMMAND\n"
        mock_serial_mod.Serial.return_value = mock_instance

        with patch('sorting_cell.serial_interface.SERIAL_AVAILABLE', True), \
             patch('sorting_cell.serial_interface.serial', mock_serial_mod):
            real_ser = SerialInterface(port='/dev/ttyACM0', mock_mode=False, boot_delay=0)
            real_ser.connect()
            ctrl = ArmControllerCore(parser=self.parser, serial_interface=real_ser)
            final_status = ctrl.process_command("BASE 90")
            assert ctrl.status_history == ["EXECUTING:BASE", "ERROR:INVALID_COMMAND"]
            assert final_status == "ERROR:INVALID_COMMAND"



class TestShippedPositionsWithinDefaultLimits:
    """Every measured position in config/arm_positions.yaml must pass the default
    ArmCommandParser limits, or the real arm rejects it at runtime."""

    def test_all_shipped_positions_pass_default_limits(self):
        from sorting_cell.positions import load_positions

        positions = load_positions(str(pkg_root / "config" / "arm_positions.yaml"))
        parser = ArmCommandParser()
        failures = []
        for name, position in positions.items():
            for command in position.as_commands():
                is_valid, _, _, error = parser.parse(command)
                if not is_valid:
                    failures.append(f"{name}: '{command}' -> {error}")
        assert not failures, "\n".join(failures)
