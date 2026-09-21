"""Unit tests for the high-level sorting sequence orchestration.

Covers the physical cell layout's actual flow: the arm hovers at
HOME_POSITION, descends to PICK_DOWN_POSITION to grip, returns to
HOME_POSITION, then carries the object over the fixed RFID reader
(RFID_SCAN_POSITION) before identification is possible. RFID detection is
therefore a mid-sequence blocking wait, not the trigger for the cycle.

HOME_POSITION here is the ROS-level named pickup-hover pose used by
sorting_controller's own sequencing (via BASE/SHOULDER/ELBOW/WRIST/
WRIST_PITCH commands, same as any other named position) -- NOT
arm_controller.ino's separate all-zeros firmware HOME command, which this
node no longer sends.

Uses a fake arm command sender and a fake RFID wait function (no rclpy, no
serial, no hardware) to verify: the full hover -> descend -> grip -> return
-> scan -> identify -> place sequence and its command order, unknown UID
handling, RFID timeout handling, invalid configuration handling (including
missing HOME_POSITION/PICK_DOWN_POSITION/RFID_SCAN_POSITION), and
mid-sequence arm/gripper failure recovery.
"""

import sys
from pathlib import Path

import pytest

pkg_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(pkg_root))

from sorting_cell.object_map import ObjectMap
from sorting_cell.positions import ArmPosition
from sorting_cell.sorting_controller import SortingControllerCore


OBJECT_MAP = ObjectMap.from_dict({
    "objects": {
        "UID_A": {"object_name": "OBJECT_A", "bin": "BIN_A"},
        "UID_B": {"object_name": "OBJECT_B", "bin": "BIN_B"},
    }
})

# Every position uses distinct joint values so each step's commands are
# unambiguous in assertions below -- except HOME_POSITION, which the pick
# sequence legitimately visits twice (hover approach, then return after
# grip) and therefore emits the same five commands both times.
POSITIONS = {
    "HOME_POSITION": ArmPosition(base=1, shoulder=2, elbow=3, wrist=4),
    "PICK_DOWN_POSITION": ArmPosition(base=11, shoulder=12, elbow=13, wrist=14),
    "RFID_SCAN_POSITION": ArmPosition(base=21, shoulder=22, elbow=23, wrist=24),
    "BIN_A_POSITION": ArmPosition(base=31, shoulder=32, elbow=33, wrist=34),
}

HOME_COMMANDS = ["BASE 1", "SHOULDER 2", "ELBOW 3", "WRIST 4", "WRIST_PITCH 0.0"]
PICK_DOWN_COMMANDS = ["BASE 11", "SHOULDER 12", "ELBOW 13", "WRIST 14", "WRIST_PITCH 0.0"]
RFID_SCAN_COMMANDS = ["BASE 21", "SHOULDER 22", "ELBOW 23", "WRIST 24", "WRIST_PITCH 0.0"]
BIN_A_COMMANDS = ["BASE 31", "SHOULDER 32", "ELBOW 33", "WRIST 34", "WRIST_PITCH 0.0"]

FULL_PICK_TO_SCAN_COMMANDS = (
    HOME_COMMANDS  # move to HOME_POSITION (pickup-hover)
    + PICK_DOWN_COMMANDS  # move to PICK_DOWN_POSITION
    + ["GRIPPER CLOSE"]
    + HOME_COMMANDS  # return to HOME_POSITION
    + RFID_SCAN_COMMANDS  # move to RFID_SCAN_POSITION
)


class FakeArmSender:
    """Records every command sent and returns a scripted (success, status).

    A failure can be scripted either by exact command text
    (failing_command, matching every time that text is sent) or by call
    position (fail_at_call_index, 0-based, matching only that one call).
    The latter is needed to target one occurrence of a position -- like
    HOME_POSITION -- that is sent more than once in a cycle and would
    otherwise be indistinguishable by text alone.
    """

    def __init__(self, failing_command=None, fail_at_call_index=None):
        self.sent_commands = []
        self.failing_command = failing_command
        self.fail_at_call_index = fail_at_call_index

    def __call__(self, command):
        index = len(self.sent_commands)
        self.sent_commands.append(command)
        fails = (
            self.failing_command is not None and command == self.failing_command
        ) or (
            self.fail_at_call_index is not None and index == self.fail_at_call_index
        )
        if fails:
            return False, "ERROR:SERIAL_TIMEOUT"
        action = command.split()[0]
        return True, f"OK:{action}"


class FakeRfidWaiter:
    """Returns a scripted UID (or None to simulate a timeout) when waited on."""

    def __init__(self, uid_to_return):
        self.uid_to_return = uid_to_return
        self.calls = []

    def __call__(self, timeout):
        self.calls.append(timeout)
        return self.uid_to_return


def make_core(sender, rfid_waiter, object_map=OBJECT_MAP, positions=POSITIONS, rfid_timeout=15.0):
    return SortingControllerCore(
        object_map=object_map,
        positions=positions,
        send_arm_command=sender,
        wait_for_rfid_tag=rfid_waiter,
        rfid_timeout=rfid_timeout,
    )


class TestSuccessfulCycle:
    def setup_method(self):
        self.sender = FakeArmSender()
        self.rfid_waiter = FakeRfidWaiter("UID_A")
        self.core = make_core(self.sender, self.rfid_waiter)

    def test_full_command_sequence_generated_in_order(self):
        final_status = self.core.run_cycle()

        assert self.sender.sent_commands == (
            FULL_PICK_TO_SCAN_COMMANDS
            + BIN_A_COMMANDS  # move to bin A
            + ["GRIPPER OPEN"]
            + HOME_COMMANDS  # final move back to HOME_POSITION
        )
        assert final_status == "IDLE"

    def test_rfid_wait_happens_after_pick_and_before_bin_move(self):
        """The arm must have hovered, descended, gripped, returned to
        HOME_POSITION, and parked at the scan position before any RFID wait
        occurs, and no bin motion may occur before that wait resolves."""
        sender = FakeArmSender()
        captured = {}

        def waiter(timeout):
            captured["commands_at_wait_time"] = list(sender.sent_commands)
            captured["timeout"] = timeout
            return "UID_A"

        core = make_core(sender, waiter)
        core.run_cycle()

        assert captured["commands_at_wait_time"] == FULL_PICK_TO_SCAN_COMMANDS
        assert captured["timeout"] == 15.0

    def test_status_history_reports_full_flow(self):
        self.core.run_cycle()
        assert self.core.status_history == [
            "CYCLE_START",
            "EXECUTING:MOVE_TO_HOME_POSITION",
            "EXECUTING:MOVE_TO_PICK_DOWN_POSITION",
            "EXECUTING:GRIP",
            "EXECUTING:RETURN_TO_HOME_POSITION",
            "EXECUTING:MOVE_TO_RFID_SCAN",
            "WAITING_FOR_RFID",
            "RFID_DETECTED:UID_A",
            "IDENTIFIED:OBJECT_A",
            "BIN_SELECTED:BIN_A",
            "EXECUTING:MOVE_TO_BIN:BIN_A",
            "EXECUTING:RELEASE",
            "REPORT:SORTED:UID_A:OBJECT_A:BIN_A",
            "IDLE",
        ]

    def test_correct_bin_selected_for_different_uid(self):
        positions = dict(POSITIONS)
        positions["BIN_B_POSITION"] = ArmPosition(base=135, shoulder=90, elbow=90, wrist=90)
        rfid_waiter = FakeRfidWaiter("UID_B")
        core = make_core(self.sender, rfid_waiter, positions=positions)
        core.run_cycle()
        assert "BASE 135" in self.sender.sent_commands


class TestWristPitchCommand:
    """Positions may optionally specify wrist_pitch; when they do, the same
    move-to-position loop that sends BASE/SHOULDER/ELBOW/WRIST must also
    send WRIST_PITCH and wait for its OK, via ArmPosition.as_commands()."""

    def test_position_with_explicit_wrist_pitch_sends_it_after_wrist(self):
        positions = dict(POSITIONS)
        positions["PICK_DOWN_POSITION"] = ArmPosition(
            base=11, shoulder=12, elbow=13, wrist=14, wrist_pitch=45
        )
        sender = FakeArmSender()
        rfid_waiter = FakeRfidWaiter("UID_A")
        core = make_core(sender, rfid_waiter, positions=positions)

        core.run_cycle()

        # WRIST_PITCH follows WRIST for the PICK_DOWN_POSITION move, which
        # comes right after the initial 5-command move to HOME_POSITION and
        # precedes the GRIPPER CLOSE that follows the descend-to-grip move.
        assert sender.sent_commands[5:11] == [
            "BASE 11", "SHOULDER 12", "ELBOW 13", "WRIST 14", "WRIST_PITCH 45",
            "GRIPPER CLOSE",
        ]

    def test_position_without_wrist_pitch_defaults_to_zero(self):
        # None of POSITIONS's entries specify wrist_pitch.
        core = make_core(FakeArmSender(), FakeRfidWaiter("UID_A"))
        core.run_cycle()
        assert "WRIST_PITCH 0.0" in FULL_PICK_TO_SCAN_COMMANDS

    def test_wrist_pitch_failure_is_reported_as_arm_failure(self):
        positions = dict(POSITIONS)
        positions["PICK_DOWN_POSITION"] = ArmPosition(
            base=11, shoulder=12, elbow=13, wrist=14, wrist_pitch=45
        )
        sender = FakeArmSender(failing_command="WRIST_PITCH 45")
        rfid_waiter = FakeRfidWaiter("UID_A")
        core = make_core(sender, rfid_waiter, positions=positions)

        final_status = core.run_cycle()

        assert final_status.startswith("ERROR:ARM_FAILURE:MOVE_TO_PICK_DOWN_POSITION")
        # A failure reaching the arm before the RFID wait must never invoke it.
        assert rfid_waiter.calls == []


class TestRfidTimeout:
    def test_timeout_aborts_after_scan_position_and_homes(self):
        sender = FakeArmSender()
        rfid_waiter = FakeRfidWaiter(None)  # simulates no tag detected in time
        core = make_core(sender, rfid_waiter, rfid_timeout=3.0)

        final_status = core.run_cycle()

        assert final_status == "ERROR:RFID_TIMEOUT"
        assert sender.sent_commands == FULL_PICK_TO_SCAN_COMMANDS + HOME_COMMANDS
        assert rfid_waiter.calls == [3.0]
        assert core.status_history[-1] == "IDLE"
        assert "WAITING_FOR_RFID" in core.status_history
        # No bin was ever selected -- the object is still held, un-identified.
        assert not any(s.startswith("BIN_SELECTED") for s in core.status_history)


class TestUnknownUid:
    def test_unknown_uid_after_scan_sends_no_bin_commands(self):
        sender = FakeArmSender()
        rfid_waiter = FakeRfidWaiter("UID_NOT_REGISTERED")
        core = make_core(sender, rfid_waiter)

        final_status = core.run_cycle()

        assert final_status == "ERROR:UNKNOWN_UID:UID_NOT_REGISTERED"
        # Pick + carry-to-scan already happened (object is physically held);
        # only a best-effort return to HOME_POSITION follows -- no bin motion.
        assert sender.sent_commands == FULL_PICK_TO_SCAN_COMMANDS + HOME_COMMANDS
        assert core.status_history == [
            "CYCLE_START",
            "EXECUTING:MOVE_TO_HOME_POSITION",
            "EXECUTING:MOVE_TO_PICK_DOWN_POSITION",
            "EXECUTING:GRIP",
            "EXECUTING:RETURN_TO_HOME_POSITION",
            "EXECUTING:MOVE_TO_RFID_SCAN",
            "WAITING_FOR_RFID",
            "RFID_DETECTED:UID_NOT_REGISTERED",
            "ERROR:UNKNOWN_UID:UID_NOT_REGISTERED",
            "IDLE",
        ]


class TestInvalidConfiguration:
    def test_missing_home_position_aborts_before_any_motion(self):
        sender = FakeArmSender()
        rfid_waiter = FakeRfidWaiter("UID_A")
        positions_missing_home = {k: v for k, v in POSITIONS.items() if k != "HOME_POSITION"}
        core = make_core(sender, rfid_waiter, positions=positions_missing_home)

        final_status = core.run_cycle()

        assert "ERROR:INVALID_CONFIGURATION" in final_status
        assert "HOME_POSITION" in final_status
        # Best-effort recovery can't move to HOME_POSITION either, since it's
        # undefined -- the arm must never move at all, and this must not crash.
        assert sender.sent_commands == []
        assert rfid_waiter.calls == []

    def test_missing_rfid_scan_position_aborts_before_any_motion(self):
        sender = FakeArmSender()
        rfid_waiter = FakeRfidWaiter("UID_A")
        positions_missing_scan = {k: v for k, v in POSITIONS.items() if k != "RFID_SCAN_POSITION"}
        core = make_core(sender, rfid_waiter, positions=positions_missing_scan)

        final_status = core.run_cycle()

        assert "ERROR:INVALID_CONFIGURATION" in final_status
        assert "RFID_SCAN_POSITION" in final_status
        # Config is invalid for every cycle regardless of the object, so the
        # arm must never move at all -- only the best-effort HOME_POSITION recovery.
        assert sender.sent_commands == HOME_COMMANDS
        assert rfid_waiter.calls == []

    def test_missing_pick_down_position_aborts_before_any_motion(self):
        sender = FakeArmSender()
        rfid_waiter = FakeRfidWaiter("UID_A")
        positions_missing_pick_down = {
            k: v for k, v in POSITIONS.items() if k != "PICK_DOWN_POSITION"
        }
        core = make_core(sender, rfid_waiter, positions=positions_missing_pick_down)

        final_status = core.run_cycle()

        assert "ERROR:INVALID_CONFIGURATION" in final_status
        assert "PICK_DOWN_POSITION" in final_status
        assert sender.sent_commands == HOME_COMMANDS
        assert rfid_waiter.calls == []

    def test_missing_bin_position_is_reported_after_identification(self):
        sender = FakeArmSender()
        rfid_waiter = FakeRfidWaiter("UID_A")
        positions_missing_bin = {k: v for k, v in POSITIONS.items() if k != "BIN_A_POSITION"}
        core = make_core(sender, rfid_waiter, positions=positions_missing_bin)

        final_status = core.run_cycle()

        assert "ERROR:INVALID_CONFIGURATION" in final_status
        assert "BIN_A_POSITION" in final_status
        # Pick + scan already happened; only a best-effort return to
        # HOME_POSITION follows.
        assert sender.sent_commands == FULL_PICK_TO_SCAN_COMMANDS + HOME_COMMANDS


class TestMidSequenceArmFailure:
    @pytest.mark.parametrize("sender_kwargs,expected_prefix", [
        ({"fail_at_call_index": 1}, "ERROR:ARM_FAILURE:MOVE_TO_HOME_POSITION"),  # SHOULDER 2
        ({"fail_at_call_index": 6}, "ERROR:ARM_FAILURE:MOVE_TO_PICK_DOWN_POSITION"),  # SHOULDER 12
        ({"failing_command": "GRIPPER CLOSE"}, "ERROR:ARM_FAILURE:GRIP"),
        ({"fail_at_call_index": 12}, "ERROR:ARM_FAILURE:RETURN_TO_HOME_POSITION"),  # 2nd SHOULDER 2
        ({"failing_command": "BASE 21"}, "ERROR:ARM_FAILURE:MOVE_TO_RFID_SCAN"),
    ])
    def test_failure_before_rfid_wait_aborts_and_recovers_home(self, sender_kwargs, expected_prefix):
        sender = FakeArmSender(**sender_kwargs)
        rfid_waiter = FakeRfidWaiter("UID_A")
        core = make_core(sender, rfid_waiter)

        final_status = core.run_cycle()

        assert final_status.startswith(expected_prefix)
        assert core.status_history[-1] == "IDLE"
        # Best-effort recovery moves back to HOME_POSITION (5 commands),
        # not a literal "HOME" command.
        assert sender.sent_commands[-5:] == HOME_COMMANDS
        # A failure reaching the arm before the RFID wait must never invoke it.
        assert rfid_waiter.calls == []

    @pytest.mark.parametrize("failing_command,expected_prefix", [
        ("BASE 31", "ERROR:ARM_FAILURE:MOVE_TO_BIN:BIN_A"),
        ("GRIPPER OPEN", "ERROR:ARM_FAILURE:RELEASE"),
    ])
    def test_failure_after_rfid_detection_aborts_and_attempts_home(self, failing_command, expected_prefix):
        sender = FakeArmSender(failing_command=failing_command)
        rfid_waiter = FakeRfidWaiter("UID_A")
        core = make_core(sender, rfid_waiter)

        final_status = core.run_cycle()

        assert final_status.startswith(expected_prefix)
        assert core.status_history[-1] == "IDLE"
        assert sender.sent_commands[-5:] == HOME_COMMANDS
        assert rfid_waiter.calls == [15.0]

    def test_final_home_move_failing_is_reported(self):
        # Fail only the first command of the FINAL move to HOME_POSITION
        # (after a successful release), not the identical-valued approach
        # move earlier in the same cycle.
        fail_index = len(FULL_PICK_TO_SCAN_COMMANDS) + len(BIN_A_COMMANDS) + 1  # + GRIPPER OPEN
        sender = FakeArmSender(fail_at_call_index=fail_index)
        rfid_waiter = FakeRfidWaiter("UID_A")
        core = make_core(sender, rfid_waiter)

        final_status = core.run_cycle()

        assert final_status == "ERROR:ARM_FAILURE:HOME:BASE 1:ERROR:SERIAL_TIMEOUT"
        # This is a hard failure at the very last step, not routed through
        # _fail(), so no further recovery attempt follows it.
        assert core.status_history[-1] == final_status
