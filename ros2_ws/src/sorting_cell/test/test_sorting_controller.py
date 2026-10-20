"""Unit tests for the high-level sorting sequence orchestration.

Covers the physical cell layout's actual flow: the arm starts at
HOME_POSITION, opens the gripper, descends to PICK_DOWN_POSITION to grip,
lifts to LIFT_POSITION, then carries the object over the fixed RFID reader
(RFID_SCAN_POSITION) before identification is possible, and passes through
FLOATING_POSITION on the way to the selected bin. RFID detection is
therefore a mid-sequence blocking wait, not the trigger for the cycle.

HOME_POSITION here is the ROS-level named cycle start/end pose used by
sorting_controller's own sequencing (via BASE/SHOULDER/ELBOW/WRIST/
WRIST_PITCH commands, same as any other named position) -- NOT
arm_controller.ino's separate all-zeros firmware HOME command, which this
node no longer sends.

Uses a fake arm command sender and a fake RFID wait function (no rclpy, no
serial, no hardware) to verify: the full home -> open -> descend -> grip
-> lift -> scan -> identify -> float -> place sequence and its command order,
unknown UID handling, RFID timeout handling, invalid configuration handling
(including missing HOME_POSITION/PICK_DOWN_POSITION/LIFT_POSITION/
RFID_SCAN_POSITION/FLOATING_POSITION),
mid-sequence arm/gripper failure recovery, and that the post-OK settle
waits (via an injected fake sleep, so tests run instantly) happen in the
right place in the sequence, including the per-joint gap that makes a
position's joints move one at a time.
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
        "1DF058BA141080": {"object_name": "OBJECT_A2", "bin": "BIN_A"},
        "1DEC58BA141080": {"object_name": "OBJECT_B2", "bin": "BIN_B"},
    }
})

# Every position uses distinct joint values so each step's commands are
# unambiguous in assertions below -- except HOME_POSITION, which the cycle
# legitimately visits twice (start and end) and therefore emits the same
# five commands both times. GRIPPER OPEN is likewise sent twice (before
# descending, and to release at the bin).
POSITIONS = {
    "HOME_POSITION": ArmPosition(base=1, shoulder=2, elbow=3, wrist=4),
    "PICK_DOWN_POSITION": ArmPosition(base=11, shoulder=12, elbow=13, wrist=14),
    "LIFT_POSITION": ArmPosition(base=51, shoulder=52, elbow=53, wrist=54),
    "RFID_SCAN_POSITION": ArmPosition(base=21, shoulder=22, elbow=23, wrist=24),
    "FLOATING_POSITION": ArmPosition(base=41, shoulder=42, elbow=43, wrist=44),
    "BIN_A_POSITION": ArmPosition(base=31, shoulder=32, elbow=33, wrist=34),
}

HOME_COMMANDS = ["BASE 1", "SHOULDER 2", "ELBOW 3", "WRIST 4", "WRIST_PITCH 0.0"]
PICK_DOWN_COMMANDS = ["BASE 11", "SHOULDER 12", "ELBOW 13", "WRIST 14", "WRIST_PITCH 0.0"]
LIFT_COMMANDS = ["BASE 51", "SHOULDER 52", "ELBOW 53", "WRIST 54", "WRIST_PITCH 0.0"]
RFID_SCAN_COMMANDS = ["BASE 21", "SHOULDER 22", "ELBOW 23", "WRIST 24", "WRIST_PITCH 0.0"]
FLOATING_COMMANDS = ["BASE 41", "SHOULDER 42", "ELBOW 43", "WRIST 44", "WRIST_PITCH 0.0"]
BIN_A_COMMANDS = ["BASE 31", "SHOULDER 32", "ELBOW 33", "WRIST 34", "WRIST_PITCH 0.0"]

FULL_PICK_TO_SCAN_COMMANDS = (
    HOME_COMMANDS  # move to HOME_POSITION
    + ["GRIPPER OPEN"]  # make sure the gripper is open before descending
    + PICK_DOWN_COMMANDS  # move to PICK_DOWN_POSITION
    + ["GRIPPER CLOSE"]
    + LIFT_COMMANDS  # lift clear of the pickup area
    + RFID_SCAN_COMMANDS  # then to RFID_SCAN_POSITION, no return to HOME
)

# Call indices of the two GRIPPER OPEN commands in a successful 1DF058BA141080 (OBJECT_A2 -> BIN_A) cycle.
OPEN_BEFORE_PICK_INDEX = len(HOME_COMMANDS)
RELEASE_INDEX = len(FULL_PICK_TO_SCAN_COMMANDS) + len(FLOATING_COMMANDS) + len(BIN_A_COMMANDS)


class FakeArmSender:
    """Records every command sent and returns a scripted (success, status).

    A failure can be scripted either by exact command text
    (failing_command, matching every time that text is sent) or by call
    position (fail_at_call_index, 0-based, matching only that one call).
    The latter is needed to target one occurrence of a command -- like
    HOME_POSITION or GRIPPER OPEN -- that is sent more than once in a cycle
    and would otherwise be indistinguishable by text alone.
    """

    def __init__(self, failing_command=None, fail_at_call_index=None, timeline=None):
        self.sent_commands = []
        self.failing_command = failing_command
        self.fail_at_call_index = fail_at_call_index
        self.timeline = timeline

    def __call__(self, command):
        index = len(self.sent_commands)
        self.sent_commands.append(command)
        if self.timeline is not None:
            self.timeline.append(command)
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


class FakeSleep:
    """Records each requested sleep instead of blocking.

    If given a timeline (shared with a FakeArmSender), each sleep is also
    appended to it as ("SLEEP", seconds), interleaved with the commands, so
    tests can assert exactly where each settle wait falls in the sequence.
    """

    def __init__(self, timeline=None):
        self.calls = []
        self.timeline = timeline

    def __call__(self, seconds):
        self.calls.append(seconds)
        if self.timeline is not None:
            self.timeline.append(("SLEEP", seconds))


def make_core(
    sender,
    rfid_waiter,
    object_map=OBJECT_MAP,
    positions=POSITIONS,
    rfid_timeout=15.0,
    sleep=None,
    **settle_kwargs,
):
    return SortingControllerCore(
        object_map=object_map,
        positions=positions,
        send_arm_command=sender,
        wait_for_rfid_tag=rfid_waiter,
        rfid_timeout=rfid_timeout,
        sleep=sleep if sleep is not None else FakeSleep(),
        **settle_kwargs,
    )


class TestSuccessfulCycle:
    def setup_method(self):
        self.sender = FakeArmSender()
        self.rfid_waiter = FakeRfidWaiter("1DF058BA141080")
        self.core = make_core(self.sender, self.rfid_waiter)

    def test_full_command_sequence_generated_in_order(self):
        final_status = self.core.run_cycle()

        assert self.sender.sent_commands == (
            FULL_PICK_TO_SCAN_COMMANDS
            + FLOATING_COMMANDS  # move to FLOATING_POSITION
            + BIN_A_COMMANDS  # move to bin A
            + ["GRIPPER OPEN"]  # release
            + HOME_COMMANDS  # final move back to HOME_POSITION
        )
        assert final_status == "IDLE"

    def test_gripper_opened_before_descending(self):
        self.core.run_cycle()
        assert self.sender.sent_commands[:len(HOME_COMMANDS) + 1] == HOME_COMMANDS + ["GRIPPER OPEN"]
        assert self.sender.sent_commands[OPEN_BEFORE_PICK_INDEX + 1:OPEN_BEFORE_PICK_INDEX + 6] == (
            PICK_DOWN_COMMANDS
        )

    def test_lift_then_scan_after_grip_with_no_return_to_home(self):
        self.core.run_cycle()
        close = self.sender.sent_commands.index("GRIPPER CLOSE")
        assert self.sender.sent_commands[close + 1:close + 11] == LIFT_COMMANDS + RFID_SCAN_COMMANDS
        # HOME_POSITION is only visited at the start and end of the cycle.
        assert self.sender.sent_commands.count("BASE 1") == 2

    def test_floating_move_between_rfid_scan_and_bin(self):
        sender = FakeArmSender()
        captured = {}

        def waiter(timeout):
            captured["commands_at_wait_time"] = list(sender.sent_commands)
            return "1DF058BA141080"

        core = make_core(sender, waiter)
        core.run_cycle()

        after_wait = sender.sent_commands[len(captured["commands_at_wait_time"]):]
        assert after_wait[:10] == FLOATING_COMMANDS + BIN_A_COMMANDS

    def test_rfid_wait_happens_after_pick_and_before_bin_move(self):
        """The arm must have homed, opened the gripper, descended, gripped,
        and parked at the scan position before any RFID wait occurs, and no
        floating or bin motion may occur before that wait resolves."""
        sender = FakeArmSender()
        captured = {}

        def waiter(timeout):
            captured["commands_at_wait_time"] = list(sender.sent_commands)
            captured["timeout"] = timeout
            return "1DF058BA141080"

        core = make_core(sender, waiter)
        core.run_cycle()

        assert captured["commands_at_wait_time"] == FULL_PICK_TO_SCAN_COMMANDS
        assert captured["timeout"] == 15.0

    def test_status_history_reports_full_flow(self):
        self.core.run_cycle()
        assert self.core.status_history == [
            "CYCLE_START",
            "EXECUTING:MOVE_TO_HOME_POSITION",
            "EXECUTING:OPEN_GRIPPER",
            "EXECUTING:MOVE_TO_PICK_DOWN_POSITION",
            "EXECUTING:GRIP",
            "EXECUTING:LIFT_AFTER_GRIP",
            "EXECUTING:MOVE_TO_RFID_SCAN",
            "WAITING_FOR_RFID",
            "RFID_DETECTED:1DF058BA141080",
            "IDENTIFIED:OBJECT_A2",
            "BIN_SELECTED:BIN_A",
            "EXECUTING:MOVE_TO_FLOATING",
            "EXECUTING:MOVE_TO_BIN:BIN_A",
            "EXECUTING:RELEASE",
            "REPORT:SORTED:1DF058BA141080:OBJECT_A2:BIN_A",
            "IDLE",
        ]

    def test_correct_bin_selected_for_different_uid(self):
        positions = dict(POSITIONS)
        positions["BIN_B_POSITION"] = ArmPosition(base=135, shoulder=90, elbow=90, wrist=90)
        rfid_waiter = FakeRfidWaiter("1DEC58BA141080")
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
        rfid_waiter = FakeRfidWaiter("1DF058BA141080")
        core = make_core(sender, rfid_waiter, positions=positions)

        core.run_cycle()

        # WRIST_PITCH follows WRIST for the PICK_DOWN_POSITION move, which
        # comes right after the initial 5-command move to HOME_POSITION and
        # the GRIPPER OPEN, and precedes the GRIPPER CLOSE.
        assert sender.sent_commands[5:12] == [
            "GRIPPER OPEN",
            "BASE 11", "SHOULDER 12", "ELBOW 13", "WRIST 14", "WRIST_PITCH 45",
            "GRIPPER CLOSE",
        ]

    def test_position_without_wrist_pitch_defaults_to_zero(self):
        # None of POSITIONS's entries specify wrist_pitch.
        core = make_core(FakeArmSender(), FakeRfidWaiter("1DF058BA141080"))
        core.run_cycle()
        assert "WRIST_PITCH 0.0" in FULL_PICK_TO_SCAN_COMMANDS

    def test_wrist_pitch_failure_is_reported_as_arm_failure(self):
        positions = dict(POSITIONS)
        positions["PICK_DOWN_POSITION"] = ArmPosition(
            base=11, shoulder=12, elbow=13, wrist=14, wrist_pitch=45
        )
        sender = FakeArmSender(failing_command="WRIST_PITCH 45")
        rfid_waiter = FakeRfidWaiter("1DF058BA141080")
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
        assert "EXECUTING:MOVE_TO_FLOATING" not in core.status_history


class TestUnknownUid:
    def test_unknown_uid_after_scan_sends_no_bin_commands(self):
        sender = FakeArmSender()
        rfid_waiter = FakeRfidWaiter("UID_NOT_REGISTERED")
        core = make_core(sender, rfid_waiter)

        final_status = core.run_cycle()

        assert final_status == "ERROR:UNKNOWN_UID:UID_NOT_REGISTERED"
        # Pick + carry-to-scan already happened (object is physically held);
        # only a best-effort return to HOME_POSITION follows -- no floating
        # or bin motion.
        assert sender.sent_commands == FULL_PICK_TO_SCAN_COMMANDS + HOME_COMMANDS
        assert core.status_history == [
            "CYCLE_START",
            "EXECUTING:MOVE_TO_HOME_POSITION",
            "EXECUTING:OPEN_GRIPPER",
            "EXECUTING:MOVE_TO_PICK_DOWN_POSITION",
            "EXECUTING:GRIP",
            "EXECUTING:LIFT_AFTER_GRIP",
            "EXECUTING:MOVE_TO_RFID_SCAN",
            "WAITING_FOR_RFID",
            "RFID_DETECTED:UID_NOT_REGISTERED",
            "ERROR:UNKNOWN_UID:UID_NOT_REGISTERED",
            "IDLE",
        ]


class TestInvalidConfiguration:
    def test_missing_home_position_aborts_before_any_motion(self):
        sender = FakeArmSender()
        rfid_waiter = FakeRfidWaiter("1DF058BA141080")
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
        rfid_waiter = FakeRfidWaiter("1DF058BA141080")
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
        rfid_waiter = FakeRfidWaiter("1DF058BA141080")
        positions_missing_pick_down = {
            k: v for k, v in POSITIONS.items() if k != "PICK_DOWN_POSITION"
        }
        core = make_core(sender, rfid_waiter, positions=positions_missing_pick_down)

        final_status = core.run_cycle()

        assert "ERROR:INVALID_CONFIGURATION" in final_status
        assert "PICK_DOWN_POSITION" in final_status
        assert sender.sent_commands == HOME_COMMANDS
        assert rfid_waiter.calls == []

    def test_missing_lift_position_aborts_before_any_motion(self):
        sender = FakeArmSender()
        rfid_waiter = FakeRfidWaiter("1DF058BA141080")
        positions_missing_lift = {k: v for k, v in POSITIONS.items() if k != "LIFT_POSITION"}
        core = make_core(sender, rfid_waiter, positions=positions_missing_lift)

        final_status = core.run_cycle()

        assert "ERROR:INVALID_CONFIGURATION" in final_status
        assert "LIFT_POSITION" in final_status
        assert sender.sent_commands == HOME_COMMANDS
        assert rfid_waiter.calls == []

    def test_missing_floating_position_aborts_before_any_motion(self):
        sender = FakeArmSender()
        rfid_waiter = FakeRfidWaiter("1DF058BA141080")
        positions_missing_floating = {
            k: v for k, v in POSITIONS.items() if k != "FLOATING_POSITION"
        }
        core = make_core(sender, rfid_waiter, positions=positions_missing_floating)

        final_status = core.run_cycle()

        assert "ERROR:INVALID_CONFIGURATION" in final_status
        assert "FLOATING_POSITION" in final_status
        # Checked at startup, not when the floating move is reached, so the
        # object is never picked up: only the best-effort HOME_POSITION recovery.
        assert sender.sent_commands == HOME_COMMANDS
        assert rfid_waiter.calls == []

    def test_missing_bin_position_is_reported_after_identification(self):
        sender = FakeArmSender()
        rfid_waiter = FakeRfidWaiter("1DF058BA141080")
        positions_missing_bin = {k: v for k, v in POSITIONS.items() if k != "BIN_A_POSITION"}
        core = make_core(sender, rfid_waiter, positions=positions_missing_bin)

        final_status = core.run_cycle()

        assert "ERROR:INVALID_CONFIGURATION" in final_status
        assert "BIN_A_POSITION" in final_status
        # Pick + scan already happened; the bin is checked before the floating
        # move, so only a best-effort return to HOME_POSITION follows.
        assert sender.sent_commands == FULL_PICK_TO_SCAN_COMMANDS + HOME_COMMANDS


class TestMidSequenceArmFailure:
    @pytest.mark.parametrize("sender_kwargs,expected_prefix", [
        ({"fail_at_call_index": 1}, "ERROR:ARM_FAILURE:MOVE_TO_HOME_POSITION"),  # SHOULDER 2
        ({"fail_at_call_index": OPEN_BEFORE_PICK_INDEX}, "ERROR:ARM_FAILURE:OPEN_GRIPPER"),
        ({"fail_at_call_index": 7}, "ERROR:ARM_FAILURE:MOVE_TO_PICK_DOWN_POSITION"),  # SHOULDER 12
        ({"failing_command": "GRIPPER CLOSE"}, "ERROR:ARM_FAILURE:GRIP"),
        ({"failing_command": "BASE 51"}, "ERROR:ARM_FAILURE:LIFT_AFTER_GRIP"),
        ({"failing_command": "BASE 21"}, "ERROR:ARM_FAILURE:MOVE_TO_RFID_SCAN"),
    ])
    def test_failure_before_rfid_wait_aborts_and_recovers_home(self, sender_kwargs, expected_prefix):
        sender = FakeArmSender(**sender_kwargs)
        rfid_waiter = FakeRfidWaiter("1DF058BA141080")
        core = make_core(sender, rfid_waiter)

        final_status = core.run_cycle()

        assert final_status.startswith(expected_prefix)
        assert core.status_history[-1] == "IDLE"
        # Best-effort recovery moves back to HOME_POSITION (5 commands),
        # not a literal "HOME" command.
        assert sender.sent_commands[-5:] == HOME_COMMANDS
        # A failure reaching the arm before the RFID wait must never invoke it.
        assert rfid_waiter.calls == []

    def test_open_gripper_failure_never_descends(self):
        sender = FakeArmSender(fail_at_call_index=OPEN_BEFORE_PICK_INDEX)
        core = make_core(sender, FakeRfidWaiter("1DF058BA141080"))

        core.run_cycle()

        assert sender.sent_commands == HOME_COMMANDS + ["GRIPPER OPEN"] + HOME_COMMANDS
        assert "BASE 11" not in sender.sent_commands

    def test_lift_failure_never_moves_to_scan(self):
        sender = FakeArmSender(failing_command="SHOULDER 52")
        core = make_core(sender, FakeRfidWaiter("1DF058BA141080"))

        final_status = core.run_cycle()

        assert final_status.startswith("ERROR:ARM_FAILURE:LIFT_AFTER_GRIP:SHOULDER 52")
        close = sender.sent_commands.index("GRIPPER CLOSE")
        assert sender.sent_commands[close + 1:] == ["BASE 51", "SHOULDER 52"] + HOME_COMMANDS
        assert "BASE 21" not in sender.sent_commands
        assert core.status_history[-3:] == ["EXECUTING:LIFT_AFTER_GRIP", final_status, "IDLE"]

    @pytest.mark.parametrize("sender_kwargs,expected_prefix", [
        ({"failing_command": "BASE 41"}, "ERROR:ARM_FAILURE:MOVE_TO_FLOATING"),
        ({"failing_command": "BASE 31"}, "ERROR:ARM_FAILURE:MOVE_TO_BIN:BIN_A"),
        ({"fail_at_call_index": RELEASE_INDEX}, "ERROR:ARM_FAILURE:RELEASE"),
    ])
    def test_failure_after_rfid_detection_aborts_and_attempts_home(self, sender_kwargs, expected_prefix):
        sender = FakeArmSender(**sender_kwargs)
        rfid_waiter = FakeRfidWaiter("1DF058BA141080")
        core = make_core(sender, rfid_waiter)

        final_status = core.run_cycle()

        assert final_status.startswith(expected_prefix)
        assert core.status_history[-1] == "IDLE"
        assert sender.sent_commands[-5:] == HOME_COMMANDS
        assert rfid_waiter.calls == [15.0]

    def test_floating_failure_sends_no_bin_commands(self):
        sender = FakeArmSender(failing_command="BASE 41")
        core = make_core(sender, FakeRfidWaiter("1DF058BA141080"))

        core.run_cycle()

        assert sender.sent_commands == FULL_PICK_TO_SCAN_COMMANDS + ["BASE 41"] + HOME_COMMANDS

    def test_final_home_move_failing_is_reported(self):
        # Fail only the first command of the FINAL move to HOME_POSITION
        # (after a successful release), not the identical-valued approach
        # move earlier in the same cycle.
        fail_index = RELEASE_INDEX + 1
        sender = FakeArmSender(fail_at_call_index=fail_index)
        rfid_waiter = FakeRfidWaiter("1DF058BA141080")
        core = make_core(sender, rfid_waiter)

        final_status = core.run_cycle()

        assert final_status == "ERROR:ARM_FAILURE:HOME:BASE 1:ERROR:SERIAL_TIMEOUT"
        # This is a hard failure at the very last step, not routed through
        # _fail(), so no further recovery attempt follows it.
        assert core.status_history[-1] == final_status


class TestSettleWaits:
    """The Arduino's OK:<JOINT> only means "target accepted", not "servo
    arrived", so the core must sleep after each acknowledged move/gripper
    action before sending the next step. Distinct settle values make each
    kind of wait identifiable in the timeline. The per-joint gap is turned
    off here (joint_gap_seconds=0.0) so these timelines show only the settle
    waits; TestJointGap covers the gap."""

    MOVE = ("SLEEP", 2.0)
    GRIP = ("SLEEP", 1.0)
    RFID = ("SLEEP", 0.5)

    def make(self, rfid_uid="1DF058BA141080", **sender_kwargs):
        self.timeline = []
        self.sender = FakeArmSender(timeline=self.timeline, **sender_kwargs)
        self.sleep = FakeSleep(timeline=self.timeline)
        self.rfid_waiter = FakeRfidWaiter(rfid_uid)

        def waiter(timeout):
            self.timeline.append("WAIT_FOR_RFID")
            return self.rfid_waiter(timeout)

        return make_core(
            self.sender,
            waiter,
            sleep=self.sleep,
            move_settle_seconds=2.0,
            gripper_settle_seconds=1.0,
            rfid_settle_seconds=0.5,
            joint_gap_seconds=0.0,
        )

    def test_defaults_match_hardware_tuning(self):
        core = SortingControllerCore(
            object_map=OBJECT_MAP,
            positions=POSITIONS,
            send_arm_command=FakeArmSender(),
            wait_for_rfid_tag=FakeRfidWaiter("1DF058BA141080"),
        )
        assert core.move_settle_seconds == 2.0
        assert core.gripper_settle_seconds == 1.0
        assert core.rfid_settle_seconds == 0.5
        assert core.joint_gap_seconds == 0.6

    def test_full_cycle_timeline(self):
        core = self.make()
        assert core.run_cycle() == "IDLE"

        assert self.timeline == (
            HOME_COMMANDS + [self.MOVE]
            + ["GRIPPER OPEN", self.GRIP]
            + PICK_DOWN_COMMANDS + [self.MOVE]
            + ["GRIPPER CLOSE", self.GRIP]
            + LIFT_COMMANDS + [self.MOVE]
            + RFID_SCAN_COMMANDS + [self.MOVE, self.RFID]
            + ["WAIT_FOR_RFID"]
            + FLOATING_COMMANDS + [self.MOVE]
            + BIN_A_COMMANDS + [self.MOVE]
            + ["GRIPPER OPEN", self.GRIP]
            + HOME_COMMANDS + [self.MOVE]
        )

    def test_waits_after_pick_down_before_gripper_closes(self):
        core = self.make()
        core.run_cycle()
        close = self.timeline.index("GRIPPER CLOSE")
        assert self.timeline[close - 2:close] == ["WRIST_PITCH 0.0", self.MOVE]
        assert self.timeline[close - 6:close - 1] == PICK_DOWN_COMMANDS

    def test_waits_after_gripper_open_before_descending(self):
        core = self.make()
        core.run_cycle()
        open_ = self.timeline.index("GRIPPER OPEN")
        assert self.timeline[open_ - 1] == self.MOVE
        assert self.timeline[open_ + 1] == self.GRIP
        assert self.timeline[open_ + 2:open_ + 7] == PICK_DOWN_COMMANDS

    def test_waits_after_gripper_close_before_lifting(self):
        core = self.make()
        core.run_cycle()
        close = self.timeline.index("GRIPPER CLOSE")
        assert self.timeline[close + 1] == self.GRIP
        assert self.timeline[close + 2:close + 7] == LIFT_COMMANDS

    def test_lift_settles_before_scan_move(self):
        core = self.make()
        core.run_cycle()
        close = self.timeline.index("GRIPPER CLOSE")
        assert self.timeline[close + 2:close + 13] == (
            LIFT_COMMANDS + [self.MOVE] + RFID_SCAN_COMMANDS
        )

    def test_rfid_settle_happens_after_scan_move_before_tag_wait(self):
        core = self.make()
        core.run_cycle()
        wait = self.timeline.index("WAIT_FOR_RFID")
        assert self.timeline[wait - 7:wait] == RFID_SCAN_COMMANDS + [self.MOVE, self.RFID]

    def test_floating_settles_before_bin_move(self):
        core = self.make()
        core.run_cycle()
        wait = self.timeline.index("WAIT_FOR_RFID")
        assert self.timeline[wait + 1:wait + 13] == (
            FLOATING_COMMANDS + [self.MOVE] + BIN_A_COMMANDS + [self.MOVE]
        )

    def test_waits_after_release(self):
        core = self.make()
        core.run_cycle()
        release = len(self.timeline) - 1 - self.timeline[::-1].index("GRIPPER OPEN")
        assert self.timeline[release + 1] == self.GRIP

    def test_no_settle_after_failed_move_but_recovery_home_settles(self):
        # SHOULDER 12 fails mid PICK_DOWN move.
        core = self.make(fail_at_call_index=7)
        final_status = core.run_cycle()

        assert final_status.startswith("ERROR:ARM_FAILURE:MOVE_TO_PICK_DOWN_POSITION")
        assert core.status_history[-1] == "IDLE"
        assert self.timeline == (
            HOME_COMMANDS + [self.MOVE]
            + ["GRIPPER OPEN", self.GRIP]
            + ["BASE 11", "SHOULDER 12"]  # failed: no settle, no gripper
            + HOME_COMMANDS + [self.MOVE]  # best-effort recovery to HOME_POSITION
        )
        assert self.rfid_waiter.calls == []

    def test_gripper_close_failure_skips_grip_settle_and_recovers(self):
        core = self.make(failing_command="GRIPPER CLOSE")
        final_status = core.run_cycle()

        assert final_status.startswith("ERROR:ARM_FAILURE:GRIP")
        close = self.timeline.index("GRIPPER CLOSE")
        assert self.timeline[close + 1:] == HOME_COMMANDS + [self.MOVE]
        # Only the earlier GRIPPER OPEN settled; the failed close did not.
        assert self.timeline.count(self.GRIP) == 1
        assert core.status_history[-1] == "IDLE"

    def test_rfid_timeout_recovery_still_works_with_settles(self):
        core = self.make(rfid_uid=None)
        final_status = core.run_cycle()

        assert final_status == "ERROR:RFID_TIMEOUT"
        wait = self.timeline.index("WAIT_FOR_RFID")
        assert self.timeline[wait + 1:] == HOME_COMMANDS + [self.MOVE]
        assert core.status_history[-1] == "IDLE"

    def test_failed_recovery_home_does_not_settle(self):
        # Fail the recovery move too: BASE 1 always fails, so the very first
        # move fails and the best-effort HOME_POSITION move fails as well.
        core = self.make(failing_command="BASE 1")
        final_status = core.run_cycle()

        assert final_status.startswith("ERROR:ARM_FAILURE:MOVE_TO_HOME_POSITION")
        assert self.timeline == ["BASE 1", "BASE 1"]
        assert self.sleep.calls == []
        assert core.status_history[-1] == "IDLE"

    def test_zero_settle_times_are_allowed(self):
        sleep = FakeSleep()
        core = make_core(
            FakeArmSender(),
            FakeRfidWaiter("1DF058BA141080"),
            sleep=sleep,
            move_settle_seconds=0.0,
            gripper_settle_seconds=0.0,
            rfid_settle_seconds=0.0,
            joint_gap_seconds=0.0,
        )
        assert core.run_cycle() == "IDLE"
        assert set(sleep.calls) == {0.0}

    @pytest.mark.parametrize("param", [
        "move_settle_seconds", "gripper_settle_seconds", "rfid_settle_seconds",
        "joint_gap_seconds",
    ])
    def test_negative_settle_time_rejected(self, param):
        with pytest.raises(ValueError, match=param):
            make_core(FakeArmSender(), FakeRfidWaiter("1DF058BA141080"), **{param: -1.0})


def with_gaps(commands, gap):
    """Interleave a ("SLEEP", gap) after every command except the last."""
    timeline = []
    for index, cmd in enumerate(commands):
        timeline.append(cmd)
        if index < len(commands) - 1:
            timeline.append(("SLEEP", gap))
    return timeline


class TestJointGap:
    """Within a position move, the core waits joint_gap_seconds after each
    joint command so joints move one at a time. The gap is skipped after the
    last joint (move_settle_seconds covers it) and never applied to gripper
    commands. Distinct values keep each kind of wait identifiable."""

    GAP = ("SLEEP", 0.6)
    MOVE = ("SLEEP", 2.0)
    GRIP = ("SLEEP", 1.0)
    RFID = ("SLEEP", 0.5)

    def make(self, rfid_uid="1DF058BA141080", joint_gap_seconds=0.6, **sender_kwargs):
        self.timeline = []
        self.sender = FakeArmSender(timeline=self.timeline, **sender_kwargs)
        self.sleep = FakeSleep(timeline=self.timeline)
        self.rfid_waiter = FakeRfidWaiter(rfid_uid)

        def waiter(timeout):
            self.timeline.append("WAIT_FOR_RFID")
            return self.rfid_waiter(timeout)

        return make_core(
            self.sender,
            waiter,
            sleep=self.sleep,
            move_settle_seconds=2.0,
            gripper_settle_seconds=1.0,
            rfid_settle_seconds=0.5,
            joint_gap_seconds=joint_gap_seconds,
        )

    def test_full_cycle_timeline(self):
        core = self.make()
        assert core.run_cycle() == "IDLE"

        assert self.timeline == (
            with_gaps(HOME_COMMANDS, 0.6) + [self.MOVE]
            + ["GRIPPER OPEN", self.GRIP]
            + with_gaps(PICK_DOWN_COMMANDS, 0.6) + [self.MOVE]
            + ["GRIPPER CLOSE", self.GRIP]
            + with_gaps(LIFT_COMMANDS, 0.6) + [self.MOVE]
            + with_gaps(RFID_SCAN_COMMANDS, 0.6) + [self.MOVE, self.RFID]
            + ["WAIT_FOR_RFID"]
            + with_gaps(FLOATING_COMMANDS, 0.6) + [self.MOVE]
            + with_gaps(BIN_A_COMMANDS, 0.6) + [self.MOVE]
            + ["GRIPPER OPEN", self.GRIP]
            + with_gaps(HOME_COMMANDS, 0.6) + [self.MOVE]
        )

    def test_joint_order_is_unchanged(self):
        core = self.make()
        core.run_cycle()
        assert self.sender.sent_commands == (
            FULL_PICK_TO_SCAN_COMMANDS
            + FLOATING_COMMANDS
            + BIN_A_COMMANDS
            + ["GRIPPER OPEN"]
            + HOME_COMMANDS
        )

    def test_gap_after_each_joint_except_last(self):
        core = self.make()
        core.run_cycle()
        first_move = self.timeline[:self.timeline.index(self.MOVE) + 1]
        assert first_move == [
            "BASE 1", self.GAP,
            "SHOULDER 2", self.GAP,
            "ELBOW 3", self.GAP,
            "WRIST 4", self.GAP,
            "WRIST_PITCH 0.0", self.MOVE,  # no gap: move settle follows directly
        ]

    def test_one_gap_fewer_than_joints_per_move(self):
        core = self.make()
        core.run_cycle()
        moves = 7  # HOME, PICK_DOWN, LIFT, RFID_SCAN, FLOATING, BIN_A, HOME
        assert self.sleep.calls.count(0.6) == moves * (len(HOME_COMMANDS) - 1)

    def test_no_gap_around_gripper_commands(self):
        core = self.make()
        core.run_cycle()
        gripper_indices = [
            i for i, entry in enumerate(self.timeline)
            if entry in ("GRIPPER OPEN", "GRIPPER CLOSE")
        ]
        assert len(gripper_indices) == 3  # open before pick, close, release
        for i in gripper_indices:
            assert self.timeline[i - 1] == self.MOVE
            assert self.timeline[i + 1] == self.GRIP
            assert self.timeline[i + 2] != self.GAP

    def test_zero_gap_behaves_as_before(self):
        core = self.make(joint_gap_seconds=0.0)
        assert core.run_cycle() == "IDLE"

        assert 0.0 not in self.sleep.calls
        assert self.timeline == (
            HOME_COMMANDS + [self.MOVE]
            + ["GRIPPER OPEN", self.GRIP]
            + PICK_DOWN_COMMANDS + [self.MOVE]
            + ["GRIPPER CLOSE", self.GRIP]
            + LIFT_COMMANDS + [self.MOVE]
            + RFID_SCAN_COMMANDS + [self.MOVE, self.RFID]
            + ["WAIT_FOR_RFID"]
            + FLOATING_COMMANDS + [self.MOVE]
            + BIN_A_COMMANDS + [self.MOVE]
            + ["GRIPPER OPEN", self.GRIP]
            + HOME_COMMANDS + [self.MOVE]
        )

    def test_negative_gap_rejected(self):
        with pytest.raises(ValueError, match="joint_gap_seconds"):
            self.make(joint_gap_seconds=-0.1)

    def test_failed_joint_stops_move_and_recovery_home_uses_gaps(self):
        # SHOULDER 12 (call index 7) fails mid PICK_DOWN move.
        core = self.make(fail_at_call_index=7)
        final_status = core.run_cycle()

        assert final_status.startswith("ERROR:ARM_FAILURE:MOVE_TO_PICK_DOWN_POSITION")
        assert core.status_history[-1] == "IDLE"
        assert self.timeline == (
            with_gaps(HOME_COMMANDS, 0.6) + [self.MOVE]
            + ["GRIPPER OPEN", self.GRIP]
            + ["BASE 11", self.GAP, "SHOULDER 12"]  # failed: no gap, no settle
            + with_gaps(HOME_COMMANDS, 0.6) + [self.MOVE]  # best-effort recovery
        )
        assert self.rfid_waiter.calls == []

    def test_rfid_timeout_recovery_uses_gaps(self):
        core = self.make(rfid_uid=None)
        assert core.run_cycle() == "ERROR:RFID_TIMEOUT"

        wait = self.timeline.index("WAIT_FOR_RFID")
        assert self.timeline[wait + 1:] == with_gaps(HOME_COMMANDS, 0.6) + [self.MOVE]
        assert core.status_history[-1] == "IDLE"
