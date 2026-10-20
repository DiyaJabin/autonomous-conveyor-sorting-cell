"""ROS 2 Sorting Controller Node for the Autonomous Conveyor Sorting Cell.

Sequences the MVP flow (matches the actual physical cell layout: pickup area
in front of the arm, PN532 RFID reader fixed to the LEFT, bins fixed to the
RIGHT -- the arm carries the object to the reader, not the other way round):

  cycle start -> move to HOME_POSITION -> open gripper
  -> move to PICK_DOWN_POSITION -> grip -> move to LIFT_POSITION
  -> move to RFID scan position
  -> wait for RFID detection -> identify object -> decide bin
  -> move to FLOATING_POSITION -> move to bin -> release -> report
  -> move to HOME_POSITION -> idle

  NOTE: HOME_POSITION here is a ROS-level named joint target (the
  cycle start/end pose), not the all-zeros baseline.
  That all-zeros baseline still exists, but only as arm_controller.ino's
  own firmware-level HOME command -- a separate, lower-level safety
  baseline that this node no longer sends and does not replace.

A cycle is started by a message on /sorting/trigger (there is currently no
object-presence sensor; this is the external "object is ready at pickup"
signal, and can later be wired to a real sensor without changing the
sequencing logic). RFID detection is a WAITING EVENT in the middle of the
sequence, not the trigger for it: the arm must physically carry the object
over the fixed reader before a UID can be known.

Subscribes to /sorting/trigger (cycle start), /rfid/tag (UID detected while
parked at the scan position), and /arm/status (execution feedback).
Publishes to /arm/command (joint/gripper/home commands) and /sorting/status
(high-level sequence progress and errors).

The sequencing logic (SortingControllerCore) is deliberately decoupled from
rclpy so it can be fully unit tested with fake command senders and a fake
RFID wait function, without any ROS 2 runtime or hardware.

Settle waits: the Arduino replies OK:<JOINT> as soon as it has accepted a
servo target, before the servo has physically moved, so an OK only means
"command accepted". After each named-position move, each gripper action,
and on arrival at RFID_SCAN_POSITION, the core therefore sleeps for a
configurable settle time so the arm has actually arrived (e.g. is really at
PICK_DOWN_POSITION before the gripper closes, and the gripper has really
closed before the arm lifts). Within a position move, the core also waits
joint_gap_seconds after each joint command except the last, so joints move
one at a time instead of all starting together (which makes the loaded
joints fall short on the real arm). The sleep function is injectable so
tests run instantly.
"""

import threading
import time
from pathlib import Path
from typing import Callable, Dict, Optional, Tuple

# Conditional import to allow pure Python testing on systems without rclpy
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    RCLPY_AVAILABLE = True
except ImportError:
    rclpy = None
    Node = object
    String = None
    RCLPY_AVAILABLE = False

from sorting_cell.object_map import ObjectMap, ObjectMapConfigError, load_object_map
from sorting_cell.positions import ArmPosition, PositionConfigError, load_positions


class SortingControllerCore:
    """Orchestrates the pick -> RFID scan -> identify -> decide -> place sequence.

    Decoupled from ROS 2 middleware: `send_arm_command` and
    `wait_for_rfid_tag` are injected callables so this can be unit tested on
    any platform, without rclpy, serial hardware, or a real PN532 reader.
    """

    # Positions required before a cycle can even attempt to start.
    STARTUP_REQUIRED_POSITIONS = (
        "HOME_POSITION",
        "PICK_DOWN_POSITION",
        "LIFT_POSITION",
        "RFID_SCAN_POSITION",
        "FLOATING_POSITION",
    )

    def __init__(
        self,
        object_map: ObjectMap,
        positions: Dict[str, ArmPosition],
        send_arm_command: Callable[[str], Tuple[bool, str]],
        wait_for_rfid_tag: Callable[[float], Optional[str]],
        report_status: Optional[Callable[[str], None]] = None,
        logger: Optional[Callable[[str], None]] = None,
        rfid_timeout: float = 15.0,
        move_settle_seconds: float = 2.0,
        gripper_settle_seconds: float = 1.0,
        rfid_settle_seconds: float = 0.5,
        joint_gap_seconds: float = 0.6,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        """
        :param object_map: UID -> object/bin lookup.
        :param positions: name -> ArmPosition, e.g. 'PICK_DOWN_POSITION', 'BIN_A_POSITION'.
        :param send_arm_command: sends one /arm/command string and blocks for the
               terminal /arm/status result, returning (success, status_text).
        :param wait_for_rfid_tag: blocks (up to the given timeout in seconds)
               for the next /rfid/tag detection, returning the UID string, or
               None if no tag was detected before the timeout.
        :param report_status: called with each high-level /sorting/status update.
        :param logger: optional logging callable.
        :param rfid_timeout: seconds to wait for an RFID detection once the
               arm is parked at RFID_SCAN_POSITION, before giving up.
        :param move_settle_seconds: seconds to wait after every named-position
               move has been acknowledged, so the servos physically arrive.
        :param gripper_settle_seconds: seconds to wait after GRIPPER OPEN /
               GRIPPER CLOSE has been acknowledged.
        :param rfid_settle_seconds: extra seconds to wait after arriving at
               RFID_SCAN_POSITION (on top of move_settle_seconds) before
               starting to wait for a tag.
        :param joint_gap_seconds: seconds to wait after each joint command
               within a position move, before sending the next joint. Not
               applied after the last joint (move_settle_seconds covers it)
               or to gripper commands. 0 sends joints back to back.
        :param sleep: blocking sleep function; injectable so tests run instantly.
        """
        for name, value in (
            ("move_settle_seconds", move_settle_seconds),
            ("gripper_settle_seconds", gripper_settle_seconds),
            ("rfid_settle_seconds", rfid_settle_seconds),
            ("joint_gap_seconds", joint_gap_seconds),
        ):
            if value < 0:
                raise ValueError(f"{name} must be >= 0, got {value}")
        self.object_map = object_map
        self.positions = positions
        self.send_arm_command = send_arm_command
        self.wait_for_rfid_tag = wait_for_rfid_tag
        self._report_status = report_status or (lambda status: None)
        self._logger = logger or (lambda msg: None)
        self.rfid_timeout = rfid_timeout
        self.move_settle_seconds = move_settle_seconds
        self.gripper_settle_seconds = gripper_settle_seconds
        self.rfid_settle_seconds = rfid_settle_seconds
        self.joint_gap_seconds = joint_gap_seconds
        self.sleep = sleep
        self.status_history = []

    def report_status(self, status_text: str) -> None:
        self.status_history.append(status_text)
        self._report_status(status_text)

    def log(self, msg: str) -> None:
        self._logger(msg)

    def _send_position(self, position: ArmPosition, label: str) -> Optional[str]:
        commands = position.as_commands()
        for index, cmd in enumerate(commands):
            success, status = self.send_arm_command(cmd)
            if not success:
                return f"ERROR:ARM_FAILURE:{label}:{cmd}:{status}"
            # Move joints one at a time; the settle below covers the last one.
            if self.joint_gap_seconds > 0 and index < len(commands) - 1:
                self.sleep(self.joint_gap_seconds)
        # OK only means the targets were accepted; give the servos time to arrive.
        self.sleep(self.move_settle_seconds)
        return None

    def _send_gripper(self, state: str, label: str) -> Optional[str]:
        success, status = self.send_arm_command(f"GRIPPER {state}")
        if not success:
            return f"ERROR:ARM_FAILURE:{label}:{status}"
        self.sleep(self.gripper_settle_seconds)
        return None

    def _fail(self, reason: str) -> str:
        """Report a failure, attempt a best-effort return to HOME_POSITION, and settle at IDLE.

        Deliberately does NOT force the gripper open: if the failure happens
        after the object was gripped (e.g. RFID timeout, unknown UID, a
        failure moving to the bin), releasing it at an arbitrary arm position
        would drop it uncontrolled. Homing with the gripper as-is is the
        safer default until a human inspects the cell.

        "HOME" here means the ROS-level HOME_POSITION (cycle start/end pose),
        via the same joint-command path as any other named position --
        NOT arm_controller.ino's own all-zeros firmware HOME command, which
        this node no longer sends. If HOME_POSITION itself is undefined
        (e.g. a startup config-validation failure), the KeyError is caught
        below like any other best-effort-homing failure.
        """
        self.log(reason)
        self.report_status(reason)
        try:
            error = self._send_position(self.positions["HOME_POSITION"], "HOME")
            if error:
                self.log(f"Best-effort move to HOME_POSITION after failure also failed: {error}")
        except Exception as err:  # never let a homing attempt mask the original failure
            self.log(f"Best-effort move to HOME_POSITION after failure also failed: {err}")
        self.report_status("IDLE")
        return reason

    def run_cycle(self) -> str:
        """Run one full pick -> RFID scan -> identify -> place cycle.

        :return: final status string (e.g. 'IDLE' on success, or the
                 'ERROR:...' reason if the sequence was aborted).
        """
        self.report_status("CYCLE_START")

        missing = [name for name in self.STARTUP_REQUIRED_POSITIONS if name not in self.positions]
        if missing:
            return self._fail(f"ERROR:INVALID_CONFIGURATION:MISSING_POSITION:{','.join(missing)}")

        self.report_status("EXECUTING:MOVE_TO_HOME_POSITION")
        error = self._send_position(self.positions["HOME_POSITION"], "MOVE_TO_HOME_POSITION")
        if error:
            return self._fail(error)

        # Make sure the gripper is open before descending onto the object.
        self.report_status("EXECUTING:OPEN_GRIPPER")
        error = self._send_gripper("OPEN", "OPEN_GRIPPER")
        if error:
            return self._fail(error)

        self.report_status("EXECUTING:MOVE_TO_PICK_DOWN_POSITION")
        error = self._send_position(self.positions["PICK_DOWN_POSITION"], "MOVE_TO_PICK_DOWN_POSITION")
        if error:
            return self._fail(error)

        self.report_status("EXECUTING:GRIP")
        error = self._send_gripper("CLOSE", "GRIP")
        if error:
            return self._fail(error)

        # Lift the gripped object clear of the pickup area before swinging
        # it over to the reader.
        self.report_status("EXECUTING:LIFT_AFTER_GRIP")
        error = self._send_position(self.positions["LIFT_POSITION"], "LIFT_AFTER_GRIP")
        if error:
            return self._fail(error)

        self.report_status("EXECUTING:MOVE_TO_RFID_SCAN")
        error = self._send_position(self.positions["RFID_SCAN_POSITION"], "MOVE_TO_RFID_SCAN")
        if error:
            return self._fail(error)

        # Let the held object stop swinging over the reader before scanning.
        self.sleep(self.rfid_settle_seconds)

        self.report_status("WAITING_FOR_RFID")
        uid = self.wait_for_rfid_tag(self.rfid_timeout)
        if uid is None:
            return self._fail("ERROR:RFID_TIMEOUT")

        self.report_status(f"RFID_DETECTED:{uid}")

        record = self.object_map.lookup(uid)
        if record is None:
            return self._fail(f"ERROR:UNKNOWN_UID:{uid}")

        self.report_status(f"IDENTIFIED:{record.object_name}")
        self.report_status(f"BIN_SELECTED:{record.bin_name}")

        bin_position_name = f"{record.bin_name}_POSITION"
        if bin_position_name not in self.positions:
            return self._fail(f"ERROR:INVALID_CONFIGURATION:MISSING_POSITION:{bin_position_name}")

        self.report_status("EXECUTING:MOVE_TO_FLOATING")
        error = self._send_position(self.positions["FLOATING_POSITION"], "MOVE_TO_FLOATING")
        if error:
            return self._fail(error)

        self.report_status(f"EXECUTING:MOVE_TO_BIN:{record.bin_name}")
        error = self._send_position(self.positions[bin_position_name], f"MOVE_TO_BIN:{record.bin_name}")
        if error:
            return self._fail(error)

        self.report_status("EXECUTING:RELEASE")
        error = self._send_gripper("OPEN", "RELEASE")
        if error:
            return self._fail(error)

        self.report_status(f"REPORT:SORTED:{uid}:{record.object_name}:{record.bin_name}")

        # "HOME" here is the ROS-level HOME_POSITION (cycle start/end pose), not
        # arm_controller.ino's own all-zeros firmware HOME command -- see the
        # module docstring and _fail() for why those are no longer the same.
        error = self._send_position(self.positions["HOME_POSITION"], "HOME")
        if error:
            self.report_status(error)
            return error

        self.report_status("IDLE")
        return "IDLE"


class SortingControllerNode(Node):
    """ROS 2 Node running the high-level sorting sequence."""

    def __init__(self) -> None:
        if not RCLPY_AVAILABLE:
            raise RuntimeError("rclpy is not available in this environment")

        super().__init__('sorting_controller')

        self.declare_parameter('object_map_path', '')
        self.declare_parameter('positions_path', '')
        self.declare_parameter('arm_command_timeout', 5.0)
        self.declare_parameter('rfid_timeout', 15.0)
        self.declare_parameter('move_settle_seconds', 2.0)
        self.declare_parameter('gripper_settle_seconds', 1.0)
        self.declare_parameter('rfid_settle_seconds', 0.5)
        self.declare_parameter('joint_gap_seconds', 0.6)

        self.arm_command_timeout = self.get_parameter(
            'arm_command_timeout'
        ).get_parameter_value().double_value
        self.rfid_timeout = self.get_parameter('rfid_timeout').get_parameter_value().double_value
        self.move_settle_seconds = self.get_parameter(
            'move_settle_seconds'
        ).get_parameter_value().double_value
        self.gripper_settle_seconds = self.get_parameter(
            'gripper_settle_seconds'
        ).get_parameter_value().double_value
        self.rfid_settle_seconds = self.get_parameter(
            'rfid_settle_seconds'
        ).get_parameter_value().double_value
        self.joint_gap_seconds = self.get_parameter(
            'joint_gap_seconds'
        ).get_parameter_value().double_value

        object_map_path = self._resolve_config_path('object_map_path', 'object_map.yaml')
        positions_path = self._resolve_config_path('positions_path', 'arm_positions.yaml')

        try:
            self.object_map = load_object_map(object_map_path)
        except ObjectMapConfigError as err:
            self.get_logger().error(f"Failed to load object map ({object_map_path}): {err}")
            self.object_map = ObjectMap({})

        try:
            self.positions = load_positions(positions_path)
        except PositionConfigError as err:
            self.get_logger().error(f"Failed to load arm positions ({positions_path}): {err}")
            self.positions = {}

        self.command_publisher = self.create_publisher(String, '/arm/command', 10)
        self.status_publisher = self.create_publisher(String, '/sorting/status', 10)

        # /arm/status synchronization (request/response over pub-sub).
        # The lock makes "callback writes value + sets event" and "waiter
        # resets value + clears event" mutually exclusive, so a response
        # can never be partially observed (e.g. event left set with the
        # value wiped back to None, or the reverse) if it races the setup
        # for the next wait. The blocking .wait() itself is called with the
        # lock released, so the callback can still acquire it meanwhile.
        self._arm_status_lock = threading.Lock()
        self._arm_status_event = threading.Event()
        self._latest_arm_status: Optional[str] = None
        self.arm_status_subscription = self.create_subscription(
            String, '/arm/status', self._arm_status_callback, 10
        )

        # /rfid/tag synchronization: set by the subscription callback (runs on
        # the executor thread), waited on from the worker thread below. This
        # mirrors the /arm/status pattern above (including the lock) so the
        # executor is never blocked and stays free to deliver both
        # /arm/status and /rfid/tag while a cycle is mid-sequence.
        # _wait_for_rfid_tag() clears any earlier detection when it starts,
        # so a tag published before WAITING_FOR_RFID (e.g. during the move
        # to the scan pose or its settle waits) is discarded. rfid_reader
        # does not republish a tag that stays in the field, so in that case
        # the wait can end in ERROR:RFID_TIMEOUT.
        self._rfid_lock = threading.Lock()
        self._rfid_event = threading.Event()
        self._latest_rfid_uid: Optional[str] = None
        self.rfid_subscription = self.create_subscription(
            String, '/rfid/tag', self._rfid_tag_callback, 10
        )

        self.core = SortingControllerCore(
            object_map=self.object_map,
            positions=self.positions,
            send_arm_command=self._send_arm_command_and_wait,
            wait_for_rfid_tag=self._wait_for_rfid_tag,
            report_status=self._publish_sorting_status,
            logger=lambda msg: self.get_logger().info(msg),
            rfid_timeout=self.rfid_timeout,
            move_settle_seconds=self.move_settle_seconds,
            gripper_settle_seconds=self.gripper_settle_seconds,
            rfid_settle_seconds=self.rfid_settle_seconds,
            joint_gap_seconds=self.joint_gap_seconds,
        )

        # Cycle triggers ("object is present at pickup") are handled on a
        # dedicated worker thread so that waiting for /arm/status or
        # /rfid/tag never blocks the node's executor (which must stay free
        # to deliver those very messages).
        #
        # This is a fixed physical cell: triggers must NOT accumulate. A
        # trigger arriving while a cycle is already running is rejected
        # outright (not queued for later) so the arm never runs extra
        # cycles after the current object has already been sorted. A single
        # Event (not a queue) signals "a cycle should start"; _cycle_running
        # gates whether a new trigger is accepted, and both are guarded by
        # the same lock so the check-and-signal is atomic.
        self._cycle_state_lock = threading.Lock()
        self._cycle_running = False
        self._trigger_event = threading.Event()
        self.trigger_subscription = self.create_subscription(
            String, '/sorting/trigger', self._trigger_callback, 10
        )
        self._worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker_thread.start()

        self._publish_sorting_status("IDLE")
        self.get_logger().info(
            "sorting_controller node started; publish to /sorting/trigger to start a cycle"
        )

    def _resolve_config_path(self, param_name: str, filename: str) -> str:
        configured = self.get_parameter(param_name).get_parameter_value().string_value
        if configured:
            return configured
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('sorting_cell')
            return str(Path(share_dir) / 'config' / filename)
        except Exception:
            # Fallback when running from source without an installed share dir
            return str(Path(__file__).resolve().parent.parent / 'config' / filename)

    def _publish_sorting_status(self, status_text: str) -> None:
        msg = String()
        msg.data = status_text
        self.status_publisher.publish(msg)

    def _arm_status_callback(self, msg: "String") -> None:
        with self._arm_status_lock:
            self._latest_arm_status = msg.data
            if msg.data.startswith("OK:") or msg.data.startswith("ERROR:"):
                self._arm_status_event.set()

    def _send_arm_command_and_wait(self, command: str) -> Tuple[bool, str]:
        with self._arm_status_lock:
            self._arm_status_event.clear()
            self._latest_arm_status = None

        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)

        got_response = self._arm_status_event.wait(self.arm_command_timeout)
        if not got_response:
            return False, "ERROR:ARM_TIMEOUT"

        with self._arm_status_lock:
            status = self._latest_arm_status
        success = status is not None and status.startswith("OK:")
        return success, status

    def _rfid_tag_callback(self, msg: "String") -> None:
        with self._rfid_lock:
            self._latest_rfid_uid = msg.data
            self._rfid_event.set()

    def _wait_for_rfid_tag(self, timeout: float) -> Optional[str]:
        with self._rfid_lock:
            self._rfid_event.clear()
            self._latest_rfid_uid = None

        got_tag = self._rfid_event.wait(timeout)
        if not got_tag:
            return None

        with self._rfid_lock:
            return self._latest_rfid_uid

    def _trigger_callback(self, msg: "String") -> None:
        with self._cycle_state_lock:
            if self._cycle_running:
                self.get_logger().warn(
                    "Sorting cycle already running; ignoring /sorting/trigger"
                )
                return
            self._trigger_event.set()

    def _worker_loop(self) -> None:
        while rclpy.ok():
            got_trigger = self._trigger_event.wait(timeout=0.5)
            if not got_trigger:
                continue

            with self._cycle_state_lock:
                self._trigger_event.clear()
                self._cycle_running = True

            try:
                self.core.run_cycle()
            finally:
                with self._cycle_state_lock:
                    self._cycle_running = False


def main(args=None) -> None:
    """Main entry point for sorting_controller node."""
    if not RCLPY_AVAILABLE:
        import sys
        print("ERROR: rclpy is required to run the ROS 2 node.", file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=args)
    node = SortingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
