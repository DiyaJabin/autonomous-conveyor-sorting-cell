"""ROS 2 RFID Reader Node for the Autonomous Conveyor Sorting Cell.

Publishes detected RFID UIDs on /rfid/tag (std_msgs/String). The real
hardware read path is a PN532 NFC reader wired directly to the Raspberry
Pi's I2C1 bus (/dev/i2c-1, SDA=GPIO2/pin3, SCL=GPIO3/pin5) -- not the
Arduino -- using the adafruit-circuitpython-pn532 I2C driver (see
Pn532Backend below). The I2C address is a ROS 2 parameter ('i2c_address'),
never hardcoded: confirm the real address with `i2cdetect -y 1` once the
reader is wired, since it can differ between PN532 breakout boards.

For development and testing without the PN532 connected, publish a UID to
/rfid/simulate and it will be forwarded (through the same debounce logic
used for real scans) to /rfid/tag.
"""

from typing import Callable, Optional

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

# Conditional import: the CircuitPython/Blinka hardware stack is only
# installed (and only importable without error) on the Raspberry Pi, once
# adafruit-blinka + adafruit-circuitpython-pn532 are installed (see
# requirements.txt). Keeping this optional lets the rest of this module,
# and mock_mode, load and be unit tested on any machine -- the same
# conditional-import pattern used for rclpy/pyserial/pyyaml elsewhere in
# this package.
try:
    import board
    import busio
    from adafruit_pn532.i2c import PN532_I2C
    PN532_LIB_AVAILABLE = True
except ImportError:
    board = None
    busio = None
    PN532_I2C = None
    PN532_LIB_AVAILABLE = False


class Pn532NotConfiguredError(RuntimeError):
    """Raised when the PN532 driver libraries are not installed/importable."""


class Pn532HardwareError(RuntimeError):
    """Raised when the PN532 cannot be initialized or read over I2C."""


class Pn532Backend:
    """Real PN532 hardware backend: NFC reader on the Raspberry Pi's I2C1
    bus (/dev/i2c-1, SDA=GPIO2, SCL=GPIO3), via adafruit-circuitpython-pn532.

    The I2C address is never hardcoded here: it is passed in by the caller
    (the 'i2c_address' ROS 2 parameter, see RfidReaderNode) and must be
    confirmed with `i2cdetect -y 1` once the reader is physically wired --
    PN532 breakout boards can differ.

    Connecting to the bus/chip is deferred to the first read_uid() call
    (not done at construction time), so this class can be constructed --
    and this module imported and unit tested -- on a machine without the
    PN532 driver stack installed, without touching any hardware.
    """

    #: Standard PN532 I2C address (matches the adafruit_pn532 library's own
    #: default). Used only as the ROS parameter default -- always confirm
    #: with `i2cdetect -y 1` before relying on it.
    DEFAULT_I2C_ADDRESS = 0x24

    def __init__(
        self,
        i2c_address: int = DEFAULT_I2C_ADDRESS,
        read_timeout: float = 0.1,
        pn532_factory: Optional[Callable[[int], object]] = None,
    ) -> None:
        """
        :param i2c_address: PN532 I2C address (confirm via `i2cdetect -y 1`).
        :param read_timeout: seconds read_passive_target() waits for a tag
               on each poll.
        :param pn532_factory: test seam only. If given, called with
               i2c_address to obtain an object exposing SAM_configuration()
               and read_passive_target(timeout=...), instead of constructing
               real board/busio/PN532_I2C hardware objects. Unit tests use
               this to mock the PN532 driver without any real I2C bus.
        """
        self.i2c_address = i2c_address
        self.read_timeout = read_timeout
        self._pn532_factory = pn532_factory
        self._pn532 = None

    def _connect(self):
        """Lazily construct and configure the PN532 driver instance."""
        if self._pn532 is not None:
            return self._pn532

        try:
            if self._pn532_factory is not None:
                pn532 = self._pn532_factory(self.i2c_address)
            else:
                if not PN532_LIB_AVAILABLE:
                    raise Pn532NotConfiguredError(
                        "PN532 driver libraries are not installed in this "
                        "Python environment (need 'adafruit-blinka' and "
                        "'adafruit-circuitpython-pn532'; see "
                        "requirements.txt). Install them on the Raspberry "
                        "Pi, or use mock_mode:=true and /rfid/simulate for "
                        "development without the libraries/hardware."
                    )
                i2c = busio.I2C(board.SCL, board.SDA)
                pn532 = PN532_I2C(i2c, address=self.i2c_address, debug=False)

            pn532.SAM_configuration()
        except Pn532NotConfiguredError:
            raise
        except Exception as err:
            raise Pn532HardwareError(
                f"Failed to initialize the PN532 over I2C at address "
                f"{hex(self.i2c_address)}: {err}. Confirm wiring and the "
                f"real address with `i2cdetect -y 1` on the Raspberry Pi, "
                f"then set the 'i2c_address' ROS 2 parameter accordingly."
            ) from err

        self._pn532 = pn532
        return self._pn532

    def read_uid(self) -> Optional[str]:
        """Poll the PN532 once for a passive (ISO14443A) tag.

        :return: uppercase hex UID string with no separators (e.g.
                 '04A224B2D95E80', matching config/object_map.yaml's
                 documented UID format), or None if no tag is present.
        :raises Pn532NotConfiguredError: driver libraries are not installed.
        :raises Pn532HardwareError: the PN532 could not be initialized or
                 read over I2C.
        """
        pn532 = self._connect()

        try:
            uid_bytes = pn532.read_passive_target(timeout=self.read_timeout)
        except Exception as err:
            raise Pn532HardwareError(f"PN532 read over I2C failed: {err}") from err

        if not uid_bytes:
            return None

        return "".join(f"{byte:02X}" for byte in uid_bytes)


class RfidReaderCore:
    """Debounces raw tag reads into discrete 'new tag detected' events.

    Decoupled from ROS 2 and hardware so it can be unit tested directly.
    """

    def __init__(self) -> None:
        self._last_uid: Optional[str] = None

    def process_scan(self, uid: Optional[str]) -> Optional[str]:
        """Feed one raw read (a UID string, or None if no tag is present).

        :return: the UID to publish if this is a newly-detected tag,
                 otherwise None (same tag still present, or no tag present).
        """
        if uid is None or not isinstance(uid, str) or not uid.strip():
            self._last_uid = None
            return None

        clean_uid = uid.strip()
        if clean_uid == self._last_uid:
            return None

        self._last_uid = clean_uid
        return clean_uid


class RfidReaderNode(Node):
    """ROS 2 Node publishing detected RFID UIDs."""

    def __init__(self) -> None:
        if not RCLPY_AVAILABLE:
            raise RuntimeError("rclpy is not available in this environment")

        super().__init__('rfid_reader')

        self.declare_parameter('mock_mode', True)
        self.declare_parameter('poll_interval', 0.2)
        # PN532 I2C address (7-bit). The PN532 chip's address is normally
        # fixed, but breakout boards can differ -- confirm with
        # `i2cdetect -y 1` on the Raspberry Pi once the reader is wired,
        # and override this parameter (not the code) if it differs from
        # Pn532Backend.DEFAULT_I2C_ADDRESS.
        self.declare_parameter('i2c_address', Pn532Backend.DEFAULT_I2C_ADDRESS)

        self.mock_mode = self.get_parameter('mock_mode').get_parameter_value().bool_value
        poll_interval = self.get_parameter('poll_interval').get_parameter_value().double_value
        i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value

        self.core = RfidReaderCore()
        self.backend: Optional[Pn532Backend] = None

        self.tag_publisher = self.create_publisher(String, '/rfid/tag', 10)

        # Test/dev injection point: publish a UID here to simulate a tag scan,
        # regardless of mock_mode. Goes through the same debounce logic as
        # real hardware reads.
        self.simulate_subscription = self.create_subscription(
            String,
            '/rfid/simulate',
            self._simulate_callback,
            10,
        )

        if self.mock_mode:
            self.get_logger().info(
                "rfid_reader started in mock_mode: publish a UID to /rfid/simulate to test detection"
            )
        else:
            self.backend = Pn532Backend(i2c_address=i2c_address)
            try:
                self.backend.read_uid()
            except (Pn532NotConfiguredError, Pn532HardwareError) as err:
                self.get_logger().error(str(err))
                self.get_logger().error(
                    "Real RFID scanning disabled at startup. "
                    "Use mock_mode:=true and /rfid/simulate for now."
                )
                self.backend = None

        self.poll_timer = None
        if self.backend is not None:
            self.poll_timer = self.create_timer(poll_interval, self._poll_callback)

    def _publish_tag(self, uid: str) -> None:
        msg = String()
        msg.data = uid
        self.tag_publisher.publish(msg)
        self.get_logger().info(f"RFID tag detected: {uid}")

    def _simulate_callback(self, msg: "String") -> None:
        uid_to_publish = self.core.process_scan(msg.data)
        if uid_to_publish:
            self._publish_tag(uid_to_publish)

    def _poll_callback(self) -> None:
        if self.backend is None:
            return
        try:
            uid = self.backend.read_uid()
        except Exception as err:
            self.get_logger().error(f"RFID read failure: {err}")
            return

        uid_to_publish = self.core.process_scan(uid)
        if uid_to_publish:
            self._publish_tag(uid_to_publish)


def main(args=None) -> None:
    """Main entry point for rfid_reader node."""
    if not RCLPY_AVAILABLE:
        import sys
        print("ERROR: rclpy is required to run the ROS 2 node.", file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=args)
    node = RfidReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
