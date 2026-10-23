"""Unit tests for the RFID reader's debounce logic and PN532 I2C backend.

These tests exercise RfidReaderCore and Pn532Backend directly, without
rclpy, matching the decoupled-core pattern used for the arm controller.
Pn532Backend's pn532_factory test seam is used to inject a fake driver
object standing in for adafruit_pn532.i2c.PN532_I2C, so these tests never
open a real I2C bus and pass regardless of whether adafruit-blinka /
adafruit-circuitpython-pn532 (see requirements.txt) are installed on the
machine running pytest.
"""

import sys
from pathlib import Path

import pytest

pkg_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(pkg_root))

from sorting_cell import rfid_reader
from sorting_cell.rfid_reader import (
    Pn532Backend,
    Pn532HardwareError,
    Pn532NotConfiguredError,
    RfidReaderCore,
)


class TestRfidReaderCoreDebounce:
    def setup_method(self):
        self.core = RfidReaderCore()

    def test_first_scan_of_new_tag_is_published(self):
        assert self.core.process_scan("UID_A") == "UID_A"

    def test_repeated_scan_of_same_tag_is_not_republished(self):
        assert self.core.process_scan("UID_A") == "UID_A"
        assert self.core.process_scan("UID_A") is None
        assert self.core.process_scan("UID_A") is None

    def test_tag_removed_then_rescanned_is_republished(self):
        assert self.core.process_scan("UID_A") == "UID_A"
        assert self.core.process_scan(None) is None  # tag lifted off reader
        assert self.core.process_scan("UID_A") == "UID_A"  # scanned again

    def test_switching_to_a_different_tag_publishes_immediately(self):
        assert self.core.process_scan("UID_A") == "UID_A"
        assert self.core.process_scan("UID_B") == "UID_B"

    def test_whitespace_uid_is_trimmed(self):
        assert self.core.process_scan("  UID_A  ") == "UID_A"

    @pytest.mark.parametrize("empty_value", [None, "", "   "])
    def test_no_tag_present_never_publishes(self, empty_value):
        assert self.core.process_scan(empty_value) is None


class FakePn532:
    """Minimal fake standing in for adafruit_pn532.i2c.PN532_I2C."""

    def __init__(self):
        self.sam_configuration_calls = 0
        self.read_timeouts = []
        self.next_uid = None
        self.raise_on_sam_configuration = None
        self.raise_on_read = None

    def SAM_configuration(self):
        if self.raise_on_sam_configuration is not None:
            raise self.raise_on_sam_configuration
        self.sam_configuration_calls += 1

    def read_passive_target(self, timeout=1):
        self.read_timeouts.append(timeout)
        if self.raise_on_read is not None:
            raise self.raise_on_read
        return self.next_uid


class TestPn532BackendWithoutDriverLibraries:
    """When adafruit-blinka / adafruit-circuitpython-pn532 are not
    installed (the normal case on a dev machine, not the Raspberry Pi),
    read_uid() must fail loudly and clearly instead of guessing/crashing."""

    def test_read_uid_raises_not_configured(self, monkeypatch):
        monkeypatch.setattr(rfid_reader, "PN532_LIB_AVAILABLE", False)
        backend = Pn532Backend()
        with pytest.raises(Pn532NotConfiguredError):
            backend.read_uid()


class TestPn532BackendMockedHardware:
    """Exercises the real read_uid()/_connect() code path (including the
    I2C address hand-off and error wrapping) against a fake PN532 driver
    injected via pn532_factory -- never a real I2C bus or PN532 chip."""

    def _make_backend(self, fake, i2c_address=0x24):
        return Pn532Backend(i2c_address=i2c_address, pn532_factory=lambda addr: fake)

    def test_tag_present_returns_uppercase_hex_uid_no_separators(self):
        fake = FakePn532()
        fake.next_uid = bytearray([0x04, 0xA2, 0x24, 0xB2, 0xD9, 0x5E, 0x80])
        backend = self._make_backend(fake)

        # Matches the UID format documented in config/object_map.yaml.
        assert backend.read_uid() == "04A224B2D95E80"

    def test_no_tag_present_returns_none(self):
        fake = FakePn532()
        fake.next_uid = None
        backend = self._make_backend(fake)

        assert backend.read_uid() is None

    def test_empty_uid_bytes_returns_none(self):
        fake = FakePn532()
        fake.next_uid = bytearray()
        backend = self._make_backend(fake)

        assert backend.read_uid() is None

    def test_configured_i2c_address_is_passed_to_factory(self):
        fake = FakePn532()
        seen_addresses = []
        backend = Pn532Backend(
            i2c_address=0x42,
            pn532_factory=lambda addr: (seen_addresses.append(addr), fake)[1],
        )

        backend.read_uid()

        assert seen_addresses == [0x42]

    def test_connection_and_sam_configuration_happen_once_and_are_cached(self):
        fake = FakePn532()
        call_count = {"n": 0}

        def factory(addr):
            call_count["n"] += 1
            return fake

        backend = Pn532Backend(pn532_factory=factory)

        backend.read_uid()
        backend.read_uid()
        backend.read_uid()

        assert call_count["n"] == 1
        assert fake.sam_configuration_calls == 1

    def test_sam_configuration_failure_raises_hardware_error(self):
        fake = FakePn532()
        fake.raise_on_sam_configuration = OSError("no ACK from device")
        backend = self._make_backend(fake)

        with pytest.raises(Pn532HardwareError):
            backend.read_uid()

    def test_read_failure_raises_hardware_error(self):
        fake = FakePn532()
        fake.raise_on_read = OSError("I2C read timed out")
        backend = self._make_backend(fake)

        with pytest.raises(Pn532HardwareError):
            backend.read_uid()

    def test_read_timeout_parameter_is_forwarded(self):
        fake = FakePn532()
        backend = Pn532Backend(read_timeout=0.05, pn532_factory=lambda addr: fake)

        backend.read_uid()

        assert fake.read_timeouts == [0.05]
