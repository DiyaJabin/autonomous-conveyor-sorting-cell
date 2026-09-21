"""Unit tests for named arm position configuration."""

import sys
import textwrap
from pathlib import Path

import pytest

pkg_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(pkg_root))

from sorting_cell.positions import ArmPosition, PositionConfigError, load_positions, positions_from_dict


VALID_CONFIG = {
    "positions": {
        "HOME_POSITION": {"base": 90, "shoulder": 90, "elbow": 90, "wrist": 90},
        "PICK_DOWN_POSITION": {"base": 45, "shoulder": 60, "elbow": 30, "wrist": 90},
        "RFID_SCAN_POSITION": {"base": 20, "shoulder": 60, "elbow": 30, "wrist": 90},
        "BIN_A_POSITION": {"base": 0, "shoulder": 60, "elbow": 30, "wrist": 90},
    }
}


class TestArmPosition:
    def test_as_commands_order_and_format(self):
        position = ArmPosition(base=10, shoulder=20, elbow=30, wrist=40)
        assert position.as_commands() == [
            "BASE 10",
            "SHOULDER 20",
            "ELBOW 30",
            "WRIST 40",
            "WRIST_PITCH 0.0",
        ]

    def test_as_commands_includes_explicit_wrist_pitch(self):
        position = ArmPosition(base=10, shoulder=20, elbow=30, wrist=40, wrist_pitch=15)
        assert position.as_commands() == [
            "BASE 10",
            "SHOULDER 20",
            "ELBOW 30",
            "WRIST 40",
            "WRIST_PITCH 15",
        ]


class TestPositionsFromDict:
    def test_valid_config_parses_all_positions(self):
        positions = positions_from_dict(VALID_CONFIG)
        assert set(positions.keys()) == {
            "HOME_POSITION", "PICK_DOWN_POSITION", "RFID_SCAN_POSITION", "BIN_A_POSITION",
        }
        assert positions["PICK_DOWN_POSITION"] == ArmPosition(base=45, shoulder=60, elbow=30, wrist=90)

    def test_wrist_pitch_is_optional_and_defaults_to_zero(self):
        # None of VALID_CONFIG's positions specify wrist_pitch.
        positions = positions_from_dict(VALID_CONFIG)
        assert positions["PICK_DOWN_POSITION"].wrist_pitch == 0.0
        assert positions["PICK_DOWN_POSITION"] == ArmPosition(base=45, shoulder=60, elbow=30, wrist=90)

    def test_wrist_pitch_is_parsed_when_specified(self):
        config = {
            "positions": {
                **VALID_CONFIG["positions"],
                "PICK_DOWN_POSITION": {
                    "base": 45, "shoulder": 60, "elbow": 30, "wrist": 90, "wrist_pitch": 15,
                },
            }
        }
        positions = positions_from_dict(config)
        assert positions["PICK_DOWN_POSITION"] == ArmPosition(
            base=45, shoulder=60, elbow=30, wrist=90, wrist_pitch=15
        )
        # Unrelated positions that still omit wrist_pitch keep defaulting.
        assert positions["HOME_POSITION"].wrist_pitch == 0.0

    @pytest.mark.parametrize("bad_config", [
        {},
        {"not_positions": {}},
        {"positions": {}},
        {"positions": "not_a_dict"},
        {"positions": {"HOME_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1}}},  # missing PICK_DOWN_POSITION, RFID_SCAN_POSITION
        {"positions": {
            "HOME_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "PICK_DOWN_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            # RFID_SCAN_POSITION intentionally missing
        }},
        {"positions": {
            "HOME_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "PICK_DOWN_POSITION": {"base": 1, "shoulder": 1, "elbow": 1},  # missing wrist
            "RFID_SCAN_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
        }},
        {"positions": {
            "HOME_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "PICK_DOWN_POSITION": {"base": "not_a_number", "shoulder": 1, "elbow": 1, "wrist": 1},
            "RFID_SCAN_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
        }},
        {"positions": {
            "HOME_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "PICK_DOWN_POSITION": {
                "base": 1, "shoulder": 1, "elbow": 1, "wrist": 1, "wrist_pitch": "not_a_number",
            },
            "RFID_SCAN_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
        }},
        {"positions": {
            "HOME_POSITION": "not_a_dict",
            "PICK_DOWN_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "RFID_SCAN_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
        }},
    ])
    def test_malformed_config_raises(self, bad_config):
        with pytest.raises(PositionConfigError):
            positions_from_dict(bad_config)


class TestLoadPositionsFromFile:
    def test_load_valid_yaml(self, tmp_path):
        config_file = tmp_path / "arm_positions.yaml"
        config_file.write_text(textwrap.dedent("""
            positions:
              HOME_POSITION:
                base: 90
                shoulder: 90
                elbow: 90
                wrist: 90
              PICK_DOWN_POSITION:
                base: 45
                shoulder: 60
                elbow: 30
                wrist: 90
                wrist_pitch: 15
              RFID_SCAN_POSITION:
                base: 20
                shoulder: 60
                elbow: 30
                wrist: 90
        """))
        positions = load_positions(str(config_file))
        assert "PICK_DOWN_POSITION" in positions
        assert "RFID_SCAN_POSITION" in positions
        # PICK_DOWN_POSITION specifies wrist_pitch explicitly...
        assert positions["PICK_DOWN_POSITION"].wrist_pitch == 15.0
        # ...while positions that omit it (like HOME/RFID_SCAN here) default.
        assert positions["HOME_POSITION"].wrist_pitch == 0.0
        assert positions["RFID_SCAN_POSITION"].wrist_pitch == 0.0

    def test_missing_file_raises(self, tmp_path):
        with pytest.raises(PositionConfigError):
            load_positions(str(tmp_path / "does_not_exist.yaml"))

    def test_invalid_yaml_raises(self, tmp_path):
        config_file = tmp_path / "arm_positions.yaml"
        config_file.write_text("positions: [this is not, valid: yaml: at all")
        with pytest.raises(PositionConfigError):
            load_positions(str(config_file))

    def test_repo_default_config_loads(self):
        """The shipped placeholder config must itself be valid and complete."""
        config_path = pkg_root / "config" / "arm_positions.yaml"
        positions = load_positions(str(config_path))
        for required in (
            "HOME_POSITION", "PICK_DOWN_POSITION", "RFID_SCAN_POSITION",
            "BIN_A_POSITION", "BIN_B_POSITION", "BIN_C_POSITION",
        ):
            assert required in positions
