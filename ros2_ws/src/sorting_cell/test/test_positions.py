"""Unit tests for named arm position configuration."""

import sys
import textwrap
from pathlib import Path

import pytest

pkg_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(pkg_root))

from sorting_cell.arm_controller import ArmCommandParser
from sorting_cell.positions import (
    REQUIRED_POSITION_NAMES,
    ArmPosition,
    PositionConfigError,
    load_positions,
    positions_from_dict,
)


VALID_CONFIG = {
    "positions": {
        "HOME_POSITION": {"base": 90, "shoulder": 90, "elbow": 90, "wrist": 90},
        "PICK_DOWN_POSITION": {"base": 45, "shoulder": 60, "elbow": 30, "wrist": 90},
        "LIFT_POSITION": {"base": 45, "shoulder": 60, "elbow": 60, "wrist": 90},
        "RFID_SCAN_POSITION": {"base": 20, "shoulder": 60, "elbow": 30, "wrist": 90},
        "FLOATING_POSITION": {"base": 20, "shoulder": 60, "elbow": 60, "wrist": 90},
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
            "HOME_POSITION", "PICK_DOWN_POSITION", "LIFT_POSITION", "RFID_SCAN_POSITION",
            "FLOATING_POSITION", "BIN_A_POSITION",
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
            "LIFT_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            # RFID_SCAN_POSITION intentionally missing
            "FLOATING_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
        }},
        {"positions": {
            "HOME_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "PICK_DOWN_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "LIFT_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "RFID_SCAN_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            # FLOATING_POSITION intentionally missing
        }},
        {"positions": {
            "HOME_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "PICK_DOWN_POSITION": {"base": 1, "shoulder": 1, "elbow": 1},  # missing wrist
            "LIFT_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "RFID_SCAN_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "FLOATING_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
        }},
        {"positions": {
            "HOME_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "PICK_DOWN_POSITION": {"base": "not_a_number", "shoulder": 1, "elbow": 1, "wrist": 1},
            "LIFT_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "RFID_SCAN_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "FLOATING_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
        }},
        {"positions": {
            "HOME_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "PICK_DOWN_POSITION": {
                "base": 1, "shoulder": 1, "elbow": 1, "wrist": 1, "wrist_pitch": "not_a_number",
            },
            "LIFT_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "RFID_SCAN_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "FLOATING_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
        }},
        {"positions": {
            "HOME_POSITION": "not_a_dict",
            "PICK_DOWN_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "LIFT_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "RFID_SCAN_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
            "FLOATING_POSITION": {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1},
        }},
    ])
    def test_malformed_config_raises(self, bad_config):
        with pytest.raises(PositionConfigError):
            positions_from_dict(bad_config)

    def test_required_position_names(self):
        assert REQUIRED_POSITION_NAMES == (
            "HOME_POSITION", "PICK_DOWN_POSITION", "LIFT_POSITION", "RFID_SCAN_POSITION",
            "FLOATING_POSITION",
        )

    def test_missing_lift_position_is_named_in_error(self):
        config = {"positions": {
            k: v for k, v in VALID_CONFIG["positions"].items() if k != "LIFT_POSITION"
        }}
        with pytest.raises(PositionConfigError, match="LIFT_POSITION"):
            positions_from_dict(config)

    def test_missing_floating_position_is_named_in_error(self):
        config = {"positions": {
            k: v for k, v in VALID_CONFIG["positions"].items() if k != "FLOATING_POSITION"
        }}
        with pytest.raises(PositionConfigError, match="FLOATING_POSITION"):
            positions_from_dict(config)


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
              LIFT_POSITION:
                base: 45
                shoulder: 60
                elbow: 60
                wrist: 90
              RFID_SCAN_POSITION:
                base: 20
                shoulder: 60
                elbow: 30
                wrist: 90
              FLOATING_POSITION:
                base: 20
                shoulder: 60
                elbow: 60
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
        """The shipped measured config must itself be valid and complete."""
        config_path = pkg_root / "config" / "arm_positions.yaml"
        positions = load_positions(str(config_path))
        for required in (
            "HOME_POSITION", "PICK_DOWN_POSITION", "LIFT_POSITION", "RFID_SCAN_POSITION",
            "FLOATING_POSITION", "BIN_A_POSITION", "BIN_B_POSITION",
        ):
            assert required in positions

    def test_repo_default_config_values(self):
        """Pin the shipped values (exactly these seven positions, two bins) so
        an accidental edit is caught."""
        config_path = pkg_root / "config" / "arm_positions.yaml"
        positions = load_positions(str(config_path))
        assert positions == {
            "HOME_POSITION": ArmPosition(base=0, shoulder=105, elbow=95, wrist=100, wrist_pitch=90),
            "PICK_DOWN_POSITION": ArmPosition(base=0, shoulder=85, elbow=175, wrist=35, wrist_pitch=95),
            "LIFT_POSITION": ArmPosition(base=0, shoulder=85, elbow=125, wrist=35, wrist_pitch=95),
            "RFID_SCAN_POSITION": ArmPosition(base=55, shoulder=90, elbow=175, wrist=0, wrist_pitch=100),
            "FLOATING_POSITION": ArmPosition(base=55, shoulder=110, elbow=155, wrist=15, wrist_pitch=100),
            "BIN_A_POSITION": ArmPosition(base=85, shoulder=110, elbow=155, wrist=15, wrist_pitch=100),
            "BIN_B_POSITION": ArmPosition(base=110, shoulder=110, elbow=155, wrist=15, wrist_pitch=100),
        }

    def test_repo_default_floating_aligned_with_scan_and_bins(self):
        """FLOATING_POSITION shares RFID_SCAN_POSITION's base (straight lift
        off the reader) and every other joint with the bins (bin move is
        base-only)."""
        config_path = pkg_root / "config" / "arm_positions.yaml"
        positions = load_positions(str(config_path))
        floating = positions["FLOATING_POSITION"]
        assert floating.base == positions["RFID_SCAN_POSITION"].base
        for bin_name in ("BIN_A_POSITION", "BIN_B_POSITION"):
            bin_pose = positions[bin_name]
            assert (floating.shoulder, floating.elbow, floating.wrist, floating.wrist_pitch) == (
                bin_pose.shoulder, bin_pose.elbow, bin_pose.wrist, bin_pose.wrist_pitch
            ), bin_name

    def test_repo_default_config_within_default_joint_limits(self):
        """Every joint command of every shipped position must pass the
        ArmCommandParser default limits, or the arm would reject it mid-cycle."""
        config_path = pkg_root / "config" / "arm_positions.yaml"
        parser = ArmCommandParser()
        for name, position in load_positions(str(config_path)).items():
            for command in position.as_commands():
                is_valid, _, _, error_detail = parser.parse(command)
                assert is_valid, f"{name}: {command} -> {error_detail}"
