"""Named arm positions for the sorting_cell subsystem.

Kept deliberately free of rclpy so it can be loaded and unit tested on any
platform. Real joint angles are data (config/arm_positions.yaml), never
hard-coded here, since the physical cell and bin layout are not finalized.
"""

from dataclasses import dataclass
from typing import Dict, List

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    yaml = None
    YAML_AVAILABLE = False


REQUIRED_POSITION_NAMES = (
    "HOME_POSITION",
    "PICK_DOWN_POSITION",
    "LIFT_POSITION",
    "RFID_SCAN_POSITION",
    "FLOATING_POSITION",
)

REQUIRED_JOINTS = ("base", "shoulder", "elbow", "wrist")

# wrist_pitch is optional per-position: a position that omits it defaults to
# the HOME wrist_pitch angle (arm_controller.ino's HOME_ANGLE_WRIST_PITCH),
# so existing positions that only care about wrist roll don't need updating.
WRIST_PITCH_DEFAULT = 0.0


class PositionConfigError(Exception):
    """Raised when the named arm position configuration is missing or malformed."""


@dataclass(frozen=True)
class ArmPosition:
    """A named joint-angle target for the arm (gripper state is handled separately).

    wrist_pitch defaults to WRIST_PITCH_DEFAULT (0.0) when a position's YAML
    entry omits it.
    """
    base: float
    shoulder: float
    elbow: float
    wrist: float
    wrist_pitch: float = WRIST_PITCH_DEFAULT

    def as_commands(self) -> List[str]:
        """Return the ordered /arm/command strings needed to reach this position."""
        return [
            f"BASE {self.base}",
            f"SHOULDER {self.shoulder}",
            f"ELBOW {self.elbow}",
            f"WRIST {self.wrist}",
            f"WRIST_PITCH {self.wrist_pitch}",
        ]


def _build_position(name: str, entry: dict) -> ArmPosition:
    if not isinstance(entry, dict):
        raise PositionConfigError(f"Position '{name}' must be a mapping of joint angles")

    values = {}
    for joint in REQUIRED_JOINTS:
        if joint not in entry:
            raise PositionConfigError(f"Position '{name}' is missing required joint '{joint}'")
        try:
            values[joint] = float(entry[joint])
        except (TypeError, ValueError) as err:
            raise PositionConfigError(
                f"Position '{name}' has a non-numeric value for joint '{joint}': {entry[joint]!r}"
            ) from err

    if "wrist_pitch" in entry:
        try:
            values["wrist_pitch"] = float(entry["wrist_pitch"])
        except (TypeError, ValueError) as err:
            raise PositionConfigError(
                f"Position '{name}' has a non-numeric value for joint 'wrist_pitch': {entry['wrist_pitch']!r}"
            ) from err

    return ArmPosition(**values)


def positions_from_dict(data: dict) -> Dict[str, ArmPosition]:
    """Build a name -> ArmPosition dict from an already-parsed dict (as loaded from YAML)."""
    if not isinstance(data, dict) or "positions" not in data:
        raise PositionConfigError("Config must contain a top-level 'positions' mapping")

    raw_positions = data["positions"]
    if not isinstance(raw_positions, dict) or not raw_positions:
        raise PositionConfigError("'positions' must be a non-empty mapping of name -> joint angles")

    positions: Dict[str, ArmPosition] = {}
    for name, entry in raw_positions.items():
        if not isinstance(name, str) or not name.strip():
            raise PositionConfigError(f"Invalid position name: {name!r}")
        positions[name] = _build_position(name, entry)

    missing_required = [name for name in REQUIRED_POSITION_NAMES if name not in positions]
    if missing_required:
        raise PositionConfigError(f"Config is missing required position(s): {', '.join(missing_required)}")

    return positions


def load_positions(path: str) -> Dict[str, ArmPosition]:
    """Load and validate named arm positions from a YAML file.

    :raises PositionConfigError: if the file is missing, unreadable, or malformed.
    """
    if not YAML_AVAILABLE:
        raise PositionConfigError("PyYAML is required to load arm_positions.yaml but is not installed")

    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f)
    except FileNotFoundError as err:
        raise PositionConfigError(f"Arm positions config not found: {path}") from err
    except yaml.YAMLError as err:
        raise PositionConfigError(f"Arm positions config is not valid YAML: {path} ({err})") from err

    return positions_from_dict(data or {})
