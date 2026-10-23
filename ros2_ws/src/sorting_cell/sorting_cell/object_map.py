"""RFID UID -> object -> bin mapping for the sorting_cell subsystem.

Kept deliberately free of rclpy so it can be loaded and unit tested on any
platform. The UID list is data (config/object_map.yaml), never
hard-coded here, so tags can be added without code changes.
"""

from dataclasses import dataclass
from typing import Dict, Optional

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    yaml = None
    YAML_AVAILABLE = False


class ObjectMapConfigError(Exception):
    """Raised when the UID -> object -> bin configuration is missing or malformed."""


@dataclass(frozen=True)
class ObjectRecord:
    """A single identified object and its destination bin."""
    object_name: str
    bin_name: str


class ObjectMap:
    """Looks up the object/bin for a scanned RFID UID."""

    def __init__(self, records: Dict[str, ObjectRecord]) -> None:
        self._records = records

    def lookup(self, uid: str) -> Optional[ObjectRecord]:
        """Return the ObjectRecord for a UID, or None if the UID is unknown."""
        if not uid or not isinstance(uid, str):
            return None
        return self._records.get(uid.strip())

    def known_uids(self):
        return list(self._records.keys())

    def bin_names(self):
        """Distinct bin names referenced by this mapping (order-preserving)."""
        seen = []
        for record in self._records.values():
            if record.bin_name not in seen:
                seen.append(record.bin_name)
        return seen

    @classmethod
    def from_dict(cls, data: dict) -> "ObjectMap":
        """Build an ObjectMap from an already-parsed dict (as loaded from YAML)."""
        if not isinstance(data, dict) or "objects" not in data:
            raise ObjectMapConfigError("Config must contain a top-level 'objects' mapping")

        raw_objects = data["objects"]
        if not isinstance(raw_objects, dict) or not raw_objects:
            raise ObjectMapConfigError("'objects' must be a non-empty mapping of UID -> {object_name, bin}")

        records: Dict[str, ObjectRecord] = {}
        for uid, entry in raw_objects.items():
            if not isinstance(uid, str) or not uid.strip():
                raise ObjectMapConfigError(f"Invalid UID key: {uid!r}")
            if not isinstance(entry, dict):
                raise ObjectMapConfigError(f"Entry for UID '{uid}' must be a mapping")

            object_name = entry.get("object_name")
            bin_name = entry.get("bin")

            if not object_name or not isinstance(object_name, str):
                raise ObjectMapConfigError(f"UID '{uid}' is missing a valid 'object_name'")
            if not bin_name or not isinstance(bin_name, str):
                raise ObjectMapConfigError(f"UID '{uid}' is missing a valid 'bin'")

            records[uid.strip()] = ObjectRecord(object_name=object_name, bin_name=bin_name)

        return cls(records)


def load_object_map(path: str) -> ObjectMap:
    """Load and validate the UID -> object -> bin mapping from a YAML file.

    :raises ObjectMapConfigError: if the file is missing, unreadable, or malformed.
    """
    if not YAML_AVAILABLE:
        raise ObjectMapConfigError("PyYAML is required to load object_map.yaml but is not installed")

    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f)
    except FileNotFoundError as err:
        raise ObjectMapConfigError(f"Object map config not found: {path}") from err
    except yaml.YAMLError as err:
        raise ObjectMapConfigError(f"Object map config is not valid YAML: {path} ({err})") from err

    return ObjectMap.from_dict(data or {})
