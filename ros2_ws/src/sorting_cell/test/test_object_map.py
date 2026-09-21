"""Unit tests for the RFID UID -> object -> bin mapping."""

import sys
import textwrap
from pathlib import Path

import pytest

pkg_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(pkg_root))

from sorting_cell.object_map import ObjectMap, ObjectMapConfigError, ObjectRecord, load_object_map


VALID_CONFIG = {
    "objects": {
        "UID_A": {"object_name": "OBJECT_A", "bin": "BIN_A"},
        "UID_B": {"object_name": "OBJECT_B", "bin": "BIN_B"},
    }
}


class TestObjectMapLookup:
    def setup_method(self):
        self.object_map = ObjectMap.from_dict(VALID_CONFIG)

    def test_known_uid_returns_record(self):
        record = self.object_map.lookup("UID_A")
        assert record == ObjectRecord(object_name="OBJECT_A", bin_name="BIN_A")

    def test_unknown_uid_returns_none(self):
        assert self.object_map.lookup("UID_DOES_NOT_EXIST") is None

    def test_whitespace_is_trimmed(self):
        assert self.object_map.lookup("  UID_A  ") is not None

    @pytest.mark.parametrize("bad_uid", [None, "", "   ", 123, 4.5])
    def test_invalid_input_returns_none(self, bad_uid):
        assert self.object_map.lookup(bad_uid) is None

    def test_bin_names_are_distinct_and_ordered(self):
        assert self.object_map.bin_names() == ["BIN_A", "BIN_B"]

    def test_known_uids(self):
        assert set(self.object_map.known_uids()) == {"UID_A", "UID_B"}


class TestObjectMapConfigValidation:
    @pytest.mark.parametrize("bad_config", [
        {},
        {"not_objects": {}},
        {"objects": {}},
        {"objects": "not_a_dict"},
        {"objects": {"UID_A": "not_a_dict"}},
        {"objects": {"UID_A": {"bin": "BIN_A"}}},  # missing object_name
        {"objects": {"UID_A": {"object_name": "OBJECT_A"}}},  # missing bin
        {"objects": {"UID_A": {"object_name": "", "bin": "BIN_A"}}},
        {"objects": {"UID_A": {"object_name": "OBJECT_A", "bin": ""}}},
        {"objects": {"": {"object_name": "OBJECT_A", "bin": "BIN_A"}}},
    ])
    def test_malformed_config_raises(self, bad_config):
        with pytest.raises(ObjectMapConfigError):
            ObjectMap.from_dict(bad_config)

    def test_valid_config_does_not_raise(self):
        ObjectMap.from_dict(VALID_CONFIG)


class TestLoadObjectMapFromFile:
    def test_load_valid_yaml(self, tmp_path):
        config_file = tmp_path / "object_map.yaml"
        config_file.write_text(textwrap.dedent("""
            objects:
              UID_1:
                object_name: OBJECT_1
                bin: BIN_A
        """))
        object_map = load_object_map(str(config_file))
        assert object_map.lookup("UID_1") == ObjectRecord(object_name="OBJECT_1", bin_name="BIN_A")

    def test_missing_file_raises(self, tmp_path):
        with pytest.raises(ObjectMapConfigError):
            load_object_map(str(tmp_path / "does_not_exist.yaml"))

    def test_invalid_yaml_raises(self, tmp_path):
        config_file = tmp_path / "object_map.yaml"
        config_file.write_text("objects: [this is not, valid: yaml: at all")
        with pytest.raises(ObjectMapConfigError):
            load_object_map(str(config_file))

    def test_repo_default_config_loads(self):
        """The shipped placeholder config must itself be valid."""
        config_path = pkg_root / "config" / "object_map.yaml"
        object_map = load_object_map(str(config_path))
        assert len(object_map.known_uids()) >= 1
