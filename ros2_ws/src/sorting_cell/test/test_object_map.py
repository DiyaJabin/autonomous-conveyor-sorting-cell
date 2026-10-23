"""Unit tests for the RFID UID -> object -> bin mapping."""

import sys
import textwrap
from pathlib import Path

import pytest

pkg_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(pkg_root))

from sorting_cell.object_map import ObjectMap, ObjectMapConfigError, ObjectRecord, load_object_map
from sorting_cell.positions import load_positions


VALID_CONFIG = {
    "objects": {
        "1DF058BA141080": {"object_name": "OBJECT_A2", "bin": "BIN_A"},
        "1DEC58BA141080": {"object_name": "OBJECT_B2", "bin": "BIN_B"},
    }
}


class TestObjectMapLookup:
    def setup_method(self):
        self.object_map = ObjectMap.from_dict(VALID_CONFIG)

    def test_known_uid_returns_record(self):
        record = self.object_map.lookup("1DF058BA141080")
        assert record == ObjectRecord(object_name="OBJECT_A2", bin_name="BIN_A")

    def test_unknown_uid_returns_none(self):
        assert self.object_map.lookup("UID_DOES_NOT_EXIST") is None

    def test_whitespace_is_trimmed(self):
        assert self.object_map.lookup("  1DF058BA141080  ") is not None

    @pytest.mark.parametrize("bad_uid", [None, "", "   ", 123, 4.5])
    def test_invalid_input_returns_none(self, bad_uid):
        assert self.object_map.lookup(bad_uid) is None

    def test_bin_names_are_distinct_and_ordered(self):
        assert self.object_map.bin_names() == ["BIN_A", "BIN_B"]

    def test_known_uids(self):
        assert set(self.object_map.known_uids()) == {"1DF058BA141080", "1DEC58BA141080"}


class TestObjectMapConfigValidation:
    @pytest.mark.parametrize("bad_config", [
        {},
        {"not_objects": {}},
        {"objects": {}},
        {"objects": "not_a_dict"},
        {"objects": {"1DF058BA141080": "not_a_dict"}},
        {"objects": {"1DF058BA141080": {"bin": "BIN_A"}}},  # missing object_name
        {"objects": {"1DF058BA141080": {"object_name": "OBJECT_A2"}}},  # missing bin
        {"objects": {"1DF058BA141080": {"object_name": "", "bin": "BIN_A"}}},
        {"objects": {"1DF058BA141080": {"object_name": "OBJECT_A2", "bin": ""}}},
        {"objects": {"": {"object_name": "OBJECT_A2", "bin": "BIN_A"}}},
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

    def test_several_uids_may_share_one_bin(self):
        object_map = ObjectMap.from_dict({
            "objects": {
                "1DF058BA141080": {"object_name": "OBJECT_A2", "bin": "BIN_A"},
                "1DEF58BA141080": {"object_name": "OBJECT_A3", "bin": "BIN_A"},
            }
        })
        assert object_map.lookup("1DF058BA141080").bin_name == "BIN_A"
        assert object_map.lookup("1DEF58BA141080").bin_name == "BIN_A"
        assert object_map.bin_names() == ["BIN_A"]

    def test_repo_default_config_loads(self):
        """The shipped config must itself be valid."""
        config_path = pkg_root / "config" / "object_map.yaml"
        object_map = load_object_map(str(config_path))
        assert len(object_map.known_uids()) >= 1

    def test_repo_default_config_entries(self):
        """Pin the shipped UID -> object -> bin entries."""
        config_path = pkg_root / "config" / "object_map.yaml"
        object_map = load_object_map(str(config_path))
        expected = {
            "1DF058BA141080": ObjectRecord(object_name="OBJECT_A2", bin_name="BIN_A"),
            "1DEF58BA141080": ObjectRecord(object_name="OBJECT_A3", bin_name="BIN_A"),
            "1DEC58BA141080": ObjectRecord(object_name="OBJECT_B2", bin_name="BIN_B"),
            "1DEE58BA141080": ObjectRecord(object_name="OBJECT_B3", bin_name="BIN_B"),
            "1DED58BA141080": ObjectRecord(object_name="OBJECT_B4", bin_name="BIN_B"),
        }
        assert set(object_map.known_uids()) == set(expected)
        for uid, record in expected.items():
            assert object_map.lookup(uid) == record
        assert object_map.bin_names() == ["BIN_A", "BIN_B"]

    def test_repo_default_bins_have_positions(self):
        """Every bin in the shipped object map must have a *_POSITION in the
        shipped arm_positions.yaml, or that object's cycle would fail."""
        positions = load_positions(str(pkg_root / "config" / "arm_positions.yaml"))
        object_map = load_object_map(str(pkg_root / "config" / "object_map.yaml"))
        for bin_name in object_map.bin_names():
            assert f"{bin_name}_POSITION" in positions
