"""Integration tests for Blender Naming and Synchronization."""

from __future__ import annotations

import pytest

from tests.blender_test_utils import (
    create_test_object,
    safe_get_linkforge,
    safe_get_sensor,
    safe_update,
)

# Name Synchronization


class TestNaming:
    def test_link_outliner_rename_sync(self, blender_clean_scene) -> None:
        """Verify that link outliner renames are synchronized to properties."""
        obj = create_test_object("base_link", None)
        obj_lf = safe_get_linkforge(obj)
        obj_lf.is_robot_link = True

        # Rename in Blender
        obj.name = "chassis"
        safe_update()

        assert obj_lf.link_name == "chassis"

    def test_name_sanitization(self, blender_clean_scene) -> None:
        """Verify that renames are sanitized for URDF compatibility."""
        obj = create_test_object("link", None)
        obj_lf = safe_get_linkforge(obj)
        obj_lf.is_robot_link = True

        # Rename with spaces
        obj.name = "front left wheel"
        safe_update()

        assert obj_lf.link_name == "front_left_wheel"
        assert obj.name == "front_left_wheel"

    # Robustness and Edge Cases
    def test_empty_name_guard(self, blender_clean_scene) -> None:
        """Verify that empty names are rejected and revert to object name."""
        obj = create_test_object("my_sensor", None)
        obj_lf = safe_get_sensor(obj)
        obj_lf.is_robot_sensor = True

        # Try setting empty
        obj_lf.sensor_name = ""
        assert obj_lf.sensor_name == "my_sensor"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
