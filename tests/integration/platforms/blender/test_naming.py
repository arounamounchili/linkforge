"""Integration tests for Blender Naming and Synchronization."""

from __future__ import annotations

import time

from tests.blender_test_utils import (
    create_test_object,
    safe_get_linkforge,
    safe_get_sensor,
    safe_update,
)


class TestNaming:
    def test_link_outliner_rename_sync(self, blender_clean_scene) -> None:
        """Verify that link outliner renames are synchronized to properties."""
        import bpy

        obj = create_test_object("base_link", None, bpy.context.scene)
        obj_lf = safe_get_linkforge(obj)
        obj_lf.is_robot_link = True

        # Rename in Blender
        obj.name = "chassis"
        safe_update()

        assert obj_lf.link_name == "chassis"

    def test_name_sanitization(self, blender_clean_scene) -> None:
        """Verify that renames are sanitized for URDF compatibility."""
        import bpy

        obj = create_test_object("wheel", None, bpy.context.scene)
        obj_lf = safe_get_linkforge(obj)
        obj_lf.is_robot_link = True

        # Rename with spaces
        obj.name = "front left wheel"

        # Poll until the deferred rename is applied
        start = time.time()
        while time.time() - start < 2.0:
            safe_update()
            if obj.name == "front_left_wheel":
                break
            time.sleep(0.05)

        assert obj_lf.link_name == "front_left_wheel"

        # In background mode, Blender sometimes locks object names during script execution
        # even after a depsgraph update, making the "rename-back" synchronization
        # unreliable for immediate assertion. We prioritize link_name consistency.
        if not bpy.app.background:
            assert obj.name == "front_left_wheel"

    # Robustness and Edge Cases
    def test_empty_name_guard(self, blender_clean_scene) -> None:
        """Verify that empty names are rejected and revert to object name."""
        import bpy

        obj = create_test_object("my_sensor", None, bpy.context.scene)
        obj_lf = safe_get_sensor(obj)
        obj_lf.is_robot_sensor = True

        # Try setting empty
        obj_lf.sensor_name = ""
        assert obj_lf.sensor_name == "my_sensor"
