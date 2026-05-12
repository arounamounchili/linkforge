"""Integration tests for Blender Sensors and Gazebo Plugins."""

from __future__ import annotations

from typing import Any

import bpy
import pytest

from tests.blender_test_utils import (
    create_robot_link,
    safe_get_linkforge,
    safe_get_linkforge_scene,
    safe_get_sensor,
    safe_update,
)


class TestSensorsPluginsIntegration:
    def test_sensor_creation_and_export(self, blender_clean_scene, tmp_path) -> None:
        """Verify that a sensor added in Blender is correctly exported to URDF."""
        scene = bpy.context.scene
        ops: Any = bpy.ops

        # 1. Create an Empty as link frame
        ops.linkforge.add_empty_link()
        link = bpy.context.active_object
        assert link is not None, "Failed to create or set active link frame"
        assert link.name == "base_link"

        # Ensure it has a name for the sensor naming logic
        safe_get_linkforge(link).link_name = "base_link"

        # 2. Add sensor
        if bpy.context.view_layer is not None:
            bpy.context.view_layer.objects.active = link
        else:
            bpy.context.active_object = link
        link.select_set(True)

        safe_update()

        # Execute sensor creation
        res = ops.linkforge.create_sensor()
        assert res == {"FINISHED"}

        # Find the sensor object (usually a child of the link)
        sensor_obj = next((c for c in link.children if "_sensor" in c.name), None)
        assert sensor_obj is not None

        s_props = safe_get_sensor(sensor_obj)
        s_props.sensor_type = "LIDAR"
        s_props.update_rate = 30.0
        s_props.lidar_horizontal_samples = 640

        safe_update()

        # Export and verify XML
        export_path = tmp_path / "sensor_test.urdf"
        res = ops.linkforge.export_robot_model(filepath=str(export_path))
        assert res == {"FINISHED"}

        urdf_content = export_path.read_text()
        assert "<sensor" in urdf_content
        assert 'type="gpu_lidar"' in urdf_content
        assert "<update_rate>30</update_rate>" in urdf_content
        assert "<samples>640</samples>" in urdf_content

    def test_gazebo_plugin_scene_export(self, blender_clean_scene, tmp_path) -> None:
        """Verify that a robot-level Gazebo plugin is correctly exported."""
        scene = bpy.context.scene
        lf_scene = safe_get_linkforge_scene(scene)
        ops: Any = bpy.ops

        # Add a robot link so there is something to export
        create_robot_link("base_link", scene)

        # Set scene-level Gazebo plugin (simplified for now, might need dedicated operator)
        # Note: In linkforge, robot-level plugins are often managed in scene properties
        lf_scene.gazebo_plugin_name = "libmy_plugin.so"

        safe_update()

        export_path = tmp_path / "plugin_test.urdf"
        res = ops.linkforge.export_robot_model(filepath=str(export_path))
        assert res == {"FINISHED"}

        urdf_content = export_path.read_text()
        assert "<gazebo>" in urdf_content
        assert '<plugin name="libmy_plugin.so" filename="libmy_plugin.so"' in urdf_content


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
