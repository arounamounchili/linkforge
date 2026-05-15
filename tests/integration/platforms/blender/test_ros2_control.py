"""Integration tests for Blender ROS 2 Control."""

from __future__ import annotations

import bpy

from tests.blender_test_utils import (
    safe_get_linkforge_scene,
)

# ROS 2 Control Parameter Extraction


class TestROS2ControlIntegration:
    def test_ros2_control_parameters(self, blender_clean_scene) -> None:
        """Verify that joint parameters are correctly stored in the scene config."""
        scene = bpy.context.scene
        lf_scene = safe_get_linkforge_scene(scene)

        # Add a joint to control config
        item = lf_scene.ros2_control_joints.add()
        item.name = "joint1"
        item.cmd_position = True
        item.state_position = True

        assert len(lf_scene.ros2_control_joints) == 1
        assert lf_scene.ros2_control_joints[0].name == "joint1"
        assert lf_scene.ros2_control_joints[0].cmd_position is True
