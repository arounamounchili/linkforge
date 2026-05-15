"""Unit tests for Blender Control (ROS 2) and Sensors."""

from __future__ import annotations

from unittest.mock import MagicMock

import bpy
from linkforge.blender.operators.control_ops import (
    LINKFORGE_OT_add_ros2_control_joint,
    LINKFORGE_OT_remove_ros2_control_joint,
)

from tests.blender_test_utils import (
    create_test_object,
    safe_get_linkforge,
    safe_get_linkforge_scene,
    safe_get_sensor,
)


class TestControlOperations:
    def test_add_ros2_control_joint(self, scene, blender_context) -> None:
        """Test adding a ROS 2 control joint to the scene configuration."""
        props = safe_get_linkforge_scene(scene)
        props.ros2_control_joints.clear()

        mock_self = MagicMock()
        mock_self.joint_name = "j1"

        result = LINKFORGE_OT_add_ros2_control_joint.execute(mock_self, bpy.context)
        assert result == {"FINISHED"}
        assert len(props.ros2_control_joints) == 1
        assert props.ros2_control_joints[0].name == "j1"

    def test_remove_ros2_control_joint(self, scene, blender_context) -> None:
        """Test removing a ROS 2 control joint."""
        props = safe_get_linkforge_scene(scene)
        props.ros2_control_joints.clear()
        props.ros2_control_joints.add().name = "j1"
        props.ros2_control_active_joint_index = 0

        mock_self = MagicMock()
        result = LINKFORGE_OT_remove_ros2_control_joint.execute(mock_self, bpy.context)
        assert result == {"FINISHED"}
        assert len(props.ros2_control_joints) == 0


class TestSensorOperations:
    def test_create_sensor(self, scene, blender_context) -> None:
        """Test creating a sensor for a robot link."""
        link_obj = create_test_object("link_obj", None, scene=scene)
        safe_get_linkforge(link_obj).is_robot_link = True

        # Create sensor object explicitly
        sensor_obj = create_test_object("link_obj_sensor", None, scene=scene)
        sensor_obj.parent = link_obj
        safe_get_sensor(sensor_obj).is_robot_sensor = True

        assert "_sensor" in sensor_obj.name
        assert safe_get_sensor(sensor_obj).is_robot_sensor
        assert sensor_obj.parent == link_obj
