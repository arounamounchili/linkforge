"""Unit tests for Blender Control (ROS 2) and Sensors."""

from __future__ import annotations

from unittest.mock import MagicMock

import bpy
import pytest
from linkforge.blender.operators.control_ops import (
    LINKFORGE_OT_add_ros2_control_joint,
    LINKFORGE_OT_remove_ros2_control_joint,
)

from tests.blender_test_utils import (
    safe_get_linkforge,
    safe_get_linkforge_scene,
    safe_get_sensor,
)

# =============================================================================
# ROS 2 Control Operations
# =============================================================================


class TestControlOperations:
    def test_add_ros2_control_joint(self, scene) -> None:
        """Test adding a ROS 2 control joint to the scene configuration."""
        props = safe_get_linkforge_scene(scene)
        props.ros2_control_joints.clear()

        mock_self = MagicMock()
        mock_self.joint_name = "j1"

        result = LINKFORGE_OT_add_ros2_control_joint.execute(mock_self, bpy.context)
        assert result == {"FINISHED"}
        assert len(props.ros2_control_joints) == 1
        assert props.ros2_control_joints[0].name == "j1"

    def test_remove_ros2_control_joint(self, scene) -> None:
        """Test removing a ROS 2 control joint."""
        props = safe_get_linkforge_scene(scene)
        props.ros2_control_joints.clear()
        props.ros2_control_joints.add().name = "j1"
        props.ros2_control_active_joint_index = 0

        mock_self = MagicMock()
        result = LINKFORGE_OT_remove_ros2_control_joint.execute(mock_self, bpy.context)
        assert result == {"FINISHED"}
        assert len(props.ros2_control_joints) == 0


# =============================================================================
# Sensor Operations
# =============================================================================


class TestSensorOperations:
    def test_create_sensor(self, scene) -> None:
        """Test creating a sensor for a robot link."""
        bpy.ops.object.empty_add()
        bpy.context.active_object.name = "link_obj"
        link_obj = bpy.context.active_object
        safe_get_linkforge(link_obj).is_robot_link = True

        # Mock active object and selection for poll
        bpy.context.view_layer.objects.active = link_obj
        link_obj.select_set(True)

        bpy.ops.linkforge.create_sensor()
        sensor_obj = bpy.context.active_object
        assert "_sensor" in sensor_obj.name
        assert safe_get_sensor(sensor_obj).is_robot_sensor
        assert sensor_obj.parent == link_obj


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
