import typing
from unittest.mock import MagicMock, patch

import bpy
import pytest
from linkforge.blender.adapters.blender_to_core import (
    blender_ros2_control_to_core,
    scene_to_robot,
)
from linkforge_core.exceptions import RobotModelError
from linkforge_core.models import CameraInfo, Link, Sensor, SensorType

from tests.blender_test_utils import create_test_object

if typing.TYPE_CHECKING:
    from linkforge.blender.properties.robot_props import RobotPropertyGroup


def test_scene_to_robot_strict_mode(scene) -> None:
    """Test that strict mode correctly raises vs collects errors."""
    scene.linkforge.robot_name = "test_robot"
    obj = create_test_object("broken_link", None, scene)
    obj.linkforge.is_robot_link = True
    obj.linkforge.link_name = "broken_link"

    with patch(
        "linkforge.blender.adapters.blender_to_core.blender_link_to_core_with_origin",
        side_effect=RobotModelError("Link error"),
    ):
        # Strict mode = True
        scene.linkforge.strict_mode = True
        with pytest.raises(RobotModelError, match="Link error"):
            scene_to_robot(bpy.context)

        # Strict mode = False
        scene.linkforge.strict_mode = False
        with pytest.raises(RobotModelError, match=r"Multiple configuration errors found"):
            scene_to_robot(bpy.context)


def test_sensor_origin_correction(scene) -> None:
    """Test that sensors correctly calculate world offset relative to links."""
    # Parent Link at (1, 1, 1)
    link_obj = create_test_object("base_link", None, scene)
    link_obj.location = (1.0, 1.0, 1.0)
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.link_name = "base_link"

    # Sensor at (2, 2, 2)
    sensor_obj = create_test_object("Sensor", None, scene)
    sensor_obj.location = (2.0, 2.0, 2.0)

    # Ensure matrices are updated after setting locations
    bpy.context.view_layer.update()

    sensor_obj.linkforge_sensor.is_robot_sensor = True
    sensor_obj.linkforge_sensor.attached_link = link_obj
    sensor_obj.linkforge_sensor.sensor_type = "CAMERA"

    bpy.context.view_layer.update()

    with (
        patch(
            "linkforge.blender.adapters.blender_to_core.blender_link_to_core_with_origin",
            return_value=Link(name="base_link"),
        ),
        patch(
            "linkforge.blender.adapters.blender_to_core.blender_sensor_to_core",
            return_value=Sensor(
                name="cam", type=SensorType.CAMERA, link_name="base_link", camera_info=CameraInfo()
            ),
        ),
    ):
        robot, _ = scene_to_robot(bpy.context)
        assert len(robot.sensors) == 1
        # Check relative origin: (2-1, 2-1, 2-1) = (1,1,1)
        vec = robot.sensors[0].origin.xyz
        assert (vec.x, vec.y, vec.z) == pytest.approx((1.0, 1.0, 1.0))


def test_ros2_control_conversion(scene) -> None:
    """Test conversion of global ROS2 control properties."""
    props = scene.linkforge
    props.use_ros2_control = True
    props.ros2_control_name = "RealRobot"
    props.ros2_control_type = "system"
    props.hardware_plugin = "my_hardware/RobotHW"

    # Add a joint
    joint = props.ros2_control_joints.add()
    joint.name = "j1"
    joint.cmd_position = True

    ctrl = blender_ros2_control_to_core(props)
    assert ctrl is not None
    assert ctrl.name == "RealRobot"
    assert ctrl.type == "system"
    assert ctrl.hardware_plugin == "my_hardware/RobotHW"
    assert len(ctrl.joints) == 1


def test_gazebo_plugin_extraction(scene) -> None:
    """Test extraction of Gazebo ros2_control plugin when configured."""
    props = scene.linkforge

    assert scene is not None
    props = typing.cast("RobotPropertyGroup", scene.linkforge)
    props.use_ros2_control = True
    props.gazebo_plugin_name = "gazebo_ros2_control"
    props.controllers_yaml_path = "/config/ctrl.yaml"

    with (
        patch(
            "linkforge.blender.adapters.blender_to_core._categorize_scene_objects",
            return_value=({}, [], [], [], {}, None),
        ),
        patch(
            "linkforge.blender.adapters.blender_to_core.blender_ros2_control_to_core",
            return_value=MagicMock(),
        ),
    ):
        robot, _ = scene_to_robot(bpy.context)

    assert len(robot.gazebo_elements) == 1
    plugin = robot.gazebo_elements[0].plugins[0]
    assert plugin.name == "gazebo_ros2_control"
    assert plugin.parameters["parameters"] == "/config/ctrl.yaml"
