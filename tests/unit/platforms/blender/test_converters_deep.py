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

if typing.TYPE_CHECKING:
    from linkforge.blender.properties.robot_props import RobotPropertyGroup
    from linkforge.blender.properties.sensor_props import SensorPropertyGroup


def test_scene_to_robot_strict_mode() -> None:
    """Test that strict mode correctly raises vs collects errors."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    import linkforge.blender

    linkforge.blender.register()
    scene = bpy.context.scene
    assert scene is not None
    props = typing.cast("RobotPropertyGroup", scene.linkforge)
    props.robot_name = "test_robot"

    # Create a link
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.linkforge.is_robot_link = True
    obj.linkforge.link_name = "broken_link"

    with patch(
        "linkforge.blender.adapters.blender_to_core.blender_link_to_core_with_origin",
        side_effect=RobotModelError("Link error"),
    ):
        # 1. Strict mode = True
        scene.linkforge.strict_mode = True
        with pytest.raises(RobotModelError, match="Link error"):
            scene_to_robot(bpy.context)

        # 2. Strict mode = False
        scene.linkforge.strict_mode = False
        with pytest.raises(RobotModelError, match=r"Multiple configuration errors found"):
            scene_to_robot(bpy.context)


def test_sensor_origin_correction() -> None:
    """Test that sensors correctly calculate world offset relative to links."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    import linkforge.blender

    linkforge.blender.register()

    # Parent Link at (1, 1, 1)
    bpy.ops.object.empty_add(location=(1, 1, 1))
    link_obj = bpy.context.active_object
    link_obj.name = "base_link"
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.link_name = "base_link"

    # Sensor at (2, 2, 2)
    bpy.ops.object.empty_add(location=(2, 2, 2))
    sensor_obj = bpy.context.active_object
    assert sensor_obj is not None
    sensor_props = typing.cast("SensorPropertyGroup", sensor_obj.linkforge_sensor)
    sensor_props.is_robot_sensor = True
    sensor_props.attached_link = link_obj
    sensor_props.sensor_type = "CAMERA"

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


def test_ros2_control_conversion() -> None:
    """Test conversion of global ROS2 control properties."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    import linkforge.blender

    linkforge.blender.register()
    props = bpy.context.scene.linkforge
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


def test_gazebo_plugin_extraction() -> None:
    """Test extraction of Gazebo ros2_control plugin when configured."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    import linkforge.blender

    linkforge.blender.register()
    scene = bpy.context.scene
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
