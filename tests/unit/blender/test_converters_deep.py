from unittest.mock import MagicMock

import bpy
import pytest
from linkforge.blender.converters import (
    blender_ros2_control_to_core,
    scene_to_robot,
)
from linkforge_core.models import CameraInfo, Link, Sensor, SensorType


def test_scene_to_robot_strict_mode(mocker):
    """Test that strict mode correctly raises vs collects errors."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    import linkforge.blender

    linkforge.blender.register()
    scene = bpy.context.scene
    scene.linkforge.robot_name = "test_robot"

    # Create a link
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.linkforge.is_robot_link = True
    obj.linkforge.link_name = "broken_link"

    # Mock blender_link_to_core to throw
    mocker.patch(
        "linkforge.blender.converters.blender_link_to_core_with_origin",
        side_effect=ValueError("Link error"),
    )

    # 1. Strict mode = True
    scene.linkforge.strict_mode = True
    with pytest.raises(ValueError, match="Link error"):
        scene_to_robot(bpy.context)

    # 2. Strict mode = False
    scene.linkforge.strict_mode = False
    with pytest.raises(ValueError, match="Unable to build robot model"):
        scene_to_robot(bpy.context)


def test_sensor_origin_correction(mocker):
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
    sensor_obj.linkforge_sensor.is_robot_sensor = True
    sensor_obj.linkforge_sensor.attached_link = link_obj
    sensor_obj.linkforge_sensor.sensor_type = "CAMERA"

    bpy.context.view_layer.update()

    # Scenario: Deep correction in scene_to_robot
    mocker.patch(
        "linkforge.blender.converters.blender_link_to_core_with_origin",
        return_value=Link(name="base_link"),
    )
    mocker.patch(
        "linkforge.blender.converters.blender_sensor_to_core",
        return_value=Sensor(
            name="cam", type=SensorType.CAMERA, link_name="base_link", camera_info=CameraInfo()
        ),
    )

    robot, _ = scene_to_robot(bpy.context)
    assert len(robot.sensors) == 1
    # Check relative origin: (2-1, 2-1, 2-1) = (1,1,1)
    vec = robot.sensors[0].origin.xyz
    assert (vec.x, vec.y, vec.z) == pytest.approx((1.0, 1.0, 1.0))


def test_ros2_control_conversion():
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
    assert ctrl.name == "RealRobot"
    assert ctrl.type == "system"
    assert ctrl.hardware_plugin == "my_hardware/RobotHW"
    assert len(ctrl.joints) == 1


def test_gazebo_plugin_extraction(mocker):
    """Test extraction of Gazebo ros2_control plugin when configured."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    import linkforge.blender

    linkforge.blender.register()
    props = bpy.context.scene.linkforge
    props.use_ros2_control = True
    props.gazebo_plugin_name = "gazebo_ros2_control"
    props.controllers_yaml_path = "/config/ctrl.yaml"

    mocker.patch(
        "linkforge.blender.converters._categorize_scene_objects",
        return_value=({}, [], [], {}, None),
    )
    mocker.patch(
        "linkforge.blender.converters.blender_ros2_control_to_core", return_value=MagicMock()
    )

    robot, _ = scene_to_robot(bpy.context)

    assert len(robot.gazebo_elements) == 1
    plugin = robot.gazebo_elements[0].plugins[0]
    assert plugin.name == "gazebo_ros2_control"
    assert plugin.parameters["parameters"] == "/config/ctrl.yaml"
