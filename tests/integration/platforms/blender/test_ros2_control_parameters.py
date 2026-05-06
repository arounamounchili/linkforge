"""Integration test for ros2_control hardware parameters and sensor type."""

from typing import Any

import bpy
from linkforge.linkforge_core.parsers.urdf_parser import URDFParser


def test_ros2_control_sensor_and_parameters_export(clean_scene) -> None:
    """Test that choosing 'sensor' type exports read-only and includes parameters."""
    scene = bpy.context.scene or bpy.data.scenes[0]
    props: Any = getattr(scene, "linkforge")

    # 1. Create a simple robot structure
    bpy.ops.mesh.primitive_cube_add()
    base_obj = bpy.context.active_object
    assert base_obj is not None
    base_obj.name = "base_link"
    base_lf: Any = getattr(base_obj, "linkforge")
    base_lf.is_robot_link = True

    bpy.ops.mesh.primitive_cube_add()
    sensor_obj = bpy.context.active_object
    assert sensor_obj is not None
    sensor_obj.name = "sensor_link"
    sensor_lf: Any = getattr(sensor_obj, "linkforge")
    sensor_lf.is_robot_link = True

    # Create joint
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    joint_obj = bpy.context.active_object
    assert joint_obj is not None
    # Setup Link & Joint
    scene = bpy.context.scene or bpy.data.scenes[0]
    collection = scene.collection
    assert collection is not None
    joint_obj.name = "joint1"
    joint_lf: Any = getattr(joint_obj, "linkforge_joint")
    joint_lf.is_robot_joint = True
    joint_lf.parent_link = base_obj
    joint_lf.child_link = sensor_obj

    # 2. Configure ros2_control
    props.use_ros2_control = True
    props.ros2_control_name = "MySensorSystem"
    props.ros2_control_type = "sensor"
    props.hardware_plugin = "my_sensor_plugin"

    # Add global parameter
    p = props.ros2_control_parameters.add()
    p.name = "ip_address"
    p.value = "192.168.1.100"

    # Add joint to control system
    # Instead of bpy.ops.linkforge (unregistered), add manually to the collection
    joint_item = props.ros2_control_joints.add()
    joint_item.name = "joint1"

    # Configure joint interfaces (state only)
    joint_item.cmd_position = False
    joint_item.state_position = True

    # Add joint parameter
    jp = joint_item.parameters.add()
    jp.name = "encoder_res"
    jp.value = "4096"

    # 3. Export to URDF (using the adapter to get the Robot model)
    from pathlib import Path

    from linkforge.blender.adapters.blender_to_core import scene_to_robot

    robot, _ = scene_to_robot(bpy.context, Path("/tmp"), dry_run=True)

    from linkforge.linkforge_core import URDFGenerator

    exported_urdf = URDFGenerator().generate(robot)

    # 4. Verify URDF Content
    # We use the parser to verify the structure easily
    parsed_robot = URDFParser().parse_string(exported_urdf)
    assert len(parsed_robot.ros2_controls) == 1
    rc = parsed_robot.ros2_controls[0]

    assert rc.name == "MySensorSystem"
    assert rc.type == "sensor"
    assert rc.hardware_plugin == "my_sensor_plugin"
    assert rc.parameters["ip_address"] == "192.168.1.100"

    assert len(rc.joints) == 1
    rj = rc.joints[0]
    assert rj.name == "joint1"
    assert not rj.command_interfaces
    assert rj.state_interfaces == ["position", "velocity"]
    assert rj.parameters["encoder_res"] == "4096"

    # Verify XML tags manually for peace of mind
    import xml.etree.ElementTree as ET

    root = ET.fromstring(exported_urdf)
    rc_xml = root.find("ros2_control")
    assert rc_xml is not None, "ros2_control element not found in URDF"
    hw_xml = rc_xml.find("hardware")
    assert hw_xml is not None, "hardware element not found in ros2_control"

    # Check global param
    param_xml = hw_xml.find(".//param[@name='ip_address']")
    assert param_xml is not None
    assert param_xml.text == "192.168.1.100"

    # Check joint param
    joint_xml = rc_xml.find(".//joint[@name='joint1']")
    assert joint_xml is not None, "joint1 not found in ros2_control"
    joint_param_xml = joint_xml.find(".//param[@name='encoder_res']")
    assert joint_param_xml is not None
    assert joint_param_xml.text == "4096"

    # 5. ROUNDTRIP VERIFICATION: Import back to Blender and check properties
    # Clear current scene properties to simulate fresh import
    props.ros2_control_parameters.clear()
    props.ros2_control_joints.clear()
    props.use_ros2_control = False

    from linkforge.blender.adapters.core_to_blender import import_robot_to_scene

    # We pass the parsed_robot back into the scene
    import_robot_to_scene(parsed_robot, Path("/tmp/Untitled.urdf"), bpy.context)

    # Check if properties were restored from the parsed model
    assert props.use_ros2_control is True
    assert props.ros2_control_name == "MySensorSystem"
    assert props.ros2_control_type == "sensor"

    # Check global param restoration
    assert len(props.ros2_control_parameters) == 1
    assert props.ros2_control_parameters[0].name == "ip_address"
    assert props.ros2_control_parameters[0].value == "192.168.1.100"

    # Check joint param restoration
    assert len(props.ros2_control_joints) == 1
    rj_item = props.ros2_control_joints[0]
    assert rj_item.name == "joint1"
    assert len(rj_item.parameters) == 1
    assert rj_item.parameters[0].name == "encoder_res"
    assert rj_item.parameters[0].value == "4096"
