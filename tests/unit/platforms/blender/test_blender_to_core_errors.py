from unittest import mock

import bpy
import pytest
from linkforge.blender.adapters.blender_to_core import (
    blender_link_to_core_with_origin,
    blender_ros2_control_to_core,
    blender_sensor_to_core,
    scene_to_robot,
)
from linkforge.linkforge_core.exceptions import RobotModelError


def test_scene_to_robot_strict_mode_links(clean_scene) -> None:
    """Verify that strict_mode=True raises the original exception on link conversion error."""
    scene = bpy.context.scene
    scene.linkforge.strict_mode = True

    # Setup a root link
    root = bpy.data.objects.new("Root", None)
    scene.collection.objects.link(root)
    root.linkforge.is_robot_link = True

    # Mocking blender_link_to_core_with_origin to fail
    with (
        mock.patch(
            "linkforge.blender.adapters.blender_to_core.blender_link_to_core_with_origin",
            side_effect=RobotModelError("Link Fail"),
        ),
        pytest.raises(RobotModelError, match="Link Fail"),
    ):
        from unittest.mock import MagicMock

        context = MagicMock()
        context.scene = scene
        scene_to_robot(context)


def test_scene_to_robot_strict_mode_others(clean_scene) -> None:
    """Verify strict_mode=True for joints, sensors, and transmissions."""
    scene = bpy.context.scene
    scene.linkforge.strict_mode = True

    # 1. Joint failure
    j_obj = bpy.data.objects.new("JointObj", None)
    scene.collection.objects.link(j_obj)
    j_obj.linkforge_joint.is_robot_joint = True

    # Signatures: link_objects, joint_objects, sensor_objects, transmission_objects, joints_map, root_link
    with (
        mock.patch(
            "linkforge.blender.adapters.blender_to_core._categorize_scene_objects",
            return_value=({}, [j_obj], [], [], {}, None),
        ),
        mock.patch(
            "linkforge.blender.adapters.blender_to_core.blender_joint_to_core",
            side_effect=RobotModelError("Joint Fail"),
        ),
    ):
        from unittest.mock import MagicMock

        context = MagicMock()
        context.scene = scene
        with pytest.raises(RobotModelError, match="Joint Fail"):
            scene_to_robot(context)


def test_sensor_attachment_error(clean_scene) -> None:
    """Verify RobotModelError when sensor has no attached link."""
    s = bpy.data.objects.new("Sensor", None)
    bpy.context.collection.objects.link(s)
    s.linkforge_sensor.is_robot_sensor = True
    s.linkforge_sensor.attached_link = None

    with pytest.raises(RobotModelError, match="is not attached to any link"):
        blender_sensor_to_core(s)


def test_contact_sensor_fallback(clean_scene) -> None:
    """Verify contact sensor collision name fallback."""
    link = bpy.data.objects.new("Link", None)
    bpy.context.collection.objects.link(link)
    link.linkforge.is_robot_link = True
    link.linkforge.link_name = "test_link"

    s = bpy.data.objects.new("Contact", None)
    bpy.context.collection.objects.link(s)
    s.linkforge_sensor.is_robot_sensor = True
    s.linkforge_sensor.sensor_type = "CONTACT"
    s.linkforge_sensor.attached_link = link
    s.linkforge_sensor.contact_collision = ""  # Trigger fallback

    core = blender_sensor_to_core(s)
    assert core.contact_info.collision == "test_link_collision"


def test_ros2_control_logic_gaps(clean_scene) -> None:
    """Hit remaining branches in ROS2 control conversion."""
    # props is None
    assert blender_ros2_control_to_core(None) is None

    # Missing name
    scene = bpy.context.scene
    scene.linkforge.use_ros2_control = True
    scene.linkforge.ros2_control_name = ""
    # Should return None because name is empty
    assert blender_ros2_control_to_core(scene.linkforge) is None

    # interfaces selected with one empty (should return defaults for others)
    scene.linkforge.ros2_control_name = "test_controller"
    item = scene.linkforge.ros2_control_joints.add()
    item.name = "joint1"
    item.cmd_position = False
    item.cmd_velocity = False
    item.cmd_effort = False
    item.state_position = True  # Need at least one True to enter the block
    item.state_velocity = False
    item.state_effort = False

    ctrl = blender_ros2_control_to_core(scene.linkforge)
    assert ctrl is not None
    # Verify defaults are hit for command_interfaces
    # cmd_ifs should default to position because state_ifs is not empty
    assert ctrl.joints[0].command_interfaces == ["position"]
    assert ctrl.joints[0].state_interfaces == ["position"]


def test_inertia_mesh_fallback(clean_scene) -> None:
    """Verify mesh inertia fallback to bounding box."""
    from linkforge.blender.adapters.blender_to_core import blender_link_to_core_with_origin

    o = bpy.data.objects.new("BadMesh", bpy.data.meshes.new("EmptyMesh"))
    bpy.context.collection.objects.link(o)
    o.linkforge.is_robot_link = True
    o.linkforge.use_auto_inertia = True
    o.linkforge.mass = 1.0
    o.dimensions = (1.0, 1.0, 1.0)  # Ensure non-zero dimensions

    vis = bpy.data.objects.new("BadMesh_visual", None)
    vis.parent = o
    bpy.context.collection.objects.link(vis)

    # Mock extract_mesh_triangles to return None (triggering Box fallback in blender_link_to_core_with_origin)
    with mock.patch(
        "linkforge.blender.adapters.blender_to_core.extract_mesh_triangles", return_value=None
    ):
        core = blender_link_to_core_with_origin(o, dry_run=True)
        # Bounding box of Empty/Mesh with dimensions should give non-zero inertia
        if core and core.inertial:
            assert core.inertial.inertia.ixx > 0


def test_scene_to_robot_non_strict_errors(clean_scene) -> None:
    """Verify that strict_mode=False collects errors instead of raising."""
    scene = bpy.context.scene
    scene.linkforge.strict_mode = False

    # Setup some objects that will fail conversion
    j_obj = bpy.data.objects.new("BadJoint", None)
    scene.collection.objects.link(j_obj)
    j_obj.linkforge_joint.is_robot_joint = True

    s_obj = bpy.data.objects.new("BadSensor", None)
    scene.collection.objects.link(s_obj)
    s_obj.linkforge_sensor.is_robot_sensor = True

    with (
        mock.patch(
            "linkforge.blender.adapters.blender_to_core.blender_joint_to_core",
            side_effect=Exception("Joint Fail"),
        ),
        mock.patch(
            "linkforge.blender.adapters.blender_to_core.blender_sensor_to_core",
            side_effect=Exception("Sensor Fail"),
        ),
    ):
        # Also trigger transmission and ros2_control errors
        t_obj = bpy.data.objects.new("BadTrans", None)
        scene.collection.objects.link(t_obj)
        t_obj.linkforge_transmission.is_robot_transmission = True

        with mock.patch(
            "linkforge.blender.adapters.blender_to_core.blender_transmission_to_core",
            side_effect=Exception("Trans Fail"),
        ):
            scene.linkforge.use_ros2_control = True
            scene.linkforge.ros2_control_name = "test"
            with mock.patch(
                "linkforge.blender.adapters.blender_to_core.blender_ros2_control_to_core",
                side_effect=Exception("Control Fail"),
            ):
                from unittest.mock import MagicMock

                context = MagicMock()
                context.scene = scene
                # It always raises RobotModelError at the end if errors exist
                with pytest.raises(RobotModelError, match=r"Multiple configuration errors found"):
                    scene_to_robot(context)


def test_material_default_fallback(clean_scene) -> None:
    """Verify material default gray color fallback."""
    from linkforge.blender.adapters.blender_to_core import get_object_material

    o = bpy.data.objects.new("NoMat", None)
    bpy.context.collection.objects.link(o)

    props = mock.MagicMock()
    props.use_material = True

    # No material slots
    mat = get_object_material(o, props)
    assert mat.color.r == 0.8


def test_link_conversion_edge_cases(clean_scene) -> None:
    """Verify link conversion with custom urdf_name and invalid objects."""
    o = bpy.data.objects.new("Link", None)
    bpy.context.collection.objects.link(o)
    o.linkforge.is_robot_link = True

    # 1. urdf_name on child
    child_mesh = bpy.data.meshes.new("VMesh")
    import bmesh

    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bm.to_mesh(child_mesh)
    bm.free()

    child = bpy.data.objects.new("obj_visual", child_mesh)
    child.parent = o
    child["urdf_name"] = "custom_vis"
    bpy.context.collection.objects.link(child)

    core = blender_link_to_core_with_origin(o)
    assert core.visuals[0].name == "custom_vis"

    # 2. non-robot link
    o_non = bpy.data.objects.new("NonRobot", None)
    o_non.linkforge.is_robot_link = False
    assert blender_link_to_core_with_origin(o_non) is None


def test_scene_to_robot_empty_context() -> None:
    """Verify scene_to_robot returns early if context is invalid."""
    robot, errors = scene_to_robot(None)
    assert robot.name == "empty_robot"
    assert errors == []


def test_extract_mesh_triangles_none() -> None:
    """Verify extract_mesh_triangles returns None for non-meshes."""
    from linkforge.blender.adapters.blender_to_core import extract_mesh_triangles

    assert extract_mesh_triangles(None) is None
    o = bpy.data.objects.new("Empty", None)
    assert extract_mesh_triangles(o) is None


def test_ros2_control_state_default(clean_scene) -> None:
    """Hit state_ifs default."""
    scene = bpy.context.scene
    scene.linkforge.ros2_control_name = "test"
    item = scene.linkforge.ros2_control_joints.add()
    item.name = "joint1"
    item.cmd_position = True
    item.state_position = False
    item.state_velocity = False
    item.state_effort = False

    core = blender_ros2_control_to_core(scene.linkforge)
    assert core.joints[0].state_interfaces == ["position"]


def test_ros2_control_sensor_strips_commands(clean_scene) -> None:
    """Verify that sensor-type control systems strip command interfaces during export."""
    scene = bpy.context.scene
    scene.linkforge.ros2_control_name = "test_sensor"
    scene.linkforge.ros2_control_type = "sensor"

    item = scene.linkforge.ros2_control_joints.add()
    item.name = "joint1"
    # Select command interfaces, which should be stripped by the adapter
    item.cmd_position = True
    item.cmd_velocity = True
    # Configure state interfaces explicitly
    item.state_position = True
    item.state_velocity = False
    item.state_effort = False

    core = blender_ros2_control_to_core(scene.linkforge)
    assert core is not None
    assert core.type == "sensor"
    # Verify command interfaces were stripped
    assert len(core.joints[0].command_interfaces) == 0
    # Verify state interface was preserved
    assert core.joints[0].state_interfaces == ["position"]


def test_ros2_control_actuator_strips_extra_joints(clean_scene) -> None:
    """Verify that 'actuator' type systems strip extra joints and log a warning."""
    scene = bpy.context.scene
    robot_props = scene.linkforge
    robot_props.use_ros2_control = True
    robot_props.ros2_control_type = "actuator"

    # Add two joints to the control configuration
    j1 = robot_props.ros2_control_joints.add()
    j1.name = "joint1"
    j1.cmd_position = True

    j2 = robot_props.ros2_control_joints.add()
    j2.name = "joint2"
    j2.cmd_position = True

    with mock.patch("linkforge.blender.adapters.blender_to_core.logger") as mock_logger:
        core = blender_ros2_control_to_core(scene.linkforge)

    # Should only have one joint
    assert core is not None
    assert core.type == "actuator"
    assert len(core.joints) == 1
    assert core.joints[0].name == "joint1"

    # Should have logged a warning
    assert mock_logger.warning.called
    args, _ = mock_logger.warning.call_args
    assert "limited to exactly one joint" in args[0]
