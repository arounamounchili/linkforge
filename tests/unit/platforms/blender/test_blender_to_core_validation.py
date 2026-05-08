"""Validation and robustness tests for Blender-to-Core adapter."""

from __future__ import annotations

from pathlib import Path
from unittest import mock

import bmesh
import bpy
import pytest
from linkforge.blender.adapters.blender_to_core import (
    blender_joint_to_core,
    blender_link_to_core_with_origin,
    blender_ros2_control_to_core,
    blender_sensor_to_core,
    detect_primitive_type,
    extract_mesh_triangles,
    get_object_geometry,
    matrix_to_transform,
    scene_to_robot,
)
from linkforge_core.exceptions import RobotValidationError, ValidationErrorCode
from linkforge_core.models import JointType, Vector3
from mathutils import Matrix

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_sensor,
)


def test_scene_to_robot_strict_mode(mock_context, scene) -> None:
    """Verify that strict_mode=True raises exceptions on conversion errors."""
    scene.linkforge.strict_mode = True

    # Setup a root link
    root = create_test_object("Root", None, scene)
    safe_get_linkforge(root).is_robot_link = True

    # Mock failure
    with (
        mock.patch(
            "linkforge.blender.adapters.blender_to_core.blender_link_to_core_with_origin",
            side_effect=RobotValidationError(ValidationErrorCode.INVALID_VALUE, "Link Fail"),
        ),
        pytest.raises(RobotValidationError, match=r"\[INVALID_VALUE\] Link Fail"),
    ):
        scene_to_robot(mock_context)


def test_scene_to_robot_non_strict_errors(mock_context, scene) -> None:
    """Verify that strict_mode=False collects errors instead of raising immediately."""
    scene.linkforge.strict_mode = False

    # Setup failing objects
    j_obj = create_test_object("BadJoint", None, scene)
    safe_get_joint(j_obj).is_robot_joint = True

    with (
        mock.patch(
            "linkforge.blender.adapters.blender_to_core.blender_joint_to_core",
            side_effect=RobotValidationError(ValidationErrorCode.INVALID_VALUE, "Joint Fail"),
        ),
        pytest.raises(RobotValidationError, match=r"Multiple configuration errors found"),
    ):
        scene_to_robot(mock_context)


def test_detect_primitive_type_robustness(scene) -> None:
    """Test detect_primitive_type with complex/invalid mesh edge cases."""
    # Non-quad box
    m = bpy.data.meshes.new("NonQuadBox")
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    bm.to_mesh(m)
    bm.free()
    o = create_test_object("NonQuadBox", m, scene)
    assert detect_primitive_type(o) is None

    # Distorted Sphere
    bpy.ops.mesh.primitive_uv_sphere_add(radius=1.0)
    sphere = bpy.context.active_object
    if sphere:
        sphere.dimensions = (1.0, 1.0, 5.0)
    if bpy.context.view_layer:
        bpy.context.view_layer.update()
    assert detect_primitive_type(sphere) is None

    # None/Empty
    assert detect_primitive_type(None) is None
    assert detect_primitive_type(create_test_object("Empty", None)) is None


def test_blender_sensor_to_core_basic(scene) -> None:
    """Verify basic sensor property extraction."""
    # Setup link first
    link = create_test_object("Link", None, scene)
    safe_get_linkforge(link).is_robot_link = True
    safe_get_linkforge(link).link_name = "SensorLink"

    o = create_test_object("Lidar", None, scene)
    safe_get_sensor(o).is_robot_sensor = True
    safe_get_sensor(o).sensor_type = "LIDAR"
    safe_get_sensor(o).attached_link = link


def test_joint_to_core_edge_cases(scene) -> None:
    """Test custom axis fallbacks and dynamics logic."""
    p = create_test_object("Parent", None, scene)
    c = create_test_object("Child", None, scene)
    safe_get_linkforge(p).is_robot_link = True
    safe_get_linkforge(c).is_robot_link = True

    j = create_test_object("Joint", None, scene)
    props = safe_get_joint(j)
    props.is_robot_joint = True
    props.parent_link = p
    props.child_link = c

    # Zero Custom Axis -> Fallback to Z
    props.axis = "CUSTOM"
    props.custom_axis_x = 0.0
    props.custom_axis_y = 0.0
    props.custom_axis_z = 0.0
    core = blender_joint_to_core(j)
    assert core and core.axis and core.axis.z == 1.0

    # Continuous with limits (strips range, keeps effort/vel)
    props.joint_type = "CONTINUOUS"
    props.use_limits = True
    props.limit_effort = 100.0
    core = blender_joint_to_core(j)
    assert core and core.type == JointType.CONTINUOUS
    assert core.limits and core.limits.lower is None
    assert core.limits.effort == 100.0


def test_ros2_control_conversion_logic(scene) -> None:
    """Verify ROS2 control type-specific stripping and defaults."""
    scene.linkforge.ros2_control_name = "test"

    # Sensor type strips command interfaces
    scene.linkforge.ros2_control_type = "sensor"
    item = scene.linkforge.ros2_control_joints.add()
    item.name = "joint1"
    item.cmd_position = True
    item.state_position = True

    core = blender_ros2_control_to_core(scene.linkforge)
    assert core and core.type == "sensor"
    assert core.joints and len(core.joints) > 0
    assert len(core.joints[0].command_interfaces) == 0

    # Actuator type limited to one joint
    scene.linkforge.ros2_control_type = "actuator"
    scene.linkforge.ros2_control_joints.add().name = "joint2"
    with mock.patch("linkforge.blender.adapters.blender_to_core.logger") as mock_log:
        core = blender_ros2_control_to_core(scene.linkforge)
        assert core and core.joints and len(core.joints) == 1
        assert mock_log.warning.called


def test_mesh_export_path(tmp_path, scene) -> None:
    """Verify mesh extraction path in get_object_geometry."""
    bpy.ops.mesh.primitive_monkey_add()
    monkey = bpy.context.active_object

    with mock.patch(
        "linkforge.blender.adapters.mesh_io.export_link_mesh",
        return_value=(Path("monkey.stl"), Matrix.Identity(4)),
    ):
        geom, _ = get_object_geometry(
            obj=monkey,
            geometry_type="MESH",
            link_name="Monkey",
            meshes_dir=tmp_path,
            dry_run=False,
        )
        from linkforge_core.models import Mesh

        assert isinstance(geom, Mesh)
        assert geom.resource and "monkey.stl" in geom.resource


def test_sensor_attachment_logic(mock_context, scene) -> None:
    """Verify sensor origin calculation and attachment errors."""
    link = create_test_object("Link", None, scene)
    safe_get_linkforge(link).is_robot_link = True
    link.matrix_world = Matrix.Translation((1, 1, 1))

    # Valid attachment with origin offset (tested via scene_to_robot)
    s = create_test_object("Camera", None, scene)
    safe_get_sensor(s).is_robot_sensor = True
    safe_get_sensor(s).attached_link = link
    s.matrix_world = Matrix.Translation((2, 2, 2))

    robot, _ = scene_to_robot(mock_context)
    assert robot and robot.sensors and len(robot.sensors) > 0
    assert robot.sensors[0].origin and robot.sensors[0].origin.xyz == Vector3(1.0, 1.0, 1.0)

    # Missing attachment error
    safe_get_sensor(s).attached_link = None
    with pytest.raises(RobotValidationError, match="not attached to any link"):
        blender_sensor_to_core(s)


def test_blender_link_to_core_basic(scene) -> None:
    """Verify basic link property extraction."""
    o = create_test_object("Link1", None, scene)
    safe_get_linkforge(o).is_robot_link = True
    safe_get_linkforge(o).link_name = "BaseLink"
    safe_get_linkforge(o).mass = 5.0
    safe_get_linkforge(o).use_auto_inertia = False
    safe_get_linkforge(o).inertia_origin_xyz = (0.5, 0.6, 0.7)

    core = blender_link_to_core_with_origin(o)
    assert core and core.inertial and core.inertial.origin
    assert core.inertial.origin.xyz.x == pytest.approx(0.5)


def test_adapter_fallbacks(scene) -> None:
    """Test various adapter fallbacks for matrices, meshes, and materials."""
    # Matrix fallback
    assert matrix_to_transform(None).xyz.x == 0.0

    # Mesh extraction fallback
    assert extract_mesh_triangles(None) is None

    # Material gray fallback
    o = create_test_object("NoMat", None, scene)
    from linkforge.blender.adapters.blender_to_core import get_object_material

    props = mock.MagicMock(use_material=True)
    mat = get_object_material(o, props)
    assert mat and mat.color
    assert mat.color.r == 0.8
