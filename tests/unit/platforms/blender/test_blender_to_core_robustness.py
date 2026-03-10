from pathlib import Path
from unittest.mock import patch

import bpy
import pytest
from linkforge.blender.adapters.blender_to_core import (
    blender_joint_to_core,
    blender_link_to_core_with_origin,
    detect_primitive_type,
    extract_mesh_triangles,
    get_object_geometry,
    matrix_to_transform,
    scene_to_robot,
)
from linkforge.linkforge_core.exceptions import RobotModelError
from linkforge.linkforge_core.models import (
    JointType,
)
from mathutils import Matrix


def test_detect_primitive_type_robustness(clean_scene):
    """Test detect_primitive_type with edge cases."""
    # 1. Box with non-quad faces (should return None)
    m = bpy.data.meshes.new("NonQuadBox")
    import bmesh

    bm = bmesh.new()
    # Create a box but triangulate it
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    bm.to_mesh(m)
    bm.free()
    o = bpy.data.objects.new("NonQuadBox", m)
    bpy.context.collection.objects.link(o)

    assert detect_primitive_type(o) is None

    # 2. Non-uniform Sphere (should return None)
    bpy.ops.mesh.primitive_uv_sphere_add(radius=1.0)
    sphere = bpy.context.active_object
    sphere.dimensions = (1.0, 1.0, 5.0)  # Very distorted
    bpy.context.view_layer.update()
    assert detect_primitive_type(sphere) is None

    # 3. Cylinder with spherical proportions (should return None)
    bpy.ops.mesh.primitive_cylinder_add(radius=1.0, depth=1.0)
    cyl = bpy.context.active_object
    # Ratio dims.z / max(dims.x, dims.y) = 2.0 / 2.0 = 1.0
    # In Cylinder detection: xy_ratio > 0.9, z_vs_xy = 1.0.
    # It fails if (z_vs_xy < 0.9 or z_vs_xy > 1.1) is False.
    # So if it IS a sphere-like cylinder, it returns None correctly.
    cyl.dimensions = (2.0, 2.0, 2.0)
    bpy.context.view_layer.update()
    assert detect_primitive_type(cyl) is None


def test_scene_to_robot_strict_mode(clean_scene):
    """Test strict_mode behavior in scene_to_robot."""
    # Set up scene with a broken link (is_robot_link=True but no geometry/faulty)
    p = bpy.data.objects.new("FaultyLink", None)
    bpy.context.collection.objects.link(p)
    p.linkforge.is_robot_link = True

    bpy.context.scene.linkforge.strict_mode = True

    # Mock blender_link_to_core_with_origin to raise error
    with (
        patch(
            "linkforge.blender.adapters.blender_to_core.blender_link_to_core_with_origin",
            side_effect=RuntimeError("Link Error"),
        ),
        pytest.raises(RuntimeError, match="Link Error"),
    ):
        scene_to_robot(bpy.context)

    # Test non-strict mode error collection
    bpy.context.scene.linkforge.strict_mode = False
    with (
        patch(
            "linkforge.blender.adapters.blender_to_core.blender_link_to_core_with_origin",
            side_effect=RobotModelError("Link Failed"),
        ),
        pytest.raises(RobotModelError, match="Link Failed"),
    ):
        # It wraps the collection of errors into one RobotModelError at the end
        scene_to_robot(bpy.context)


def test_joint_to_core_advanced_properties(clean_scene):
    """Test custom axis, dynamics, and continuous limits."""
    # Parent and Child Links
    p = bpy.data.objects.new("Parent", None)
    c = bpy.data.objects.new("Child", None)
    bpy.context.collection.objects.link(p)
    bpy.context.collection.objects.link(c)
    p.linkforge.is_robot_link = True
    c.linkforge.is_robot_link = True

    j = bpy.data.objects.new("Joint", None)
    bpy.context.collection.objects.link(j)
    props = j.linkforge_joint
    props.is_robot_joint = True
    props.parent_link = p
    props.child_link = c

    # 1. Custom Axis
    props.joint_type = "REVOLUTE"
    props.axis = "CUSTOM"
    props.custom_axis_x = 1.0
    props.custom_axis_y = 1.0
    props.custom_axis_z = 0.0
    # Zero vector case
    props.custom_axis_x = 0.0
    props.custom_axis_y = 0.0
    props.custom_axis_z = 0.0
    core = blender_joint_to_core(j, bpy.context.scene)
    assert core.axis.x == 0.0
    assert core.axis.y == 0.0
    assert core.axis.z == 1.0  # Fallback

    # 2. Dynamics
    props.use_dynamics = True
    props.dynamics_damping = 0.5
    props.dynamics_friction = 0.1
    core = blender_joint_to_core(j, bpy.context.scene)
    assert core.dynamics.damping == pytest.approx(0.5)
    assert core.dynamics.friction == pytest.approx(0.1)

    # 3. Continuous with limits (effort/velocity only)
    props.joint_type = "CONTINUOUS"
    props.use_limits = True
    props.limit_effort = 100.0
    props.limit_velocity = 1.0
    core = blender_joint_to_core(j, bpy.context.scene)
    assert core.type == JointType.CONTINUOUS
    assert core.limits.effort == 100.0
    assert core.limits.lower is None


def test_ros2_control_gazebo_plugin(clean_scene):
    """Test Gazebo plugin generation for ROS2 control."""
    scene_props = bpy.context.scene.linkforge
    scene_props.use_ros2_control = True
    scene_props.gazebo_plugin_name = "custom_gazebo_plugin"
    scene_props.controllers_yaml_path = "/path/to/controllers.yaml"

    # Minimal links to allow scene_to_robot to finish
    p = bpy.data.objects.new("Root", None)
    bpy.context.collection.objects.link(p)
    p.linkforge.is_robot_link = True

    # Mock ros2_control conversion to return something valid
    from linkforge.linkforge_core.models.ros2_control import Ros2Control

    with patch(
        "linkforge.blender.adapters.blender_to_core.blender_ros2_control_to_core",
        return_value=Ros2Control(name="test_control", hardware_plugin="mock"),
    ):
        robot, _ = scene_to_robot(bpy.context)
        assert len(robot.gazebo_elements) == 1
        assert robot.gazebo_elements[0].plugins[0].name == "gazebo_ros2_control"
        assert (
            robot.gazebo_elements[0].plugins[0].parameters["parameters"]
            == "/path/to/controllers.yaml"
        )


def test_get_object_geometry_mesh_export(clean_scene, tmp_path):
    """Test mesh export path in get_object_geometry."""
    bpy.ops.mesh.primitive_monkey_add()
    monkey = bpy.context.active_object

    # Mock export_link_mesh to simulate success
    with patch(
        "linkforge.blender.adapters.mesh_io.export_link_mesh",
        return_value=(Path("monkey.stl"), Matrix.Identity(4)),
    ):
        from linkforge.blender.adapters.blender_to_core import get_object_geometry

        geom, _ = get_object_geometry(
            obj=monkey,
            geometry_type="MESH",
            link_name="MonkeyLink",
            meshes_dir=tmp_path,
            dry_run=False,
        )
        # Core Mesh model uses 'resource'
        assert hasattr(geom, "resource")
        assert geom.resource == str(Path("monkey.stl"))


def test_sensor_origin_custom_mount(clean_scene):
    """Test sensor corrected origin calculation when not a direct child."""
    # Link
    p = bpy.data.objects.new("Base", None)
    bpy.context.collection.objects.link(p)
    p.linkforge.is_robot_link = True
    p.matrix_world = Matrix.Translation((1, 1, 1))

    # Sensor (not parented to Base in Blender, but specifying link_name)
    s = bpy.data.objects.new("Camera", None)
    bpy.context.collection.objects.link(s)
    s.linkforge_sensor.is_robot_sensor = True
    s.linkforge_sensor.sensor_type = "CAMERA"
    s.linkforge_sensor.link_name = "Base"
    s.linkforge_sensor.attached_link = p
    s.matrix_world = Matrix.Translation((2, 2, 2))

    # The relative transform should be (1, 1, 1) if Base is at (1, 1, 1) and Sensor at (2, 2, 2)
    robot, _ = scene_to_robot(bpy.context)
    assert len(robot.sensors) == 1
    # Compare with Vector3
    from linkforge.linkforge_core.models import Vector3

    assert robot.sensors[0].origin.xyz == Vector3(1.0, 1.0, 1.0)


def test_manual_inertia_origin(clean_scene):
    """Verify manual inertia origin extraction."""
    o = bpy.data.objects.new("InertialLink", None)
    bpy.context.collection.objects.link(o)
    o.linkforge.is_robot_link = True
    o.linkforge.mass = 1.0
    o.linkforge.use_auto_inertia = False
    o.matrix_world = Matrix.Identity(4)
    o.linkforge.inertia_origin_xyz = (0.1, 0.2, 0.3)

    core = blender_link_to_core_with_origin(o)
    assert core.inertial.origin.xyz.x == pytest.approx(0.1)
    assert core.inertial.origin.xyz.y == pytest.approx(0.2)
    assert core.inertial.origin.xyz.z == pytest.approx(0.3)


def test_matrix_to_transform_none():
    """Hit edge case."""
    assert matrix_to_transform(None).xyz.x == 0.0


def test_detect_primitive_type_none():
    """Hit edge cases."""
    assert detect_primitive_type(None) is None

    # Object with no data
    o = bpy.data.objects.new("NoData", None)
    assert detect_primitive_type(o) is None


def test_get_object_geometry_none_and_fallback():
    """Hit edge cases."""
    geom, mat = get_object_geometry(None)
    assert geom is None

    # (Zero size fallback)
    o = bpy.data.objects.new("ZeroBox", bpy.data.meshes.new("ZeroMesh"))
    o.dimensions = (0, 0, 0)
    geom, mat = get_object_geometry(o, geometry_type="BOX")
    assert geom is None

    # (Unknown type)
    o.dimensions = (1, 1, 1)
    geom, mat = get_object_geometry(o, geometry_type="UNKNOWN")
    assert geom is None


def test_extract_mesh_triangles_null_mesh():
    """Hit edge case."""
    from unittest import mock

    # We create a complete mock of the object.
    o = mock.MagicMock()
    # Mock return of evaluated_get
    eval_mock = mock.MagicMock()
    eval_mock.to_mesh.return_value = None
    o.evaluated_get.return_value = eval_mock

    # We need to mock the context where it's used
    with mock.patch("linkforge.blender.adapters.blender_to_core.bpy") as mock_bpy:
        # The code calls: depsgraph = bpy.context.evaluated_depsgraph_get()
        # and then: eval_obj = obj.evaluated_get(depsgraph)
        mock_bpy.context.evaluated_depsgraph_get.return_value = mock.MagicMock()
        assert extract_mesh_triangles(o) is None
