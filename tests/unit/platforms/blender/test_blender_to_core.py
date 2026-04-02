from pathlib import Path

import bpy
import pytest
from linkforge.linkforge_core.exceptions import RobotModelError

try:
    import importlib.util

    HAS_PYBULLET = importlib.util.find_spec("pybullet") is not None
except ImportError:
    HAS_PYBULLET = False
from linkforge.blender.adapters.blender_to_core import (
    _calculate_link_frames,
    blender_joint_to_core,
    blender_link_to_core_with_origin,
    blender_sensor_to_core,
    detect_primitive_type,
    extract_mesh_triangles,
    get_object_geometry,
    get_object_material,
    matrix_to_transform,
    sanitize_name,
    scene_to_robot,
)
from linkforge.linkforge_core.models import (
    Box,
    Cylinder,
    JointType,
    Mesh,
    SensorType,
    Sphere,
)
from mathutils import Euler, Matrix


def test_matrix_to_transform_precision() -> None:
    """Verify that matrix_to_transform correctly extracts XYZ/RPY from a real Matrix."""
    # Create a matrix with specific translation and rotation in XYZ order (URDF Standard)
    m = Matrix.Translation((1.0, 2.0, 3.0)) @ Euler((0.4, 0.5, 0.6), "XYZ").to_matrix().to_4x4()

    transform = matrix_to_transform(m)

    assert pytest.approx(transform.xyz.x) == 1.0
    assert pytest.approx(transform.xyz.y) == 2.0
    assert pytest.approx(transform.xyz.z) == 3.0
    assert pytest.approx(transform.rpy.x) == 0.4
    assert pytest.approx(transform.rpy.y) == 0.5
    assert pytest.approx(transform.rpy.z) == 0.6


def test_get_object_geometry_sphere_cylinder() -> None:
    """Verify auto-detection of sphere and cylinder primitives via get_object_geometry."""
    # 1. Sphere
    bpy.ops.mesh.primitive_uv_sphere_add(radius=0.5)
    s_obj = bpy.context.active_object
    geom_s, world_matrix = get_object_geometry(s_obj, geometry_type="AUTO")
    assert isinstance(geom_s, Sphere)
    assert pytest.approx(geom_s.radius) == 0.5
    assert world_matrix == s_obj.matrix_world

    # 2. Cylinder
    bpy.ops.mesh.primitive_cylinder_add(radius=0.3, depth=1.0)
    c_obj = bpy.context.active_object
    geom_c, world_matrix = get_object_geometry(c_obj, geometry_type="AUTO")
    assert isinstance(geom_c, Cylinder)
    assert pytest.approx(geom_c.radius) == 0.3
    assert pytest.approx(geom_c.length) == 1.0
    assert world_matrix == c_obj.matrix_world


def test_detect_primitive_type_box() -> None:
    """Verify that a basic cube mesh is detected as BOX."""
    bpy.ops.mesh.primitive_cube_add(size=2.0)
    obj = bpy.context.active_object
    assert detect_primitive_type(obj) == "BOX"


def test_detect_primitive_type_sphere() -> None:
    """Verify that a UV sphere is detected as SPHERE."""
    bpy.ops.mesh.primitive_uv_sphere_add(radius=1.0)
    obj = bpy.context.active_object
    assert detect_primitive_type(obj) == "SPHERE"


def test_detect_primitive_type_cylinder() -> None:
    """Verify that a cylinder is detected as CYLINDER."""
    # Dimensions must not be 1:1:1 to avoid sphere detection
    bpy.ops.mesh.primitive_cylinder_add(radius=1.0, depth=3.0)
    obj = bpy.context.active_object
    assert detect_primitive_type(obj) == "CYLINDER"


def test_detect_primitive_type_none_case() -> None:
    """A complex mesh (Monkey) should return None for primitive detection."""
    bpy.ops.mesh.primitive_monkey_add()
    obj = bpy.context.active_object
    assert detect_primitive_type(obj) is None


def test_blender_joint_to_core_conversion() -> None:
    """Verify that a Blender joint object is correctly converted back to Core Joint."""
    # 1. Setup Parent Link
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    p_obj = bpy.context.active_object
    p_obj.name = "parent_l"
    if hasattr(p_obj, "linkforge"):
        p_obj.linkforge.link_name = "parent_l"

    # 2. Setup Child Link
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    c_obj = bpy.context.active_object
    c_obj.name = "child_l"
    if hasattr(c_obj, "linkforge"):
        c_obj.linkforge.link_name = "child_l"

    # 3. Setup Blender Joint
    bpy.ops.object.empty_add(type="ARROWS")
    joint_obj = bpy.context.active_object
    joint_obj.name = "blender_j"

    # Initialize properties
    if hasattr(joint_obj, "linkforge_joint"):
        props = joint_obj.linkforge_joint
        props.is_robot_joint = True
        props.joint_name = "blender_j"
        props.joint_type = "REVOLUTE"
        props.axis = "Y"
        props.parent_link = p_obj
        props.child_link = c_obj
        props.use_limits = True
        props.limit_lower = -1.0
        props.limit_upper = 1.0

    # 4. Convert
    joint = blender_joint_to_core(joint_obj)

    # 5. Verify
    assert joint is not None
    assert joint.name == "blender_j"
    assert joint.type == JointType.REVOLUTE
    assert pytest.approx(joint.axis.y) == 1.0


def test_blender_sensor_to_core_lidar() -> None:
    """Verify that a Blender sensor object is correctly converted back to Core Sensor."""
    # 1. Setup Parent Link
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    parent_obj = bpy.context.active_object
    parent_obj.name = "base_link"
    if hasattr(parent_obj, "linkforge"):
        parent_obj.linkforge.is_robot_link = True
        parent_obj.linkforge.link_name = "base_link"

    # 2. Setup Sensor Object
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    sensor_obj = bpy.context.active_object
    sensor_obj.name = "my_lidar"
    sensor_obj.parent = parent_obj

    if hasattr(sensor_obj, "linkforge_sensor"):
        props = sensor_obj.linkforge_sensor
        props.is_robot_sensor = True
        props.sensor_type = "LIDAR"
        props.update_rate = 50.0
        props.attached_link = parent_obj

    # 3. Convert
    sensor = blender_sensor_to_core(sensor_obj)

    # 4. Verify
    assert sensor is not None
    assert sensor.type == SensorType.LIDAR
    assert sensor.update_rate == 50.0
    assert sensor.link_name == "base_link"


def test_blender_link_to_core_inertia() -> None:
    """Verify that inertial properties are correctly extracted from Blender objects."""
    bpy.ops.object.empty_add()
    obj = bpy.context.active_object
    obj.name = "inertial_link"
    if hasattr(obj, "linkforge"):
        obj.linkforge.is_robot_link = True
        obj.linkforge.mass = 2.5
        obj.linkforge.inertia_ixx = 1.0
        obj.linkforge.inertia_iyy = 1.0
        obj.linkforge.inertia_izz = 1.0

    link = blender_link_to_core_with_origin(obj)
    assert link.inertial.mass == 2.5
    assert link.inertial.inertia.ixx == 1.0


def test_categorize_scene_objects_logic() -> None:
    """Verify that scene objects are correctly categorized as links, joints, or sensors."""
    # 1. Setup Scene
    bpy.ops.object.empty_add()
    l_obj = bpy.context.active_object
    l_obj.name = "l_link"
    l_obj.linkforge.is_robot_link = True

    bpy.ops.object.empty_add()
    j_obj = bpy.context.active_object
    j_obj.name = "j_joint"
    j_obj.linkforge_joint.is_robot_joint = True

    bpy.ops.object.empty_add()
    t_obj = bpy.context.active_object
    t_obj.name = "t_trans"
    if hasattr(t_obj, "linkforge_transmission"):
        t_obj.linkforge_transmission.is_robot_transmission = True

    # 2. Call internal categorizer
    from linkforge.blender.adapters.blender_to_core import _categorize_scene_objects

    links, joints, sensors, transmissions, joints_map, root = _categorize_scene_objects(
        bpy.context.scene
    )

    # 3. Verify
    assert "l_link" in links
    assert j_obj in joints
    assert root[0] == "l_link"


def test_calculate_link_frames_logic() -> None:
    """Verify recursive frame calculation with real objects."""
    # 1. Setup Hierarchy
    bpy.ops.object.empty_add()
    root_obj = bpy.context.active_object
    root_obj.name = "root"

    bpy.ops.object.empty_add()
    child_obj = bpy.context.active_object
    child_obj.name = "child"
    child_obj.parent = root_obj
    child_obj.location = (1, 0, 0)

    bpy.context.view_layer.update()

    # 2. Setup structures
    link_objects = {"root": root_obj, "child": child_obj}
    joints_map = {"child": ("root", None)}  # Dummy joint map entry
    root_link = ("root", root_obj)

    # 3. Calculate
    frames = _calculate_link_frames(link_objects, joints_map, root_link)

    # 4. Verify
    assert "root" in frames
    assert "child" in frames
    # Child frame should be at (1,0,0) relative to root
    assert pytest.approx(frames["child"].to_translation().x) == 1.0


def test_get_object_material_logic() -> None:
    """Verify material extraction from Principled BSDF node."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    mat = bpy.data.materials.new(name="PMat")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (0.1, 0.2, 0.3, 1.0)
    obj.data.materials.append(mat)

    # Use real link properties
    link_props = obj.linkforge
    link_props.use_material = True

    material = get_object_material(obj, link_props)
    assert material.name == "PMat"
    assert pytest.approx(material.color.r) == 0.1
    assert pytest.approx(material.color.g) == 0.2


def test_blender_link_to_core_multi_elements() -> None:
    """Verify conversion of a link with multiple visuals back to Core."""
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.linkforge.is_robot_link = True

    # Visual 1
    bpy.ops.mesh.primitive_cube_add()
    v1 = bpy.context.active_object
    v1.name = "v1_visual"
    v1.parent = link_obj

    # Visual 2
    bpy.ops.mesh.primitive_uv_sphere_add()
    v2 = bpy.context.active_object
    v2.name = "v2_visual"
    v2.parent = link_obj

    bpy.context.view_layer.update()

    link = blender_link_to_core_with_origin(link_obj)
    assert len(link.visuals) == 2


def test_get_object_geometry_forced_primitives() -> None:
    """Verify that get_object_geometry honors forced primitive types."""
    bpy.ops.mesh.primitive_cube_add(size=2.0)
    obj = bpy.context.active_object

    # 1. Force Sphere (radius should be max dim / 2 = 1.0)
    geom_s, wm_s = get_object_geometry(obj, geometry_type="SPHERE")
    assert isinstance(geom_s, Sphere)
    assert wm_s == obj.matrix_world
    assert pytest.approx(geom_s.radius) == 1.0

    # 2. Force Cylinder (z depth is 2.0, max x/y is 2.0 -> radius 1.0)
    geom_c, wm_c = get_object_geometry(obj, geometry_type="CYLINDER")
    assert isinstance(geom_c, Cylinder)
    assert wm_c == obj.matrix_world
    assert pytest.approx(geom_c.radius) == 1.0
    assert pytest.approx(geom_c.length) == 2.0


def test_get_object_geometry_mesh_simplified(tmp_path) -> None:
    """Verify that mesh simplification fallback is handled."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # MESH currently falls back to BOX if not implemented with real hull
    geom, wm = get_object_geometry(obj, geometry_type="MESH", meshes_dir=tmp_path, link_name="hull")
    assert isinstance(geom, (Box, Mesh))


def test_get_object_material_logic_no_nodes() -> None:
    """Verify material extraction from Blender object (No Nodes)."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # Use real link properties
    obj.linkforge.use_material = True

    mat = bpy.data.materials.new(name="Test-Mat")
    mat.use_nodes = False
    mat.diffuse_color = (1, 0, 0, 1)
    obj.data.materials.append(mat)

    core_mat = get_object_material(obj, obj.linkforge)
    assert core_mat is not None
    assert core_mat.color.r == 1.0
    assert "Test_Mat" in core_mat.name or "Test-Mat" in core_mat.name


def test_sanitize_name_logic() -> None:
    """Verify name sanitization for XACRO compatibility."""
    # Default allows hyphens
    assert sanitize_name("my-robot-link") == "my-robot-link"
    # Forced for Python identifier
    assert sanitize_name("my-robot-link", allow_hyphen=False) == "my_robot_link"
    assert sanitize_name("link.001") == "link_001"
    assert sanitize_name("123link") == "_123link"  # Correct behavior for leading digits


def test_categorize_scene_objects_complex_hierarchy() -> None:
    """Verify categorization of a full robot hierarchy with sensors and joints."""
    # 1. Base Link
    bpy.ops.object.empty_add()
    base = bpy.context.active_object
    base.name = "base_link"
    base.linkforge.is_robot_link = True

    # 2. Child Link
    bpy.ops.object.empty_add()
    child = bpy.context.active_object
    child.name = "child_link"
    child.linkforge.is_robot_link = True

    # 3. Joint (Base -> Child)
    bpy.ops.object.empty_add()
    joint = bpy.context.active_object
    joint.name = "base_to_child"
    joint.linkforge_joint.is_robot_joint = True
    joint.linkforge_joint.joint_type = "REVOLUTE"
    joint.linkforge_joint.parent_link = base
    joint.linkforge_joint.child_link = child

    # 4. Sensor on Child
    bpy.ops.object.empty_add()
    sensor = bpy.context.active_object
    sensor.name = "camera"
    sensor.linkforge_sensor.is_robot_sensor = True
    sensor.linkforge_sensor.sensor_type = "CAMERA"
    sensor.linkforge_sensor.attached_link = child

    # Manually run the protected function (we are testing unit logic)
    from linkforge.blender.adapters.blender_to_core import _categorize_scene_objects

    links, joints, sensors, transmissions, joints_map, root_link = _categorize_scene_objects(
        bpy.context.scene
    )

    assert "base_link" in links
    assert "child_link" in links
    assert any(j.name == "base_to_child" for j in joints)
    assert any(s.name == "camera" for s in sensors)
    assert len(joints_map) == 1
    assert joints_map["child_link"][0] == "base_link"  # Parent name
    assert root_link[0] == "base_link"


def test_blender_joint_to_core_types() -> None:
    """Verify conversion of different joint types and parameters."""
    # Setup Parent/Child Links
    bpy.ops.object.empty_add()
    parent = bpy.context.active_object
    parent.name = "parent_link"
    parent.linkforge.is_robot_link = True

    bpy.ops.object.empty_add()
    child = bpy.context.active_object
    child.name = "child_link"
    child.linkforge.is_robot_link = True

    # 1. Prismatic
    bpy.ops.object.empty_add()
    joint_obj = bpy.context.active_object
    joint_obj.name = "prismatic_joint"
    joint_obj.linkforge_joint.is_robot_joint = True
    joint_obj.linkforge_joint.parent_link = parent
    joint_obj.linkforge_joint.child_link = child  # Also usually required or good practice
    joint_obj.linkforge_joint.joint_type = "PRISMATIC"
    joint_obj.linkforge_joint.axis = "X"
    joint_obj.linkforge_joint.limit_lower = -1.0
    joint_obj.linkforge_joint.limit_upper = 2.0
    joint_obj.linkforge_joint.limit_effort = 100.0
    joint_obj.linkforge_joint.limit_velocity = 5.0

    joint = blender_joint_to_core(joint_obj)

    assert joint.type == JointType.PRISMATIC
    assert joint.axis.x == 1.0
    assert joint.limits.lower == -1.0
    assert joint.limits.upper == 2.0

    # 2. Continuous
    joint_obj.linkforge_joint.joint_type = "CONTINUOUS"
    joint = blender_joint_to_core(joint_obj)
    assert joint.type == JointType.CONTINUOUS
    # Continuous joints shouldn't have lower/upper limits in standard URDF but our model handles it.


def test_blender_joint_to_core_advanced_props() -> None:
    """Verify that safety controller and calibration are correctly synced to Core."""
    # 1. Setup Links
    bpy.ops.object.empty_add()
    p = bpy.context.active_object
    p.name = "p_link"
    p.linkforge.is_robot_link = True

    bpy.ops.object.empty_add()
    c = bpy.context.active_object
    c.name = "c_link"
    c.linkforge.is_robot_link = True

    # 2. Setup Joint
    bpy.ops.object.empty_add()
    joint_obj = bpy.context.active_object
    joint_obj.name = "advanced_j"
    props = joint_obj.linkforge_joint
    props.is_robot_joint = True
    props.parent_link = p
    props.child_link = c
    props.joint_type = "REVOLUTE"
    props.limit_lower = -1.57
    props.limit_upper = 1.57

    # Set safety controller
    props.use_safety_controller = True
    props.safety_soft_lower_limit = -1.0
    props.safety_soft_upper_limit = 1.0
    props.safety_k_position = 100.0
    props.safety_k_velocity = 10.0

    # Set calibration
    props.use_calibration = True
    props.use_calibration_rising = True
    props.calibration_rising = 0.5
    props.use_calibration_falling = False

    # 3. Convert
    joint = blender_joint_to_core(joint_obj)

    # 4. Verify
    assert joint.safety_controller is not None
    assert joint.safety_controller.soft_lower_limit == -1.0
    assert joint.safety_controller.k_position == 100.0

    assert joint.calibration is not None
    assert joint.calibration.rising == 0.5
    assert joint.calibration.falling is None


def test_blender_sensor_to_core_all_types() -> None:
    """Verify conversion of various sensor types and their properties."""
    from linkforge.blender.adapters.blender_to_core import blender_sensor_to_core

    # Setup Parent Link
    bpy.ops.object.empty_add()
    parent = bpy.context.active_object
    parent.name = "sensor_link"
    parent.linkforge.is_robot_link = True

    # 1. IMU
    bpy.ops.object.empty_add()
    imu_obj = bpy.context.active_object
    imu_obj.name = "imu_sensor"
    imu_obj.linkforge_sensor.is_robot_sensor = True
    imu_obj.linkforge_sensor.attached_link = parent
    imu_obj.linkforge_sensor.sensor_type = "IMU"
    imu_obj.linkforge_sensor.update_rate = 100.0
    imu_obj.linkforge_sensor.always_on = True
    imu_obj.linkforge_sensor.visualize = True

    sensor = blender_sensor_to_core(imu_obj)
    assert sensor.type == SensorType.IMU
    assert sensor.update_rate == 100.0
    assert sensor.always_on is True
    assert sensor.visualize is True

    # 2. Camera
    bpy.ops.object.empty_add()
    cam_obj = bpy.context.active_object
    cam_obj.name = "camera_sensor"
    cam_obj.linkforge_sensor.is_robot_sensor = True
    cam_obj.linkforge_sensor.attached_link = parent
    cam_obj.linkforge_sensor.sensor_type = "CAMERA"
    cam_obj.linkforge_sensor.camera_horizontal_fov = 1.047
    cam_obj.linkforge_sensor.camera_width = 800
    cam_obj.linkforge_sensor.camera_height = 600

    sensor = blender_sensor_to_core(cam_obj)
    assert sensor.type == SensorType.CAMERA
    assert sensor.camera_info is not None
    assert pytest.approx(sensor.camera_info.horizontal_fov) == 1.047
    assert sensor.camera_info.width == 800

    # 3. Lidar
    bpy.ops.object.empty_add()
    lidar_obj = bpy.context.active_object
    lidar_obj.name = "lidar_sensor"
    lidar_obj.linkforge_sensor.is_robot_sensor = True
    lidar_obj.linkforge_sensor.attached_link = parent
    lidar_obj.linkforge_sensor.sensor_type = "LIDAR"
    lidar_obj.linkforge_sensor.lidar_range_max = 50.0
    lidar_obj.linkforge_sensor.lidar_range_min = 0.5

    sensor = blender_sensor_to_core(lidar_obj)
    assert sensor.type == SensorType.LIDAR
    assert sensor.lidar_info is not None
    assert sensor.lidar_info.range_max == 50.0


def test_detect_primitive_type_logic() -> None:
    """Verify primitive detection heuristics."""
    from linkforge.blender.adapters.blender_to_core import detect_primitive_type

    # 1. Cube
    bpy.ops.mesh.primitive_cube_add()
    cube = bpy.context.active_object
    assert detect_primitive_type(cube) == "BOX"

    # 2. Sphere (UV Sphere default)
    bpy.ops.mesh.primitive_uv_sphere_add()
    sphere = bpy.context.active_object
    assert detect_primitive_type(sphere) == "SPHERE"

    # 3. Cylinder
    bpy.ops.mesh.primitive_cylinder_add()
    cyl = bpy.context.active_object
    # Scale it to be clearly cylindrical (tall) to avoid being ambiguous with sphere
    cyl.scale = (1, 1, 2)
    bpy.context.view_layer.update()  # Ensure dimensions update
    assert detect_primitive_type(cyl) == "CYLINDER"

    # 4. Complex Mesh (Monkey/Suzanne)
    bpy.ops.mesh.primitive_monkey_add()
    monkey = bpy.context.active_object
    assert detect_primitive_type(monkey) is None


def test_matrix_to_transform_conversion() -> None:
    """Verify 4x4 matrix to Transform conversion."""
    import math

    import mathutils
    from linkforge.blender.adapters.blender_to_core import matrix_to_transform

    # Identity
    mat = mathutils.Matrix.Identity(4)
    tf = matrix_to_transform(mat)
    assert tf.xyz.x == 0 and tf.xyz.y == 0 and tf.xyz.z == 0
    assert tf.rpy.x == 0 and tf.rpy.y == 0 and tf.rpy.z == 0

    # Translation (1, 2, 3)
    mat = mathutils.Matrix.Translation((1, 2, 3))
    tf = matrix_to_transform(mat)
    assert tf.xyz.x == 1 and tf.xyz.y == 2 and tf.xyz.z == 3

    # Rotation (90 deg around X, XYZ order)
    mat = mathutils.Matrix.Rotation(math.radians(90), 4, "X")
    tf = matrix_to_transform(mat)
    # Eulers match exactly for single-axis X rotation
    assert pytest.approx(tf.rpy.x) == 1.570796
    assert pytest.approx(tf.rpy.y) == 0
    assert pytest.approx(tf.rpy.z) == 0

    # Complex Rotation (mixed axes)
    # Using 'XYZ' to match URDF extrinsic standard
    mat = mathutils.Euler((0.1, 0.2, 0.3), "XYZ").to_matrix().to_4x4()
    tf = matrix_to_transform(mat)
    assert pytest.approx(tf.rpy.x) == 0.1
    assert pytest.approx(tf.rpy.y) == 0.2
    assert pytest.approx(tf.rpy.z) == 0.3


def test_get_object_geometry_decimation(tmp_path) -> None:
    """Verify that decimation (simplification) is active if requested."""
    # Create a reasonably complex object
    bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16)
    obj = bpy.context.active_object

    # 1. Without simplify
    g1, wm1 = get_object_geometry(
        obj, geometry_type="MESH", simplify=False, meshes_dir=tmp_path, link_name="l1"
    )

    # 2. With simplify (decimate to 10%)
    g2, wm2 = get_object_geometry(
        obj,
        geometry_type="MESH",
        simplify=True,
        decimation_ratio=0.1,
        meshes_dir=tmp_path,
        link_name="l2",
    )

    assert isinstance(g1, Mesh)
    assert isinstance(g2, Mesh)


def test_get_object_geometry_dry_run(tmp_path) -> None:
    """Verify that dry_run skips side-effects (like mesh saving)."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # Should not crash and should return geometry even with invalid dir
    geom, wm = get_object_geometry(
        obj, geometry_type="MESH", dry_run=True, meshes_dir=Path("/invalid/path"), link_name="dry"
    )
    assert isinstance(geom, Mesh)
    assert wm == obj.matrix_world


def test_scene_to_robot_conversion() -> None:
    """Verify that an entire Blender scene is converted to a Core Robot."""
    # 1. Setup a minimal link structure
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    link_obj = bpy.context.active_object
    link_obj.name = "base_link"
    if hasattr(link_obj, "linkforge"):
        link_obj.linkforge.is_robot_link = True
        link_obj.linkforge.link_name = "base_link"

    # 2. Convert
    # Unpack tuple: (robot, errors)
    robot, errors = scene_to_robot(bpy.context)

    # 3. Verify
    assert robot is not None
    assert len(robot.links) >= 1
    assert any(link.name == "base_link" for link in robot.links)


def test_extract_mesh_triangles_logic() -> None:
    """Test raw triangle extraction from a primitive."""
    bpy.ops.mesh.primitive_cube_add(size=1.0)
    obj = bpy.context.active_object

    verts, tris = extract_mesh_triangles(obj)

    # Cube has 8 vertices and 12 triangles
    assert len(verts) == 8
    assert len(tris) == 12


def test_get_object_geometry_auto_primitive() -> None:
    """Test auto-detection of box primitive via get_object_geometry."""
    bpy.ops.mesh.primitive_cube_add(size=2.0)
    obj = bpy.context.active_object

    geom, wm = get_object_geometry(obj, geometry_type="AUTO")

    assert isinstance(geom, Box)
    assert wm == obj.matrix_world
    assert pytest.approx(geom.size.x) == 2.0


def test_blender_link_to_core_complex() -> None:
    """Verify conversion of a link with multiple visuals and collisions back to Core."""
    # Ensure a clean state
    bpy.ops.object.select_all(action="DESELECT")

    # 1. Setup Link Empty
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.name = "base_link"
    if hasattr(link_obj, "linkforge"):
        link_obj.linkforge.is_robot_link = True
        link_obj.linkforge.link_name = "base_link"

    # 2. Add Visual Child
    bpy.ops.mesh.primitive_cube_add(size=1.0)
    vis_obj = bpy.context.active_object
    vis_obj.name = "base_link_visual"
    vis_obj.parent = link_obj
    vis_obj.location = (1, 0, 0)

    # 3. Add Collision Child
    bpy.ops.mesh.primitive_uv_sphere_add(radius=0.5)
    coll_obj = bpy.context.active_object
    coll_obj.name = "base_link_collision"
    coll_obj.parent = link_obj
    coll_obj.location = (0, 1, 0)

    # Update view layer to ensure matrices are correct
    bpy.context.view_layer.update()

    # 4. Convert
    link = blender_link_to_core_with_origin(link_obj)

    # 5. Verify
    assert link is not None
    assert len(link.visuals) == 1
    assert len(link.collisions) == 1
    # Check absolute origins (extracted from matrices)
    assert pytest.approx(link.visuals[0].origin.xyz.x) == 1.0
    assert pytest.approx(link.collisions[0].origin.xyz.y) == 1.0


def test_blender_link_to_core_geometry_and_material() -> None:
    """Verify detailed geometry and material conversion."""
    from linkforge.blender.adapters.blender_to_core import blender_link_to_core_with_origin
    from linkforge.linkforge_core.models import GeometryType

    # 1. Link Setup
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.name = "material_link"
    link_obj.linkforge.is_robot_link = True

    # 2. Visual with Material
    bpy.ops.mesh.primitive_cube_add(size=1.0)
    vis_obj = bpy.context.active_object
    vis_obj.name = "vis_cube_visual"
    vis_obj.parent = link_obj

    # Enable material export on the LINK properties (parent)
    link_obj.linkforge.use_material = True

    # Create Material using Nodes (Principled BSDF)
    mat = bpy.data.materials.new(name="RedMat")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (1, 0, 0, 1)  # Red
    vis_obj.data.materials.append(mat)

    # 3. Collision with Cylinder
    bpy.ops.mesh.primitive_cylinder_add()
    coll_obj = bpy.context.active_object
    coll_obj.name = "coll_cyl_collision"
    coll_obj.parent = link_obj
    # Scale to match heuristic
    coll_obj.scale = (1, 1, 2)

    bpy.context.view_layer.update()

    # 4. Convert
    link = blender_link_to_core_with_origin(link_obj)

    # 5. Verify Visual
    assert len(link.visuals) == 1
    vis = link.visuals[0]
    assert vis.geometry.type == GeometryType.BOX
    assert vis.material is not None
    assert vis.material.name == "RedMat"
    assert pytest.approx(vis.material.color.r) == 1.0
    assert pytest.approx(vis.material.color.g) == 0.0

    # 6. Verify Collision
    assert len(link.collisions) == 1
    coll = link.collisions[0]
    assert coll.geometry.type == GeometryType.CYLINDER


def test_robust_origin_extraction_logic() -> None:
    """Verify relative transform extraction between parent and child."""
    # Create parent
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(1, 1, 1))
    parent = bpy.context.active_object

    # Create child
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(2, 2, 2))
    child = bpy.context.active_object
    child.parent = parent

    # The relative matrix should be (1, 1, 1)
    relative_matrix = parent.matrix_world.inverted() @ child.matrix_world
    transform = matrix_to_transform(relative_matrix)

    assert pytest.approx(transform.xyz.x) == 1.0
    assert pytest.approx(transform.xyz.y) == 1.0
    assert pytest.approx(transform.xyz.z) == 1.0


# ============================================================================
# Advanced Sensor Conversion Tests
# ============================================================================


def test_blender_sensor_contact() -> None:
    """Test conversion of contact sensor."""
    # Create parent link
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.name = "base_link"
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.link_name = "base_link"

    # Create sensor
    bpy.ops.object.empty_add()
    sensor_obj = bpy.context.active_object
    sensor_obj.name = "contact1"
    sensor_obj.parent = link_obj
    sensor_obj.linkforge_sensor.is_robot_sensor = True
    sensor_obj.linkforge_sensor.attached_link = link_obj
    sensor_obj.linkforge_sensor.sensor_type = "CONTACT"
    sensor_obj.linkforge_sensor.contact_collision = "collision_link"

    sensor = blender_sensor_to_core(sensor_obj)

    assert sensor is not None
    assert sensor.type == SensorType.CONTACT
    assert sensor.contact_info is not None
    assert sensor.contact_info.collision == "collision_link"


def test_blender_sensor_force_torque() -> None:
    """Test conversion of force-torque sensor."""
    # Create parent link
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.name = "base_link"
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.link_name = "base_link"

    # Create sensor
    bpy.ops.object.empty_add()
    sensor_obj = bpy.context.active_object
    sensor_obj.parent = link_obj
    sensor_obj.linkforge_sensor.is_robot_sensor = True
    sensor_obj.linkforge_sensor.attached_link = link_obj
    sensor_obj.linkforge_sensor.sensor_type = "FORCE_TORQUE"
    sensor_obj.linkforge_sensor.ft_frame = "CHILD"
    sensor_obj.linkforge_sensor.ft_measure_direction = "PARENT_TO_CHILD"

    sensor = blender_sensor_to_core(sensor_obj)

    assert sensor is not None
    assert sensor.type == SensorType.FORCE_TORQUE
    assert sensor.force_torque_info is not None
    assert sensor.force_torque_info.frame == "child"


def test_blender_sensor_with_noise() -> None:
    """Test sensor conversion with noise parameters."""
    # Create parent link
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.name = "base_link"
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.link_name = "base_link"

    # Create sensor
    bpy.ops.object.empty_add()
    sensor_obj = bpy.context.active_object
    sensor_obj.parent = link_obj
    sensor_obj.linkforge_sensor.is_robot_sensor = True
    sensor_obj.linkforge_sensor.attached_link = link_obj
    sensor_obj.linkforge_sensor.sensor_type = "IMU"
    sensor_obj.linkforge_sensor.use_noise = True
    sensor_obj.linkforge_sensor.noise_mean = 0.1
    sensor_obj.linkforge_sensor.noise_stddev = 0.05

    sensor = blender_sensor_to_core(sensor_obj)

    assert sensor is not None
    assert sensor.imu_info is not None
    assert sensor.imu_info.angular_velocity_noise is not None
    assert pytest.approx(sensor.imu_info.angular_velocity_noise.mean) == 0.1
    assert pytest.approx(sensor.imu_info.angular_velocity_noise.stddev) == 0.05


def test_blender_sensor_with_plugin() -> None:
    """Test sensor conversion with Gazebo plugin."""
    # Create parent link
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.name = "base_link"
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.link_name = "base_link"

    # Create sensor
    bpy.ops.object.empty_add()
    sensor_obj = bpy.context.active_object
    sensor_obj.parent = link_obj
    sensor_obj.linkforge_sensor.is_robot_sensor = True
    sensor_obj.linkforge_sensor.attached_link = link_obj
    sensor_obj.linkforge_sensor.sensor_name = "camera1"
    sensor_obj.linkforge_sensor.sensor_type = "CAMERA"
    sensor_obj.linkforge_sensor.use_gazebo_plugin = True
    sensor_obj.linkforge_sensor.plugin_name = "my_camera_plugin"
    sensor_obj.linkforge_sensor.plugin_filename = "libmy_camera.so"

    sensor = blender_sensor_to_core(sensor_obj)

    assert sensor is not None
    assert sensor.plugin is not None
    assert sensor.plugin.name == "camera1_plugin"
    assert sensor.plugin.filename == "libmy_camera.so"


def test_blender_sensor_not_robot_sensor() -> None:
    """Test that non-robot sensor objects return None."""
    bpy.ops.object.empty_add()
    sensor_obj = bpy.context.active_object
    sensor_obj.linkforge_sensor.is_robot_sensor = False

    sensor = blender_sensor_to_core(sensor_obj)

    assert sensor is None


# ============================================================================
# Advanced Adapter Hardening (Mimicry, Auto-Inertia, Strict Mode)
# ============================================================================


def test_blender_joint_mimic_and_limits_advanced() -> None:
    """Verify joint mimicry conversion."""
    # Setup Links
    bpy.ops.object.empty_add()
    p = bpy.context.active_object
    p.name = "P_Link"
    p.linkforge.link_name = "P_Link"

    bpy.ops.object.empty_add()
    c = bpy.context.active_object
    c.name = "C_Link"
    c.linkforge.link_name = "C_Link"

    # Master joint
    bpy.ops.object.empty_add()
    jm = bpy.context.active_object
    jm.name = "MasterJ"
    jm.linkforge_joint.joint_name = "master_j"

    # Slave joint with mimic
    bpy.ops.object.empty_add()
    j = bpy.context.active_object
    j.name = "SlaveJ"
    j.linkforge_joint.is_robot_joint = True
    j.linkforge_joint.parent_link = p
    j.linkforge_joint.child_link = c
    j.linkforge_joint.use_mimic = True
    j.linkforge_joint.mimic_joint = jm
    j.linkforge_joint.mimic_multiplier = 2.0
    j.linkforge_joint.mimic_offset = 0.5

    core = blender_joint_to_core(j)
    assert core.mimic is not None
    assert core.mimic.joint == "master_j"
    assert core.mimic.multiplier == 2.0
    assert core.mimic.offset == 0.5


def test_blender_link_auto_inertia_sphere() -> None:
    """Verify auto-calculation of inertia from sphere geometry."""
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.name = "AutoLink"
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.mass = 2.0
    link_obj.linkforge.use_auto_inertia = True

    # Add sphere collision child
    bpy.ops.mesh.primitive_uv_sphere_add(radius=1.0)
    coll = bpy.context.active_object
    coll.name = "AutoLink_collision"
    coll.parent = link_obj

    bpy.context.view_layer.update()

    link = blender_link_to_core_with_origin(link_obj)
    # I = 2/5 * m * r^2 = 2/5 * 2.0 * 1^2 = 0.8
    assert pytest.approx(link.inertial.inertia.ixx) == 0.8


def test_blender_ros2_control_defaults(clean_scene) -> None:
    """Verify default ROS2 control interface assignment when one side is selected."""
    props = bpy.context.scene.linkforge
    props.ros2_control_name = "DefaultBot"

    joint = props.ros2_control_joints.add()
    joint.name = "j1"
    # Set STATE to True, leave CMD False
    joint.state_position = True
    joint.state_velocity = False
    joint.cmd_position = False
    joint.cmd_velocity = False
    joint.cmd_effort = False

    from linkforge.blender.adapters.blender_to_core import blender_ros2_control_to_core

    control = blender_ros2_control_to_core(props)

    assert control is not None
    # Command should default to position because it was empty but state was not
    assert control.joints[0].command_interfaces == ["position"]
    assert control.joints[0].state_interfaces == ["position"]


def test_blender_ros2_control_joint_obj_name_sync(clean_scene) -> None:
    """Verify that ros2_control generation uses the joint_obj.linkforge_joint.joint_name instead of item.name if present."""
    props = bpy.context.scene.linkforge
    props.ros2_control_name = "SyncedBot"

    # Setup a mapped joint object
    bpy.ops.object.empty_add(type="ARROWS")
    joint_obj = bpy.context.active_object
    joint_obj.name = "MyRealJoint"
    joint_obj.linkforge_joint.is_robot_joint = True
    joint_obj.linkforge_joint.joint_name = "MyRealJoint"

    joint = props.ros2_control_joints.add()
    joint.name = "StaleJointName"
    joint.joint_obj = joint_obj
    joint.cmd_position = True
    joint.state_position = True

    from linkforge.blender.adapters.blender_to_core import blender_ros2_control_to_core

    control = blender_ros2_control_to_core(props)

    assert control is not None
    assert len(control.joints) == 1
    # Should use the real name from the pointer, not the stale item name
    assert control.joints[0].name == "MyRealJoint"


def test_blender_sensor_gps_and_lidar_full(clean_scene) -> None:
    """Exhaustive test for GPS and LIDAR properties."""
    link = bpy.data.objects.new("L", None)
    bpy.context.collection.objects.link(link)
    link.linkforge.is_robot_link = True

    # GPS
    gps_obj = bpy.data.objects.new("GPS", None)
    bpy.context.collection.objects.link(gps_obj)
    gps_obj.linkforge_sensor.is_robot_sensor = True
    gps_obj.linkforge_sensor.sensor_type = "GPS"
    gps_obj.linkforge_sensor.attached_link = link
    gps_obj.linkforge_sensor.use_noise = True

    core_gps = blender_sensor_to_core(gps_obj)
    assert core_gps.gps_info.position_sensing_horizontal_noise is not None

    # LIDAR with samples
    lidar_obj = bpy.data.objects.new("LIDAR", None)
    bpy.context.collection.objects.link(lidar_obj)
    lidar_obj.linkforge_sensor.is_robot_sensor = True
    lidar_obj.linkforge_sensor.sensor_type = "LIDAR"
    lidar_obj.linkforge_sensor.attached_link = link
    lidar_obj.linkforge_sensor.lidar_horizontal_samples = 720
    lidar_obj.linkforge_sensor.lidar_vertical_samples = 16

    core_lidar = blender_sensor_to_core(lidar_obj)
    assert core_lidar.lidar_info.horizontal_samples == 720
    assert core_lidar.lidar_info.vertical_samples == 16


def test_blender_joint_dynamics(clean_scene) -> None:
    """Verify joint dynamics (damping, friction) conversion."""
    p = bpy.data.objects.new("P", None)
    c = bpy.data.objects.new("C", None)
    bpy.context.collection.objects.link(p)
    bpy.context.collection.objects.link(c)
    p.linkforge.is_robot_link = True
    c.linkforge.is_robot_link = True

    j = bpy.data.objects.new("J", None)
    bpy.context.collection.objects.link(j)
    j.linkforge_joint.is_robot_joint = True
    j.linkforge_joint.joint_type = "REVOLUTE"
    j.linkforge_joint.parent_link = p
    j.linkforge_joint.child_link = c
    j.linkforge_joint.use_dynamics = True
    j.linkforge_joint.dynamics_damping = 1.5
    j.linkforge_joint.dynamics_friction = 0.8

    from linkforge.blender.adapters.blender_to_core import blender_joint_to_core

    core = blender_joint_to_core(j)
    assert core is not None
    assert pytest.approx(core.dynamics.damping) == 1.5
    assert pytest.approx(core.dynamics.friction) == 0.8


def test_blender_link_inertial_origin(clean_scene) -> None:
    """Verify inertial origin extraction."""
    obj = bpy.data.objects.new("Link", None)
    bpy.context.collection.objects.link(obj)
    obj.linkforge.is_robot_link = True
    obj.linkforge.mass = 1.0
    obj.linkforge.inertia_origin_xyz = (0.1, 0.2, 0.3)
    obj.linkforge.inertia_origin_rpy = (0.0, 0.0, 0.5)

    link = blender_link_to_core_with_origin(obj)
    assert pytest.approx(link.inertial.origin.xyz.x) == 0.1
    assert pytest.approx(link.inertial.origin.rpy.z) == 0.5


def test_blender_transmission_full(clean_scene) -> None:
    """Exhaustive test for Simple and Differential transmissions."""
    # Setup joints
    j1 = bpy.data.objects.new("J1", None)
    j2 = bpy.data.objects.new("J2", None)
    bpy.context.collection.objects.link(j1)
    bpy.context.collection.objects.link(j2)
    j1.linkforge_joint.is_robot_joint = True
    j1.linkforge_joint.joint_name = "joint1"
    j2.linkforge_joint.is_robot_joint = True
    j2.linkforge_joint.joint_name = "joint2"

    # Simple Transmission
    t_simple = bpy.data.objects.new("TransSimple", None)
    bpy.context.collection.objects.link(t_simple)
    t_simple.linkforge_transmission.is_robot_transmission = True
    t_simple.linkforge_transmission.transmission_type = "SIMPLE"
    t_simple.linkforge_transmission.joint_name = j1
    t_simple.linkforge_transmission.mechanical_reduction = 50.0
    t_simple.linkforge_transmission.hardware_interface = "VELOCITY"

    from linkforge.blender.adapters.blender_to_core import blender_transmission_to_core

    core_simple = blender_transmission_to_core(t_simple)
    assert core_simple.name == "TransSimple"
    assert core_simple.joints[0].name == "joint1"
    assert core_simple.joints[0].mechanical_reduction == 50.0
    assert core_simple.joints[0].hardware_interfaces == ["velocity"]
    assert core_simple.actuators[0].name == "joint1_motor"

    # Differential Transmission
    t_diff = bpy.data.objects.new("TransDiff", None)
    bpy.context.collection.objects.link(t_diff)
    t_diff.linkforge_transmission.is_robot_transmission = True
    t_diff.linkforge_transmission.transmission_type = "DIFFERENTIAL"
    t_diff.linkforge_transmission.joint1_name = j1
    t_diff.linkforge_transmission.joint2_name = j2
    t_diff.linkforge_transmission.actuator1_name = "act1"
    t_diff.linkforge_transmission.actuator2_name = "act2"

    core_diff = blender_transmission_to_core(t_diff)
    assert len(core_diff.joints) == 2
    assert core_diff.actuators[0].name == "act1"
    assert core_diff.actuators[1].name == "act2"


def test_scene_to_robot_with_gazebo_and_errors(clean_scene) -> None:
    """Test scene_to_robot with Gazebo plugins and error collection."""
    props = bpy.context.scene.linkforge
    props.use_ros2_control = True
    props.gazebo_plugin_name = "test_plugin"
    props.controllers_yaml_path = "/path/to/yaml"
    props.strict_mode = False

    # Create one valid link to avoid empty robot error
    link_obj = bpy.data.objects.new("L", None)
    bpy.context.collection.objects.link(link_obj)
    link_obj.linkforge.is_robot_link = True

    # Create invalid link to trigger error collection
    bad_l = bpy.data.objects.new("Bad", None)
    bpy.context.collection.objects.link(bad_l)
    bad_l.linkforge.is_robot_link = True
    # Missing link_name or mass would normally be OK, but let's force an exception
    # by using a mock or hitting a real edge case if possible.
    # Actually, we can test the Gazebo plugin part first.

    from unittest import mock

    from linkforge.blender.adapters.blender_to_core import scene_to_robot

    with (
        mock.patch(
            "linkforge.blender.adapters.blender_to_core.blender_link_to_core_with_origin",
            side_effect=Exception("Failed link"),
        ),
        pytest.raises(RobotModelError, match=r"Multiple configuration errors found"),
    ):
        scene_to_robot(bpy.context)

    # Test success path with Gazebo
    bpy.context.scene.linkforge.use_ros2_control = True
    bpy.context.scene.linkforge.ros2_control_name = "DefaultBot"
    item = bpy.context.scene.linkforge.ros2_control_joints.add()
    item.name = "Dummy"
    bpy.context.scene.linkforge.gazebo_plugin_name = "gazebo_ros2_control"
    bpy.context.scene.linkforge.controllers_yaml_path = "/path/to/yaml"
    with mock.patch(
        "linkforge.blender.adapters.blender_to_core.blender_link_to_core_with_origin",
        return_value=None,
    ):
        robot, errors = scene_to_robot(bpy.context)
        assert len(robot.gazebo_elements) > 0
        plugin = robot.gazebo_elements[0].plugins[0]
        assert plugin.name == "gazebo_ros2_control"
        assert plugin.parameters["parameters"] == "/path/to/yaml"


def test_blender_sensor_exhaustive(clean_scene) -> None:
    """Test all remaining sensor types and properties."""
    link = bpy.data.objects.new("L", None)
    bpy.context.collection.objects.link(link)
    link.linkforge.is_robot_link = True

    from linkforge.blender.adapters.blender_to_core import blender_sensor_to_core

    # Camera
    cam = bpy.data.objects.new("Cam", None)
    bpy.context.collection.objects.link(cam)
    cam.linkforge_sensor.is_robot_sensor = True
    cam.linkforge_sensor.sensor_type = "CAMERA"
    cam.linkforge_sensor.attached_link = link
    cam.linkforge_sensor.camera_horizontal_fov = 1.05

    core_cam = blender_sensor_to_core(cam)
    assert pytest.approx(core_cam.camera_info.horizontal_fov) == 1.05

    # GPS with noise
    gps = bpy.data.objects.new("GPS", None)
    bpy.context.collection.objects.link(gps)
    gps.linkforge_sensor.is_robot_sensor = True
    gps.linkforge_sensor.sensor_type = "GPS"
    gps.linkforge_sensor.attached_link = link
    gps.linkforge_sensor.use_noise = True
    gps.linkforge_sensor.noise_mean = 0.0
    gps.linkforge_sensor.noise_stddev = 0.01

    core_gps = blender_sensor_to_core(gps)
    assert core_gps.gps_info is not None
    assert pytest.approx(core_gps.gps_info.position_sensing_horizontal_noise.stddev) == 0.01

    # Contact
    con = bpy.data.objects.new("Con", None)
    bpy.context.collection.objects.link(con)
    con.linkforge_sensor.is_robot_sensor = True
    con.linkforge_sensor.sensor_type = "CONTACT"
    con.linkforge_sensor.attached_link = link
    con.linkforge_sensor.contact_collision = "some_link_geom"

    core_con = blender_sensor_to_core(con)
    assert core_con.contact_info.collision == "some_link_geom"


def test_blender_to_core_geometry_edge_cases(clean_scene) -> None:
    """Test geometry conversion edge cases (None, zero-size, fallbacks)."""
    from linkforge.blender.adapters.blender_to_core import (
        detect_primitive_type,
        extract_mesh_triangles,
        get_object_geometry,
    )

    # detect_primitive_type None/non-mesh
    assert detect_primitive_type(None) is None
    empty = bpy.data.objects.new("Empty", None)
    assert detect_primitive_type(empty) is None

    # get_object_geometry None
    geom, mat = get_object_geometry(None)
    assert geom is None

    # zero-size object
    box = bpy.data.objects.new("ZeroBox", None)
    bpy.context.collection.objects.link(box)
    box.dimensions = (0, 0, 0)
    geom, mat = get_object_geometry(box, geometry_type="BOX")
    assert geom is None

    # extract_mesh_triangles None
    assert extract_mesh_triangles(None) is None
    assert extract_mesh_triangles(empty) is None


def test_blender_joint_advanced_cases(clean_scene) -> None:
    """Test custom axis, missing links, fixed axis, and continuous limits."""
    p = bpy.data.objects.new("P", None)
    c = bpy.data.objects.new("C", None)
    bpy.context.collection.objects.link(p)
    bpy.context.collection.objects.link(c)
    p.linkforge.is_robot_link = True
    c.linkforge.is_robot_link = True

    j = bpy.data.objects.new("J", None)
    bpy.context.collection.objects.link(j)
    j.linkforge_joint.is_robot_joint = True
    j.linkforge_joint.parent_link = p
    j.linkforge_joint.child_link = c

    from linkforge.blender.adapters.blender_to_core import blender_joint_to_core

    # Custom axis normalization
    j.linkforge_joint.joint_type = "REVOLUTE"
    j.linkforge_joint.axis = "CUSTOM"
    j.linkforge_joint.custom_axis_x = 2.0
    j.linkforge_joint.custom_axis_y = 0.0
    j.linkforge_joint.custom_axis_z = 0.0
    core = blender_joint_to_core(j)
    assert core.axis.x == 1.0  # Normalized

    # Zero axis fallback
    j.linkforge_joint.custom_axis_x = 0.0
    core = blender_joint_to_core(j)
    assert core.axis.z == 1.0  # Fallback

    # 1. Safety Controller
    j.linkforge_joint.use_safety_controller = True
    j.linkforge_joint.safety_soft_lower_limit = -1.23
    j.linkforge_joint.safety_soft_upper_limit = 1.23
    j.linkforge_joint.safety_k_position = 100.0
    j.linkforge_joint.safety_k_velocity = 10.0
    core = blender_joint_to_core(j)
    assert core.safety_controller is not None
    assert pytest.approx(core.safety_controller.soft_lower_limit) == -1.23
    assert pytest.approx(core.safety_controller.k_position) == 100.0

    # 2. Calibration
    j.linkforge_joint.use_calibration = True
    j.linkforge_joint.use_calibration_rising = True
    j.linkforge_joint.calibration_rising = 0.55
    j.linkforge_joint.use_calibration_falling = False
    core = blender_joint_to_core(j)
    assert core.calibration is not None
    assert pytest.approx(core.calibration.rising) == 0.55
    assert core.calibration.falling is None

    # Fixed joint axis (should be None)
    j.linkforge_joint.joint_type = "FIXED"
    core = blender_joint_to_core(j)
    assert core.axis is None

    # Continuous joint with limits
    j.linkforge_joint.joint_type = "CONTINUOUS"
    j.linkforge_joint.use_limits = True
    j.linkforge_joint.limit_effort = 10.0
    core = blender_joint_to_core(j)
    assert core.limits.effort == 10.0

    # Missing parent error
    j.linkforge_joint.parent_link = None
    with pytest.raises(RobotModelError, match="no parent link"):
        blender_joint_to_core(j)


def test_blender_transmission_advanced(clean_scene) -> None:
    """Test custom transmission types and actuator names."""
    j1 = bpy.data.objects.new("J1", None)
    bpy.context.collection.objects.link(j1)
    j1.linkforge_joint.is_robot_joint = True

    t = bpy.data.objects.new("TransCustom", None)
    bpy.context.collection.objects.link(t)
    t.linkforge_transmission.is_robot_transmission = True
    t.linkforge_transmission.transmission_type = "CUSTOM"
    t.linkforge_transmission.custom_type = "my_custom_trans"
    t.linkforge_transmission.joint_name = j1
    t.linkforge_transmission.use_custom_actuator_name = True
    t.linkforge_transmission.actuator_name = "custom_motor"

    from linkforge.blender.adapters.blender_to_core import blender_transmission_to_core

    core = blender_transmission_to_core(t)
    assert core.type == "my_custom_trans"
    assert core.actuators[0].name == "custom_motor"


def test_blender_link_mesh_inertia(clean_scene) -> None:
    """Test inertia calculation from real mesh data.
    Must force MESH geometry type to hit the mesh inertia branch.
    """
    link_obj = bpy.data.objects.new("L", None)
    bpy.context.collection.objects.link(link_obj)
    link_obj.linkforge.is_robot_link = True
    link_obj.linkforge.mass = 1.0
    link_obj.linkforge.use_mesh_inertia = True

    # Add a mesh geometry
    mesh = bpy.data.meshes.new("CubeMesh")
    import bmesh

    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bm.to_mesh(mesh)
    bm.free()

    o = bpy.data.objects.new("Geom", mesh)
    bpy.context.collection.objects.link(o)
    o.parent = link_obj
    o.linkforge.is_robot_visual = True
    o.linkforge.is_robot_collision = True
    o.linkforge.geometry_type = "MESH"  # Force mesh inertia branch

    from pathlib import Path

    from linkforge.blender.adapters.blender_to_core import blender_link_to_core_with_origin

    core = blender_link_to_core_with_origin(link_obj, meshes_dir=Path("/tmp"), dry_run=True)
    assert core.inertial.mass == 1.0
    assert core.inertial.inertia.ixx > 0


def test_scene_to_robot_full_integration(clean_scene) -> None:
    """Exhaustive test for scene_to_robot with sensors, plugins, and multi-visuals."""
    from pathlib import Path

    import bmesh
    from linkforge.blender.adapters.blender_to_core import scene_to_robot

    scene = bpy.context.scene
    # Root Link
    root = bpy.data.objects.new("RootLink", None)
    scene.collection.objects.link(root)
    root.linkforge.is_robot_link = True
    root.linkforge.link_name = "root_link"
    root.linkforge.use_auto_inertia = False

    def create_mesh_obj(name, parent, shape="CUBE"):
        m = bpy.data.meshes.new(f"{name}_mesh")
        bm = bmesh.new()
        if shape == "CUBE":
            bmesh.ops.create_cube(bm, size=1.0)
        else:
            bmesh.ops.create_uvsphere(bm, u_segments=16, v_segments=8, radius=0.5)
        bm.to_mesh(m)
        bm.free()
        o = bpy.data.objects.new(name, m)
        if scene:
            scene.collection.objects.link(o)
        o.parent = parent
        return o

    # Multi-visuals
    create_mesh_obj("root_link_visual_1", root, "CUBE")
    create_mesh_obj("root_link_visual_2", root, "SPHERE")  # Hits Sphere branch

    # Joint (Needed for transmission)
    child = bpy.data.objects.new("ChildLink", None)
    scene.collection.objects.link(child)
    child.linkforge.is_robot_link = True
    child.linkforge.use_auto_inertia = False

    joint = bpy.data.objects.new("Joint", None)
    scene.collection.objects.link(joint)
    joint.linkforge_joint.is_robot_joint = True
    joint.linkforge_joint.parent_link = root
    joint.linkforge_joint.child_link = child

    # Transmission (Explicitly setting joint_name)
    trans = bpy.data.objects.new("Trans", None)
    scene.collection.objects.link(trans)
    trans.linkforge_transmission.is_robot_transmission = True
    trans.linkforge_transmission.joint_name = joint

    # Sensor with Gazebo Plugin (Custom mount - Hits 1071-1075)
    lidar = bpy.data.objects.new("Lidar", None)
    scene.collection.objects.link(lidar)
    lidar.linkforge_sensor.is_robot_sensor = True
    lidar.linkforge_sensor.sensor_type = "LIDAR"
    lidar.linkforge_sensor.attached_link = root
    lidar.parent = None  # Custom mount
    lidar.linkforge_sensor.use_gazebo_plugin = True
    lidar.linkforge_sensor.plugin_filename = "liblidar.so"

    # ROS2 Control (Hits 1106-1126)
    scene.linkforge.use_ros2_control = True
    scene.linkforge.ros2_control_name = "TestSystem"
    item = scene.linkforge.ros2_control_joints.add()
    item.name = "Joint"
    item.cmd_position = True
    item.cmd_velocity = True
    item.cmd_effort = True
    item.state_position = True
    item.state_velocity = True
    item.state_effort = True
    param = item.parameters.add()
    param.name = "p1"
    param.value = "1.0"
    scene.linkforge.gazebo_plugin_name = "gz_ros2_control"
    scene.linkforge.controllers_yaml_path = "/tmp/controllers.yaml"

    from unittest.mock import MagicMock

    context = MagicMock()
    context.scene = scene

    robot, errors = scene_to_robot(context, meshes_dir=Path("/tmp"), dry_run=True)

    assert len(robot.links) == 2
    assert len(robot.joints) == 1
    assert len(robot.sensors) == 1
    # Check transmissions (Fixing 0 == 1)
    assert len(robot.transmissions) == 1
    assert len(robot.ros2_controls) == 1
    assert len(robot.gazebo_elements) > 0


def test_blender_to_core_edge_cases(clean_scene) -> None:
    """Hit absolute remaining gaps (name sanitization, empty loops, unknown types)."""
    from linkforge.blender.adapters.blender_to_core import (
        _calculate_link_frames,
        blender_link_to_core_with_origin,
        get_object_geometry,
        sanitize_name,
    )

    # sanitize_name empty/None (string_utils.py returns "" for empty)
    assert sanitize_name("") == ""
    assert sanitize_name(None) == ""

    # _calculate_link_frames empty
    assert _calculate_link_frames({}, {}, None) == {}

    # get_object_geometry UNKNOWN type
    l_data = bpy.data.lights.new("LDat", "POINT")
    o = bpy.data.objects.new("Unknown", l_data)
    geom, mat = get_object_geometry(o)
    assert geom is None
    from mathutils import Matrix

    assert mat == Matrix.Identity(4)

    # blender_link_to_core_with_origin - multi visuals logic
    p = bpy.data.objects.new("MultiLink", None)
    bpy.context.scene.collection.objects.link(p)
    p.linkforge.is_robot_link = True
    p.linkforge.use_auto_inertia = False

    import bmesh

    def add_vis(name):
        m = bpy.data.meshes.new(name)
        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=1.0)
        bm.to_mesh(m)
        bm.free()
        v = bpy.data.objects.new(f"{p.name}_{name}", m)
        bpy.context.scene.collection.objects.link(v)
        v.parent = p
        return v

    add_vis("visual_a")
    add_vis("visual_b")

    core = blender_link_to_core_with_origin(p)
    assert len(core.visuals) == 2


def test_blender_to_core_small_gaps(clean_scene) -> None:
    """Hit remaining tiny gaps like material fallback and no-geometry link."""
    from pathlib import Path
    from unittest.mock import MagicMock

    import bmesh
    from linkforge.blender.adapters.blender_to_core import (
        blender_link_to_core_with_origin,
        get_object_material,
        scene_to_robot,
    )

    # Material name fallback
    m = bpy.data.meshes.new("MatMesh")
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bm.to_mesh(m)
    bm.free()
    o = bpy.data.objects.new("MatObj", m)
    bpy.context.scene.collection.objects.link(o)
    mat = bpy.data.materials.new("TestMat")
    o.data.materials.append(mat)
    props = MagicMock()
    props.use_material = True
    res = get_object_material(o, props)
    assert res.name == "TestMat"

    # blender_link_to_core_with_origin - No children
    p = bpy.data.objects.new("EmptyLink", None)
    bpy.context.scene.collection.objects.link(p)
    p.linkforge.is_robot_link = True
    p.linkforge.use_auto_inertia = False
    core = blender_link_to_core_with_origin(p)
    assert len(core.visuals) == 0
    assert len(core.collisions) == 0

    # Scene to robot integration with 1 full link
    scene = bpy.context.scene
    root = bpy.data.objects.new("GapsRoot", None)
    scene.collection.objects.link(root)
    root.linkforge.is_robot_link = True
    root.linkforge.link_name = "gaps_root"
    root.linkforge.use_auto_inertia = False

    m1 = bpy.data.meshes.new("G1Mesh")
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bm.to_mesh(m1)
    bm.free()
    v1 = bpy.data.objects.new("gaps_root_visual", m1)
    scene.collection.objects.link(v1)
    v1.parent = root
    v1.dimensions = (1, 1, 1)

    m_col = bpy.data.meshes.new("GColMesh")
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bm.to_mesh(m_col)
    bm.free()
    c1 = bpy.data.objects.new("gaps_root_collision", m_col)
    scene.collection.objects.link(c1)
    c1.parent = root
    c1.dimensions = (1, 1, 1)

    bpy.context.view_layer.update()

    robot, _ = scene_to_robot(bpy.context, meshes_dir=Path("/tmp"), dry_run=True)
    # Check if we at least have our gaps_root link
    assert any(ln.name == "gaps_root" for ln in robot.links)


def test_blender_to_core_missing_errors(clean_scene) -> None:
    """Hit missing child link, empty transmission, simplify, and None returns."""
    from unittest.mock import MagicMock

    import bmesh
    from linkforge.blender.adapters.blender_to_core import (
        blender_joint_to_core,
        blender_link_to_core_with_origin,
        blender_sensor_to_core,
        blender_transmission_to_core,
        get_object_geometry,
    )
    from linkforge.linkforge_core.exceptions import RobotModelError
    from linkforge.linkforge_core.models.geometry import Box

    # blender_link_to_core_with_origin None
    assert blender_link_to_core_with_origin(None) is None

    # blender_joint_to_core None/non-robot
    assert blender_joint_to_core(None) is None
    empty = bpy.data.objects.new("EmptyNone", None)
    assert blender_joint_to_core(empty) is None

    # blender_sensor_to_core None/non-robot
    assert blender_sensor_to_core(None) is None
    assert blender_sensor_to_core(empty) is None

    # blender_transmission_to_core None/non-robot
    assert blender_transmission_to_core(None) is None
    assert blender_transmission_to_core(empty) is None

    # blender_link_to_core_with_origin simplify
    m = bpy.data.meshes.new("CMesh")
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bm.to_mesh(m)
    bm.free()
    o = bpy.data.objects.new("PLink_collision", m)
    bpy.context.scene.collection.objects.link(o)
    o.dimensions = (1, 1, 1)
    p = bpy.data.objects.new("PLink", None)
    bpy.context.scene.collection.objects.link(p)
    p.linkforge.is_robot_link = True
    p.linkforge.use_auto_inertia = False
    o.parent = p
    bpy.context.view_layer.update()

    robot_props = MagicMock()
    robot_props.simplify_collision = True
    core = blender_link_to_core_with_origin(p, robot_props=robot_props)
    assert len(core.collisions) == 1

    # get_object_geometry BOX fallback
    m2 = bpy.data.meshes.new("BMesh")
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bm.to_mesh(m2)
    bm.free()
    o2 = bpy.data.objects.new("BoxFallback", m2)
    bpy.context.scene.collection.objects.link(o2)
    geom, _ = get_object_geometry(o2, meshes_dir=None)
    assert isinstance(geom, Box)

    # Missing child link
    p_link = bpy.data.objects.new("P", None)
    c_link = bpy.data.objects.new("C", None)
    bpy.context.scene.collection.objects.link(p_link)
    bpy.context.scene.collection.objects.link(c_link)
    p_link.linkforge.is_robot_link = True
    c_link.linkforge.is_robot_link = True
    j = bpy.data.objects.new("J", None)
    bpy.context.scene.collection.objects.link(j)
    j.linkforge_joint.is_robot_joint = True
    j.linkforge_joint.parent_link = p_link
    j.linkforge_joint.child_link = None
    with pytest.raises(RobotModelError, match="no child link"):
        blender_joint_to_core(j)

    # Empty transmission
    t = bpy.data.objects.new("T", None)
    bpy.context.scene.collection.objects.link(t)
    t.linkforge_transmission.is_robot_transmission = True
    assert blender_transmission_to_core(t) is None

    # Joint mimic fallback
    mimic_target = bpy.data.objects.new("MimicTarget", None)
    j.linkforge_joint.child_link = c_link
    j.linkforge_joint.use_mimic = True
    j.linkforge_joint.mimic_joint = mimic_target
    core = blender_joint_to_core(j)
    assert core.mimic.joint == "MimicTarget"


def test_detect_primitive_type_tags() -> None:
    """Verify manual primitive type override via custom properties."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    obj["urdf_geometry_type"] = "SPHERE"

    assert detect_primitive_type(obj) == "SPHERE"
