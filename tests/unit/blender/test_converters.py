from pathlib import Path

import bpy
import pytest
from linkforge.blender.converters import (
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


def test_matrix_to_transform_precision():
    """Verify that matrix_to_transform correctly extracts XYZ/RPY from a real Matrix."""
    # Create a matrix with specific translation and rotation
    m = Matrix.Translation((1.0, 2.0, 3.0)) @ Euler((0.4, 0.5, 0.6), "XYZ").to_matrix().to_4x4()

    transform = matrix_to_transform(m)

    assert pytest.approx(transform.xyz.x) == 1.0
    assert pytest.approx(transform.xyz.y) == 2.0
    assert pytest.approx(transform.xyz.z) == 3.0
    assert pytest.approx(transform.rpy.x) == 0.4
    assert pytest.approx(transform.rpy.y) == 0.5
    assert pytest.approx(transform.rpy.z) == 0.6


def test_get_object_geometry_sphere_cylinder():
    """Verify auto-detection of sphere and cylinder primitives via get_object_geometry."""
    # 1. Sphere
    bpy.ops.mesh.primitive_uv_sphere_add(radius=0.5)
    s_obj = bpy.context.active_object
    geom_s = get_object_geometry(s_obj, geometry_type="AUTO")
    assert isinstance(geom_s, Sphere)
    assert pytest.approx(geom_s.radius) == 0.5

    # 2. Cylinder
    bpy.ops.mesh.primitive_cylinder_add(radius=0.3, depth=1.0)
    c_obj = bpy.context.active_object
    geom_c = get_object_geometry(c_obj, geometry_type="AUTO")
    assert isinstance(geom_c, Cylinder)
    assert pytest.approx(geom_c.radius) == 0.3
    assert pytest.approx(geom_c.length) == 1.0


def test_detect_primitive_type_box():
    """Verify that a basic cube mesh is detected as BOX."""
    bpy.ops.mesh.primitive_cube_add(size=2.0)
    obj = bpy.context.active_object
    assert detect_primitive_type(obj) == "BOX"


def test_detect_primitive_type_sphere():
    """Verify that a UV sphere is detected as SPHERE."""
    bpy.ops.mesh.primitive_uv_sphere_add(radius=1.0)
    obj = bpy.context.active_object
    assert detect_primitive_type(obj) == "SPHERE"


def test_detect_primitive_type_cylinder():
    """Verify that a cylinder is detected as CYLINDER."""
    # Dimensions must not be 1:1:1 to avoid sphere detection
    bpy.ops.mesh.primitive_cylinder_add(radius=1.0, depth=3.0)
    obj = bpy.context.active_object
    assert detect_primitive_type(obj) == "CYLINDER"


def test_detect_primitive_type_none_case():
    """A complex mesh (Monkey) should return None for primitive detection."""
    bpy.ops.mesh.primitive_monkey_add()
    obj = bpy.context.active_object
    assert detect_primitive_type(obj) is None


def test_blender_joint_to_core_conversion():
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
    joint = blender_joint_to_core(joint_obj, bpy.context.scene)

    # 5. Verify
    assert joint is not None
    assert joint.name == "blender_j"
    assert joint.type == JointType.REVOLUTE
    assert pytest.approx(joint.axis.y) == 1.0


def test_blender_sensor_to_core_lidar():
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


def test_blender_link_to_core_inertia():
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


def test_categorize_scene_objects_logic():
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
    from linkforge.blender.converters import _categorize_scene_objects

    links, joints, sensors, joints_map, root = _categorize_scene_objects(bpy.context.scene)

    # 3. Verify
    assert "l_link" in links
    assert j_obj in joints
    assert root[0] == "l_link"


def test_calculate_link_frames_logic():
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


def test_get_object_material_logic():
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


def test_blender_link_to_core_multi_elements():
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


def test_get_object_geometry_forced_primitives():
    """Verify that get_object_geometry honors forced primitive types."""
    bpy.ops.mesh.primitive_cube_add(size=2.0)
    obj = bpy.context.active_object

    # 1. Force Sphere (radius should be max dim / 2 = 1.0)
    geom_s = get_object_geometry(obj, geometry_type="SPHERE")
    assert isinstance(geom_s, Sphere)
    assert pytest.approx(geom_s.radius) == 1.0

    # 2. Force Cylinder (z depth is 2.0, max x/y is 2.0 -> radius 1.0)
    geom_c = get_object_geometry(obj, geometry_type="CYLINDER")
    assert isinstance(geom_c, Cylinder)
    assert pytest.approx(geom_c.radius) == 1.0
    assert pytest.approx(geom_c.length) == 2.0


def test_get_object_geometry_convex_hull(tmp_path):
    """Verify that convex hull fallback is handled."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # CONVEX_HULL currently falls back to BOX if not implemented with real hull
    geom = get_object_geometry(
        obj, geometry_type="CONVEX_HULL", meshes_dir=tmp_path, link_name="hull"
    )
    assert isinstance(geom, (Box, Mesh))


def test_get_object_material_logic_no_nodes():
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


def test_sanitize_name_logic():
    """Verify name sanitization for XACRO compatibility."""
    # Default allows hyphens
    assert sanitize_name("my-robot-link") == "my-robot-link"
    # Forced for Python identifier
    assert sanitize_name("my-robot-link", allow_hyphen=False) == "my_robot_link"
    assert sanitize_name("link.001") == "link_001"
    assert sanitize_name("123link") == "_123link"  # Correct behavior for leading digits


def test_categorize_scene_objects_complex_hierarchy():
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
    from linkforge.blender.converters import _categorize_scene_objects

    links, joints, sensors, joints_map, root_link = _categorize_scene_objects(bpy.context.scene)

    assert "base_link" in links
    assert "child_link" in links
    assert any(j.name == "base_to_child" for j in joints)
    assert any(s.name == "camera" for s in sensors)
    assert len(joints_map) == 1
    assert joints_map["child_link"][0] == "base_link"  # Parent name
    assert root_link[0] == "base_link"


def test_blender_joint_to_core_types():
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

    joint = blender_joint_to_core(joint_obj, bpy.context.scene)

    assert joint.type == JointType.PRISMATIC
    assert joint.axis.x == 1.0
    assert joint.limits.lower == -1.0
    assert joint.limits.upper == 2.0

    # 2. Continuous
    joint_obj.linkforge_joint.joint_type = "CONTINUOUS"
    joint = blender_joint_to_core(joint_obj, bpy.context.scene)
    assert joint.type == JointType.CONTINUOUS
    # Continuous joints shouldn't have lower/upper limits in standard URDF but our model handles it.


def test_blender_sensor_to_core_all_types():
    """Verify conversion of various sensor types and their properties."""
    from linkforge.blender.converters import blender_sensor_to_core

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


def test_detect_primitive_type_logic():
    """Verify primitive detection heuristics."""
    from linkforge.blender.converters import detect_primitive_type

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


def test_matrix_to_transform_conversion():
    """Verify 4x4 matrix to Transform conversion."""
    import math

    import mathutils
    from linkforge.blender.converters import matrix_to_transform

    # Identity
    mat = mathutils.Matrix.Identity(4)
    tf = matrix_to_transform(mat)
    assert tf.xyz.x == 0 and tf.xyz.y == 0 and tf.xyz.z == 0
    assert tf.rpy.x == 0 and tf.rpy.y == 0 and tf.rpy.z == 0

    # Translation (1, 2, 3)
    mat = mathutils.Matrix.Translation((1, 2, 3))
    tf = matrix_to_transform(mat)
    assert tf.xyz.x == 1 and tf.xyz.y == 2 and tf.xyz.z == 3

    # Rotation (90 deg around X)
    mat = mathutils.Matrix.Rotation(math.radians(90), 4, "X")
    tf = matrix_to_transform(mat)
    # Eulers can be tricky, check approx
    assert pytest.approx(tf.rpy.x) == 1.570796
    assert pytest.approx(tf.rpy.y) == 0
    assert pytest.approx(tf.rpy.z) == 0


def test_get_object_geometry_decimation(tmp_path):
    """Verify that decimation (simplification) is active if requested."""
    # Create a reasonably complex object
    bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16)
    obj = bpy.context.active_object

    # 1. Without simplify
    g1 = get_object_geometry(
        obj, geometry_type="MESH", simplify=False, meshes_dir=tmp_path, link_name="l1"
    )

    # 2. With simplify (decimate to 10%)
    g2 = get_object_geometry(
        obj,
        geometry_type="MESH",
        simplify=True,
        decimation_ratio=0.1,
        meshes_dir=tmp_path,
        link_name="l2",
    )

    assert isinstance(g1, Mesh)
    assert isinstance(g2, Mesh)


def test_get_object_geometry_dry_run(tmp_path):
    """Verify that dry_run skips side-effects (like mesh saving)."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object

    # Should not crash and should return geometry even with invalid dir
    geom = get_object_geometry(
        obj, geometry_type="MESH", dry_run=True, meshes_dir=Path("/invalid/path"), link_name="dry"
    )
    assert isinstance(geom, Mesh)


def test_scene_to_robot_conversion():
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


def test_extract_mesh_triangles_logic():
    """Test raw triangle extraction from a primitive."""
    bpy.ops.mesh.primitive_cube_add(size=1.0)
    obj = bpy.context.active_object

    verts, tris = extract_mesh_triangles(obj)

    # Cube has 8 vertices and 12 triangles
    assert len(verts) == 8
    assert len(tris) == 12


def test_get_object_geometry_auto_primitive():
    """Test auto-detection of box primitive via get_object_geometry."""
    bpy.ops.mesh.primitive_cube_add(size=2.0)
    obj = bpy.context.active_object

    geom = get_object_geometry(obj, geometry_type="AUTO")

    assert isinstance(geom, Box)
    assert pytest.approx(geom.size.x) == 2.0


def test_blender_link_to_core_complex():
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


def test_blender_link_to_core_geometry_and_material():
    """Verify detailed geometry and material conversion."""
    from linkforge.blender.converters import blender_link_to_core_with_origin
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


def test_robust_origin_extraction_logic():
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


def test_blender_sensor_contact():
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


def test_blender_sensor_force_torque():
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


def test_blender_sensor_with_noise():
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


def test_blender_sensor_with_plugin():
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


def test_blender_sensor_not_robot_sensor():
    """Test that non-robot sensor objects return None."""
    bpy.ops.object.empty_add()
    sensor_obj = bpy.context.active_object
    sensor_obj.linkforge_sensor.is_robot_sensor = False

    sensor = blender_sensor_to_core(sensor_obj)

    assert sensor is None
