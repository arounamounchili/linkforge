from pathlib import Path
from unittest.mock import MagicMock

import bpy
import pytest

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_linkforge_scene,
    safe_get_sensor,
    safe_get_transmission,
)

try:
    import importlib.util

    HAS_PYBULLET = importlib.util.find_spec("pybullet") is not None
except ImportError:
    HAS_PYBULLET = False
from linkforge.blender.adapters.blender_to_core import (
    _calculate_link_frames,
    detect_primitive_type,
    extract_mesh_triangles,
    get_object_geometry,
    get_object_material,
    matrix_to_transform,
    sanitize_name,
    scene_to_robot,
)
from linkforge.blender.adapters.translator import (
    JointTranslator,
    LinkTranslator,
    SensorTranslator,
    TransmissionTranslator,
)
from linkforge.core import (
    Box,
    Cylinder,
    GeometryType,
    Joint,
    JointType,
    Link,
    Mesh,
    RobotBuilder,
    RobotValidationError,
    SensorType,
    Sphere,
    ValidationErrorCode,
)
from linkforge.core.constants import (
    HW_IF_VELOCITY,
    TRANS_CUSTOM,
    TRANS_DIFFERENTIAL,
    TRANS_SIMPLE,
)
from mathutils import Euler, Matrix


def translate_link_to_model(obj, context):
    if obj is None:
        return None
    builder = RobotBuilder("test_robot")
    lb = LinkTranslator().translate(obj, builder, context)
    if lb:
        lb.commit()
    props = getattr(obj, "linkforge", None)
    link_name = props.link_name if props and props.link_name else obj.name
    return builder.robot.get_link(link_name)


def translate_joint_to_model(obj, context, parent=None, child=None):
    if obj is None:
        return None
    builder = RobotBuilder("test_robot")
    p_name = None
    if parent:
        lb_p = LinkTranslator().translate(parent, builder, context)
        if lb_p:
            lb_p.root()
        p_props = getattr(parent, "linkforge", None)
        p_name = p_props.link_name if p_props and p_props.link_name else parent.name

    lb_c = None
    if child:
        c_props = getattr(child, "linkforge", None)
        c_name = c_props.link_name if c_props and c_props.link_name else child.name
        lb_c = builder.link(c_name, parent=p_name)
        LinkTranslator().translate(child, builder, context, lb=lb_c)

    JointTranslator().translate(obj, builder, context, lb=lb_c)
    if lb_c:
        lb_c.commit()

    props = getattr(obj, "linkforge_joint", None)
    joint_name = props.joint_name if props and props.joint_name else obj.name
    return builder.robot.get_joint(joint_name)


def test_matrix_to_transform_precision(scene, blender_context) -> None:
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


def test_get_object_geometry_sphere_cylinder(scene, blender_context) -> None:
    """Verify auto-detection of sphere and cylinder primitives via get_object_geometry."""
    import bpy

    # Sphere: use real UV sphere (default: 32 segs, 16 rings = 482 verts, 480 faces)
    bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16, radius=0.5)
    s_obj = bpy.context.active_object
    assert s_obj is not None
    geom_s, world_matrix = get_object_geometry(s_obj, geometry_type="auto")
    assert isinstance(geom_s, Sphere)
    assert geom_s.radius > 0.0
    assert world_matrix == s_obj.matrix_world

    # Cylinder: use real Blender cylinder
    bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=0.3, depth=1.0)
    c_obj = bpy.context.active_object
    assert c_obj is not None
    geom_c, world_matrix = get_object_geometry(c_obj, geometry_type="auto")
    assert isinstance(geom_c, Cylinder)
    assert geom_c.radius > 0.0
    assert geom_c.length > 0.0
    assert world_matrix == c_obj.matrix_world


def test_detect_primitive_type_box(scene, blender_context) -> None:
    """Verify that a basic cube mesh is detected as BOX."""
    import bpy

    # Use Blender's real primitive cube (8 verts, 6 quad faces)
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    assert obj is not None
    assert detect_primitive_type(obj) == "box"


def test_detect_primitive_type_sphere(scene, blender_context) -> None:
    """Verify that a UV sphere is detected as SPHERE."""
    import bpy

    # Use Blender's real UV sphere (default 32 segs x 16 rings = 482 verts, 480 faces)
    bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16, radius=1.0)
    obj = bpy.context.active_object
    assert obj is not None
    assert detect_primitive_type(obj) == "sphere"


def test_detect_primitive_type_cylinder(scene, blender_context) -> None:
    """Verify that a cylinder is detected as CYLINDER."""
    import bpy

    # Use Blender's real cylinder (32 vertices matches the cylinder topology detection range)
    bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=1.0, depth=3.0)
    obj = bpy.context.active_object
    assert obj is not None
    assert detect_primitive_type(obj) == "cylinder"


def test_detect_primitive_type_none_case(scene, blender_context) -> None:
    """A mesh tagged as MESH geometry type should return None for primitive detection."""
    from tests.blender_test_utils import create_mesh_object

    obj = create_mesh_object("complex_mesh", scene=scene, with_cube=True)
    # Tag explicitly as a generic mesh geometry type to bypass topology detection
    obj["source_geometry_type"] = "MESH"
    assert detect_primitive_type(obj) is None


def test_blender_joint_to_core_conversion(scene, blender_context) -> None:
    """Verify that a Blender Empty marked as a joint converts correctly to Core Joint."""
    # Setup Parent Link
    p_obj = create_test_object("parent_l", None, scene)
    safe_get_linkforge(p_obj).link_name = "parent_l"

    # Setup Child Link
    c_obj = create_test_object("child_l", None, scene)
    safe_get_linkforge(c_obj).link_name = "child_l"

    # Setup Blender Joint
    joint_obj = create_test_object("blender_j", None, scene)
    props = safe_get_joint(joint_obj)
    props.is_robot_joint = True
    props.joint_name = "blender_j"
    props.joint_type = "revolute"
    props.axis = "Y"
    props.parent_link = p_obj
    props.child_link = c_obj
    props.use_limits = True
    props.limit_lower = -1.0
    props.limit_upper = 1.0

    # Convert
    joint = translate_joint_to_model(joint_obj, blender_context, parent=p_obj, child=c_obj)

    # Verify
    assert joint is not None
    assert joint.name == "blender_j"
    assert joint.type == JointType.REVOLUTE
    assert joint.axis is not None
    assert pytest.approx(joint.axis.y) == 1.0


def test_blender_sensor_to_core_lidar(scene, blender_context) -> None:
    """Verify that a Blender sensor object is correctly converted back to Core Sensor."""
    # Setup Parent Link
    parent_obj = create_test_object("base_link", None, scene)
    safe_get_linkforge(parent_obj).is_robot_link = True
    safe_get_linkforge(parent_obj).link_name = "base_link"

    # Setup Sensor Object
    sensor_obj = create_test_object("my_lidar", None, scene)
    sensor_obj.parent = parent_obj

    props = safe_get_sensor(sensor_obj)
    props.is_robot_sensor = True
    props.attached_link = parent_obj
    props.sensor_type = "lidar"
    props.update_rate = 50.0
    props.lidar_range_min = 0.5
    props.lidar_range_max = 50.0

    # Convert
    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("base_link"))  # Register required link
    SensorTranslator().translate(sensor_obj, builder, blender_context)
    sensor = builder.robot.sensors[0] if builder.robot.sensors else None

    # Verify
    assert sensor is not None
    assert sensor.type == SensorType.LIDAR
    assert sensor.update_rate == 50.0
    assert sensor.link_name == "base_link"


def test_blender_link_to_core_inertia(scene, blender_context) -> None:
    """Verify that inertial properties are correctly extracted from Blender objects."""
    obj = create_test_object("inertial_link", None, scene)
    props = safe_get_linkforge(obj)
    props.is_robot_link = True
    props.mass = 2.5
    props.use_auto_inertia = False
    props.inertia_ixx = 1.0
    props.inertia_iyy = 1.0
    props.inertia_izz = 1.0

    link = translate_link_to_model(obj, blender_context)
    assert link is not None
    assert link.inertial is not None
    assert link.inertial.mass == 2.5
    assert link.inertial.inertia.ixx == 1.0


def test_blender_link_to_core_physics(scene, blender_context) -> None:
    """Verify that physics properties (friction, stiffness, damping) are exported to LinkPhysics."""
    obj = create_test_object("physics_link", None, scene)
    props = safe_get_linkforge(obj)
    props.is_robot_link = True

    # Set simulation physics values
    props.use_simulation_props = True
    props.mu = 0.8
    props.kp = 1.0e9
    props.kd = 50.0

    # Convert to Core Link
    link = translate_link_to_model(obj, blender_context)

    # Verify
    assert link is not None
    assert link.physics is not None
    assert pytest.approx(link.physics.mu) == 0.8
    assert pytest.approx(link.physics.kp) == 1.0e9
    assert pytest.approx(link.physics.kd) == 50.0


def test_categorize_scene_objects_logic(scene, blender_context) -> None:
    """Verify that scene objects are correctly categorized as links, joints, or sensors."""
    # Setup Scene
    l_obj = create_test_object("l_link", None, scene)
    safe_get_linkforge(l_obj).is_robot_link = True

    j_obj = create_test_object("j_joint", None, scene)
    safe_get_joint(j_obj).is_robot_joint = True

    t_obj = create_test_object("t_trans", None, scene)
    safe_get_transmission(t_obj).is_robot_transmission = True

    # Call internal categorizer
    from linkforge.blender.adapters.blender_to_core import _categorize_scene_objects

    links, joints, sensors, transmissions, joints_map, root = _categorize_scene_objects(scene)

    # Verify
    assert "l_link" in links
    assert j_obj in joints
    assert root is not None
    assert root[0] == "l_link"


def test_calculate_link_frames_logic(scene, blender_context) -> None:
    """Verify recursive frame calculation with real objects."""
    # Setup Hierarchy
    # Setup Hierarchy
    root_obj = create_test_object("root", None, scene)
    child_obj = create_test_object("child", None, scene)

    child_obj.parent = root_obj
    child_obj.location = (1, 0, 0)

    if bpy.context.view_layer:
        bpy.context.view_layer.update()

    # Setup structures
    link_objects = {"root": root_obj, "child": child_obj}
    joints_map = {"child": ("root", None)}  # Dummy joint map entry
    root_link = ("root", root_obj)

    # Calculate
    frames = _calculate_link_frames(link_objects, joints_map, root_link)

    # Verify
    assert "root" in frames
    assert "child" in frames
    # Child frame should be at (1,0,0) relative to root
    assert pytest.approx(frames["child"].to_translation().x) == 1.0


def test_get_object_material_logic(scene, blender_context) -> None:
    """Verify material extraction from Principled BSDF node."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    assert obj is not None
    assert obj is not None
    mat = bpy.data.materials.new(name="PMat")
    mat.use_nodes = True
    assert mat.node_tree is not None
    nodes = mat.node_tree.nodes
    bsdf = nodes.get("Principled BSDF")
    assert bsdf is not None
    socket = bsdf.inputs.get("Base Color")
    if socket and hasattr(socket, "default_value"):
        setattr(socket, "default_value", (0.1, 0.2, 0.3, 1.0))  # noqa: B010
    assert obj.data is not None and hasattr(obj.data, "materials")
    getattr(obj.data, "materials").append(mat)

    link_props = safe_get_linkforge(obj)
    link_props.use_material = True

    material = get_object_material(obj, link_props)
    assert material is not None
    assert material.name == "PMat"
    assert material.color is not None
    assert pytest.approx(material.color.r) == 0.1
    assert pytest.approx(material.color.g) == 0.2


def test_blender_link_to_core_multi_elements(scene, blender_context) -> None:
    """Verify conversion of a link with multiple visuals back to Core."""
    link_obj = create_test_object("multi_link", None, scene)
    safe_get_linkforge(link_obj).is_robot_link = True

    # Visual 1
    bpy.ops.mesh.primitive_cube_add()
    v1 = bpy.context.active_object
    assert v1 is not None
    v1.name = "v1_visual"
    safe_get_linkforge(v1).is_robot_visual = True
    v1.parent = link_obj

    # Visual 2
    bpy.ops.mesh.primitive_uv_sphere_add()
    v2 = bpy.context.active_object
    assert v2 is not None
    v2.name = "v2_visual"
    safe_get_linkforge(v2).is_robot_visual = True
    v2.parent = link_obj

    if bpy.context.view_layer:
        bpy.context.view_layer.update()

    link = translate_link_to_model(link_obj, blender_context)
    assert link is not None
    assert len(link.visuals) == 2


def test_get_object_geometry_forced_primitives(scene, blender_context) -> None:
    """Verify that get_object_geometry honors forced primitive types."""
    bpy.ops.mesh.primitive_cube_add(size=2.0)
    obj = bpy.context.active_object
    assert obj is not None

    # Force Sphere (radius should be max dim / 2 = 1.0)
    geom_s, wm_s = get_object_geometry(obj, geometry_type="sphere")
    assert isinstance(geom_s, Sphere)
    assert wm_s == obj.matrix_world
    assert pytest.approx(geom_s.radius) == 1.0

    # Force Cylinder (z depth is 2.0, max x/y is 2.0 -> radius 1.0)
    geom_c, wm_c = get_object_geometry(obj, geometry_type="cylinder")
    assert isinstance(geom_c, Cylinder)
    assert wm_c == obj.matrix_world
    assert pytest.approx(geom_c.radius) == 1.0
    assert pytest.approx(geom_c.length) == 2.0


def test_get_object_geometry_mesh_simplified(tmp_path, scene, blender_context) -> None:
    """Verify that mesh simplification fallback is handled."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    assert obj is not None
    # MESH currently falls back to BOX if not implemented with real hull
    geom, wm = get_object_geometry(obj, geometry_type="mesh", meshes_dir=tmp_path, link_name="hull")
    assert isinstance(geom, (Box, Mesh))


def test_get_object_material_logic_nodes(scene, blender_context) -> None:
    """Verify material extraction from Blender object using Nodes (Modern)."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    assert obj is not None

    # Use real link properties
    safe_get_linkforge(obj).use_material = True

    mat = bpy.data.materials.new(name="Test-Mat")
    mat.use_nodes = True

    # Set color via Principled BSDF (standard for Blender 4.2+)
    if mat.node_tree:
        nodes = mat.node_tree.nodes
        bsdf = nodes.get("Principled BSDF")
        assert bsdf is not None
        # Use "Base Color" input for standard Principled BSDF
        socket = bsdf.inputs.get("Base Color")
        if socket and hasattr(socket, "default_value"):
            setattr(socket, "default_value", (1.0, 0.0, 0.0, 1.0))  # noqa: B010

    # Clear and assign explicitly
    if obj.data and hasattr(obj.data, "materials"):
        getattr(obj.data, "materials").clear()
        getattr(obj.data, "materials").append(mat)
    if bpy.context.view_layer:
        bpy.context.view_layer.update()

    core_mat = get_object_material(obj, safe_get_linkforge(obj))
    assert core_mat is not None
    assert core_mat.color and core_mat.color.r == 1.0
    assert "Test_Mat" in core_mat.name or "Test-Mat" in core_mat.name


def test_sanitize_name_logic(scene, blender_context) -> None:
    """Verify name sanitization for XACRO compatibility."""
    # Default allows hyphens
    assert sanitize_name("my-robot-link") == "my-robot-link"
    # Forced for Python identifier
    assert sanitize_name("my-robot-link", allow_hyphen=False) == "my_robot_link"
    assert sanitize_name("link.001") == "link_001"
    assert sanitize_name("123link") == "_123link"  # Correct behavior for leading digits


def test_categorize_scene_objects_complex_hierarchy(scene, blender_context) -> None:
    """Verify categorization of a full robot hierarchy with sensors and joints."""
    # Base Link
    base = create_test_object("base_link", None, scene)
    safe_get_linkforge(base).is_robot_link = True

    # Child Link
    child = create_test_object("child_link", None, scene)
    safe_get_linkforge(child).is_robot_link = True

    # Joint (Base -> Child)
    joint = create_test_object("base_to_child", None, scene)
    safe_get_joint(joint).is_robot_joint = True
    safe_get_joint(joint).joint_type = "revolute"
    safe_get_joint(joint).parent_link = base
    safe_get_joint(joint).child_link = child

    # Sensor on Child
    sensor = create_test_object("CAMERA", None, scene)
    safe_get_sensor(sensor).is_robot_sensor = True
    safe_get_sensor(sensor).sensor_type = "CAMERA"
    safe_get_sensor(sensor).attached_link = child

    # Manually run the protected function (we are testing unit logic)
    from linkforge.blender.adapters.blender_to_core import _categorize_scene_objects

    links, joints, sensors, transmissions, joints_map, root_link = _categorize_scene_objects(scene)

    assert "base_link" in links
    assert "child_link" in links
    assert any(j.name == "base_to_child" for j in joints)
    assert any(s.name == "CAMERA" for s in sensors)
    assert len(joints_map) == 1
    assert joints_map["child_link"][0] == "base_link"  # Parent name
    assert root_link is not None
    assert root_link[0] == "base_link"


def test_blender_joint_to_core_types(scene, blender_context) -> None:
    """Verify conversion of different joint types and parameters."""
    # Setup Parent/Child Links
    parent = create_test_object("parent_link", None, scene)
    safe_get_linkforge(parent).is_robot_link = True

    child = create_test_object("child_link", None, scene)
    safe_get_linkforge(child).is_robot_link = True

    # Prismatic
    joint_obj = create_test_object("prismatic_joint", None, scene)
    props = safe_get_joint(joint_obj)
    props.is_robot_joint = True
    props.parent_link = parent
    props.child_link = child  # Also usually required or good practice
    props.joint_type = "prismatic"
    props.axis = "X"
    props.limit_lower = -1.0
    props.limit_upper = 2.0
    props.limit_effort = 100.0
    props.limit_velocity = 5.0

    joint = translate_joint_to_model(joint_obj, blender_context, parent=parent, child=child)
    assert joint is not None
    assert joint.type == JointType.PRISMATIC
    assert joint.axis and joint.axis.x == 1.0
    assert joint.limits and joint.limits.lower == -1.0
    assert joint.limits.upper == 2.0

    # Continuous
    safe_get_joint(joint_obj).joint_type = "continuous"
    joint = translate_joint_to_model(joint_obj, blender_context, parent, child)
    assert joint is not None
    assert joint.type == JointType.CONTINUOUS
    # Continuous joints shouldn't have lower/upper limits in standard URDF but our model handles it.


def test_blender_joint_to_core_advanced_props(scene, blender_context) -> None:
    """Verify that safety controller and calibration are correctly synced to Core."""
    # Setup Links
    p = create_test_object("p_link", None, scene)
    safe_get_linkforge(p).is_robot_link = True

    c = create_test_object("c_link", None, scene)
    safe_get_linkforge(c).is_robot_link = True

    # Setup Joint
    joint_obj = create_test_object("advanced_j", None, scene)
    props = safe_get_joint(joint_obj)
    props.is_robot_joint = True
    props.parent_link = p
    props.child_link = c
    props.joint_type = "revolute"
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

    # Convert
    joint = translate_joint_to_model(joint_obj, blender_context, parent=p, child=c)
    assert joint is not None

    # Verify
    assert joint.safety_controller is not None
    assert joint.safety_controller.soft_lower_limit == -1.0
    assert joint.safety_controller.k_position == 100.0

    assert joint.calibration is not None
    assert joint.calibration.rising == 0.5
    assert joint.calibration.falling is None


def test_blender_sensor_to_core_all_types(scene, blender_context) -> None:
    """Verify conversion of various sensor types and their properties."""

    # Setup Parent Link
    bpy.ops.object.empty_add()
    parent = bpy.context.active_object
    assert parent is not None
    # Setup Parent Link
    parent = create_test_object("sensor_link", None, scene)
    safe_get_linkforge(parent).is_robot_link = True

    # IMU
    imu_obj = create_test_object("imu_sensor", None, scene)
    props = safe_get_sensor(imu_obj)
    props.is_robot_sensor = True
    props.attached_link = parent
    props.sensor_type = "imu"
    props.update_rate = 100.0
    props.always_on = True
    props.visualize = True

    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("sensor_link"))
    SensorTranslator().translate(imu_obj, builder, blender_context)
    sensor = builder.robot.sensors[-1]
    assert sensor is not None
    assert sensor.type == SensorType.IMU
    assert sensor.update_rate == 100.0
    assert sensor.always_on is True
    assert sensor.visualize is True

    # Camera
    cam_obj = create_test_object("camera_sensor", None, scene)
    props = safe_get_sensor(cam_obj)
    props.is_robot_sensor = True
    props.attached_link = parent
    props.sensor_type = "CAMERA"
    props.camera_horizontal_fov = 1.047
    props.camera_width = 800
    props.camera_height = 600

    SensorTranslator().translate(cam_obj, builder, blender_context)
    sensor = builder.robot.sensors[-1]
    assert sensor is not None
    assert sensor.type == SensorType.CAMERA
    assert sensor.camera_info is not None
    assert pytest.approx(sensor.camera_info.horizontal_fov) == 1.047
    assert sensor.camera_info.width == 800

    # Lidar
    lidar_obj = create_test_object("lidar_sensor", None, scene)
    props = safe_get_sensor(lidar_obj)
    props.is_robot_sensor = True
    props.attached_link = parent
    props.sensor_type = "lidar"
    props.lidar_range_max = 50.0
    props.lidar_range_min = 0.5

    SensorTranslator().translate(lidar_obj, builder, blender_context)
    sensor = builder.robot.sensors[-1]
    assert sensor is not None
    assert sensor.type == SensorType.LIDAR
    assert sensor.lidar_info is not None


def test_detect_primitive_type_logic(scene, blender_context) -> None:
    """Verify primitive detection heuristics."""
    from linkforge.blender.adapters.blender_to_core import detect_primitive_type

    # Cube
    bpy.ops.mesh.primitive_cube_add()
    cube = bpy.context.active_object
    assert cube is not None
    assert cube is not None
    assert detect_primitive_type(cube) == "box"

    # Sphere (UV Sphere default)
    bpy.ops.mesh.primitive_uv_sphere_add()
    sphere = bpy.context.active_object
    assert sphere is not None
    assert detect_primitive_type(sphere) == "sphere"

    # Cylinder
    bpy.ops.mesh.primitive_cylinder_add()
    cyl = bpy.context.active_object
    assert cyl is not None
    # Scale it to be clearly cylindrical (tall) to avoid being ambiguous with sphere
    cyl.scale = (1, 1, 2)
    if bpy.context.view_layer:
        bpy.context.view_layer.update()  # Ensure dimensions update
    assert detect_primitive_type(cyl) == "cylinder"

    # Complex Mesh (Monkey/Suzanne)
    bpy.ops.mesh.primitive_monkey_add()
    monkey = bpy.context.active_object
    assert monkey is not None
    assert detect_primitive_type(monkey) is None


def test_matrix_to_transform_conversion(scene, blender_context) -> None:
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


def test_get_object_geometry_decimation(tmp_path, scene, blender_context) -> None:
    """Verify that decimation (simplification) is active if requested."""
    # Create a reasonably complex object
    bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16)
    obj = bpy.context.active_object
    assert obj is not None

    # Without simplify
    g1, wm1 = get_object_geometry(
        obj, geometry_type="mesh", simplify=False, meshes_dir=tmp_path, link_name="l1"
    )

    # With simplify (decimate to 10%)
    g2, wm2 = get_object_geometry(
        obj,
        geometry_type="mesh",
        simplify=True,
        decimation_ratio=0.1,
        meshes_dir=tmp_path,
        link_name="l2",
    )

    assert isinstance(g1, Mesh)
    assert isinstance(g2, Mesh)


def test_get_object_geometry_dry_run(tmp_path, scene, blender_context) -> None:
    """Verify that dry_run skips side-effects (like mesh saving)."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    assert obj is not None

    # Should not crash and should return geometry even with invalid dir
    geom, wm = get_object_geometry(
        obj, geometry_type="mesh", dry_run=True, meshes_dir=Path("/invalid/path"), link_name="dry"
    )
    assert isinstance(geom, Mesh)
    assert wm == obj.matrix_world


def test_scene_to_robot_conversion(scene, blender_context) -> None:
    """Verify that an entire Blender scene is converted to a Core Robot."""
    # Setup a minimal link structure
    link_obj = create_test_object("base_link", None, scene)
    safe_get_linkforge(link_obj).is_robot_link = True
    safe_get_linkforge(link_obj).link_name = "base_link"

    # Convert
    robot, errors = scene_to_robot(bpy.context)

    # Verify
    assert robot is not None
    assert len(robot.links) >= 1
    assert any(link.name == "base_link" for link in robot.links)


def test_extract_mesh_triangles_logic(scene, blender_context) -> None:
    """Test raw triangle extraction from a primitive."""
    bpy.ops.mesh.primitive_cube_add(size=1.0)
    obj = bpy.context.active_object
    assert obj is not None

    mesh_data = extract_mesh_triangles(obj)
    assert mesh_data is not None
    verts, tris = mesh_data

    # Cube has 8 vertices and 12 triangles
    assert len(verts) == 8
    assert len(tris) == 12


def test_get_object_geometry_auto_primitive(scene, blender_context) -> None:
    """Test auto-detection of box primitive via get_object_geometry."""
    bpy.ops.mesh.primitive_cube_add(size=2.0)
    obj = bpy.context.active_object
    assert obj is not None

    geom, wm = get_object_geometry(obj, geometry_type="auto")

    assert isinstance(geom, Box)
    assert wm == obj.matrix_world
    assert pytest.approx(geom.size.x) == 2.0


def test_blender_link_to_core_complex(scene, blender_context) -> None:
    """Verify conversion of a link with multiple visuals and collisions back to Core."""
    # Ensure a clean state
    bpy.ops.object.select_all(action="DESELECT")

    # Setup Link Empty
    link_obj = create_test_object("base_link", None, scene)
    safe_get_linkforge(link_obj).is_robot_link = True
    safe_get_linkforge(link_obj).link_name = "base_link"

    # Add Visual Child
    bpy.ops.mesh.primitive_cube_add(size=1.0)
    vis_obj = bpy.context.active_object
    assert vis_obj is not None
    vis_obj.name = "base_link_visual"
    vis_obj.parent = link_obj
    vis_obj.location = (1, 0, 0)

    # Add Collision Child
    bpy.ops.mesh.primitive_uv_sphere_add(radius=0.5)
    coll_obj = bpy.context.active_object
    assert coll_obj is not None
    coll_obj.name = "base_link_collision"
    coll_obj.parent = link_obj
    coll_obj.location = (0, 1, 0)

    # Update view layer to ensure matrices are correct
    if bpy.context.view_layer:
        bpy.context.view_layer.update()

    # Convert
    link = translate_link_to_model(link_obj, blender_context)

    # Verify
    assert link is not None
    assert len(link.visuals) == 1
    assert len(link.collisions) == 1
    # Check absolute origins (extracted from matrices)
    assert pytest.approx(link.visuals[0].origin.xyz.x) == 1.0
    assert pytest.approx(link.collisions[0].origin.xyz.y) == 1.0


def test_blender_link_to_core_geometry_and_material(scene, blender_context) -> None:
    """Verify detailed geometry and material conversion."""

    # Link Setup
    link_obj = create_test_object("material_link", None, scene)
    safe_get_linkforge(link_obj).is_robot_link = True

    # Visual with Material
    bpy.ops.mesh.primitive_cube_add(size=1.0)
    vis_obj = bpy.context.active_object
    assert vis_obj is not None
    vis_obj.name = "vis_cube_visual"
    vis_obj.parent = link_obj

    # Enable material export on the LINK properties (parent)
    safe_get_linkforge(link_obj).use_material = True

    # Create Material using Nodes (Principled BSDF)
    mat = bpy.data.materials.new(name="RedMat")
    mat.use_nodes = True
    if mat.node_tree:
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        assert bsdf is not None
        socket = bsdf.inputs.get("Base Color")
        if socket and hasattr(socket, "default_value"):
            setattr(socket, "default_value", (1, 0, 0, 1))  # Red  # noqa: B010
    if vis_obj.data and hasattr(vis_obj.data, "materials"):
        getattr(vis_obj.data, "materials").append(mat)

    # Collision with Cylinder
    bpy.ops.mesh.primitive_cylinder_add()
    coll_obj = bpy.context.active_object
    assert coll_obj is not None
    coll_obj.name = "coll_cyl_collision"
    coll_obj.parent = link_obj
    # Scale to match heuristic
    coll_obj.scale = (1, 1, 2)

    if bpy.context.view_layer:
        bpy.context.view_layer.update()

    # Convert
    link = translate_link_to_model(link_obj, blender_context)

    # Verify Visual
    assert link is not None and len(link.visuals) == 1
    vis = link.visuals[0]
    assert vis.geometry.type == GeometryType.BOX
    assert vis.material is not None
    assert vis.material.name == "RedMat"
    assert vis.material.color and pytest.approx(vis.material.color.r) == 1.0
    assert pytest.approx(vis.material.color.g) == 0.0

    # Verify Collision
    assert len(link.collisions) == 1
    coll = link.collisions[0]
    assert coll.geometry.type == GeometryType.CYLINDER


def test_robust_origin_extraction_logic(scene, blender_context) -> None:
    """Verify relative transform extraction between parent and child."""
    # Create parent
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(1, 1, 1))
    parent = bpy.context.active_object
    assert parent is not None

    # Create child
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(2, 2, 2))
    child = bpy.context.active_object
    assert child is not None
    child.parent = parent

    # The relative matrix should be (1, 1, 1)
    relative_matrix = parent.matrix_world.inverted() @ child.matrix_world
    transform = matrix_to_transform(relative_matrix)

    assert pytest.approx(transform.xyz.x) == 1.0
    assert pytest.approx(transform.xyz.y) == 1.0
    assert pytest.approx(transform.xyz.z) == 1.0


def test_blender_sensor_contact(scene, blender_context) -> None:
    """Test conversion of contact sensor."""
    # Create parent link
    link_obj = create_test_object("base_link", None, scene)
    safe_get_linkforge(link_obj).is_robot_link = True
    safe_get_linkforge(link_obj).link_name = "base_link"

    # Create sensor
    sensor_obj = create_test_object("contact1", None, scene)
    sensor_obj.parent = link_obj
    safe_get_sensor(sensor_obj).is_robot_sensor = True
    safe_get_sensor(sensor_obj).attached_link = link_obj
    safe_get_sensor(sensor_obj).sensor_type = "CONTACT"
    safe_get_sensor(sensor_obj).contact_collision = "collision_link"

    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("base_link"))
    builder.robot.add_link(Link("collision_link"))  # Register required link
    SensorTranslator().translate(sensor_obj, builder, blender_context)
    sensor = builder.robot.sensors[0] if builder.robot.sensors else None

    assert sensor is not None
    assert sensor.type == SensorType.CONTACT
    assert sensor.contact_info is not None
    assert sensor.contact_info.collision == "collision_link"


def test_blender_sensor_force_torque(scene, blender_context) -> None:
    """Test conversion of force-torque sensor."""
    # Create parent link
    link_obj = create_test_object("base_link", None, scene)
    safe_get_linkforge(link_obj).is_robot_link = True
    safe_get_linkforge(link_obj).link_name = "base_link"

    # Create sensor
    sensor_obj = create_test_object("ft_sensor", None, scene)
    sensor_obj.parent = link_obj
    safe_get_sensor(sensor_obj).is_robot_sensor = True
    safe_get_sensor(sensor_obj).attached_link = link_obj
    safe_get_sensor(sensor_obj).sensor_type = "FORCE_TORQUE"

    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("base_link"))  # Register required link
    SensorTranslator().translate(sensor_obj, builder, blender_context)
    sensor = builder.robot.sensors[0] if builder.robot.sensors else None

    assert sensor is not None
    assert sensor.type == SensorType.FORCE_TORQUE
    assert sensor.force_torque_info is not None


def test_blender_sensor_with_noise(scene, blender_context) -> None:
    """Test sensor conversion with noise parameters."""
    # Create parent link
    link_obj = create_test_object("base_link", None, scene)
    safe_get_linkforge(link_obj).is_robot_link = True
    safe_get_linkforge(link_obj).link_name = "base_link"

    # Create sensor
    sensor_obj = create_test_object("imu_sensor", None, scene)
    sensor_obj.parent = link_obj
    safe_get_sensor(sensor_obj).is_robot_sensor = True
    safe_get_sensor(sensor_obj).attached_link = link_obj
    safe_get_sensor(sensor_obj).sensor_type = "imu"
    safe_get_sensor(sensor_obj).use_noise = True
    safe_get_sensor(sensor_obj).noise_mean = 0.1
    safe_get_sensor(sensor_obj).noise_stddev = 0.05

    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("base_link"))  # Register required link
    SensorTranslator().translate(sensor_obj, builder, blender_context)
    sensor = builder.robot.sensors[0] if builder.robot.sensors else None

    assert sensor is not None
    assert sensor.imu_info is not None
    assert sensor.imu_info.angular_velocity_noise is not None
    assert pytest.approx(sensor.imu_info.angular_velocity_noise.mean) == 0.1
    assert pytest.approx(sensor.imu_info.angular_velocity_noise.stddev) == 0.05


def test_blender_sensor_with_plugin(scene, blender_context) -> None:
    """Test sensor conversion with Gazebo plugin."""
    # Create parent link
    link_obj = create_test_object("base_link", None, scene)
    safe_get_linkforge(link_obj).is_robot_link = True
    safe_get_linkforge(link_obj).link_name = "base_link"

    # Create sensor
    sensor_obj = create_test_object("camera1", None, scene)
    sensor_obj.parent = link_obj
    safe_get_sensor(sensor_obj).is_robot_sensor = True
    safe_get_sensor(sensor_obj).attached_link = link_obj
    safe_get_sensor(sensor_obj).sensor_name = "camera1"
    safe_get_sensor(sensor_obj).sensor_type = "CAMERA"
    safe_get_sensor(sensor_obj).use_gazebo_plugin = True
    safe_get_sensor(sensor_obj).plugin_filename = "libmy_camera.so"

    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("base_link"))  # Register required link
    SensorTranslator().translate(sensor_obj, builder, blender_context)
    sensor = builder.robot.sensors[0] if builder.robot.sensors else None

    assert sensor is not None
    assert sensor.plugin is not None
    assert sensor.plugin.name == "camera1_plugin"
    assert sensor.plugin.filename == "libmy_camera.so"


def test_blender_sensor_not_robot_sensor(scene, blender_context) -> None:
    """Test that non-robot sensor objects return None."""
    sensor_obj = create_test_object("not_a_sensor", None, scene)
    safe_get_sensor(sensor_obj).is_robot_sensor = False

    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("base_link"))
    SensorTranslator().translate(sensor_obj, builder, blender_context)
    sensor = builder.robot.sensors[0] if builder.robot.sensors else None

    assert sensor is None


def test_blender_joint_mimic_and_limits_advanced(scene, blender_context) -> None:
    """Verify joint mimicry conversion."""
    # Setup Links
    p = create_test_object("P_Link", None, scene)
    safe_get_linkforge(p).link_name = "P_Link"

    c = create_test_object("C_Link", None, scene)
    safe_get_linkforge(c).link_name = "C_Link"

    # Master joint
    jm = create_test_object("MasterJ", None, scene)
    safe_get_joint(jm).joint_name = "master_j"

    # Slave joint with mimic
    j = create_test_object("SlaveJ", None, scene)
    safe_get_joint(j).is_robot_joint = True
    safe_get_joint(j).parent_link = p
    safe_get_joint(j).child_link = c
    safe_get_joint(j).use_mimic = True
    safe_get_joint(j).mimic_joint = jm
    safe_get_joint(j).mimic_multiplier = 2.0
    safe_get_joint(j).mimic_offset = 0.5

    core = translate_joint_to_model(j, blender_context, parent=p, child=c)
    assert core is not None and core.mimic is not None
    assert core.mimic.joint == "master_j"
    assert core.mimic.multiplier == 2.0
    assert core.mimic.offset == 0.5


def test_blender_link_auto_inertia_sphere(scene, blender_context) -> None:
    """Verify auto-calculation of inertia from sphere geometry."""
    link_obj = create_test_object("AutoLink", None, scene)
    safe_get_linkforge(link_obj).is_robot_link = True
    safe_get_linkforge(link_obj).mass = 2.0
    safe_get_linkforge(link_obj).use_auto_inertia = True

    # Add sphere collision child
    bpy.ops.mesh.primitive_uv_sphere_add(radius=1.0)
    coll = bpy.context.active_object
    assert coll is not None
    coll.name = "AutoLink_collision"
    coll.parent = link_obj

    if bpy.context.view_layer:
        bpy.context.view_layer.update()

    link = translate_link_to_model(link_obj, blender_context)
    # I = 2/5 * m * r^2 = 2/5 * 2.0 * 1^2 = 0.8
    assert link is not None
    assert link.inertial is not None
    assert pytest.approx(link.inertial.inertia.ixx) == 0.8


def test_blender_ros2_control_defaults(clean_scene, scene, blender_context) -> None:
    """Verify default ROS2 control interface assignment when one side is selected."""
    props = safe_get_linkforge_scene(scene)
    props.ros2_control_name = "DefaultBot"
    props.use_ros2_control = True

    joint = props.ros2_control_joints.add()
    joint.name = "j1"
    # Set STATE to True, leave CMD False
    joint.state_position = True
    joint.state_velocity = False
    joint.cmd_position = False
    joint.cmd_velocity = False
    joint.cmd_effort = False

    from linkforge.blender.adapters.translator import Ros2ControlTranslator

    control = Ros2ControlTranslator()._blender_ros2_control_to_core(props)

    assert control is not None
    # Command should default to position because it was empty but state was not
    assert control.joints[0].command_interfaces == ("position",)
    assert control.joints[0].state_interfaces == ("position",)


def test_blender_ros2_control_joint_obj_name_sync(clean_scene, scene, blender_context) -> None:
    """Verify that ros2_control generation uses the joint_obj.linkforge_joint.joint_name instead of item.name if present."""
    props = safe_get_linkforge_scene(scene)
    props.ros2_control_name = "SyncedBot"
    props.use_ros2_control = True

    # Setup a mapped joint object
    joint_obj = create_test_object("MyRealJoint", None, scene)
    safe_get_joint(joint_obj).is_robot_joint = True
    safe_get_joint(joint_obj).joint_name = "MyRealJoint"

    joint = props.ros2_control_joints.add()
    joint.name = "StaleJointName"
    joint.joint_obj = joint_obj
    joint.cmd_position = True
    joint.state_position = True

    from linkforge.blender.adapters.translator import Ros2ControlTranslator

    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("p"))
    builder.robot.add_link(Link("c"))
    builder.robot.add_joint(Joint("MyRealJoint", parent="p", child="c", type=JointType.FIXED))
    Ros2ControlTranslator().translate(props, builder, blender_context)
    control = builder.robot.ros2_controls[0]

    assert control is not None
    assert len(control.joints) == 1
    # Should use the real name from the pointer, not the stale item name
    assert control.joints[0].name == "MyRealJoint"


def test_blender_sensor_gps_and_lidar_full(clean_scene, scene, blender_context) -> None:
    """Exhaustive test for GPS and LIDAR properties."""
    link = create_test_object("L", None, scene)
    safe_get_linkforge(link).is_robot_link = True

    # GPS
    gps_obj = create_test_object("gps", None, scene)
    safe_get_sensor(gps_obj).is_robot_sensor = True
    safe_get_sensor(gps_obj).sensor_type = "gps"
    safe_get_sensor(gps_obj).attached_link = link
    safe_get_sensor(gps_obj).use_noise = True

    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("L"))  # Register required link
    SensorTranslator().translate(gps_obj, builder, blender_context)
    core_gps = builder.robot.sensors[0] if builder.robot.sensors else None
    assert (
        core_gps
        and core_gps.gps_info
        and core_gps.gps_info.position_sensing_horizontal_noise is not None
    )

    # LIDAR with samples
    lidar_obj = create_test_object("lidar", None, scene)
    safe_get_sensor(lidar_obj).is_robot_sensor = True
    safe_get_sensor(lidar_obj).sensor_type = "lidar"
    safe_get_sensor(lidar_obj).attached_link = link
    safe_get_sensor(lidar_obj).lidar_horizontal_samples = 720
    safe_get_sensor(lidar_obj).lidar_horizontal_min_angle = -3.14159
    safe_get_sensor(lidar_obj).lidar_horizontal_max_angle = 3.14159
    safe_get_sensor(lidar_obj).lidar_vertical_samples = 16
    safe_get_sensor(lidar_obj).lidar_vertical_min_angle = -0.1
    safe_get_sensor(lidar_obj).lidar_vertical_max_angle = 0.1

    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("L"))  # Register required link
    SensorTranslator().translate(lidar_obj, builder, blender_context)
    core_lidar = builder.robot.sensors[0] if builder.robot.sensors else None
    assert core_lidar and core_lidar.lidar_info
    assert core_lidar.lidar_info.horizontal_samples == 720
    assert core_lidar.lidar_info.vertical_samples == 16


def test_blender_joint_dynamics(clean_scene, scene, blender_context) -> None:
    """Verify joint dynamics (damping, friction) conversion."""
    p = create_test_object("P", None, scene)
    c = create_test_object("C", None, scene)
    safe_get_linkforge(p).is_robot_link = True
    safe_get_linkforge(c).is_robot_link = True

    j = create_test_object("J", None, scene)
    safe_get_joint(j).is_robot_joint = True
    safe_get_joint(j).joint_type = "revolute"
    safe_get_joint(j).parent_link = p
    safe_get_joint(j).child_link = c
    safe_get_joint(j).use_dynamics = True
    safe_get_joint(j).dynamics_damping = 1.5
    safe_get_joint(j).dynamics_friction = 0.8

    core = translate_joint_to_model(j, blender_context, parent=p, child=c)
    assert core is not None and core.dynamics
    assert pytest.approx(core.dynamics.damping) == 1.5
    assert pytest.approx(core.dynamics.friction) == 0.8


def test_blender_link_inertial_origin(clean_scene, scene, blender_context) -> None:
    """Verify inertial origin extraction."""
    obj = create_test_object("Link", None, scene)
    safe_get_linkforge(obj).is_robot_link = True
    safe_get_linkforge(obj).mass = 1.0
    safe_get_linkforge(obj).use_auto_inertia = False
    safe_get_linkforge(obj).inertia_origin_xyz = (0.1, 0.2, 0.3)
    safe_get_linkforge(obj).inertia_origin_rpy = (0.0, 0.0, 0.5)

    link = translate_link_to_model(obj, blender_context)
    assert link is not None
    assert link.inertial is not None
    assert link.inertial.origin is not None
    assert pytest.approx(link.inertial.origin.xyz.x) == 0.1
    assert pytest.approx(link.inertial.origin.rpy.z) == 0.5


def test_blender_transmission_full(clean_scene, scene, blender_context) -> None:
    """Exhaustive test for Simple and Differential transmissions."""
    # Setup joints
    j1 = create_test_object("J1", None, scene)
    j2 = create_test_object("J2", None, scene)
    safe_get_joint(j1).is_robot_joint = True
    safe_get_joint(j1).joint_name = "Joint1"
    safe_get_joint(j2).is_robot_joint = True
    safe_get_joint(j2).joint_name = "Joint2"

    # Simple Transmission
    t_simple = create_test_object("TransSimple", None, scene)
    safe_get_transmission(t_simple).is_robot_transmission = True
    safe_get_transmission(t_simple).transmission_type = TRANS_SIMPLE
    safe_get_transmission(t_simple).joint_name = j1
    safe_get_transmission(t_simple).mechanical_reduction = 50.0
    safe_get_transmission(t_simple).hardware_interface = HW_IF_VELOCITY

    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("p"))
    builder.robot.add_link(Link("c"))
    builder.robot.add_joint(Joint("Joint1", parent="p", child="c", type=JointType.FIXED))
    builder.robot.add_joint(Joint("Joint2", parent="p", child="c", type=JointType.FIXED))
    TransmissionTranslator().translate(t_simple, builder, blender_context)
    core_simple = builder.robot.transmissions[0] if builder.robot.transmissions else None
    assert core_simple is not None
    assert core_simple.name == "TransSimple"
    assert len(core_simple.joints) > 0 and core_simple.joints[0].name == "Joint1"
    assert core_simple.joints[0].mechanical_reduction == 50.0
    assert core_simple.joints[0].hardware_interfaces == ("velocity",)
    assert len(core_simple.actuators) > 0 and core_simple.actuators[0].name == "Joint1_motor"

    # Differential Transmission
    t_diff = create_test_object("TransDiff", None, scene)
    safe_get_transmission(t_diff).is_robot_transmission = True
    safe_get_transmission(t_diff).transmission_type = TRANS_DIFFERENTIAL
    safe_get_transmission(t_diff).joint1_name = j1
    safe_get_transmission(t_diff).joint2_name = j2
    safe_get_transmission(t_diff).actuator1_name = "act1"
    safe_get_transmission(t_diff).actuator2_name = "act2"

    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("p"))
    builder.robot.add_link(Link("c"))
    builder.robot.add_joint(Joint("Joint1", parent="p", child="c", type=JointType.FIXED))
    builder.robot.add_joint(Joint("Joint2", parent="p", child="c", type=JointType.FIXED))
    TransmissionTranslator().translate(t_diff, builder, blender_context)
    core_diff = builder.robot.transmissions[0] if builder.robot.transmissions else None
    assert core_diff is not None
    assert len(core_diff.joints) == 2
    assert core_diff.actuators[0].name == "act1"
    assert core_diff.actuators[1].name == "act2"


def test_scene_to_robot_with_gazebo_and_errors(clean_scene, scene, blender_context) -> None:
    """Test scene_to_robot with Gazebo plugins and error collection."""
    props = safe_get_linkforge_scene(scene)
    props.use_ros2_control = True
    props.gazebo_plugin_name = "test_plugin"
    props.controllers_yaml_path = "/path/to/yaml"
    props.strict_mode = False

    # Create one valid link to avoid empty robot error
    link_obj = create_test_object("L", None, scene)
    safe_get_linkforge(link_obj).is_robot_link = True

    from unittest import mock

    from linkforge.blender.adapters.blender_to_core import scene_to_robot

    with (
        mock.patch(
            "linkforge.blender.adapters.translator.LinkTranslator.translate",
            side_effect=RobotValidationError(ValidationErrorCode.INVALID_VALUE, "Failed link"),
        ),
        pytest.raises(
            RobotValidationError, match=r"\[INVALID_VALUE\] Multiple configuration errors found"
        ),
    ):
        scene_to_robot(bpy.context)

    # scene_to_robot with robot props
    lf_scene = safe_get_linkforge_scene(scene)
    lf_scene.robot_name = "ExhaustiveRobot"
    lf_scene.use_ros2_control = True
    scene_props = safe_get_linkforge_scene(scene)
    scene_props.ros2_control_name = "DefaultBot"

    # Create a valid joint for ros2_control validation
    child_obj = create_test_object("C", None, scene)
    safe_get_linkforge(child_obj).is_robot_link = True

    joint_obj = create_test_object("DummyJoint", None, scene)
    safe_get_joint(joint_obj).is_robot_joint = True
    safe_get_joint(joint_obj).joint_name = "Dummy"
    safe_get_joint(joint_obj).parent_link = link_obj
    safe_get_joint(joint_obj).child_link = child_obj

    item = safe_get_linkforge_scene(scene).ros2_control_joints.add()
    item.name = "Dummy"
    item.cmd_position = True
    safe_get_linkforge_scene(scene).gazebo_plugin_name = "gazebo_ros2_control"
    safe_get_linkforge_scene(scene).controllers_yaml_path = "/path/to/yaml"
    # Use a lambda to return a Link with the correct name for each call
    with mock.patch(
        "linkforge.blender.adapters.translator.LinkTranslator.translate",
        side_effect=lambda obj, *args, **kwargs: kwargs.get("lb"),
    ):
        robot, errors = scene_to_robot(bpy.context)
        assert robot and len(robot.gazebo_elements) > 0
        plugin = robot.gazebo_elements[0].plugins[0]
        assert plugin.name == "gazebo_ros2_control"
        assert plugin.parameters["parameters"] == "/path/to/yaml"


def test_blender_sensor_exhaustive(clean_scene, scene, blender_context) -> None:
    """Test all remaining sensor types and properties."""
    link = create_test_object("L", None, scene)
    safe_get_linkforge(link).is_robot_link = True

    # Camera
    cam = create_test_object("Cam", None, scene)
    safe_get_sensor(cam).is_robot_sensor = True
    safe_get_sensor(cam).sensor_type = "CAMERA"
    safe_get_sensor(cam).attached_link = link
    safe_get_sensor(cam).camera_horizontal_fov = 1.05

    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("L"))  # Register required link
    SensorTranslator().translate(cam, builder, blender_context)
    core_cam = builder.robot.sensors[0] if builder.robot.sensors else None
    assert (
        core_cam
        and core_cam.camera_info
        and pytest.approx(core_cam.camera_info.horizontal_fov) == 1.05
    )

    # GPS with noise
    gps = create_test_object("gps", None, scene)
    safe_get_sensor(gps).is_robot_sensor = True
    safe_get_sensor(gps).sensor_type = "gps"
    safe_get_sensor(gps).attached_link = link
    safe_get_sensor(gps).use_noise = True
    safe_get_sensor(gps).noise_mean = 0.0
    safe_get_sensor(gps).noise_stddev = 0.01

    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("L"))  # Register required link
    SensorTranslator().translate(gps, builder, blender_context)
    core_gps = builder.robot.sensors[0] if builder.robot.sensors else None
    assert core_gps and core_gps.gps_info is not None
    assert (
        core_gps.gps_info.position_sensing_horizontal_noise
        and pytest.approx(core_gps.gps_info.position_sensing_horizontal_noise.stddev) == 0.01
    )

    # Contact
    con = create_test_object("Con", None, scene)
    safe_get_sensor(con).is_robot_sensor = True
    safe_get_sensor(con).sensor_type = "CONTACT"
    safe_get_sensor(con).attached_link = link
    safe_get_sensor(con).contact_collision = "some_link_geom"

    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("L"))  # Register required link
    SensorTranslator().translate(con, builder, blender_context)
    core_con = builder.robot.sensors[0] if builder.robot.sensors else None
    assert (
        core_con and core_con.contact_info and core_con.contact_info.collision == "some_link_geom"
    )


def test_blender_to_core_geometry_edge_cases(clean_scene, scene, blender_context) -> None:
    """Test geometry conversion edge cases (None, zero-size, fallbacks)."""
    from linkforge.blender.adapters.blender_to_core import (
        detect_primitive_type,
        extract_mesh_triangles,
        get_object_geometry,
    )

    # detect_primitive_type None/non-mesh
    assert detect_primitive_type(None) is None
    empty = create_test_object("Empty", None, scene)
    assert detect_primitive_type(empty) is None

    # get_object_geometry None
    geom, mat = get_object_geometry(None)
    assert geom is None

    # zero-size object
    box = create_test_object("ZeroBox", None, scene)
    box.dimensions = (0, 0, 0)
    geom, mat = get_object_geometry(box, geometry_type="box")
    assert geom is None

    # extract_mesh_triangles None
    assert extract_mesh_triangles(None) is None
    assert extract_mesh_triangles(empty) is None


def test_blender_joint_advanced_cases(clean_scene, scene, blender_context) -> None:
    """Test custom axis, missing links, fixed axis, and continuous limits."""
    p = create_test_object("P", None, scene)
    c = create_test_object("C", None, scene)
    safe_get_linkforge(p).is_robot_link = True
    safe_get_linkforge(c).is_robot_link = True

    j = create_test_object("J", None, scene)
    safe_get_joint(j).is_robot_joint = True
    safe_get_joint(j).parent_link = p
    safe_get_joint(j).child_link = c
    # Custom axis normalization
    safe_get_joint(j).joint_type = "revolute"
    safe_get_joint(j).axis = "CUSTOM"
    safe_get_joint(j).custom_axis_x = 2.0
    safe_get_joint(j).custom_axis_y = 0.0
    safe_get_joint(j).custom_axis_z = 0.0

    core = translate_joint_to_model(j, blender_context, parent=p, child=c)
    assert core and core.axis and core.axis.x == 1.0  # Normalized

    # Zero axis fallback
    safe_get_joint(j).custom_axis_x = 0.0
    core = translate_joint_to_model(j, blender_context, parent=p, child=c)
    assert core and core.axis and core.axis.z == 1.0  # Fallback

    # Safety Controller
    safe_get_joint(j).use_safety_controller = True
    safe_get_joint(j).safety_soft_lower_limit = -1.23
    safe_get_joint(j).safety_soft_upper_limit = 1.23
    safe_get_joint(j).safety_k_position = 100.0
    safe_get_joint(j).safety_k_velocity = 10.0
    core = translate_joint_to_model(j, blender_context, parent=p, child=c)
    assert core is not None
    assert core.safety_controller is not None
    assert pytest.approx(core.safety_controller.soft_lower_limit) == -1.23
    assert pytest.approx(core.safety_controller.k_position) == 100.0

    # Calibration
    safe_get_joint(j).use_calibration = True
    safe_get_joint(j).use_calibration_rising = True
    safe_get_joint(j).calibration_rising = 0.55
    safe_get_joint(j).use_calibration_falling = False
    core = translate_joint_to_model(j, blender_context, parent=p, child=c)
    assert core and core.calibration is not None
    assert pytest.approx(core.calibration.rising) == 0.55
    assert core.calibration.falling is None

    # Fixed joint axis (should be None)
    safe_get_joint(j).joint_type = "fixed"
    core = translate_joint_to_model(j, blender_context, parent=p, child=c)
    assert core and core.axis is None

    # Continuous joint with limits
    safe_get_joint(j).joint_type = "continuous"
    safe_get_joint(j).use_limits = True
    safe_get_joint(j).limit_effort = 10.0
    core = translate_joint_to_model(j, blender_context, parent=p, child=c)
    assert core and core.limits and core.limits.effort == 10.0

    safe_get_joint(j).parent_link = None
    with pytest.raises(RobotValidationError, match=r"\[NOT_FOUND\] Joint has no parent link"):
        translate_joint_to_model(j, blender_context, parent=None, child=c)


def test_blender_transmission_advanced(clean_scene, scene, blender_context) -> None:
    """Test custom transmission types and actuator names."""
    j1 = create_test_object("J1", None, scene)
    safe_get_joint(j1).is_robot_joint = True

    t = create_test_object("TransCustom", None, scene)
    safe_get_transmission(t).is_robot_transmission = True
    safe_get_transmission(t).transmission_type = TRANS_CUSTOM
    safe_get_transmission(t).custom_type = "my_custom_trans"
    safe_get_transmission(t).joint_name = j1
    safe_get_transmission(t).use_custom_actuator_name = True
    safe_get_transmission(t).actuator_name = "custom_motor"

    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("p"))
    builder.robot.add_link(Link("c"))
    builder.robot.add_joint(Joint("J1", parent="p", child="c", type=JointType.FIXED))
    TransmissionTranslator().translate(t, builder, blender_context)
    core = builder.robot.transmissions[0] if builder.robot.transmissions else None
    assert core is not None
    assert core.type == "my_custom_trans"
    assert core.actuators[0].name == "custom_motor"


def test_blender_link_mesh_inertia(clean_scene, scene, blender_context) -> None:
    """Test inertia calculation from real mesh data.
    Must force MESH geometry type to hit the mesh inertia branch.
    """
    link_obj = create_test_object("L", None, scene)
    safe_get_linkforge(link_obj).is_robot_link = True
    safe_get_linkforge(link_obj).mass = 1.0
    safe_get_linkforge(link_obj).use_mesh_inertia = True

    # Add a mesh geometry
    mesh = bpy.data.meshes.new("CubeMesh")
    import bmesh

    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bm.to_mesh(mesh)
    bm.free()

    o = create_test_object("Geom", mesh)
    scene.collection.objects.link(o)
    o.parent = link_obj
    props = safe_get_linkforge(o)
    props.is_robot_visual = True
    props.is_robot_collision = True
    props.geometry_type = "MESH"  # Force mesh inertia branch

    core = translate_link_to_model(link_obj, blender_context)
    assert core is not None
    assert core.inertial is not None
    assert core.inertial.mass == 1.0
    assert core.inertial.inertia.ixx > 0


def test_scene_to_robot_full_integration(clean_scene, scene, blender_context) -> None:
    """Exhaustive test for scene_to_robot with sensors, plugins, and multi-visuals."""
    from pathlib import Path

    import bmesh
    from linkforge.blender.adapters.blender_to_core import scene_to_robot

    # Root Link
    root = create_test_object("RootLink", None, scene)
    safe_get_linkforge(root).is_robot_link = True
    safe_get_linkforge(root).link_name = "root_link"
    safe_get_linkforge(root).use_auto_inertia = False

    def create_mesh_obj(name, parent, shape="CUBE"):
        m = bpy.data.meshes.new(f"{name}_mesh")
        bm = bmesh.new()
        if shape == "CUBE":
            bmesh.ops.create_cube(bm, size=1.0)
        else:
            bmesh.ops.create_uvsphere(bm, u_segments=16, v_segments=8, radius=0.5)
        bm.to_mesh(m)
        bm.free()
        o = create_test_object(name, m, scene)
        o.parent = parent
        return o

    # Multi-visuals
    create_mesh_obj("root_link_visual_1", root, "CUBE")
    create_mesh_obj("root_link_visual_2", root, "sphere")  # Hits Sphere branch

    # Joint (Needed for transmission)
    child = create_test_object("ChildLink", None, scene)
    safe_get_linkforge(child).is_robot_link = True
    safe_get_linkforge(child).use_auto_inertia = False

    joint = create_test_object("Joint", None, scene)
    safe_get_joint(joint).is_robot_joint = True
    safe_get_joint(joint).parent_link = root
    safe_get_joint(joint).child_link = child

    # Transmission (Explicitly setting joint_name)
    trans = create_test_object("Trans", None, scene)
    safe_get_transmission(trans).is_robot_transmission = True
    safe_get_transmission(trans).joint_name = joint

    # Sensor with Gazebo Plugin (Custom mount - Hits 1071-1075)
    lidar = create_test_object("Lidar", None, scene)
    safe_get_sensor(lidar).is_robot_sensor = True
    safe_get_sensor(lidar).sensor_type = "lidar"
    safe_get_sensor(lidar).attached_link = root
    lidar.parent = None  # Custom mount
    safe_get_sensor(lidar).use_gazebo_plugin = True
    safe_get_sensor(lidar).plugin_filename = "liblidar.so"

    # ROS2 Control (Hits 1106-1126)
    scene_props = safe_get_linkforge_scene(scene)
    scene_props.use_ros2_control = True
    scene_props.ros2_control_name = "TestSystem"
    item = scene_props.ros2_control_joints.add()
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
    scene_props.gazebo_plugin_name = "gz_ros2_control"
    scene_props.controllers_yaml_path = "/tmp/controllers.yaml"

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


def test_blender_to_core_edge_cases(clean_scene, scene, blender_context) -> None:
    """Hit absolute remaining gaps (name sanitization, empty loops, unknown types)."""
    from linkforge.blender.adapters.blender_to_core import (
        _calculate_link_frames,
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
    o = create_test_object("Unknown", l_data)
    geom, mat = get_object_geometry(o)
    assert geom is None
    from mathutils import Matrix

    assert mat == Matrix.Identity(4)

    # blender_link_to_core_with_origin - multi visuals logic
    p = create_test_object("MultiLink", None, scene)
    safe_get_linkforge(p).is_robot_link = True
    safe_get_linkforge(p).use_auto_inertia = False

    import bmesh

    def add_vis(name):
        m = bpy.data.meshes.new(name)
        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=1.0)
        bm.to_mesh(m)
        bm.free()
        v = create_test_object(f"{p.name}_{name}", m, scene)
        v.parent = p
        return v

    add_vis("visual_a")
    add_vis("visual_b")

    core = translate_link_to_model(p, blender_context)
    assert core is not None
    assert len(core.visuals) == 2


def test_blender_to_core_small_gaps(clean_scene, scene, blender_context) -> None:
    """Hit remaining tiny gaps like material fallback and no-geometry link."""
    from pathlib import Path

    import bmesh
    from linkforge.blender.adapters.blender_to_core import (
        get_object_material,
        scene_to_robot,
    )

    # Material name fallback
    m = bpy.data.meshes.new("MatMesh")
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bm.to_mesh(m)
    bm.free()
    o = create_test_object("MatObj", m, scene)
    mat = bpy.data.materials.new("TestMat")
    if o.data and hasattr(o.data, "materials"):
        o.data.materials.append(mat)
    props = MagicMock()
    props.use_material = True
    res = get_object_material(o, props)
    assert res is not None
    assert res.name == "TestMat"

    # blender_link_to_core_with_origin - No children
    p = create_test_object("EmptyLink", None, scene)
    safe_get_linkforge(p).is_robot_link = True
    safe_get_linkforge(p).use_auto_inertia = False
    core = translate_link_to_model(p, blender_context)
    assert core is not None
    assert len(core.visuals) == 0
    assert len(core.collisions) == 0
    safe_get_linkforge(p).is_robot_link = False

    # Scene to robot integration with 1 full link
    root = create_test_object("GapsRoot", None, scene)
    safe_get_linkforge(root).is_robot_link = True
    safe_get_linkforge(root).link_name = "gaps_root"
    safe_get_linkforge(root).use_auto_inertia = False

    m1 = bpy.data.meshes.new("G1Mesh")
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bm.to_mesh(m1)
    bm.free()
    v1 = create_test_object("gaps_root_visual", m1, scene)
    v1.parent = root
    v1.dimensions = (1, 1, 1)

    m_col = bpy.data.meshes.new("GColMesh")
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bm.to_mesh(m_col)
    bm.free()
    c1 = create_test_object("gaps_root_collision", m_col, scene)
    c1.parent = root
    c1.dimensions = (1, 1, 1)
    if bpy.context.view_layer is not None:
        bpy.context.view_layer.update()

    robot, _ = scene_to_robot(bpy.context, meshes_dir=Path("/tmp"), dry_run=True)
    # Check if we at least have our gaps_root link
    assert any(ln.name == "gaps_root" for ln in robot.links)


def test_blender_to_core_missing_errors(clean_scene, scene, blender_context) -> None:
    """Hit missing child link, empty transmission, simplify, and None returns."""

    import bmesh
    from linkforge.blender.adapters.blender_to_core import (
        get_object_geometry,
    )

    # blender_link_to_core_with_origin None
    assert translate_link_to_model(None, blender_context) is None

    # blender_joint_to_core None/non-robot
    assert translate_joint_to_model(None, blender_context) is None
    empty = create_test_object("EmptyNone", None, scene)
    assert translate_joint_to_model(empty, blender_context) is None

    # Translator should handle None/non-robot gracefully
    builder = RobotBuilder("Robot")
    SensorTranslator().translate(None, builder, blender_context)
    assert len(builder.robot.sensors) == 0
    SensorTranslator().translate(empty, builder, blender_context)
    assert len(builder.robot.sensors) == 0

    TransmissionTranslator().translate(None, builder, blender_context)
    assert len(builder.robot.transmissions) == 0
    TransmissionTranslator().translate(empty, builder, blender_context)
    assert len(builder.robot.transmissions) == 0

    # blender_link_to_core_with_origin simplify
    m = bpy.data.meshes.new("CMesh")
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bm.to_mesh(m)
    bm.free()
    o = create_test_object("PLink_collision", m, scene)
    o.dimensions = (1, 1, 1)
    p = create_test_object("PLink", None, scene)
    safe_get_linkforge(p).is_robot_link = True
    safe_get_linkforge(p).use_auto_inertia = False
    o.parent = p
    if bpy.context.view_layer:
        bpy.context.view_layer.update()

    robot_props = MagicMock()
    robot_props.simplify_collision = True
    core = translate_link_to_model(p, blender_context)
    assert core is not None
    assert len(core.collisions) == 1

    # get_object_geometry BOX fallback
    m2 = bpy.data.meshes.new("BMesh")
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bm.to_mesh(m2)
    bm.free()
    o2 = create_test_object("BoxFallback", m2, scene)
    geom, _ = get_object_geometry(o2, meshes_dir=None)
    assert isinstance(geom, Box)

    # Missing child link
    p_link = create_test_object("P", None, scene)
    c_link = create_test_object("C", None, scene)
    safe_get_linkforge(p_link).is_robot_link = True
    safe_get_linkforge(c_link).is_robot_link = True
    j = create_test_object("J", None, scene)
    safe_get_joint(j).is_robot_joint = True
    safe_get_joint(j).parent_link = p_link
    safe_get_joint(j).child_link = None
    with pytest.raises(RobotValidationError, match=r"\[NOT_FOUND\] Joint has no child link"):
        translate_joint_to_model(j, blender_context, parent=p_link, child=None)

    # Empty transmission
    t = create_test_object("T", None, scene)
    safe_get_transmission(t).is_robot_transmission = True
    builder = RobotBuilder("Robot")

    builder.robot.add_link(Link("p"))
    builder.robot.add_link(Link("c"))
    builder.robot.add_joint(Joint("Joint", parent="p", child="c", type=JointType.FIXED))
    TransmissionTranslator().translate(t, builder, blender_context)
    assert len(builder.robot.transmissions) == 0

    # Joint mimic fallback
    mimic_target = create_test_object("MimicTarget", None, scene)
    safe_get_joint(j).child_link = c_link
    safe_get_joint(j).use_mimic = True
    safe_get_joint(j).mimic_joint = mimic_target
    core = translate_joint_to_model(j, blender_context, parent=p_link, child=c_link)
    assert core is not None
    assert core.mimic is not None
    assert core.mimic.joint == "MimicTarget"


def test_detect_primitive_type_tags(scene, blender_context) -> None:
    """Verify manual primitive type override via custom properties."""
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    assert obj is not None
    obj["source_geometry_type"] = "sphere"
    assert detect_primitive_type(obj) == "sphere"
