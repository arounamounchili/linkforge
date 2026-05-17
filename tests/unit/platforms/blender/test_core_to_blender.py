from pathlib import Path
from unittest import mock

import bpy
import pytest
from linkforge.blender.adapters.context import BlenderContext
from linkforge.blender.adapters.core_to_blender import (
    create_joint_object,
    create_link_object,
    create_material_from_color,
    create_primitive_mesh,
    create_sensor_object,
    import_mesh_file,
    import_robot_to_scene,
    normalize_and_consolidate_imported_objects,
)
from linkforge.core import (
    Box,
    CameraInfo,
    Collision,
    Color,
    Cylinder,
    GazeboElement,
    GazeboPlugin,
    GPSInfo,
    IMUInfo,
    Inertial,
    Joint,
    JointCalibration,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointSafetyController,
    JointType,
    LidarInfo,
    Link,
    LinkPhysics,
    Material,
    Mesh,
    Robot,
    Ros2Control,
    Ros2ControlJoint,
    Sensor,
    SensorNoise,
    SensorType,
    Sphere,
    Transform,
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    Vector3,
    Visual,
)

from tests.blender_test_utils import (
    create_test_object,
    safe_get_joint,
    safe_get_linkforge,
    safe_get_linkforge_scene,
    safe_get_sensor,
)


def test_create_primitive_mesh_box(scene, blender_context) -> None:
    """Test creation of a real Blender Cube from Box model."""
    box = Box(size=Vector3(1, 2, 3))
    obj = create_primitive_mesh(blender_context, box, "test_box_unique")

    assert obj is not None
    assert obj.name == "test_box_unique"
    assert obj.type == "MESH"
    assert pytest.approx(obj.dimensions.x) == 1.0
    assert pytest.approx(obj.dimensions.y) == 2.0
    assert pytest.approx(obj.dimensions.z) == 3.0


def test_create_link_object_multi_collisions(scene, blender_context) -> None:
    """Test link creation with multiple collision elements."""
    coll1 = Collision(geometry=Box(size=Vector3(1, 1, 1)), name="c1")
    coll2 = Collision(geometry=Sphere(radius=0.5), name="c2")
    link = Link(name="multi_coll_link", collisions=[coll1, coll2])

    robot = Robot(name="test")
    obj = create_link_object(blender_context, link, robot, Path("/tmp"))
    assert obj is not None

    collisions = [c for c in obj.children if "_collision" in c.name]
    assert len(collisions) == 2
    assert any("c1" in c.name for c in collisions)
    assert any("c2" in c.name for c in collisions)


def test_create_link_object_zero_mass(scene, blender_context) -> None:
    """Test link creation with no inertial properties (should default to 0 mass)."""
    link = Link(name="dummy_link")  # No inertial
    robot = Robot(name="test")
    obj = create_link_object(blender_context, link, robot, Path("/tmp"))
    assert obj is not None

    assert safe_get_linkforge(obj).mass == 0.0
    assert safe_get_linkforge(obj).use_auto_inertia is False


def test_create_link_object_physics(scene, blender_context) -> None:
    """Verify that importing a Link with LinkPhysics correctly sets Blender properties."""
    physics = LinkPhysics(mu=0.7, kp=2.0e10, kd=100.0)
    link = Link(name="physics_link_import", physics=physics)
    robot = Robot(name="test")

    obj = create_link_object(blender_context, link, robot, Path("/tmp"))
    assert obj is not None

    # Verify Blender properties
    props = safe_get_linkforge(obj)
    assert pytest.approx(props.mu) == 0.7
    assert pytest.approx(props.kp) == 2.0e10
    assert pytest.approx(props.kd) == 100.0


def test_create_link_object_physics_toggle(scene, blender_context) -> None:
    """Verify that use_simulation_props is only True if physics are non-default."""
    robot = Robot(name="test")

    # 1. Default Physics -> Toggle should be FALSE
    link_default = Link(name="default_link", physics=LinkPhysics())
    obj_def = create_link_object(blender_context, link_default, robot, Path("/tmp"))
    assert safe_get_linkforge(obj_def).use_simulation_props is False

    # 2. Modified Physics -> Toggle should be TRUE
    phys_mod = LinkPhysics(mu=0.5)  # Modified mu
    link_mod = Link(name="mod_link", physics=phys_mod)
    obj_mod = create_link_object(blender_context, link_mod, robot, Path("/tmp"))
    assert safe_get_linkforge(obj_mod).use_simulation_props is True
    assert pytest.approx(safe_get_linkforge(obj_mod).mu) == 0.5

    # 3. Explicit Gazebo Element -> Toggle should be TRUE even if physics are default
    link_gz = Link(name="gz_link", physics=LinkPhysics())
    gz_elem = GazeboElement(reference="gz_link", static=True)  # Has static property
    robot_gz = Robot(name="test_gz", links=[link_gz], gazebo_elements=[gz_elem])
    obj_gz = create_link_object(blender_context, link_gz, robot_gz, Path("/tmp"))
    assert safe_get_linkforge(obj_gz).use_simulation_props is True


def test_create_link_object_primitives(scene, blender_context) -> None:
    """Test creating a Link object with multiple primitive visuals and collisions."""
    # Setup Core Link
    box_geom = Box(size=Vector3(1, 1, 1))
    sphere_geom = Sphere(radius=0.5)

    visual = Visual(geometry=box_geom, origin=Transform(xyz=Vector3(1, 0, 0)))
    collision = Collision(geometry=sphere_geom, origin=Transform(xyz=Vector3(0, 1, 0)))

    link = Link(
        name="test_link_p",
        visuals=[visual],
        collisions=[collision],
        inertial=Inertial(mass=1.0),
    )

    # Build in Blender
    collection = bpy.data.collections.new("TestCol")
    scene.collection.children.link(collection)

    robot = Robot(name="test")
    obj = create_link_object(blender_context, link, robot, Path("/tmp"), collection=collection)

    # Verify
    assert obj is not None
    assert obj.name == "test_link_p"

    # Verify relative positioning
    visual_obj = next(c for c in obj.children if "_visual" in c.name)
    assert pytest.approx(visual_obj.location.x) == 1.0


def test_create_joint_object_fixed(scene, blender_context) -> None:
    """Test creation of a Joint object in Blender."""
    # Setup Link objects
    parent_obj = create_test_object("parent_l", None, scene)
    child_obj = create_test_object("child_l", None, scene)
    child_obj.location = (1, 0, 0)

    link_objects = {"parent_l": parent_obj, "child_l": child_obj}

    # Setup Core Joint
    joint = Joint(
        name="test_j",
        type=JointType.FIXED,
        parent="parent_l",
        child="child_l",
        origin=Transform(xyz=Vector3(0.5, 0, 0)),
    )

    # Build in Blender
    joint_obj = create_joint_object(blender_context, joint, link_objects)

    # Verify
    assert joint_obj is not None
    assert joint_obj.parent == parent_obj
    assert pytest.approx(joint_obj.location.x) == 0.5


def test_create_joint_object_complex(scene, blender_context) -> None:
    """Test creation of a revolute Joint with limits and axis in Blender."""
    # Setup Links
    parent_obj = create_test_object("p_link", None, scene)
    child_obj = create_test_object("c_link", None, scene)
    child_obj.location = (0, 0, 1)

    link_objects = {"p_link": parent_obj, "c_link": child_obj}

    # Setup Core Joint
    joint = Joint(
        name="rev_joint",
        type=JointType.REVOLUTE,
        parent="p_link",
        child="c_link",
        axis=Vector3(0, 0, 1),
        limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=1.0),
        dynamics=JointDynamics(damping=0.1, friction=0.05),
    )

    # Build
    joint_obj = create_joint_object(blender_context, joint, link_objects)

    # Verify properties
    assert joint_obj is not None
    props = safe_get_joint(joint_obj)
    assert props.joint_type == "revolute"
    assert props.axis == "Z"
    assert props.use_limits is True
    assert pytest.approx(props.limit_lower) == -1.57
    assert pytest.approx(props.limit_upper) == 1.57
    assert props.use_dynamics is True
    assert pytest.approx(props.dynamics_damping) == 0.1


def test_create_joint_object_advanced_props(scene, blender_context) -> None:
    """Verify that safety controller and calibration are correctly synced to Blender properties."""

    # Setup Links
    p_obj = create_test_object("p_link_adv", None, scene)
    c_obj = create_test_object("c_link_adv", None, scene)
    link_objects = {"p_link_adv": p_obj, "c_link_adv": c_obj}

    # Setup Core Joint
    safety = JointSafetyController(
        soft_lower_limit=-1.0, soft_upper_limit=1.0, k_position=100.0, k_velocity=10.0
    )
    calib = JointCalibration(rising=0.5)
    joint = Joint(
        name="adv_j",
        type=JointType.REVOLUTE,
        parent="p_link_adv",
        child="c_link_adv",
        axis=Vector3(1, 0, 0),
        limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=1.0),
        safety_controller=safety,
        calibration=calib,
    )

    # Build in Blender
    joint_obj = create_joint_object(blender_context, joint, link_objects)

    # Verify properties
    assert joint_obj is not None
    props = safe_get_joint(joint_obj)
    assert props.use_safety_controller is True
    assert props.safety_soft_lower_limit == -1.0
    assert props.safety_k_position == 100.0

    assert props.use_calibration is True
    assert props.use_calibration_rising is True
    assert props.calibration_rising == 0.5
    assert props.use_calibration_falling is False


def test_import_robot_to_scene_full(scene, blender_context) -> None:
    """Test the full robot import entry point with a simple chain."""
    # Create Robot Model
    l1 = Link(name="base_link")
    l2 = Link(name="tool_link")
    j1 = Joint(name="j1", type=JointType.FIXED, parent="base_link", child="tool_link")

    # Correct Robot instantiation using links/joints
    robot = Robot(name="mini_robot", links=[l1, l2], joints=[j1])

    # Import
    result = import_robot_to_scene(robot, Path("dummy.urdf"), blender_context)

    # Verify
    assert result is True
    assert "base_link" in bpy.data.objects
    assert "tool_link" in bpy.data.objects
    assert "j1" in bpy.data.objects

    # Check hierarchy
    base = bpy.data.objects["base_link"]
    joint = bpy.data.objects["j1"]
    tool = bpy.data.objects["tool_link"]

    assert joint.parent == base
    assert tool.parent == joint

    # Check collection creation
    assert "mini_robot" in bpy.data.collections


def test_import_robot_complex_tree(scene, blender_context) -> None:
    """Test importing a robot with a multi-branch tree structure."""
    l1 = Link(name="root")
    l2 = Link(name="branch1")
    l3 = Link(name="branch2")
    l4 = Link(name="leaf")

    j1 = Joint(
        name="j1",
        parent="root",
        child="branch1",
        type=JointType.FIXED,
        origin=Transform(xyz=Vector3(1, 0, 0)),
    )
    j2 = Joint(
        name="j2",
        parent="root",
        child="branch2",
        type=JointType.FIXED,
        origin=Transform(xyz=Vector3(0, 1, 0)),
    )
    j3 = Joint(
        name="j3",
        parent="branch1",
        child="leaf",
        type=JointType.FIXED,
        origin=Transform(xyz=Vector3(0, 0, 1)),
    )

    robot = Robot(name="tree_bot", links=[l1, l2, l3, l4], joints=[j1, j2, j3])

    # Needs source_path and context. Returns bool.
    success = import_robot_to_scene(robot, Path("/tmp/robot.urdf"), blender_context)
    assert success is True
    # Check hierarchy
    root_obj = bpy.data.objects.get("root")
    branch1_obj = bpy.data.objects.get("branch1")
    branch2_obj = bpy.data.objects.get("branch2")
    leaf_obj = bpy.data.objects.get("leaf")

    assert branch1_obj is not None
    assert branch2_obj is not None
    assert leaf_obj is not None

    assert branch1_obj.parent is not None
    assert branch1_obj.parent.parent == root_obj  # Link -> Joint -> Parent Link
    assert branch2_obj.parent is not None
    assert branch2_obj.parent.parent == root_obj
    assert leaf_obj.parent is not None
    assert leaf_obj.parent.parent == branch1_obj


def test_import_robot_with_ros2_control_and_gazebo(scene, blender_context) -> None:
    """Verify that ros2_control and Gazebo settings are synced to scene properties."""
    l1 = Link(name="l1")
    l2 = Link(name="l2")
    j1 = Joint(name="j1", type=JointType.FIXED, parent="l1", child="l2")

    # Setup ros2_control
    rc_joint = Ros2ControlJoint(
        name="j1", command_interfaces=["position"], state_interfaces=["position"]
    )
    rc = Ros2Control(name="RealRobot", type="system", hardware_plugin="fake_hw", joints=[rc_joint])

    # Setup Gazebo
    plugin = GazeboPlugin(
        name="gazebo_ros2_control",
        filename="libgazebo_ros2_control.so",
        parameters={"parameters": "config.yaml"},
    )
    gazebo = GazeboElement(plugins=[plugin])

    robot = Robot(
        name="ctrl_bot",
        links=[l1, l2],
        joints=[j1],
        ros2_controls=[rc],
        gazebo_elements=[gazebo],
    )

    success = import_robot_to_scene(robot, Path("/tmp/robot.urdf"), blender_context)
    assert success is True

    # Verify scene properties
    lp = safe_get_linkforge_scene(scene)
    assert lp.use_ros2_control is True
    assert lp.ros2_control_name == "RealRobot"
    assert len(lp.ros2_control_joints) == 1
    assert lp.controllers_yaml_path == "config.yaml"


def test_create_sensor_object_lidar(scene, blender_context) -> None:
    """Test creation of a LIDAR sensor in Blender."""
    # Setup parent link object
    link_obj = create_test_object("base_link", None, scene)
    link_objects = {"base_link": link_obj}

    # Setup Sensor Model
    sensor = Sensor(
        name="top_lidar",
        type=SensorType.LIDAR,
        link_name="base_link",
        lidar_info=LidarInfo(),
        origin=Transform(xyz=Vector3(0, 0, 0.5)),
    )

    # Build
    sensor_obj = create_sensor_object(blender_context, sensor, link_objects)

    # Verify
    assert sensor_obj is not None
    assert sensor_obj.parent == link_obj
    assert pytest.approx(sensor_obj.location.z) == 0.5
    props = safe_get_sensor(sensor_obj)
    assert props.is_robot_sensor is True
    assert props.sensor_type == "lidar"


def test_create_sensor_object_imu_gps_camera(scene, blender_context) -> None:
    """Verify that IMU, GPS, and Camera sensors are correctly created in Blender."""
    # Setup a dummy link for sensors to attach to
    link_obj = create_test_object("base_link", None, scene)
    safe_get_linkforge(link_obj).is_robot_link = True
    link_objects = {"base_link": link_obj}

    # IMU
    imu = Sensor(name="imu_sensor", type=SensorType.IMU, link_name="base_link", imu_info=IMUInfo())
    obj_imu = create_sensor_object(blender_context, imu, link_objects)
    assert obj_imu is not None
    assert safe_get_sensor(obj_imu).sensor_type == "imu"

    # GPS
    gps = Sensor(name="gps_sensor", type=SensorType.GPS, link_name="base_link", gps_info=GPSInfo())
    obj_gps = create_sensor_object(blender_context, gps, link_objects)
    assert obj_gps is not None
    assert safe_get_sensor(obj_gps).sensor_type == "gps"

    # Camera
    cam = Sensor(
        name="cam_sensor", type=SensorType.CAMERA, link_name="base_link", camera_info=CameraInfo()
    )
    obj_cam = create_sensor_object(blender_context, cam, link_objects)
    assert obj_cam is not None
    assert safe_get_sensor(obj_cam).sensor_type == "camera"


def test_import_robot_with_mimic(scene, blender_context) -> None:
    """Test full robot import with mimic joint resolution."""
    l1 = Link(name="base")
    l2 = Link(name="j_parent")
    l3 = Link(name="j_mimic")

    j1 = Joint(
        name="driver",
        type=JointType.REVOLUTE,
        parent="base",
        child="j_parent",
        axis=Vector3(0, 0, 1),
        limits=JointLimits(0, 1, 10, 1),
    )
    j2 = Joint(
        name="follower",
        type=JointType.REVOLUTE,
        parent="base",
        child="j_mimic",
        axis=Vector3(0, 0, 1),
        limits=JointLimits(0, 1, 10, 1),
        mimic=JointMimic(joint="driver", multiplier=2.0),
    )

    robot = Robot(name="mimic_bot", links=[l1, l2, l3], joints=[j1, j2])

    # Import
    import_robot_to_scene(robot, Path("dummy.urdf"), blender_context)

    # Verify mimic link
    follower = bpy.data.objects["follower"]
    driver = bpy.data.objects["driver"]
    props = safe_get_joint(follower)
    assert props.use_mimic is True
    assert props.mimic_joint == driver
    assert pytest.approx(props.mimic_multiplier) == 2.0


def test_import_mesh_file_stl(tmp_path, scene, blender_context) -> None:
    """Test importing a real STL mesh file."""
    # Create a dummy STL file using Blender
    bpy.ops.mesh.primitive_cube_add()
    stl_path = tmp_path / "test.stl"

    # In modern Blender (4.0+), use wm.stl_export.
    # Use getattr for static analysis compatibility.
    wm_ops = getattr(bpy.ops, "wm")
    if hasattr(wm_ops, "stl_export"):
        wm_ops.stl_export(filepath=str(stl_path))
    else:
        # Fallback for legacy STL addon if present
        export_ops = getattr(bpy.ops, "export_mesh", None)
        if export_ops and hasattr(export_ops, "stl"):
            export_ops.stl(filepath=str(stl_path))
        else:
            pytest.skip("No STL exporter found in this Blender environment")

    bpy.ops.object.delete()  # Cleanup cube

    # Import it
    obj = import_mesh_file(blender_context, stl_path, "imported_stl")

    # Verify
    assert obj is not None
    assert obj.name == "imported_stl"
    assert obj.type == "MESH"


def test_create_joint_object_prismatic(scene, blender_context) -> None:
    """Test creation of a PRISMATIC joint with limits and CUSTOM axis."""
    p_obj = create_test_object("p", None, scene)
    c_obj = create_test_object("c", None, scene)

    link_objects = {"p": p_obj, "c": c_obj}

    # Use highly precise normalization to satisfy SYLVESTER_TOLERANCE_EPSILON (1e-9)
    joint = Joint(
        name="prism_j",
        type=JointType.PRISMATIC,
        parent="p",
        child="c",
        axis=Vector3(0.70710678118, 0.70710678118, 0.0),
        limits=JointLimits(lower=0, upper=1.0, effort=10, velocity=1),
    )

    obj = create_joint_object(blender_context, joint, link_objects)
    assert obj is not None
    props = safe_get_joint(obj)
    assert props.joint_type == "prismatic"
    assert props.axis == "CUSTOM"
    # Expect normalized 1/sqrt(2) approx 0.707
    assert pytest.approx(props.custom_axis_x) == 0.70710678
    assert pytest.approx(props.custom_axis_y) == 0.70710678


def test_create_joint_object_continuous_floating(scene, blender_context) -> None:
    """Test creation of CONTINUOUS and FLOATING joints."""
    p_obj = create_test_object("p_c", None, scene)
    c_obj = create_test_object("c_c", None, scene)
    link_objects = {"p_c": p_obj, "c_c": c_obj}

    j_cont = Joint(
        name="cont", type=JointType.CONTINUOUS, parent="p_c", child="c_c", axis=Vector3(1, 0, 0)
    )
    obj_cont = create_joint_object(blender_context, j_cont, link_objects)
    assert obj_cont is not None
    assert safe_get_joint(obj_cont).joint_type == "continuous"

    # Floating
    j_float = Joint(name="float_j", type=JointType.FLOATING, parent="p_c", child="c_c")
    obj_float = create_joint_object(blender_context, j_float, link_objects)
    assert obj_float is not None
    assert safe_get_joint(obj_float).joint_type == "floating"


def test_create_link_object_with_mesh_visual(tmp_path, scene, blender_context) -> None:
    """Test creating a link that uses a mesh file for visual."""
    # Create dummy STL
    bpy.ops.mesh.primitive_uv_sphere_add()
    mesh_path = tmp_path / "v.stl"
    wm_ops = getattr(bpy.ops, "wm")
    if hasattr(wm_ops, "stl_export"):
        wm_ops.stl_export(filepath=str(mesh_path))
    else:
        export_ops = getattr(bpy.ops, "export_mesh", None)
        if export_ops and hasattr(export_ops, "stl"):
            export_ops.stl(filepath=str(mesh_path))
        else:
            pytest.skip("No STL exporter found")
    bpy.ops.object.delete()

    # Model
    mesh_geom = Mesh(resource="v.stl")
    visual = Visual(geometry=mesh_geom)
    link = Link(name="mesh_link", visuals=[visual])

    # Build (providing tmp_path as source_directory)
    robot = Robot(name="test")
    obj = create_link_object(blender_context, link, robot, tmp_path)

    # Verify
    assert obj is not None
    visual_obj = next(c for c in obj.children if "_visual" in c.name)
    assert visual_obj.type == "MESH"


def test_create_material_existing(scene, blender_context) -> None:
    """Test that existing materials are reused."""
    mat_name = "ExistingMat"
    mat = bpy.data.materials.new(name=mat_name)
    color = Color(1, 0, 0, 1)

    mat_out = create_material_from_color(blender_context, color, mat_name)
    assert mat_out == mat


def test_import_mesh_file_unsupported(scene, blender_context) -> None:
    """Test handling of unsupported mesh formats."""
    obj = import_mesh_file(blender_context, Path("unsupported.txt"), "test")
    assert obj is None


def test_create_primitive_mesh_invalid(scene, blender_context) -> None:
    """Test handling of unsupported geometry types."""
    obj = create_primitive_mesh(blender_context, None, "test")  # type: ignore
    assert obj is None


def test_create_sensor_with_gazebo_plugin(scene, blender_context) -> None:
    """Test that Gazebo plugins are preserved during import (no legacy filtering)."""
    link_obj = create_test_object("base", None, scene)
    link_objects = {"base": link_obj}

    # Create sensor with legacy plugin
    plugin = GazeboPlugin(
        name="gazebo_ros_camera", filename="libgazebo_ros_camera.so", raw_xml="<plugin>...</plugin>"
    )
    sensor = Sensor(
        name="cam",
        type=SensorType.CAMERA,
        link_name="base",
        camera_info=CameraInfo(),
        plugin=plugin,
    )

    obj = create_sensor_object(blender_context, sensor, link_objects)
    assert obj is not None
    # All plugins should now be preserved (no legacy filtering)
    props = safe_get_sensor(obj)
    assert props.use_gazebo_plugin is True
    assert props.plugin_filename == "libgazebo_ros_camera.so"


def test_create_sensor_with_custom_plugin(scene, blender_context) -> None:
    """Test that custom (non-legacy) plugins are preserved."""
    link_obj = create_test_object("base", None, scene)
    link_objects = {"base": link_obj}

    # Create sensor with custom plugin (not libgazebo_ros_*)
    plugin = GazeboPlugin(
        name="my_custom_plugin", filename="libmy_custom.so", raw_xml="<plugin>...</plugin>"
    )
    sensor = Sensor(
        name="custom_sensor",
        type=SensorType.LIDAR,
        link_name="base",
        lidar_info=LidarInfo(),
        plugin=plugin,
    )

    obj = create_sensor_object(blender_context, sensor, link_objects)
    assert obj is not None
    # Custom plugin should be preserved
    props = safe_get_sensor(obj)
    assert props.use_gazebo_plugin is True
    assert props.plugin_filename == "libmy_custom.so"


def test_import_robot_with_legacy_transmissions_skipped(scene, blender_context) -> None:
    """Test that legacy transmissions are skipped (no auto-conversion)."""

    l1 = Link(name="base")
    l2 = Link(name="arm")
    j1 = Joint(
        name="shoulder",
        type=JointType.REVOLUTE,
        parent="base",
        child="arm",
        axis=Vector3(0, 0, 1),
        limits=JointLimits(0, 1, 10, 1),
    )

    # Create legacy transmission
    trans = Transmission(
        name="trans1",
        type="transmission_interface/SimpleTransmission",
        joints=[
            TransmissionJoint(
                name="shoulder", hardware_interfaces=["hardware_interface/PositionJointInterface"]
            )
        ],
        actuators=[
            TransmissionActuator(
                name="motor1", hardware_interfaces=["hardware_interface/PositionJointInterface"]
            )
        ],
    )

    robot = Robot(
        name="legacy_bot",
        links=[l1, l2],
        joints=[j1],
        transmissions=[trans],
    )

    # Import
    success = import_robot_to_scene(robot, Path("/tmp/robot.urdf"), blender_context)
    assert success is True

    # Auto-conversion is now disabled/removed.
    scene_props = safe_get_linkforge_scene(scene)
    assert scene_props.use_ros2_control is False
    assert len(scene_props.ros2_control_joints) == 0


def test_import_robot_skips_transmissions_when_ros2_control_exists(scene, blender_context) -> None:
    """Test that transmissions are skipped when ros2_control is present."""

    l1 = Link(name="base")
    l2 = Link(name="arm")
    j1 = Joint(
        name="j1",
        type=JointType.REVOLUTE,
        parent="base",
        child="arm",
        axis=Vector3(1, 0, 0),
        limits=JointLimits(0, 1, 10, 1),
    )

    # Both ros2_control and transmission
    rc_joint = Ros2ControlJoint(
        name="j1", command_interfaces=["position"], state_interfaces=["position"]
    )
    rc = Ros2Control(name="RealRobot", type="system", hardware_plugin="fake_hw", joints=[rc_joint])

    trans = Transmission(
        name="trans1",
        type="transmission_interface/SimpleTransmission",
        joints=[
            TransmissionJoint(
                name="j1", hardware_interfaces=["hardware_interface/PositionJointInterface"]
            )
        ],
        actuators=[
            TransmissionActuator(
                name="motor1", hardware_interfaces=["hardware_interface/PositionJointInterface"]
            )
        ],
    )

    robot = Robot(
        name="hybrid_bot",
        links=[l1, l2],
        joints=[j1],
        ros2_controls=[rc],
        transmissions=[trans],
    )

    success = import_robot_to_scene(robot, Path("/tmp/robot.urdf"), blender_context)
    assert success is True

    # ros2_control should take priority, transmission should be skipped
    assert len(safe_get_linkforge_scene(scene).ros2_control_joints) == 1


def test_create_link_with_material(scene, blender_context) -> None:
    """Test link creation with visual material."""

    color = Color(r=1.0, g=0.0, b=0.0, a=1.0)
    material = Material(name="RedMat", color=color)
    visual = Visual(geometry=Box(size=Vector3(1, 1, 1)), material=material)
    link = Link(name="colored_link", visuals=[visual])

    robot = Robot(name="test")
    obj = create_link_object(blender_context, link, robot, Path("/tmp"))

    assert obj is not None
    assert safe_get_linkforge(obj).use_material is True
    # Check that visual child has material
    visual_obj = next(c for c in obj.children if "_visual" in c.name)
    assert visual_obj.type == "MESH"
    assert visual_obj.data is not None
    assert hasattr(visual_obj.data, "materials") and len(getattr(visual_obj.data, "materials")) > 0


def test_create_joint_with_custom_axis(scene, blender_context) -> None:
    """Test joint creation with non-standard axis."""
    p_obj = create_test_object("p", None, scene)
    c_obj = create_test_object("c", None, scene)

    link_objects = {"p": p_obj, "c": c_obj}

    # Custom axis (not X, Y, or Z)
    joint = Joint(
        name="custom_j",
        type=JointType.REVOLUTE,
        parent="p",
        child="c",
        axis=Vector3(0.5, 0.5, 0.70710678),
        limits=JointLimits(0, 1, 10, 1),
    )

    obj = create_joint_object(blender_context, joint, link_objects)
    assert obj is not None
    props = safe_get_joint(obj)
    assert props.axis == "CUSTOM"
    assert props.custom_axis_x != 0.0


def test_import_mesh_file_nonexistent(scene, blender_context) -> None:
    """Test handling of non-existent mesh files."""
    obj = import_mesh_file(blender_context, Path("/nonexistent/file.stl"), "test")
    assert obj is None


def test_create_link_with_collision_mesh(tmp_path, scene, blender_context) -> None:
    """Test link creation with mesh collision geometry."""
    # Create dummy STL
    bpy.ops.mesh.primitive_cube_add()
    mesh_path = tmp_path / "collision.stl"
    wm_ops = getattr(bpy.ops, "wm")
    if hasattr(wm_ops, "stl_export"):
        wm_ops.stl_export(filepath=str(mesh_path))
    else:
        export_ops = getattr(bpy.ops, "export_mesh", None)
        if export_ops and hasattr(export_ops, "stl"):
            export_ops.stl(filepath=str(mesh_path))
        else:
            pytest.skip("No STL exporter found")
    bpy.ops.object.delete()

    # Model
    mesh_geom = Mesh(resource="collision.stl")
    collision = Collision(geometry=mesh_geom)
    link = Link(name="mesh_coll_link", collisions=[collision])

    # Build
    robot = Robot(name="test")
    obj = create_link_object(blender_context, link, robot, tmp_path)

    # Verify
    assert obj is not None
    coll_obj = next(c for c in obj.children if "_collision" in c.name)
    assert coll_obj["imported_from_source"] is True
    assert coll_obj["collision_geometry_type"] == "mesh"
    assert safe_get_linkforge(obj).collision_quality == 100.0


def test_create_primitive_mesh_cylinder_sphere(scene, blender_context) -> None:
    """Test creation of Cylinder and Sphere primitives."""
    # Cylinder
    cyl = Cylinder(radius=0.5, length=2.0)
    obj_cyl = create_primitive_mesh(blender_context, cyl, "test_cyl")
    assert obj_cyl is not None
    assert obj_cyl["source_geometry_type"] == "cylinder"
    assert pytest.approx(obj_cyl.dimensions.z) == 2.0

    # Sphere
    sphere = Sphere(radius=1.0)
    obj_sphere = create_primitive_mesh(blender_context, sphere, "test_sphere")
    assert obj_sphere is not None
    assert obj_sphere["source_geometry_type"] == "sphere"
    assert pytest.approx(obj_sphere.dimensions.x) == 2.0


def test_sensor_unknown_link(scene, blender_context) -> None:
    """Test sensor creation with unknown link name."""
    sensor = Sensor(
        name="orphan_sensor",
        type=SensorType.CAMERA,
        link_name="nonexistent_link",
        camera_info=CameraInfo(),
    )

    obj = create_sensor_object(blender_context, sensor, {})
    assert obj is None


def test_joint_axis_standard_axes(scene, blender_context) -> None:
    """Test joint axis detection for standard X, Y, Z axes."""
    bpy.ops.object.empty_add()
    p_obj = bpy.context.active_object
    assert p_obj is not None
    p_obj.name = "p"

    bpy.ops.object.empty_add()
    c_obj = bpy.context.active_object
    assert c_obj is not None
    c_obj.name = "c"

    link_objects = {"p": p_obj, "c": c_obj}

    # Test X axis
    j_x = Joint(
        name="j_x",
        type=JointType.REVOLUTE,
        parent="p",
        child="c",
        axis=Vector3(1, 0, 0),
        limits=JointLimits(0, 1, 10, 1),
    )
    obj_x = create_joint_object(blender_context, j_x, link_objects)
    assert obj_x is not None
    assert safe_get_joint(obj_x).axis == "X"

    # Test Y axis
    j_y = Joint(
        name="j_y",
        type=JointType.REVOLUTE,
        parent="p",
        child="c",
        axis=Vector3(0, 1, 0),
        limits=JointLimits(0, 1, 10, 1),
    )
    obj_y = create_joint_object(blender_context, j_y, link_objects)
    assert obj_y is not None
    assert safe_get_joint(obj_y).axis == "Y"


def test_import_robot_topological_sort(scene, blender_context) -> None:
    """Test that joints are created in correct topological order."""
    # Create a chain where joints must be sorted
    l1 = Link(name="root")
    l2 = Link(name="mid")
    l3 = Link(name="leaf")

    # Define joints out of order
    j2 = Joint(name="j2", parent="mid", child="leaf", type=JointType.FIXED)
    j1 = Joint(name="j1", parent="root", child="mid", type=JointType.FIXED)

    robot = Robot(name="chain_bot", links=[l1, l2, l3], joints=[j2, j1])

    success = import_robot_to_scene(robot, Path("/tmp/robot.urdf"), blender_context)
    assert success is True

    # Verify hierarchy is correct despite out-of-order definition
    root = bpy.data.objects["root"]
    mid = bpy.data.objects["mid"]
    leaf = bpy.data.objects["leaf"]

    assert mid.parent is not None
    assert mid.parent.parent == root
    assert leaf.parent is not None
    assert leaf.parent.parent == mid


def test_normalize_and_consolidate_imported_objects(scene, blender_context) -> None:
    """Test the robust mesh normalization and consolidation logic."""
    from linkforge.blender.adapters.core_to_blender import (
        normalize_and_consolidate_imported_objects,
    )

    # Create a hierarchy: Empty -> Mesh -> Mesh
    root = create_test_object("root_empty", None, scene)
    root.location = (1.0, 1.0, 1.0)

    import bmesh

    m1_mesh = bpy.data.meshes.new("mesh1")
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bm.to_mesh(m1_mesh)
    bm.free()
    m1 = create_test_object("mesh1", m1_mesh, scene)
    m1.location = (2.0, 2.0, 2.0)
    m1.parent = root

    m2_mesh = bpy.data.meshes.new("mesh2")
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=8, v_segments=8, radius=0.5)
    bm.to_mesh(m2_mesh)
    bm.free()
    m2 = create_test_object("mesh2", m2_mesh, scene)
    m2.location = (3.0, 3.0, 3.0)
    m2.parent = m1

    if bpy.context.view_layer is not None:
        bpy.context.view_layer.update()

    # Store names before they are potentially deleted
    root_name = root.name
    m1_name = m1.name
    m2_name = m2.name

    # Consolidate
    res = normalize_and_consolidate_imported_objects(blender_context, [root], "consolidated")

    assert res is not None
    assert res.name == "consolidated"
    assert res.type == "MESH"
    assert res.data is not None
    assert hasattr(res.data, "vertices") and len(getattr(res.data, "vertices")) > 0
    # Check that root empty and original meshes are queued for deletion (implicitly cleaned by the function)
    assert root_name not in bpy.data.objects
    assert m1_name not in bpy.data.objects
    assert m2_name not in bpy.data.objects


def test_create_joint_object_mimic_logic(scene, blender_context) -> None:
    """Test that mimics are correctly resolved even if created out of order."""
    from linkforge.blender.adapters.core_to_blender import create_joint_object

    # Parent/Child link shells
    p = create_test_object("p", None, scene)
    c = create_test_object("c", None, scene)

    # Target joint object
    driver_obj = create_test_object("driver_joint", None, scene)

    link_objects = {"p": p, "c": c}
    # Mocking the discovery of the driver joint in scene
    # The actual implementation looks for objects by name

    joint = Joint(
        name="follower",
        type=JointType.REVOLUTE,
        parent="p",
        child="c",
        axis=Vector3(1, 0, 0),
        limits=JointLimits(lower=-1.0, upper=1.0, effort=10.0, velocity=1.0),
        mimic=JointMimic(joint="driver_joint", multiplier=0.5),
    )

    res = create_joint_object(blender_context, joint, link_objects)
    assert res is not None
    res_props = safe_get_joint(res)
    assert res_props.use_mimic is True
    assert res_props.mimic_multiplier == 0.5


def test_sensor_noise_properties(clean_scene, scene, blender_context) -> None:
    """Verify sensor noise property mapping for LIDAR, IMU, and GPS."""
    # LIDAR with noise
    lidar = Sensor(
        name="Lidar",
        type=SensorType.LIDAR,
        link_name="base_link",
        lidar_info=LidarInfo(
            horizontal_samples=640, noise=SensorNoise(type="gaussian", mean=0.0, stddev=0.01)
        ),
    )

    # IMU with noise
    imu = Sensor(
        name="imu",
        type=SensorType.IMU,
        link_name="base_link",
        imu_info=IMUInfo(
            angular_velocity_noise=SensorNoise(type="gaussian", mean=0.0, stddev=0.001)
        ),
    )

    # GPS with noise
    gps = Sensor(
        name="gps",
        type=SensorType.GPS,
        link_name="base_link",
        gps_info=GPSInfo(
            position_sensing_horizontal_noise=SensorNoise(type="gaussian", mean=0.0, stddev=0.5)
        ),
    )

    base_link = create_test_object("base_link", None)
    scene.collection.objects.link(base_link)
    link_map = {"base_link": base_link}

    for s in [lidar, imu, gps]:
        obj = create_sensor_object(blender_context, s, link_map)
        assert obj is not None
        sensor_props = safe_get_sensor(obj)
        assert sensor_props.use_noise is True
        assert sensor_props.noise_type == "gaussian"

        if s.type == SensorType.LIDAR:
            assert sensor_props.noise_stddev == pytest.approx(0.01)
        elif s.type == SensorType.IMU:
            assert sensor_props.noise_stddev == pytest.approx(0.001)
        elif s.type == SensorType.GPS:
            assert sensor_props.noise_stddev == pytest.approx(0.5)


def test_multi_visual_collision_naming(clean_scene, scene, blender_context) -> None:
    """Verify suffix naming for multiple unnamed visuals and collisions."""
    box_geom = Box(size=Vector3(1, 1, 1))

    link = Link(
        name="multi_link",
        visuals=[Visual(geometry=box_geom), Visual(geometry=box_geom)],
        collisions=[Collision(geometry=box_geom), Collision(geometry=box_geom)],
    )

    robot = Robot(name="test")
    obj = create_link_object(
        blender_context, link, robot, Path("/cwd"), collection=scene.collection
    )
    assert obj is not None

    # Check visuals: naming should follow {link_name}_visual_{idx}
    visuals = [c for c in obj.children if "_visual_" in c.name]
    assert len(visuals) == 2, f"Expected 2 visuals, found {[c.name for c in obj.children]}"

    # Check collisions: naming should follow {link_name}_collision_{idx}
    collisions = [c for c in obj.children if "_collision_" in c.name]
    assert len(collisions) == 2, f"Expected 2 collisions, found {[c.name for c in obj.children]}"


def test_normalize_consolidate_empty_cleanup(clean_scene, scene, blender_context) -> None:
    """Verify cleanup when no meshes are found in imported objects."""
    empty = create_test_object("EmptyContainer", None)
    scene.collection.objects.link(empty)

    res = normalize_and_consolidate_imported_objects(blender_context, [empty], "Final")
    assert res is None
    assert "EmptyContainer" not in bpy.data.objects


def test_import_robot_sensor_creation_failure(clean_scene, scene, blender_context) -> None:
    """Verify import_robot_to_scene handles sensor creation failure."""
    robot = Robot(
        name="test_robot",
        links=[Link(name="base_link")],
        sensors=[
            Sensor(
                name="BadSensor",
                type=SensorType.CAMERA,
                link_name="base_link",
                camera_info=CameraInfo(),
            )
        ],
    )

    context = blender_context
    with mock.patch(
        "linkforge.blender.adapters.core_to_blender.create_sensor_object", return_value=None
    ):
        import_robot_to_scene(robot, Path("test.urdf"), context)
        assert "base_link" in bpy.data.objects


def test_import_mesh_file_removes_non_mesh_stragglers(tmp_path, scene, blender_context) -> None:
    """Verify that import side-effects (Camera, Lamp, Empties) are cleaned up."""
    # Create a real STL file to import
    bpy.ops.mesh.primitive_cube_add()
    stl_path = tmp_path / "cube.stl"
    wm_ops = getattr(bpy.ops, "wm")
    if hasattr(wm_ops, "stl_export"):
        wm_ops.stl_export(filepath=str(stl_path))
    else:
        export_ops = getattr(bpy.ops, "export_mesh", None)
        if export_ops and hasattr(export_ops, "stl"):
            export_ops.stl(filepath=str(stl_path))
        else:
            pytest.skip("No STL exporter found")
    bpy.ops.object.delete()

    # Wrap the real importer so we can inject a Camera side-effect during the call,
    # matching exactly what Blender's Collada importer does for embedded scene nodes.
    real_import = bpy.ops.wm.stl_import
    injected_cam_name = "DAE_Camera_SideEffect"

    def import_with_sideeffect(**kwargs):
        result = real_import(**kwargs)
        cam_data = bpy.data.cameras.new("_tmp_cam")
        cam_obj = create_test_object(injected_cam_name, cam_data)
        scene.collection.objects.link(cam_obj)
        return result

    with mock.patch(
        "linkforge.blender.adapters.core_to_blender.bpy.ops.wm.stl_import",
        side_effect=import_with_sideeffect,
    ):
        result = import_mesh_file(blender_context, stl_path, "mesh_only")

    assert result is not None
    assert result.type == "MESH"
    assert injected_cam_name not in bpy.data.objects, (
        "Camera straggler was not removed by import_mesh_file"
    )


class TestCoreToBlenderExhaustiveCoverage:
    """Exhaustive edge-case unit tests to maximize core_to_blender.py coverage."""

    def test_create_material_exhaustive(self, scene, blender_context) -> None:
        """Verify create_material_from_color handling of missing node trees or inputs."""
        # 1. No node tree
        from tests.mock_bpy_env import MockMaterial

        with mock.patch.object(MockMaterial, "use_nodes", new_callable=mock.PropertyMock):
            mat_no_tree = bpy.data.materials.new(name="no_tree")
            mat_no_tree._values["node_tree"] = None
            # Mock context.data.materials to return our customized material
            with mock.patch.object(blender_context.data.materials, "new", return_value=mat_no_tree):
                res = create_material_from_color(blender_context, Color(1, 1, 1, 1), "no_tree_mat")
                assert res == mat_no_tree

        # 2. node_principled input has no default_value
        mat_no_val = bpy.data.materials.new(name="no_val")
        original_new = mat_no_val.node_tree.nodes.new

        def mock_nodes_new(*args, **kwargs):
            type_name = kwargs.get("type", args[0] if args else None)
            node = original_new(*args, **kwargs)
            if (
                type_name == "ShaderNodeBsdfPrincipled"
                and "default_value" in node.inputs[0]._values
            ):
                del node.inputs[0]._values["default_value"]
            return node

        with (
            mock.patch.object(mat_no_val.node_tree.nodes, "new", side_effect=mock_nodes_new),
            mock.patch.object(blender_context.data.materials, "new", return_value=mat_no_val),
        ):
            res = create_material_from_color(blender_context, Color(1, 1, 1, 1), "no_val_mat")
            assert res == mat_no_val

    def test_create_primitive_mesh_exhaustive(self, mocker, scene, blender_context) -> None:
        """Verify create_primitive_mesh boundary cases, missing scene updates, and errors."""
        # 1. View layer is None during Box creation
        mocker.patch(
            "linkforge.blender.adapters.context.BlenderContext.view_layer",
            new_callable=mocker.PropertyMock,
            return_value=None,
        )
        box = Box(size=Vector3(1, 1, 1))
        obj = create_primitive_mesh(blender_context, box, "test_box_vl_none")
        assert obj is not None

        # 2. Scene update missing / scene is None
        mocker.patch(
            "linkforge.blender.adapters.context.BlenderContext.scene",
            new_callable=mocker.PropertyMock,
            return_value=None,
        )
        cyl = Cylinder(radius=0.5, length=2.0)
        obj_cyl = create_primitive_mesh(blender_context, cyl, "test_cyl_scene_none")
        assert obj_cyl is not None

        # 3. Exception caught in create_primitive_mesh (ValueError)
        mocker.patch(
            "linkforge.blender.adapters.core_to_blender.bpy.ops.mesh.primitive_cube_add",
            side_effect=ValueError("Simulated cube error"),
        )
        obj_err = create_primitive_mesh(blender_context, box, "test_box_err")
        assert obj_err is None

    def test_import_mesh_file_exhaustive(self, mocker, scene, blender_context, tmp_path) -> None:
        """Verify import_mesh_file versions check, importer exceptions, and GLTF collections."""
        # 1. Collada deprecation check in Blender 5.0+
        mocker.patch("linkforge.blender.adapters.core_to_blender.bpy.app.version", (5, 0, 0))
        dae_path = tmp_path / "test.dae"
        dae_path.write_text("dummy dae content")
        obj_dae = import_mesh_file(blender_context, dae_path, "test_dae")
        assert obj_dae is None

        # 2. Importers failure and fallback
        stl_path = tmp_path / "test.stl"
        stl_path.write_text("dummy stl")

        mocker.patch(
            "linkforge.blender.adapters.core_to_blender.bpy.ops.wm.stl_import",
            side_effect=RuntimeError("Importers crash"),
        )
        obj_stl = import_mesh_file(blender_context, stl_path, "test_stl_err")
        assert obj_stl is None

        # 3. GLTF collection clean-up branch
        gltf_path = tmp_path / "test.glb"
        gltf_path.write_text("dummy glb")

        def mock_gltf_import(**kwargs):
            col = bpy.data.collections.new("New_GLTF_Collection")
            bpy.context.scene.collection.children.link(col)
            mesh_data = bpy.data.meshes.new("gltf_mesh")
            obj = create_test_object("gltf_obj", mesh_data)
            col.objects.link(obj)
            return {"FINISHED"}

        mocker.patch.object(blender_context.ops.wm, "gltf_import", side_effect=mock_gltf_import)
        obj_gltf = import_mesh_file(blender_context, gltf_path, "gltf_consolidated")
        assert obj_gltf is not None
        assert "New_GLTF_Collection" not in bpy.data.collections

        # 4. Processing exception (RuntimeError) during import_mesh_file
        mocker.patch(
            "linkforge.blender.adapters.core_to_blender.normalize_and_consolidate_imported_objects",
            side_effect=RuntimeError("Processing failed"),
        )
        obj_proc_err = import_mesh_file(blender_context, gltf_path, "gltf_proc_err")
        assert obj_proc_err is None

    def test_normalize_and_consolidate_imported_objects_boundaries(
        self, mocker, scene, blender_context
    ) -> None:
        """Verify normalize_and_consolidate_imported_objects returns None for empty objects and view layer is None."""
        # 1. No objects
        res = normalize_and_consolidate_imported_objects(blender_context, [], "no_objs")
        assert res is None

        # 2. View layer is None
        mocker.patch(
            "linkforge.blender.adapters.context.BlenderContext.view_layer",
            new_callable=mocker.PropertyMock,
            return_value=None,
        )
        obj = create_test_object("mesh_vl_none", bpy.data.meshes.new("mesh_vl"), scene)
        res_vl = normalize_and_consolidate_imported_objects(blender_context, [obj], "mesh_vl_none")
        assert res_vl is not None

    def test_create_link_object_exhaustive(self, mocker, scene, blender_context) -> None:
        """Verify create_link_object when properties are None or missing."""
        robot = Robot(name="test")

        # 1. get_link_props returns None
        mocker.patch("linkforge.blender.adapters.core_to_blender.get_link_props", return_value=None)
        link = Link(name="no_props_link")
        obj_no_props = create_link_object(blender_context, link, robot, Path("/tmp"))
        assert obj_no_props is not None

        # 2. FileNotFoundError for visual mesh resolution & fallback
        mocker.stopall()
        vis_mesh = Visual(geometry=Mesh(resource="non_existent.stl"))
        link_vis_mesh = Link(name="non_existent_mesh_link", visuals=[vis_mesh])
        obj_non_exist = create_link_object(blender_context, link_vis_mesh, robot, Path("/tmp"))
        assert obj_non_exist is not None
        assert len(obj_non_exist.children) == 0

        # 3. Collision Cylinder, visual with no origin and no name, collision with no origin
        coll_cyl = Collision(geometry=Cylinder(radius=0.5, length=1.0))
        vis_prim = Visual(geometry=Box(size=Vector3(1, 1, 1)))
        link_cyl = Link(name="cyl_link", visuals=[vis_prim], collisions=[coll_cyl])
        obj_cyl = create_link_object(blender_context, link_cyl, robot, Path("/tmp"))
        assert obj_cyl is not None
        coll_child = next(c for c in obj_cyl.children if "_collision" in c.name)
        assert coll_child["collision_geometry_type"] == "cylinder"

        # 4. Inertial without inertia tensor or without origin
        inertial_no_tensor = Inertial(mass=2.5, origin=Transform(xyz=Vector3(1, 2, 3)))
        link_inertial = Link(name="inertial_link", inertial=inertial_no_tensor)
        obj_inertial = create_link_object(blender_context, link_inertial, robot, Path("/tmp"))
        assert obj_inertial is not None
        props = safe_get_linkforge(obj_inertial)
        assert props.mass == 2.5
        assert props.inertia_origin_xyz[0] == 1.0

    def test_create_joint_object_exhaustive(self, mocker, scene, blender_context) -> None:
        """Verify joint creation when parent or child is missing, properties are None, or context has no scene."""
        p = create_test_object("p", None, scene)
        c = create_test_object("c", None, scene)
        link_objects = {"p": p, "c": c}

        # 1. Parent/child link missing (returns joint Empty and logs error)
        joint_err = Joint(
            name="err_j", type=JointType.FIXED, parent="nonexistent_p", child="nonexistent_c"
        )
        obj_err = create_joint_object(blender_context, joint_err, link_objects)
        assert obj_err is not None

        # 2. get_joint_props returns None
        mocker.patch(
            "linkforge.blender.adapters.core_to_blender.get_joint_props", return_value=None
        )
        joint_normal = Joint(name="normal_j", type=JointType.FIXED, parent="p", child="c")
        obj_normal = create_joint_object(blender_context, joint_normal, link_objects)
        assert obj_normal is not None

        # 3. Calibration rising is None, falling is None
        mocker.stopall()
        calib = JointCalibration()
        joint_calib = Joint(
            name="calib_j", type=JointType.FIXED, parent="p", child="c", calibration=calib
        )
        obj_calib = create_joint_object(blender_context, joint_calib, link_objects)
        assert obj_calib is not None
        assert safe_get_joint(obj_calib).use_calibration is True
        assert safe_get_joint(obj_calib).use_calibration_rising is False
        assert safe_get_joint(obj_calib).use_calibration_falling is False

        # 4. Joint collection is an object
        another_obj = create_test_object("another_obj", None, scene)
        joint_coll_obj = Joint(name="coll_obj_j", type=JointType.FIXED, parent="p", child="c")
        obj_coll_obj = create_joint_object(
            blender_context, joint_coll_obj, link_objects, collection=another_obj
        )
        assert obj_coll_obj is not None

        # 5. context.scene is None
        mocker.patch(
            "linkforge.blender.adapters.context.BlenderContext.scene",
            new_callable=mocker.PropertyMock,
            return_value=None,
        )
        joint_scene_none = Joint(name="scene_none_j", type=JointType.FIXED, parent="p", child="c")
        obj_scene_none = create_joint_object(blender_context, joint_scene_none, link_objects)
        assert obj_scene_none is not None

    def test_create_sensor_object_exhaustive(self, mocker, scene, blender_context) -> None:
        """Verify sensor creation when properties are None or missing."""
        link_obj = create_test_object("base_link", None, scene)
        link_objects = {"base_link": link_obj}

        # 1. PROP_SENSOR missing from empty object
        mocker.patch("linkforge.blender.adapters.core_to_blender.hasattr", return_value=False)
        sensor_no_prop = Sensor(
            name="no_prop_sensor",
            type=SensorType.CAMERA,
            link_name="base_link",
            camera_info=CameraInfo(),
        )
        obj_no_prop = create_sensor_object(blender_context, sensor_no_prop, link_objects)
        assert obj_no_prop is not None

        # 2. Camera details format, near_clip, far_clip are None; topic/update_rate are None
        mocker.stopall()
        cam_info = CameraInfo(horizontal_fov=1.0, width=640, height=480)
        sensor_cam = Sensor(
            name="cam_none", type=SensorType.CAMERA, link_name="base_link", camera_info=cam_info
        )
        obj_cam = create_sensor_object(blender_context, sensor_cam, link_objects)
        assert obj_cam is not None
        props = safe_get_sensor(obj_cam)
        assert props.camera_horizontal_fov == 1.0

        # 3. Lidar vertical_samples is None
        lidar_info = LidarInfo(horizontal_samples=360, range_min=0.1, range_max=10.0)
        sensor_lidar = Sensor(
            name="lidar_none", type=SensorType.LIDAR, link_name="base_link", lidar_info=lidar_info
        )
        obj_lidar = create_sensor_object(blender_context, sensor_lidar, link_objects)
        assert obj_lidar is not None
        assert safe_get_sensor(obj_lidar).lidar_horizontal_samples == 360

        # 4. Plugin raw_xml is None; sensor origin is None
        plugin = GazeboPlugin(name="plug", filename="libplug.so")
        sensor_plug = Sensor(
            name="plug_none",
            type=SensorType.CAMERA,
            link_name="base_link",
            camera_info=cam_info,
            plugin=plugin,
        )
        obj_plug = create_sensor_object(blender_context, sensor_plug, link_objects)
        assert obj_plug is not None
        assert safe_get_sensor(obj_plug).use_gazebo_plugin is True
        assert safe_get_sensor(obj_plug).plugin_filename == "libplug.so"

    def test_setup_scene_for_robot_exhaustive(self, scene, blender_context) -> None:
        """Verify scene setup for robots with empty ros2_control or varied gazebo plugins."""
        from linkforge.blender.adapters.core_to_blender import setup_scene_for_robot

        # 1. Empty ros2_control (resets scene props)
        robot_empty = Robot(name="empty_bot")
        setup_scene_for_robot(blender_context, robot_empty)
        lp = safe_get_linkforge_scene(scene)
        assert lp.use_ros2_control is False

        # 2. Gazebo elements loop
        plugin_other = GazeboPlugin(
            name="other_plugin", filename="libother.so", parameters={"another": "val"}
        )
        gazebo = GazeboElement(plugins=[plugin_other])
        robot_gz = Robot(name="gz_other_bot", gazebo_elements=[gazebo])
        setup_scene_for_robot(blender_context, robot_gz)
        lp_gz = safe_get_linkforge_scene(scene)
        assert lp_gz.gazebo_plugin_name == "gz_ros2_control::GazeboSimROS2ControlPlugin"

    def test_import_robot_to_scene_exhaustive(self, mocker, scene, blender_context) -> None:
        """Verify import_robot_to_scene auto-wraps raw context and handles individual creation failures."""
        l1 = Link(name="base")
        l2 = Link(name="child")
        j1 = Joint(name="j", type=JointType.FIXED, parent="base", child="child")
        s1 = Sensor(name="s", type=SensorType.CAMERA, link_name="base", camera_info=CameraInfo())
        robot = Robot(name="ex_bot", links=[l1, l2], joints=[j1], sensors=[s1])

        # 1. Raw context autowrapping
        raw_ctx = bpy.context
        with mock.patch("linkforge.blender.adapters.core_to_blender.setup_scene_for_robot"):
            mocker.patch(
                "linkforge.blender.adapters.core_to_blender.create_link_object", return_value=None
            )
            success = import_robot_to_scene(robot, Path("test.urdf"), raw_ctx)
            assert success is True

        # 2. Individual components creation returning None
        mocker.stopall()
        mocker.patch(
            "linkforge.blender.adapters.core_to_blender.create_link_object", return_value=None
        )
        mocker.patch(
            "linkforge.blender.adapters.core_to_blender.create_joint_object", return_value=None
        )
        mocker.patch(
            "linkforge.blender.adapters.core_to_blender.create_sensor_object", return_value=None
        )
        success_none = import_robot_to_scene(robot, Path("test.urdf"), blender_context)
        assert success_none is True

        # 3. View layer and scene are None during import
        mocker.stopall()
        mocker.patch(
            "linkforge.blender.adapters.context.BlenderContext.view_layer",
            new_callable=mocker.PropertyMock,
            return_value=None,
        )
        mocker.patch(
            "linkforge.blender.adapters.context.BlenderContext.scene",
            new_callable=mocker.PropertyMock,
            return_value=None,
        )
        success_scene_none = import_robot_to_scene(robot, Path("test.urdf"), blender_context)
        assert success_scene_none is True

    def test_core_to_blender_uncovered_branches(self, scene, blender_context, tmp_path) -> None:
        """Verify uncovered branches in core_to_blender.py including edge exceptions, unsupported meshes, warnings, and mimic calibrations."""
        # 1. create_primitive_mesh unexpected error
        mock_ops = mock.Mock()
        mock_ops.mesh.primitive_cube_add.side_effect = RuntimeError("unexpected error")
        with (
            mock.patch.object(
                BlenderContext, "ops", new_callable=mock.PropertyMock, return_value=mock_ops
            ),
            pytest.raises(RuntimeError),
        ):
            create_primitive_mesh(blender_context, Box(size=Vector3(x=1, y=1, z=1)), "test_box")

        # 2. import_mesh_file unsupported mesh file extension
        unsupported_path = tmp_path / "test.xyz"
        unsupported_path.touch()
        assert import_mesh_file(blender_context, unsupported_path, "test_xyz") is None

        # 3. import_mesh_file Collada support warning on Blender < 5.0.0 or error on >= 5.0.0
        dae_path = tmp_path / "test.dae"
        dae_path.touch()
        with mock.patch("bpy.app.version", (4, 2, 0)):
            # Should warning, but succeed under mock env
            assert import_mesh_file(blender_context, dae_path, "test_dae") is not None

        with mock.patch("bpy.app.version", (5, 0, 0)):
            assert import_mesh_file(blender_context, dae_path, "test_dae") is None

        # 4. Importer unexpected error
        with mock.patch("linkforge.blender.adapters.context.BlenderContext.ops") as mock_ops:
            mock_callable = mock.Mock(side_effect=ValueError("Unexpected importer error"))
            mock_ops.wm.obj_import = mock_callable
            obj_path = tmp_path / "test.obj"
            obj_path.touch()
            assert import_mesh_file(blender_context, obj_path, "test_obj") is None

        # 5. Cyclic references in process_recursive
        dup_obj = create_test_object("mesh_dup", None, scene=scene)
        # Call normalize_and_consolidate_imported_objects twice with same object to hit recursive seen check
        normalize_and_consolidate_imported_objects(
            blender_context, [dup_obj, dup_obj], "consolidated"
        )

        # 6. create_link_object with no origin but name, and box collision quality str
        visual = mock.Mock()
        visual.geometry = Box(size=Vector3(x=1, y=1, z=1))
        visual.origin = None
        visual.name = "test_visual_name"
        visual.material = None

        collision = mock.Mock()
        collision.geometry = Box(size=Vector3(x=1, y=1, z=1))
        collision.name = "test_col_box"
        collision.origin = None

        dummy_robot = Robot(name="dummy_robot")
        link = Link(name="test_link_col_box", visuals=(visual,), collisions=(collision,))
        link_obj = create_link_object(blender_context, link, dummy_robot, Path("."))
        assert link_obj is not None

        # 7. create_link_object with FileNotFoundError mesh resolution
        robot = Robot(name="test_robot", version="1.1", materials={}, metadata={})
        with mock.patch.object(
            robot, "resolve_resource", side_effect=FileNotFoundError("Mock file not found")
        ):
            collision_err = mock.Mock()
            collision_err.geometry = Mesh(
                resource="package://nonexistent.stl", scale=Vector3(x=1, y=1, z=1)
            )
            collision_err.name = "test_col_err"
            collision_err.origin = None
            link_err = Link(name="test_link_col_err", visuals=(), collisions=(collision_err,))
            create_link_object(blender_context, link_err, robot, Path("."))

        # 8. Joint calibration falling & safety limits
        calibration = JointCalibration(rising=None, falling=0.5)
        joint = Joint(
            name="test_joint_cal",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
            calibration=calibration,
            origin=None,
        )
        with mock.patch("linkforge.blender.adapters.core_to_blender.get_joint_props") as mock_get:
            mock_props = mock.Mock()
            mock_get.return_value = mock_props
            create_joint_object(blender_context, joint, {}, {})
            assert mock_props.use_calibration_falling is True
            assert mock_props.calibration_falling == 0.5

        # 9. Sensor topic name mapping
        link_obj = create_test_object("link1", None, scene=scene)
        sensor = Sensor(
            name="test_sensor_topic",
            type=SensorType.CAMERA,
            link_name="link1",
            update_rate=30.0,
            topic="/camera/image_raw",
            camera_info=CameraInfo(),
            origin=None,
        )
        sensor_props = mock.Mock()
        # Mock hasattr to return True for linkforge_sensor
        mock_sensor_obj = mock.Mock()
        mock_sensor_obj.linkforge_sensor = sensor_props
        with mock.patch.object(blender_context.data.objects, "new", return_value=mock_sensor_obj):
            sensor_obj = create_sensor_object(blender_context, sensor, {"link1": link_obj})
            assert sensor_obj is not None
            assert sensor_props.topic_name == "/camera/image_raw"

        # 10. Import robot with ROS2 Control parameters & joint parameters
        scene.linkforge_robot = mock.Mock()
        rc_joint = Ros2ControlJoint(
            name="joint_1",
            command_interfaces=["position"],
            state_interfaces=["position"],
            parameters={"joint_p": "j_val"},
        )
        rc = Ros2Control(
            name="test_control",
            type="system",
            hardware_plugin="mock_hw",
            parameters={"global_p": "g_val"},
            joints=[rc_joint],
        )
        gz_plugin = GazeboPlugin(
            name="test_ros2_control",
            filename="libgz_ros2_control.so",
            parameters={"parameters": "test_controllers.yaml"},
        )
        gz_element = GazeboElement(plugins=(gz_plugin,))
        robot_rc = Robot(
            name="rc_bot",
            links=[Link("base"), Link("child")],
            joints=[Joint("joint_1", JointType.FIXED, "base", "child")],
            ros2_controls=[rc],
            gazebo_elements=(gz_element,),
        )
        with (
            mock.patch("linkforge.blender.adapters.core_to_blender.setup_scene_for_robot"),
            mock.patch(
                "linkforge.blender.adapters.core_to_blender.create_link_object", return_value=None
            ),
            mock.patch(
                "linkforge.blender.adapters.core_to_blender.create_joint_object", return_value=None
            ),
        ):
            success = import_robot_to_scene(robot_rc, Path("test.urdf"), blender_context)
            assert success is True

        from linkforge.blender.adapters.core_to_blender import _get_geometry_type_str

        assert _get_geometry_type_str(None) == "mesh"

    def test_create_material_no_default_value(self, scene, blender_context, mocker) -> None:
        """Verify create_material_from_color when node_principled input has no default_value attribute (covers 114->118 false branch)."""
        mat = bpy.data.materials.new(name="no_def_val")

        # Mock node_tree.nodes.new to return a mock node whose inputs[0] has no default_value
        mock_node = mocker.MagicMock()
        mock_input = mocker.MagicMock(spec=[])
        mock_node.inputs = [mock_input]

        mocker.patch.object(mat.node_tree.nodes, "new", return_value=mock_node)

        with mocker.patch.object(blender_context.data.materials, "new", return_value=mat):
            res = create_material_from_color(blender_context, Color(1, 1, 1, 1), "no_def_val_mat")
            assert res == mat

    def test_setup_scene_for_robot_uncovered_branches(self, scene, blender_context, mocker) -> None:
        """Verify setup_scene_for_robot covered completely (996->1002 False, 1052->1054 False)."""
        from linkforge.blender.adapters.core_to_blender import setup_scene_for_robot

        # 1. scene is missing PROP_ROBOT (covers 996->1002 False branch)
        mock_scene_no_prop = mocker.MagicMock(spec=[])
        mock_context = mocker.MagicMock()
        mock_context.scene = mock_scene_no_prop

        robot_empty = Robot(name="empty_bot")
        setup_scene_for_robot(mock_context, robot_empty)

        # 2. Gazebo element plugin with ros2_control but parameters dict has no "parameters" key (covers 1052->1054 False branch)
        mock_scene_with_prop = mocker.MagicMock()
        mock_scene_with_prop.linkforge_robot = mocker.MagicMock()
        mock_context_with_prop = mocker.MagicMock()
        mock_context_with_prop.scene = mock_scene_with_prop

        plugin_no_yaml = GazeboPlugin(
            name="ros2_control_plugin",
            filename="libros2_control.so",
            parameters={"kp": "100.0"},
        )
        elem = GazeboElement(plugins=[plugin_no_yaml])
        robot_gz = Robot(name="gz_bot", gazebo_elements=[elem])

        setup_scene_for_robot(mock_context_with_prop, robot_gz)
        assert mock_scene_with_prop.linkforge_robot.controllers_yaml_path == ""

    def test_core_to_blender_comprehensive_exhaustiveness(
        self, mocker, scene, blender_context, tmp_path
    ) -> None:
        """Verify all remaining uncovered branches in core_to_blender.py to achieve 100% statement and branch coverage."""
        from linkforge.blender.adapters.core_to_blender import (
            create_joint_object,
            create_link_object,
            create_primitive_mesh,
            create_sensor_object,
            import_mesh_file,
            setup_scene_for_robot,
        )
        from linkforge.core.constants import (
            GEOM_BOX,
            GEOM_CYLINDER,
            GEOM_SPHERE,
        )

        # 1. Sphere scene update missing branch (174->176 False branch)
        mocker.patch(
            "linkforge.blender.adapters.context.BlenderContext.scene",
            new_callable=mocker.PropertyMock,
            return_value=None,
        )
        sph = Sphere(radius=0.5)
        obj_sph = create_primitive_mesh(blender_context, sph, "test_sph_scene_none")
        assert obj_sph is not None
        mocker.stopall()

        # 2. Primitive mesh creation when obj = context.get_active_object() is None (149->188, 160->188, 171->188 False branches)
        mocker.patch(
            "linkforge.blender.adapters.context.BlenderContext.get_active_object",
            return_value=None,
        )
        assert create_primitive_mesh(blender_context, Box(size=Vector3(1, 1, 1)), "t_box") is None
        assert (
            create_primitive_mesh(blender_context, Cylinder(radius=0.5, length=2.0), "t_cyl")
            is None
        )
        assert create_primitive_mesh(blender_context, Sphere(radius=0.5), "t_sph") is None
        mocker.stopall()

        # 3. import_mesh_file collections and exception handling (291->False, 297->False, 307, 312-314)
        stl_path = tmp_path / "test_import.stl"
        stl_path.touch()

        # 3a. res_obj.name is ALREADY in current_col.objects (covers 291 False branch)
        # 3b. res_obj.name is NOT in new_col.objects (covers 297 False branch)
        def mock_stl_import_ok(**kwargs):
            col = bpy.data.collections.new("New_STL_Collection")
            bpy.context.scene.collection.children.link(col)
            mesh_data = bpy.data.meshes.new("stl_mesh")
            obj = create_test_object("stl_obj_already_linked", mesh_data)
            bpy.context.scene.collection.objects.link(obj)
            return {"FINISHED"}

        mocker.patch(
            "linkforge.blender.adapters.core_to_blender.bpy.ops.wm.stl_import", mock_stl_import_ok
        )
        res = import_mesh_file(blender_context, stl_path, "stl_obj_already_linked")
        assert res is not None
        mocker.stopall()

        # 3c. normalize_and_consolidate returns None (covers 307)
        mocker.patch(
            "linkforge.blender.adapters.core_to_blender.bpy.ops.wm.stl_import",
            return_value={"FINISHED"},
        )
        mocker.patch(
            "linkforge.blender.adapters.core_to_blender.normalize_and_consolidate_imported_objects",
            return_value=None,
        )
        assert import_mesh_file(blender_context, stl_path, "none_imported") is None
        mocker.stopall()

        # 3d. normalize_and_consolidate raises generic Exception (covers 312-314)
        mocker.patch(
            "linkforge.blender.adapters.core_to_blender.bpy.ops.wm.stl_import",
            return_value={"FINISHED"},
        )
        mocker.patch(
            "linkforge.blender.adapters.core_to_blender.normalize_and_consolidate_imported_objects",
            side_effect=ValueError("Unexpected processing error"),
        )
        with pytest.raises(ValueError, match="Unexpected processing error"):
            import_mesh_file(blender_context, stl_path, "raises_err")
        mocker.stopall()

        # 3e. res_obj.name is NOT in current_col.objects (covers 292 statement)
        def mock_stl_import_not_linked(**kwargs):
            col = bpy.data.collections.new("New_STL_Collection_Not_Linked")
            bpy.context.scene.collection.children.link(col)
            return {"FINISHED"}

        mocker.patch(
            "linkforge.blender.adapters.core_to_blender.bpy.ops.wm.stl_import",
            mock_stl_import_not_linked,
        )
        obj_not_linked = bpy.data.objects.new(
            "stl_obj_not_linked", bpy.data.meshes.new("stl_mesh_not_linked")
        )
        mocker.patch(
            "linkforge.blender.adapters.core_to_blender.normalize_and_consolidate_imported_objects",
            return_value=obj_not_linked,
        )

        mock_scene = mocker.MagicMock()
        mock_scene.collection = mocker.MagicMock()
        mock_scene.collection.objects = mocker.MagicMock()
        mock_scene.collection.objects.__contains__.return_value = False

        mocker.patch(
            "linkforge.blender.adapters.context.BlenderContext.scene",
            new_callable=mocker.PropertyMock,
            return_value=mock_scene,
        )

        res_not_linked = import_mesh_file(blender_context, stl_path, "stl_obj_not_linked")
        assert res_not_linked is not None
        mock_scene.collection.objects.link.assert_called_once_with(obj_not_linked)
        mocker.stopall()

        # Create unrecognized geometry class
        class CustomUnrecognizedGeometry:
            pass

        # Link with:
        # - Visual whose created object data has no materials (evaluates 511 False -> jumps to 448)
        # - Collision whose created object data has no materials (evaluates 581 False -> jumps to 586)
        # - Collision with unrecognized geometry (evaluates 599 False -> loops to 518)
        # - Inertial without inertia (covers 606 False branch)
        # - Inertial without origin (covers 617 False branch)
        vis = Visual(
            geometry=Box(size=Vector3(1, 1, 1)),
            material=Material(name="missing_mat", color=Color(1, 1, 1, 1)),
        )
        coll_unrecognized = Collision(geometry=CustomUnrecognizedGeometry())
        coll_box = Collision(geometry=Box(size=Vector3(1, 1, 1)))

        # Mock create_primitive_mesh to return a MagicMock with data = None to trigger visual/collision false branches
        mock_mesh_obj = mocker.MagicMock()
        mock_mesh_obj.data = None
        mocker.patch(
            "linkforge.blender.adapters.core_to_blender.create_primitive_mesh",
            return_value=mock_mesh_obj,
        )

        inertial = Inertial(mass=2.0)
        object.__setattr__(inertial, "inertia", None)
        object.__setattr__(inertial, "origin", None)

        link = Link(
            name="exhaustive_link",
            visuals=[vis],
            collisions=[coll_unrecognized, coll_box],
            inertial=inertial,
        )
        robot = Robot(name="dummy")
        obj_link = create_link_object(blender_context, link, robot, tmp_path)
        assert obj_link is not None
        mocker.stopall()

        # Verify lowercase collision_type is set (covers 659 for BOX, CYLINDER, SPHERE)
        link_box = Link(
            name="link_box",
            collisions=[Collision(geometry=Box(size=Vector3(1, 1, 1)))],
        )
        obj_box = create_link_object(blender_context, link_box, robot, tmp_path)
        assert obj_box is not None
        props = safe_get_linkforge(obj_box)
        assert props.collision_type == GEOM_BOX

        link_cyl = Link(
            name="link_cyl",
            collisions=[Collision(geometry=Cylinder(radius=0.5, length=2.0))],
        )
        obj_cyl = create_link_object(blender_context, link_cyl, robot, tmp_path)
        assert obj_cyl is not None
        assert safe_get_linkforge(obj_cyl).collision_type == GEOM_CYLINDER

        link_sph = Link(
            name="link_sph",
            collisions=[Collision(geometry=Sphere(radius=0.5))],
        )
        obj_sph_l = create_link_object(blender_context, link_sph, robot, tmp_path)
        assert obj_sph_l is not None
        assert safe_get_linkforge(obj_sph_l).collision_type == GEOM_SPHERE

        # Collision unrecognized type (covers 660->666 False branch)
        mocker.patch(
            "linkforge.blender.adapters.core_to_blender._get_geometry_type_str",
            return_value="UNKNOWN",
        )
        link_unrecognized_type = Link(
            name="link_unrecognized_type",
            collisions=[Collision(geometry=Box(size=Vector3(1, 1, 1)))],
        )
        obj_unrecognized = create_link_object(
            blender_context, link_unrecognized_type, robot, tmp_path
        )
        assert obj_unrecognized is not None
        mocker.stopall()

        # 5. create_joint_object without origin (covers 806 False branch)
        p = create_test_object("parent_l", None, scene)
        c = create_test_object("child_l", None, scene)
        link_objects = {"parent_l": p, "child_l": c}
        joint_no_origin = Joint(
            name="joint_no_origin",
            type=JointType.FIXED,
            parent="parent_l",
            child="child_l",
            origin=None,
        )
        obj_j = create_joint_object(blender_context, joint_no_origin, link_objects)
        assert obj_j is not None

        # 6. create_sensor_object and setup_scene_for_robot edge cases (895->897, 906->908, 908->910, 910->914, 919->921, 1013-1015, 1032-1034, 1052->1054)
        # 6a. Sensor values are None
        sensor_none = Sensor(
            name="sensor_none",
            type=SensorType.CAMERA,
            link_name="parent_l",
            update_rate=30.0,
            topic=None,
            camera_info=CameraInfo(
                horizontal_fov=1.0,
                width=640,
                height=480,
                format="R8G8B8",
                near_clip=0.1,
                far_clip=100.0,
            ),
        )
        object.__setattr__(sensor_none, "update_rate", None)
        object.__setattr__(sensor_none.camera_info, "format", None)
        object.__setattr__(sensor_none.camera_info, "near_clip", None)
        object.__setattr__(sensor_none.camera_info, "far_clip", None)

        obj_s = create_sensor_object(blender_context, sensor_none, link_objects)
        assert obj_s is not None

        # 6b. Lidar vertical_samples is None
        lidar_none = Sensor(
            name="lidar_none",
            type=SensorType.LIDAR,
            link_name="parent_l",
            lidar_info=LidarInfo(
                horizontal_samples=360, range_min=0.1, range_max=10.0, vertical_samples=1
            ),
        )
        object.__setattr__(lidar_none.lidar_info, "vertical_samples", None)

        obj_lid = create_sensor_object(blender_context, lidar_none, link_objects)
        assert obj_lid is not None

        # 6c. Setup scene for robot with ROS2 Control parameters (covers 1013-1015 and 1032-1034)
        # and Gazebo element with plugin containing "ros2_control" and parameters (covers 1052->1054)
        rc_joint = Ros2ControlJoint(
            name="j1",
            command_interfaces=["position"],
            state_interfaces=["position"],
            parameters={"kp": "100.0"},
        )
        control = Ros2Control(
            name="scene_control",
            type="system",
            hardware_plugin="mock_hw",
            parameters={"global_param": "global_val"},
            joints=[rc_joint],
        )
        plugin_gz = GazeboPlugin(
            name="gz_ros2_control_plugin",
            filename="libgz_ros2_control.so",
            parameters={"parameters": "config.yaml"},
        )
        plugin_other = GazeboPlugin(
            name="other_plugin",
            filename="libother.so",
        )
        elem = GazeboElement(plugins=[plugin_other, plugin_gz])
        robot_scene = Robot(
            name="scene_bot",
            ros2_controls=[control],
            gazebo_elements=[elem],
        )

        # Configure high-fidelity MagicMock and lambda-based lookup for scene.linkforge_robot
        lp = mocker.MagicMock()

        mock_param = mocker.MagicMock()
        mock_param.name = "global_param"
        lp.ros2_control_parameters.add.return_value = mock_param
        lp.ros2_control_parameters.__getitem__ = lambda *args, **kwargs: mock_param

        mock_joint_param = mocker.MagicMock()
        mock_joint_param.name = "kp"
        mock_joint = mocker.MagicMock()
        mock_joint.parameters.add.return_value = mock_joint_param
        mock_joint.parameters.__getitem__ = lambda *args, **kwargs: mock_joint_param

        lp.ros2_control_joints.add.return_value = mock_joint
        lp.ros2_control_joints.__getitem__ = lambda *args, **kwargs: mock_joint

        scene.linkforge_robot = lp

        setup_scene_for_robot(blender_context, robot_scene)

        assert lp.use_ros2_control is True
        assert lp.ros2_control_parameters[0].name == "global_param"
        assert lp.ros2_control_joints[0].parameters[0].name == "kp"
        assert lp.controllers_yaml_path == "config.yaml"
