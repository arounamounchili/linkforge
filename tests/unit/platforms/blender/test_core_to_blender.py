from pathlib import Path
from unittest import mock

import bpy
import pytest
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
from linkforge_core.models import (
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
    JointDynamics,
    JointLimits,
    JointMimic,
    JointType,
    LidarInfo,
    Link,
    Mesh,
    Robot,
    Ros2Control,
    Ros2ControlJoint,
    Sensor,
    SensorNoise,
    SensorType,
    Sphere,
    Transform,
    Vector3,
    Visual,
)


def test_create_primitive_mesh_box() -> None:
    """Test creation of a real Blender Cube from Box model."""
    box = Box(size=Vector3(1, 2, 3))
    obj = create_primitive_mesh(box, "test_box_unique")

    assert obj is not None
    assert obj.name == "test_box_unique"
    assert obj.type == "MESH"
    assert pytest.approx(obj.dimensions.x) == 1.0
    assert pytest.approx(obj.dimensions.y) == 2.0
    assert pytest.approx(obj.dimensions.z) == 3.0


def test_create_link_object_multi_collisions() -> None:
    """Test link creation with multiple collision elements."""
    coll1 = Collision(geometry=Box(size=Vector3(1, 1, 1)), name="c1")
    coll2 = Collision(geometry=Sphere(radius=0.5), name="c2")
    link = Link(name="multi_coll_link", initial_collisions=[coll1, coll2])

    robot = Robot(name="test")
    obj = create_link_object(link, robot, Path("/tmp"))

    collisions = [c for c in obj.children if "_collision" in c.name]
    assert len(collisions) == 2
    assert any("c1" in c.name for c in collisions)
    assert any("c2" in c.name for c in collisions)


def test_create_link_object_zero_mass() -> None:
    """Test link creation with no inertial properties (should default to 0 mass)."""
    link = Link(name="dummy_link")  # No inertial
    robot = Robot(name="test")
    obj = create_link_object(link, robot, Path("/tmp"))

    assert obj.linkforge.mass == 0.0
    assert obj.linkforge.use_auto_inertia is False


def test_create_link_object_primitives() -> None:
    """Test creating a Link object with multiple primitive visuals and collisions."""
    # 1. Setup Core Link
    box_geom = Box(size=Vector3(1, 1, 1))
    sphere_geom = Sphere(radius=0.5)

    visual = Visual(geometry=box_geom, origin=Transform(xyz=Vector3(1, 0, 0)))
    collision = Collision(geometry=sphere_geom, origin=Transform(xyz=Vector3(0, 1, 0)))

    link = Link(
        name="test_link_p",
        initial_visuals=[visual],
        initial_collisions=[collision],
        inertial=Inertial(mass=1.0),
    )

    # 2. Build in Blender
    collection = bpy.data.collections.new("TestCol")
    bpy.context.scene.collection.children.link(collection)

    robot = Robot(name="test")
    obj = create_link_object(link, robot, Path("/tmp"), collection=collection)

    # 3. Verify
    assert obj is not None
    assert obj.name == "test_link_p"

    # Verify relative positioning
    visual_obj = next(c for c in obj.children if "_visual" in c.name)
    assert pytest.approx(visual_obj.location.x) == 1.0


def test_create_joint_object_fixed() -> None:
    """Test creation of a Joint object in Blender."""
    # 1. Setup Link objects
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(0, 0, 0))
    parent_obj = bpy.context.active_object
    parent_obj.name = "parent_l"

    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(1, 0, 0))
    child_obj = bpy.context.active_object
    child_obj.name = "child_l"

    link_objects = {"parent_l": parent_obj, "child_l": child_obj}

    # 2. Setup Core Joint
    joint = Joint(
        name="test_j",
        type=JointType.FIXED,
        parent="parent_l",
        child="child_l",
        origin=Transform(xyz=Vector3(0.5, 0, 0)),
    )

    # 3. Build in Blender
    joint_obj = create_joint_object(joint, link_objects)

    # 4. Verify
    assert joint_obj is not None
    assert joint_obj.parent == parent_obj
    assert pytest.approx(joint_obj.location.x) == 0.5


def test_create_joint_object_complex() -> None:
    """Test creation of a revolute Joint with limits and axis in Blender."""
    # 1. Setup Links
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(0, 0, 0))
    parent_obj = bpy.context.active_object
    parent_obj.name = "p_link"

    bpy.ops.object.empty_add(type="PLAIN_AXES", location=(0, 0, 1))
    child_obj = bpy.context.active_object
    child_obj.name = "c_link"

    link_objects = {"p_link": parent_obj, "c_link": child_obj}

    # 2. Setup Core Joint
    joint = Joint(
        name="rev_joint",
        type=JointType.REVOLUTE,
        parent="p_link",
        child="c_link",
        axis=Vector3(0, 0, 1),
        limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=1.0),
        dynamics=JointDynamics(damping=0.1, friction=0.05),
    )

    # 3. Build
    joint_obj = create_joint_object(joint, link_objects)

    # 4. Verify properties
    assert joint_obj is not None
    props = joint_obj.linkforge_joint
    assert props.joint_type == "REVOLUTE"
    assert props.axis == "Z"
    assert props.use_limits is True
    assert pytest.approx(props.limit_lower) == -1.57
    assert pytest.approx(props.limit_upper) == 1.57
    assert props.use_dynamics is True
    assert pytest.approx(props.dynamics_damping) == 0.1


def test_create_joint_object_advanced_props() -> None:
    """Verify that safety controller and calibration are correctly synced to Blender properties."""
    from linkforge_core.models import JointCalibration, JointSafetyController

    # 1. Setup Links
    bpy.ops.object.empty_add()
    p_obj = bpy.context.active_object
    p_obj.name = "p_link_adv"
    bpy.ops.object.empty_add()
    c_obj = bpy.context.active_object
    c_obj.name = "c_link_adv"
    link_objects = {"p_link_adv": p_obj, "c_link_adv": c_obj}

    # 2. Setup Core Joint
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

    # 3. Build in Blender
    joint_obj = create_joint_object(joint, link_objects)

    # 4. Verify properties
    props = joint_obj.linkforge_joint
    assert props.use_safety_controller is True
    assert props.safety_soft_lower_limit == -1.0
    assert props.safety_k_position == 100.0

    assert props.use_calibration is True
    assert props.use_calibration_rising is True
    assert props.calibration_rising == 0.5
    assert props.use_calibration_falling is False


def test_import_robot_to_scene_full() -> None:
    """Test the full robot import entry point with a simple chain."""
    # 1. Create Robot Model
    l1 = Link(name="base_link")
    l2 = Link(name="tool_link")
    j1 = Joint(name="j1", type=JointType.FIXED, parent="base_link", child="tool_link")

    # Correct Robot instantiation using initial_links/initial_joints
    robot = Robot(name="mini_robot", initial_links=[l1, l2], initial_joints=[j1])

    # 2. Import
    result = import_robot_to_scene(robot, Path("dummy.urdf"), bpy.context)

    # 3. Verify
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


def test_import_robot_complex_tree() -> None:
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

    robot = Robot(name="tree_bot", initial_links=[l1, l2, l3, l4], initial_joints=[j1, j2, j3])

    # Needs urdf_path and context. Returns bool.
    success = import_robot_to_scene(robot, Path("/tmp/robot.urdf"), bpy.context)
    assert success is True
    # Check hierarchy
    root_obj = bpy.data.objects.get("root")
    branch1_obj = bpy.data.objects.get("branch1")
    branch2_obj = bpy.data.objects.get("branch2")
    leaf_obj = bpy.data.objects.get("leaf")

    assert branch1_obj.parent.parent == root_obj  # Link -> Joint -> Parent Link
    assert branch2_obj.parent.parent == root_obj
    assert leaf_obj.parent.parent == branch1_obj


def test_import_robot_with_ros2_control_and_gazebo() -> None:
    """Verify that ros2_control and Gazebo settings are synced to scene properties."""
    l1 = Link(name="l1")

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
        initial_links=[l1],
        initial_ros2_controls=[rc],
        initial_gazebo_elements=[gazebo],
    )

    success = import_robot_to_scene(robot, Path("/tmp/robot.urdf"), bpy.context)
    assert success is True

    # Verify scene properties
    lp = bpy.context.scene.linkforge
    assert lp.use_ros2_control is True
    assert lp.ros2_control_name == "RealRobot"
    assert len(lp.ros2_control_joints) == 1
    assert lp.controllers_yaml_path == "config.yaml"


def test_create_sensor_object_lidar() -> None:
    """Test creation of a LIDAR sensor in Blender."""
    # 1. Setup parent link object
    bpy.ops.object.empty_add(type="PLAIN_AXES")
    link_obj = bpy.context.active_object
    link_obj.name = "base_link"
    link_objects = {"base_link": link_obj}

    # 2. Setup Sensor Model
    sensor = Sensor(
        name="top_lidar",
        type=SensorType.LIDAR,
        link_name="base_link",
        lidar_info=LidarInfo(),
        origin=Transform(xyz=Vector3(0, 0, 0.5)),
    )

    # 3. Build
    sensor_obj = create_sensor_object(sensor, link_objects)

    # 4. Verify
    assert sensor_obj is not None
    assert sensor_obj.parent == link_obj
    assert pytest.approx(sensor_obj.location.z) == 0.5
    assert sensor_obj.linkforge_sensor.is_robot_sensor is True
    assert sensor_obj.linkforge_sensor.sensor_type == "LIDAR"


def test_create_sensor_object_imu_gps_camera() -> None:
    """Verify that IMU, GPS, and Camera sensors are correctly created in Blender."""
    # Setup a dummy link for sensors to attach to
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.name = "base_link"
    link_obj.linkforge.is_robot_link = True
    link_objects = {"base_link": link_obj}

    # 1. IMU
    imu = Sensor(name="imu_sensor", type=SensorType.IMU, link_name="base_link", imu_info=IMUInfo())
    obj_imu = create_sensor_object(imu, link_objects)
    assert obj_imu.linkforge_sensor.sensor_type == "IMU"

    # 2. GPS
    gps = Sensor(name="gps_sensor", type=SensorType.GPS, link_name="base_link", gps_info=GPSInfo())
    obj_gps = create_sensor_object(gps, link_objects)
    assert obj_gps.linkforge_sensor.sensor_type == "GPS"

    # 3. Camera
    cam = Sensor(
        name="cam_sensor", type=SensorType.CAMERA, link_name="base_link", camera_info=CameraInfo()
    )
    obj_cam = create_sensor_object(cam, link_objects)
    assert obj_cam.linkforge_sensor.sensor_type == "CAMERA"


def test_import_robot_with_mimic() -> None:
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

    robot = Robot(name="mimic_bot", initial_links=[l1, l2, l3], initial_joints=[j1, j2])

    # Import
    import_robot_to_scene(robot, Path("dummy.urdf"), bpy.context)

    # Verify mimic link
    follower = bpy.data.objects["follower"]
    driver = bpy.data.objects["driver"]
    assert follower.linkforge_joint.use_mimic is True
    assert follower.linkforge_joint.mimic_joint == driver
    assert pytest.approx(follower.linkforge_joint.mimic_multiplier) == 2.0


def test_import_mesh_file_stl(tmp_path) -> None:
    """Test importing a real STL mesh file."""
    # 1. Create a dummy STL file using Blender
    bpy.ops.mesh.primitive_cube_add()
    stl_path = tmp_path / "test.stl"
    # In modern Blender, stl_export might be in wm or export_mesh
    if hasattr(bpy.ops.wm, "stl_export"):
        bpy.ops.wm.stl_export(filepath=str(stl_path))
    else:
        # Fallback for older versions if needed, but we are on 4.5
        bpy.ops.export_mesh.stl(filepath=str(stl_path))

    bpy.ops.object.delete()  # Cleanup cube

    # 2. Import it
    obj = import_mesh_file(stl_path, "imported_stl")

    # 3. Verify
    assert obj is not None
    assert obj.name == "imported_stl"
    assert obj.type == "MESH"


def test_create_joint_object_prismatic() -> None:
    """Test creation of a PRISMATIC joint with limits and CUSTOM axis."""
    bpy.ops.object.empty_add()
    p_obj = bpy.context.active_object
    p_obj.name = "p"

    bpy.ops.object.empty_add()
    c_obj = bpy.context.active_object
    c_obj.name = "c"

    link_objects = {"p": p_obj, "c": c_obj}

    joint = Joint(
        name="prism_j",
        type=JointType.PRISMATIC,
        parent="p",
        child="c",
        axis=Vector3(0.70710678, 0.70710678, 0.0),  # Correct unit vector
        limits=JointLimits(lower=0, upper=1.0, effort=10, velocity=1),
    )

    obj = create_joint_object(joint, link_objects)
    assert obj is not None
    props = obj.linkforge_joint
    assert props.joint_type == "PRISMATIC"
    assert props.axis == "CUSTOM"
    # Expect normalized 1/sqrt(2) approx 0.707
    assert pytest.approx(props.custom_axis_x) == 0.7071067
    assert pytest.approx(props.custom_axis_y) == 0.7071067


def test_create_joint_object_continuous_floating() -> None:
    """Test creation of CONTINUOUS and FLOATING joints."""
    bpy.ops.object.empty_add()
    p_obj = bpy.context.active_object
    p_obj.name = "p_c"
    bpy.ops.object.empty_add()
    c_obj = bpy.context.active_object
    c_obj.name = "c_c"
    link_objects = {"p_c": p_obj, "c_c": c_obj}

    # 1. Continuous
    j_cont = Joint(
        name="cont", type=JointType.CONTINUOUS, parent="p_c", child="c_c", axis=Vector3(1, 0, 0)
    )
    obj_cont = create_joint_object(j_cont, link_objects)
    assert obj_cont.linkforge_joint.joint_type == "CONTINUOUS"

    # 2. Floating
    j_float = Joint(name="float_j", type=JointType.FLOATING, parent="p_c", child="c_c")
    obj_float = create_joint_object(j_float, link_objects)
    assert obj_float.linkforge_joint.joint_type == "FLOATING"


def test_create_link_object_with_mesh_visual(tmp_path) -> None:
    """Test creating a link that uses a mesh file for visual."""
    # 1. Create dummy STL
    bpy.ops.mesh.primitive_uv_sphere_add()
    mesh_path = tmp_path / "v.stl"
    if hasattr(bpy.ops.wm, "stl_export"):
        bpy.ops.wm.stl_export(filepath=str(mesh_path))
    else:
        bpy.ops.export_mesh.stl(filepath=str(mesh_path))
    bpy.ops.object.delete()

    # 2. Model
    mesh_geom = Mesh(resource="v.stl")
    visual = Visual(geometry=mesh_geom)
    link = Link(name="mesh_link", initial_visuals=[visual])

    # 3. Build (providing tmp_path as urdf_dir)
    robot = Robot(name="test")
    obj = create_link_object(link, robot, tmp_path)

    # 4. Verify
    assert obj is not None
    visual_obj = next(c for c in obj.children if "_visual" in c.name)
    assert visual_obj.type == "MESH"


def test_create_material_existing() -> None:
    """Test that existing materials are reused."""
    mat_name = "ExistingMat"
    mat = bpy.data.materials.new(name=mat_name)
    color = Color(1, 0, 0, 1)

    mat_out = create_material_from_color(color, mat_name)
    assert mat_out == mat


def test_import_mesh_file_unsupported() -> None:
    """Test handling of unsupported mesh formats."""
    obj = import_mesh_file(Path("unsupported.txt"), "test")
    assert obj is None


def test_create_primitive_mesh_invalid() -> None:
    """Test handling of unsupported geometry types."""
    obj = create_primitive_mesh(None, "test")
    assert obj is None


def test_create_sensor_with_gazebo_plugin() -> None:
    """Test that Gazebo plugins are preserved during import (no legacy filtering)."""
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.name = "base"
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

    obj = create_sensor_object(sensor, link_objects)
    assert obj is not None
    # All plugins should now be preserved (no legacy filtering)
    assert obj.linkforge_sensor.use_gazebo_plugin is True
    assert obj.linkforge_sensor.plugin_filename == "libgazebo_ros_camera.so"


def test_create_sensor_with_custom_plugin() -> None:
    """Test that custom (non-legacy) plugins are preserved."""
    bpy.ops.object.empty_add()
    link_obj = bpy.context.active_object
    link_obj.name = "base"
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

    obj = create_sensor_object(sensor, link_objects)
    assert obj is not None
    # Custom plugin should be preserved
    assert obj.linkforge_sensor.use_gazebo_plugin is True
    assert obj.linkforge_sensor.plugin_filename == "libmy_custom.so"


def test_import_robot_with_legacy_transmissions_skipped() -> None:
    """Test that legacy transmissions are skipped (no auto-conversion)."""
    from linkforge_core.models import Transmission, TransmissionActuator, TransmissionJoint

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
        initial_links=[l1, l2],
        initial_joints=[j1],
        initial_transmissions=[trans],
    )

    # Import
    success = import_robot_to_scene(robot, Path("/tmp/robot.urdf"), bpy.context)
    assert success is True

    # Auto-conversion is now disabled/removed.
    scene = bpy.context.scene
    assert scene.linkforge.use_ros2_control is False
    assert len(scene.linkforge.ros2_control_joints) == 0


def test_import_robot_skips_transmissions_when_ros2_control_exists() -> None:
    """Test that transmissions are skipped when ros2_control is present."""
    from linkforge_core.models import Transmission, TransmissionJoint

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
    )

    robot = Robot(
        name="hybrid_bot",
        initial_links=[l1, l2],
        initial_joints=[j1],
        initial_ros2_controls=[rc],
        initial_transmissions=[trans],
    )

    success = import_robot_to_scene(robot, Path("/tmp/robot.urdf"), bpy.context)
    assert success is True

    # ros2_control should take priority, transmission should be skipped
    scene = bpy.context.scene
    assert len(scene.linkforge.ros2_control_joints) == 1


def test_create_link_with_material() -> None:
    """Test link creation with visual material."""
    from linkforge_core.models import Material

    color = Color(r=1.0, g=0.0, b=0.0, a=1.0)
    material = Material(name="RedMat", color=color)
    visual = Visual(geometry=Box(size=Vector3(1, 1, 1)), material=material)
    link = Link(name="colored_link", initial_visuals=[visual])

    robot = Robot(name="test")
    obj = create_link_object(link, robot, Path("/tmp"))

    assert obj is not None
    assert obj.linkforge.use_material is True
    # Check that visual child has material
    visual_obj = next(c for c in obj.children if "_visual" in c.name)
    assert len(visual_obj.data.materials) > 0


def test_create_joint_with_custom_axis() -> None:
    """Test joint creation with non-standard axis."""
    bpy.ops.object.empty_add()
    p_obj = bpy.context.active_object
    p_obj.name = "p"

    bpy.ops.object.empty_add()
    c_obj = bpy.context.active_object
    c_obj.name = "c"

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

    obj = create_joint_object(joint, link_objects)
    assert obj is not None
    props = obj.linkforge_joint
    assert props.axis == "CUSTOM"
    assert props.custom_axis_x != 0.0


def test_import_mesh_file_nonexistent() -> None:
    """Test handling of non-existent mesh files."""
    obj = import_mesh_file(Path("/nonexistent/file.stl"), "test")
    assert obj is None


def test_create_link_with_collision_mesh(tmp_path) -> None:
    """Test link creation with mesh collision geometry."""
    # Create dummy STL
    bpy.ops.mesh.primitive_cube_add()
    mesh_path = tmp_path / "collision.stl"
    if hasattr(bpy.ops.wm, "stl_export"):
        bpy.ops.wm.stl_export(filepath=str(mesh_path))
    else:
        bpy.ops.export_mesh.stl(filepath=str(mesh_path))
    bpy.ops.object.delete()

    # Model
    mesh_geom = Mesh(resource="collision.stl")
    collision = Collision(geometry=mesh_geom)
    link = Link(name="mesh_coll_link", initial_collisions=[collision])

    # Build
    robot = Robot(name="test")
    obj = create_link_object(link, robot, tmp_path)

    # Verify
    assert obj is not None
    coll_obj = next(c for c in obj.children if "_collision" in c.name)
    assert coll_obj["imported_from_urdf"] is True
    assert coll_obj["collision_geometry_type"] == "MESH"
    assert obj.linkforge.collision_quality == 100.0


def test_create_primitive_mesh_cylinder_sphere() -> None:
    """Test creation of Cylinder and Sphere primitives."""
    # Cylinder
    cyl = Cylinder(radius=0.5, length=2.0)
    obj_cyl = create_primitive_mesh(cyl, "test_cyl")
    assert obj_cyl is not None
    assert obj_cyl["urdf_geometry_type"] == "CYLINDER"
    assert pytest.approx(obj_cyl.dimensions.z) == 2.0

    # Sphere
    sphere = Sphere(radius=1.0)
    obj_sphere = create_primitive_mesh(sphere, "test_sphere")
    assert obj_sphere is not None
    assert obj_sphere["urdf_geometry_type"] == "SPHERE"
    assert pytest.approx(obj_sphere.dimensions.x) == 2.0


def test_sensor_unknown_link() -> None:
    """Test sensor creation with unknown link name."""
    sensor = Sensor(
        name="orphan_sensor",
        type=SensorType.CAMERA,
        link_name="nonexistent_link",
        camera_info=CameraInfo(),
    )

    obj = create_sensor_object(sensor, {})
    assert obj is None


def test_joint_axis_standard_axes() -> None:
    """Test joint axis detection for standard X, Y, Z axes."""
    bpy.ops.object.empty_add()
    p_obj = bpy.context.active_object
    p_obj.name = "p"

    bpy.ops.object.empty_add()
    c_obj = bpy.context.active_object
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
    obj_x = create_joint_object(j_x, link_objects)
    assert obj_x.linkforge_joint.axis == "X"

    # Test Y axis
    j_y = Joint(
        name="j_y",
        type=JointType.REVOLUTE,
        parent="p",
        child="c",
        axis=Vector3(0, 1, 0),
        limits=JointLimits(0, 1, 10, 1),
    )
    obj_y = create_joint_object(j_y, link_objects)
    assert obj_y.linkforge_joint.axis == "Y"


def test_import_robot_topological_sort() -> None:
    """Test that joints are created in correct topological order."""
    # Create a chain where joints must be sorted
    l1 = Link(name="root")
    l2 = Link(name="mid")
    l3 = Link(name="leaf")

    # Define joints out of order
    j2 = Joint(name="j2", parent="mid", child="leaf", type=JointType.FIXED)
    j1 = Joint(name="j1", parent="root", child="mid", type=JointType.FIXED)

    robot = Robot(name="chain_bot", initial_links=[l1, l2, l3], initial_joints=[j2, j1])

    success = import_robot_to_scene(robot, Path("/tmp/robot.urdf"), bpy.context)
    assert success is True

    # Verify hierarchy is correct despite out-of-order definition
    root = bpy.data.objects["root"]
    mid = bpy.data.objects["mid"]
    leaf = bpy.data.objects["leaf"]

    assert mid.parent.parent == root
    assert leaf.parent.parent == mid


def test_normalize_and_consolidate_imported_objects() -> None:
    """Test the robust mesh normalization and consolidation logic."""
    from linkforge.blender.adapters.core_to_blender import (
        normalize_and_consolidate_imported_objects,
    )

    bpy.ops.wm.read_factory_settings(use_empty=True)
    import linkforge.blender

    linkforge.blender.register()

    # Create a hierarchy: Empty -> Mesh -> Mesh
    bpy.ops.object.empty_add(location=(1, 1, 1))
    root = bpy.context.active_object
    root.name = "root_empty"

    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(2, 2, 2))
    m1 = bpy.context.active_object
    m1.name = "mesh1"
    m1.parent = root

    bpy.ops.mesh.primitive_uv_sphere_add(radius=0.5, location=(3, 3, 3))
    m2 = bpy.context.active_object
    m2.name = "mesh2"
    m2.parent = m1

    bpy.context.view_layer.update()

    # Store names before they are potentially deleted
    root_name = root.name
    m1_name = m1.name
    m2_name = m2.name

    # Consolidate
    res = normalize_and_consolidate_imported_objects([root], "consolidated")

    assert res is not None
    assert res.name == "consolidated"
    assert res.type == "MESH"
    assert len(res.data.vertices) > 0
    # Check that root empty and original meshes are queued for deletion (implicitly cleaned by the function)
    assert root_name not in bpy.data.objects
    assert m1_name not in bpy.data.objects
    assert m2_name not in bpy.data.objects


def test_create_joint_object_mimic_logic() -> None:
    """Test that mimics are correctly resolved even if created out of order."""
    from linkforge.blender.adapters.core_to_blender import create_joint_object
    from linkforge_core.models import Joint, JointMimic, JointType

    bpy.ops.wm.read_factory_settings(use_empty=True)
    import linkforge.blender

    linkforge.blender.register()

    # Parent/Child link shells
    bpy.ops.object.empty_add()
    p = bpy.context.active_object
    p.name = "p"
    bpy.ops.object.empty_add()
    c = bpy.context.active_object
    c.name = "c"

    # Target joint object
    bpy.ops.object.empty_add()
    driver_obj = bpy.context.active_object
    driver_obj.name = "driver_joint"

    link_objects = {"p": p, "c": c}
    # Mocking the discovery of the driver joint in scene
    # The actual implementation looks for objects by name

    from linkforge_core.models import JointLimits

    joint = Joint(
        name="follower",
        type=JointType.REVOLUTE,
        parent="p",
        child="c",
        axis=Vector3(1, 0, 0),
        limits=JointLimits(lower=-1.0, upper=1.0, effort=10.0, velocity=1.0),
        mimic=JointMimic(joint="driver_joint", multiplier=0.5),
    )

    res = create_joint_object(joint, link_objects)
    assert res.linkforge_joint.use_mimic is True

    assert res.linkforge_joint.mimic_multiplier == 0.5


def test_sensor_noise_properties(clean_scene) -> None:
    """Verify sensor noise property mapping for LIDAR, IMU, and GPS."""
    # 1. LIDAR with noise
    lidar = Sensor(
        name="Lidar",
        type=SensorType.LIDAR,
        link_name="base_link",
        lidar_info=LidarInfo(
            horizontal_samples=640, noise=SensorNoise(type="gaussian", mean=0.0, stddev=0.01)
        ),
    )

    # 2. IMU with noise
    imu = Sensor(
        name="IMU",
        type=SensorType.IMU,
        link_name="base_link",
        imu_info=IMUInfo(
            angular_velocity_noise=SensorNoise(type="gaussian", mean=0.0, stddev=0.001)
        ),
    )

    # 3. GPS with noise
    gps = Sensor(
        name="GPS",
        type=SensorType.GPS,
        link_name="base_link",
        gps_info=GPSInfo(
            position_sensing_horizontal_noise=SensorNoise(type="gaussian", mean=0.0, stddev=0.5)
        ),
    )

    base_link = bpy.data.objects.new("base_link", None)
    bpy.context.collection.objects.link(base_link)
    link_map = {"base_link": base_link}

    for s in [lidar, imu, gps]:
        obj = create_sensor_object(s, link_map)
        assert obj is not None
        assert obj.linkforge_sensor.use_noise is True
        assert obj.linkforge_sensor.noise_type == "gaussian"

        if s.type == SensorType.LIDAR:
            assert obj.linkforge_sensor.noise_stddev == pytest.approx(0.01)
        elif s.type == SensorType.IMU:
            assert obj.linkforge_sensor.noise_stddev == pytest.approx(0.001)
        elif s.type == SensorType.GPS:
            assert obj.linkforge_sensor.noise_stddev == pytest.approx(0.5)


def test_multi_visual_collision_naming(clean_scene) -> None:
    """Verify suffix naming for multiple unnamed visuals and collisions."""
    box_geom = Box(size=Vector3(1, 1, 1))

    link = Link(
        name="multi_link",
        initial_visuals=[Visual(geometry=box_geom), Visual(geometry=box_geom)],
        initial_collisions=[Collision(geometry=box_geom), Collision(geometry=box_geom)],
    )

    robot = Robot(name="test")
    obj = create_link_object(link, robot, Path("/cwd"), collection=bpy.context.collection)
    assert obj is not None

    # Check visuals: naming should follow {link_name}_visual_{idx}
    visuals = [c for c in obj.children if "_visual_" in c.name]
    assert len(visuals) == 2, f"Expected 2 visuals, found {[c.name for c in obj.children]}"

    # Check collisions: naming should follow {link_name}_collision_{idx}
    collisions = [c for c in obj.children if "_collision_" in c.name]
    assert len(collisions) == 2, f"Expected 2 collisions, found {[c.name for c in obj.children]}"


def test_normalize_consolidate_empty_cleanup(clean_scene) -> None:
    """Verify cleanup when no meshes are found in imported objects."""
    empty = bpy.data.objects.new("EmptyContainer", None)
    bpy.context.collection.objects.link(empty)

    res = normalize_and_consolidate_imported_objects([empty], "Final")
    assert res is None
    assert "EmptyContainer" not in bpy.data.objects


def test_import_robot_sensor_creation_failure(clean_scene) -> None:
    """Verify import_robot_to_scene handles sensor creation failure."""
    robot = Robot(
        name="test_robot",
        initial_links=[Link(name="base_link")],
        initial_sensors=[
            Sensor(
                name="BadSensor",
                type=SensorType.CAMERA,
                link_name="base_link",
                camera_info=CameraInfo(),
            )
        ],
    )

    context = mock.MagicMock()
    context.scene = bpy.context.scene
    context.view_layer = bpy.context.view_layer

    with mock.patch(
        "linkforge.blender.adapters.core_to_blender.create_sensor_object", return_value=None
    ):
        import_robot_to_scene(robot, Path("test.urdf"), context)
        assert "base_link" in bpy.data.objects


def test_import_mesh_file_removes_non_mesh_stragglers(tmp_path) -> None:
    """Verify that import side-effects (Camera, Lamp, Empties) are cleaned up."""
    # Create a real STL file to import
    bpy.ops.mesh.primitive_cube_add()
    stl_path = tmp_path / "cube.stl"
    if hasattr(bpy.ops.wm, "stl_export"):
        bpy.ops.wm.stl_export(filepath=str(stl_path))
    else:
        bpy.ops.export_mesh.stl(filepath=str(stl_path))
    bpy.ops.object.delete()

    # Wrap the real importer so we can inject a Camera side-effect during the call,
    # matching exactly what Blender's Collada importer does for embedded scene nodes.
    real_import = bpy.ops.wm.stl_import
    injected_cam_name = "DAE_Camera_SideEffect"

    def import_with_sideeffect(**kwargs):
        result = real_import(**kwargs)
        cam_data = bpy.data.cameras.new("_tmp_cam")
        cam_obj = bpy.data.objects.new(injected_cam_name, cam_data)
        bpy.context.scene.collection.objects.link(cam_obj)
        return result

    with mock.patch(
        "linkforge.blender.adapters.core_to_blender.bpy.ops.wm.stl_import",
        side_effect=import_with_sideeffect,
    ):
        result = import_mesh_file(stl_path, "mesh_only")

    assert result is not None
    assert result.type == "MESH"
    assert injected_cam_name not in bpy.data.objects, (
        "Camera straggler was not removed by import_mesh_file"
    )
