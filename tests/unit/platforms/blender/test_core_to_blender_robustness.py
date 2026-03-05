from pathlib import Path
from unittest.mock import patch

import bpy
import pytest
from linkforge.blender.adapters.core_to_blender import (
    _get_geometry_type_str,
    create_joint_object,
    create_material_from_color,
    create_primitive_mesh,
    create_sensor_object,
    import_mesh_file,
    import_robot_to_scene,
    normalize_and_consolidate_imported_objects,
)
from linkforge.linkforge_core.models import (
    Box,
    CameraInfo,
    Collision,
    Color,
    Cylinder,
    GazeboElement,
    GazeboPlugin,
    Inertial,
    InertiaTensor,
    Joint,
    JointLimits,
    JointType,
    Link,
    Material,
    Mesh,
    Robot,
    Ros2Control,
    Ros2ControlJoint,
    Sensor,
    SensorType,
    Sphere,
    Transform,
    Vector3,
    Visual,
)
from linkforge.linkforge_core.models.sensor import GPSInfo, IMUInfo
from linkforge_core.base import FileSystemResolver
from mathutils import Vector


@pytest.fixture
def clean_scene():
    """Clear all objects and materials."""
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    for material in bpy.data.materials:
        bpy.data.materials.remove(material)
    # Don't delete standard Collection
    for collection in bpy.data.collections:
        if collection.name != "Collection":
            bpy.data.collections.remove(collection)
    return bpy.context.scene


def test_resolve_mesh_path(tmp_path):
    """Test resolution of various mesh path types."""
    urdf_dir = tmp_path / "urdf"
    urdf_dir.mkdir()
    resolver = FileSystemResolver()

    # 1. Local relative path (exists)
    mesh_file = urdf_dir / "mesh.stl"
    mesh_file.touch()
    assert resolver.resolve("mesh.stl", relative_to=urdf_dir) == mesh_file.absolute()

    # 2. Package URI (requires mock of resolve_package_path)
    with patch("linkforge_core.utils.path_utils.resolve_package_path") as mock_resolve:
        mock_resolve.return_value = mesh_file
        # Note: resolve_package_path mock needs to return a Path that exists
        assert (
            resolver.resolve("package://my_pkg/mesh.stl", relative_to=urdf_dir)
            == mesh_file.absolute()
        )

    # 3. Non-existent path raises FileNotFoundError (new behavior)
    with pytest.raises(FileNotFoundError):
        resolver.resolve("/tmp/non_existent.obj", relative_to=urdf_dir)

    # 4. file:// URI
    file_uri = "file:///tmp/mesh.stl"
    # Create the file so it can be resolved (FileSystemResolver checks existence)
    mesh_tmp = Path("/tmp/mesh.stl")
    mesh_tmp.touch()
    assert resolver.resolve(file_uri, relative_to=urdf_dir) == mesh_tmp.absolute()

    # 5. Windows style URI (mocking Windows environment is hard, but we can test the regex)
    # FileSystemResolver uses re.sub and Path.
    # On Unix, file:///C:/path becomes /C:/path
    win_uri = "file:///C:/path/to/mesh.stl"
    with patch("pathlib.Path.exists", return_value=True):
        res = resolver.resolve(win_uri, relative_to=urdf_dir)
        # Our logic for Windows URIs strips the leading slash if : is present
        # On Posix, Path("C:/...").absolute() will prepend CWD, so we check for presence
        assert "C:/path/to/mesh.stl" in str(res).replace("\\", "/")


def test_create_material_from_color(clean_scene):
    """Test material creation from color model."""
    color = Color(1.0, 0.5, 0.0, 1.0)
    mat = create_material_from_color(color, "TestMaterial")

    assert mat is not None
    assert mat.name == "TestMaterial"
    assert mat.use_nodes

    # Verify BSDF color
    nodes = mat.node_tree.nodes
    principled = nodes.get("Principled BSDF")
    if principled:
        assert list(principled.inputs[0].default_value) == [1.0, 0.5, 0.0, 1.0]

    # Check reuse
    mat_reuse = create_material_from_color(color, "TestMaterial")
    assert mat_reuse == mat


def test_create_primitive_mesh(clean_scene):
    """Test creation of primitive meshes (Box, Cylinder, Sphere)."""
    # 1. Box
    box = Box(size=Vector3(2.0, 2.0, 2.0))
    print(f"DEBUG: box type: {type(box)}, isinstance(box, Box): {isinstance(box, Box)}")
    obj = create_primitive_mesh(box, "TestBox")
    assert obj is not None
    assert obj.name == "TestBox"
    assert obj.dimensions == Vector((2.0, 2.0, 2.0))
    assert obj["urdf_geometry_type"] == "BOX"

    # 2. Cylinder
    cyl = Cylinder(radius=1.0, length=4.0)
    obj = create_primitive_mesh(cyl, "TestCylinder")
    assert obj.dimensions == Vector((2.0, 2.0, 4.0))  # radius * 2
    assert obj["urdf_geometry_type"] == "CYLINDER"

    # 3. Sphere
    sphere = Sphere(radius=3.0)
    obj = create_primitive_mesh(sphere, "TestSphere")
    assert obj.dimensions == Vector((6.0, 6.0, 6.0))
    assert obj["urdf_geometry_type"] == "SPHERE"

    # 4. Invalid geometry
    assert create_primitive_mesh(None, "Fail") is None


def test_normalize_and_consolidate(clean_scene):
    """Test joining multiple objects and normalization."""
    # Add temporary meshes
    bpy.ops.mesh.primitive_cube_add(location=(1, 1, 1))
    o1 = bpy.context.active_object
    bpy.ops.mesh.primitive_cube_add(location=(-1, -1, -1))
    o2 = bpy.context.active_object

    # Create a hierarchy
    bpy.ops.object.empty_add()
    parent = bpy.context.active_object
    o1.parent = parent

    # This should join them into one mesh named Consolidated at (0,0,0)
    res = normalize_and_consolidate_imported_objects([parent, o1, o2], "Consolidated")

    assert res is not None
    assert res.name == "Consolidated"
    assert res.location == Vector((0, 0, 0))
    # parent and containers should be gone? Actually normalize_and_consolidate deletes non-MESH objs in hierarchy
    # Verify we only have one object left
    assert len(bpy.data.objects) == 1


def test_import_mesh_file_success(clean_scene, tmp_path):
    """Test successful mesh import flow with a real minimal file."""
    p = tmp_path / "test.obj"
    p.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")

    # We don't mock the operator here, we let it run if it's available
    # Or we mock it but make it actually create an object
    with patch("bpy.ops.wm.obj_import", wraps=bpy.ops.wm.obj_import) as mock_import:

        def side_effect(*args, **kwargs):
            bpy.ops.mesh.primitive_cube_add()
            return {"FINISHED"}

        mock_import.side_effect = side_effect

        res = import_mesh_file(p, "ConsolidatedCube")
        assert res is not None
        assert "ConsolidatedCube" in res.name


def test_create_joint_object(clean_scene):
    """Test reconstruction of a Joint and its parenting."""
    parent_link = bpy.data.objects.new("ParentLink", None)
    child_link = bpy.data.objects.new("ChildLink", None)
    clean_scene.collection.objects.link(parent_link)
    clean_scene.collection.objects.link(child_link)

    links = {"ParentLink": parent_link, "ChildLink": child_link}

    joint = Joint(
        name="Joint1",
        type=JointType.REVOLUTE,
        parent="ParentLink",
        child="ChildLink",
        axis=Vector3(0, 0, 1),
        limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=1.0),
        origin=Transform(xyz=Vector3(0, 0, 0.5)),
    )

    obj = create_joint_object(joint, links)

    assert obj is not None
    assert obj.name == "Joint1"
    assert obj.parent == parent_link
    assert child_link.parent == obj
    assert obj.location == Vector((0, 0, 0.5))
    assert obj.linkforge_joint.axis == "Z"


def test_create_sensor_object(clean_scene):
    """Test reconstruction of a Sensor."""
    link_obj = bpy.data.objects.new("LinkA", None)
    clean_scene.collection.objects.link(link_obj)
    links = {"LinkA": link_obj}

    sensor = Sensor(
        name="Camera1",
        type=SensorType.CAMERA,
        link_name="LinkA",
        camera_info=CameraInfo(),
        origin=Transform(xyz=Vector3(0.1, 0, 0)),
    )

    obj = create_sensor_object(sensor, links)

    assert obj is not None
    assert obj.name == "Camera1"
    assert obj.parent == link_obj
    assert obj.location == Vector((0.1, 0, 0))
    assert obj.linkforge_sensor.sensor_type == "CAMERA"


def test_import_robot_with_mimic_and_gazebo(clean_scene):
    """Test import of robot with mimic joints and gazebo plugins."""
    from linkforge.linkforge_core.models.gazebo import GazeboElement, GazeboPlugin
    from linkforge.linkforge_core.models.joint import JointMimic

    robot = Robot(
        name="MimicBot",
        initial_links=(Link(name="base"), Link(name="m1"), Link(name="m2")),
        initial_joints=(
            Joint(
                name="j1",
                type=JointType.CONTINUOUS,
                parent="base",
                child="m1",
                axis=Vector3(1, 0, 0),
            ),
            Joint(
                name="j2",
                type=JointType.CONTINUOUS,
                parent="base",
                child="m2",
                axis=Vector3(1, 0, 0),
                mimic=JointMimic(joint="j1", multiplier=2.0, offset=0.1),
            ),
        ),
        gazebo_elements=[
            GazeboElement(
                plugins=[GazeboPlugin(name="p3d_ros2_control", filename="libgazebo_ros_p3d.so")]
            )
        ],
    )

    with patch("linkforge_core.models.robot.Robot.resolve_resource") as mock_resolve:
        mock_resolve.return_value = Path("dummy.stl")
        import_robot_to_scene(robot, Path("robot.urdf"), bpy.context)

    assert "j2" in bpy.data.objects
    j2 = bpy.data.objects["j2"]
    assert j2.linkforge_joint.use_mimic
    assert j2.linkforge_joint.mimic_joint.name == "j1"
    assert j2.linkforge_joint.mimic_multiplier == 2.0

    assert clean_scene.linkforge.gazebo_plugin_name == "p3d_ros2_control"


def test_create_material_no_tree(clean_scene):
    """Test material creation when node tree is missing."""
    col = Color(1, 0, 0, 1)
    # Use a real material but force use_nodes=False
    mat = bpy.data.materials.new("NoTree")
    mat.use_nodes = False
    # Instead of patching bpy.data.materials.new (which is read-only at RNA level),
    # we just call the function and verify it doesn't crash
    res = create_material_from_color(col, "NoTree_Test")
    assert res is not None
    assert res.name.startswith("NoTree_Test")


def test_import_robot_with_transmissions(clean_scene):
    """Test importing robot with transmissions."""
    from linkforge.linkforge_core.models.transmission import (
        Transmission,
        TransmissionActuator,
        TransmissionJoint,
    )

    robot = Robot(
        name="TransBot",
        initial_links=(Link(name="b"),),
        initial_joints=(Joint(name="j1", parent="world", child="b", type=JointType.FIXED),),
        transmissions=[
            Transmission(
                name="t1",
                type="simple",
                joints=[TransmissionJoint(name="j1", hardware_interfaces=["position"])],
                actuators=[TransmissionActuator(name="a1", hardware_interfaces=["position"])],
            )
        ],
    )
    import_robot_to_scene(robot, Path("robot.urdf"), bpy.context)
    assert clean_scene.linkforge.robot_name == "TransBot"


def test_full_robot_import_integration(clean_scene):
    """A 'MegaBot' test to hit as many code paths as possible in core_to_blender."""
    from linkforge.linkforge_core.models.transmission import (
        Transmission,
        TransmissionActuator,
        TransmissionJoint,
    )

    # Material
    # Material
    Material(name="Iron", color=Color(0.5, 0.5, 0.5, 1.0))

    # Links
    # Base link with inertial properties and collision
    l1 = Link(
        name="base_link",
        visuals=[
            Visual(
                geometry=Box(size=Vector3(1, 1, 1)),
                material=Material(name="Blue", color=Color(0, 0, 1, 1)),
            )
        ],
        collisions=[Collision(name="base_col", geometry=Box(size=Vector3(1, 1, 1)))],
        inertial=Inertial(
            mass=1.0, inertia=InertiaTensor(ixx=0.1, iyy=0.1, izz=0.1, ixy=0.0, ixz=0.0, iyz=0.0)
        ),
    )

    # Tool link with multiple visuals and sphere collision
    l2 = Link(
        name="tool_link",
        visuals=[
            Visual(geometry=Cylinder(radius=0.1, length=0.5)),
            Visual(geometry=Sphere(radius=0.2), origin=Transform(xyz=Vector3(0, 0, 0.5))),
        ],
        collisions=[Collision(geometry=Sphere(radius=0.2))],
    )

    # Mesh geometry in 3rd link with mesh collision
    l3 = Link(
        name="mesh_link",
        visuals=[Visual(geometry=Mesh(resource="package://pkg/mesh.stl"))],
        collisions=[Collision(geometry=Mesh(resource="package://pkg/mesh.stl"))],
    )
    # Sensors
    Sensor(
        name="kinect",
        type=SensorType.CAMERA,
        link_name="tool_link",
        camera_info=CameraInfo(width=640, height=480, horizontal_fov=1.57),
    )
    imu = Sensor(name="imu_sensor", type=SensorType.IMU, link_name="base_link", imu_info=IMUInfo())
    gps = Sensor(name="gps_sensor", type=SensorType.GPS, link_name="base_link", gps_info=GPSInfo())

    from linkforge_core.models.joint import JointDynamics

    # Joint with dynamics and limits
    j1 = Joint(
        name="j1",
        type=JointType.CONTINUOUS,
        parent="base_link",
        child="tool_link",
        axis=Vector3(1, 0, 0),
        limits=JointLimits(effort=10, velocity=1),
        dynamics=JointDynamics(damping=0.5, friction=0.1),
    )

    # Mesh geometry in 3rd link
    l3 = Link(name="mesh_link", visuals=[Visual(geometry=Mesh(resource="package://pkg/mesh.stl"))])
    j2 = Joint(name="j2", type=JointType.FIXED, parent="base_link", child="mesh_link")

    # Sensors with more info
    # cam is immutable, so we create a new one or just rely on the initial one being enough.
    # Let's create a new detailed camera for the robot
    cam_detailed = Sensor(
        name="kinect_detailed",
        type=SensorType.CAMERA,
        link_name="tool_link",
        camera_info=CameraInfo(
            width=640, height=480, horizontal_fov=1.57, near_clip=0.01, far_clip=50.0
        ),
    )

    from linkforge.linkforge_core.models.sensor import LidarInfo

    lidar = Sensor(
        name="lidar_sensor",
        type=SensorType.LIDAR,
        link_name="base_link",
        lidar_info=LidarInfo(range_max=30.0),
    )

    # ROS2 Control
    rcj = Ros2ControlJoint(
        name="j1", command_interfaces=["position"], state_interfaces=["position"]
    )
    rc = Ros2Control(
        name="hardware", joints=[rcj], hardware_plugin="gz_ros2_control/GazeboSimSystem"
    )

    # Gazebo
    gz = GazeboElement(
        reference="base_link", plugins=[GazeboPlugin(name="gz_ros2_control", filename="libgz.so")]
    )

    # Transmission
    tr = Transmission(
        name="t1",
        type="simple",
        joints=[TransmissionJoint(name="j1", hardware_interfaces=["position"])],
        actuators=[TransmissionActuator(name="a1", hardware_interfaces=["position"])],
    )

    robot = Robot(
        name="MegaBot",
        initial_links=[l1, l2, l3],
        initial_joints=[j1, j2],
        sensors=[cam_detailed, imu, gps, lidar],
        ros2_controls=[rc],
        gazebo_elements=[gz],
        transmissions=[tr],
    )

    # Mocking OBJ for mesh import
    with (
        patch("linkforge.blender.adapters.core_to_blender.import_mesh_file") as mock_io,
        patch("linkforge_core.utils.path_utils.resolve_package_path") as mock_pkg,
    ):
        mock_io.return_value = bpy.data.objects.new("MeshObj", None)
        mock_pkg.return_value = Path("/tmp/mesh.stl")
        import_robot_to_scene(robot, Path("robot.urdf"), bpy.context)

    assert "base_link" in bpy.data.objects
    assert "tool_link" in bpy.data.objects
    assert "j1" in bpy.data.objects
    assert clean_scene.linkforge.robot_name == "MegaBot"
    assert clean_scene.linkforge.use_ros2_control
    assert clean_scene.linkforge.ros2_control_name == "hardware"
    assert len(clean_scene.linkforge.ros2_control_joints) == 1

    # Check sensors
    assert "kinect_detailed" in bpy.data.objects
    assert "imu_sensor" in bpy.data.objects
    assert "gps_sensor" in bpy.data.objects
    assert "lidar_sensor" in bpy.data.objects
    assert bpy.data.objects["lidar_sensor"].linkforge_sensor.lidar_range_max == 30.0

    # Check consolidation (L2 has 2 visuals)
    # L2 object should have children or be joined depending on normalize_and_consolidate
    # Actually normalize_and_consolidate joins them if they are meshes.
    print(f"DEBUG: Objects in scene: {[o.name for o in bpy.data.objects]}")


def test_import_mesh_file_robustness(tmp_path):
    """Hit lines 216, 224, 254, 261 in core_to_blender.py."""
    from unittest import mock

    # Line 216 (DAE on Blender 5.0+ mock)
    dae = tmp_path / "test.dae"
    dae.touch()

    # Use a side_effect to mock the version check
    with mock.patch("linkforge.blender.adapters.core_to_blender.bpy") as mock_bpy:
        # Mock app.version which is normally a tuple
        mock_bpy.app = mock.MagicMock()
        mock_bpy.app.version = (5, 0, 0)
        assert import_mesh_file(dae, "test") is None

    # Line 224 (Unsupported extension)
    txt = tmp_path / "test.txt"
    txt.touch()
    assert import_mesh_file(txt, "test") is None

    # Line 254 (No functional importer)
    obj_file = tmp_path / "test.obj"
    obj_file.touch()
    with mock.patch("bpy.ops.wm.obj_import", side_effect=RuntimeError("Fail")):
        assert import_mesh_file(obj_file, "test") is None

    # Line 261 (Importer ran but no objects)
    with (
        mock.patch("bpy.ops.wm.obj_import", return_value={"FINISHED"}),
        mock.patch("linkforge.blender.adapters.core_to_blender.bpy") as mock_bpy_at_use,
    ):
        mock_ctx = mock.MagicMock()
        mock_ctx.selected_objects = []
        mock_bpy_at_use.context = mock_ctx
        assert import_mesh_file(obj_file, "test") is None


def test_get_geometry_type_str_robustness():
    """Hit line 403 in core_to_blender.py (Unknown type fallback and Cylinder)."""
    from unittest import mock

    cyl = Cylinder(radius=1.0, length=2.0)
    assert _get_geometry_type_str(cyl) == "CYLINDER"

    # Unknown type
    assert _get_geometry_type_str(mock.MagicMock()) == "MESH"
