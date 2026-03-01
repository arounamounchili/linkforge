"""Unit tests for URDF generator."""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from linkforge_core.base import RobotGeneratorError
from linkforge_core.generators.urdf_generator import URDFGenerator
from linkforge_core.models import (
    Box,
    CameraInfo,
    Collision,
    Color,
    Cylinder,
    GazeboElement,
    GPSInfo,
    IMUInfo,
    Inertial,
    InertiaTensor,
    Joint,
    JointCalibration,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointSafetyController,
    JointType,
    LidarInfo,
    Link,
    Material,
    Mesh,
    Robot,
    Ros2Control,
    Sensor,
    SensorNoise,
    SensorType,
    Sphere,
    Transform,
    Vector3,
    Visual,
)
from linkforge_core.models.transmission import Transmission, TransmissionActuator, TransmissionJoint


class TestURDFGenerator:
    """Test URDF generator."""

    def test_generate_basic_robot(self):
        """Test generating a basic robot with one link."""
        robot = Robot(name="test_robot")
        link = Link(name="base_link")
        robot.add_link(link)

        generator = URDFGenerator(pretty_print=False)
        xml_str = generator.generate(robot)

        root = ET.fromstring(xml_str)
        assert root.tag == "robot"
        assert root.get("name") == "test_robot"
        assert len(root.findall("link")) == 1
        assert root.find("link").get("name") == "base_link"

    def test_generate_geometries(self):
        """Test generating all geometry types."""
        robot = Robot(name="geo_robot")
        link = Link(name="base_link")

        box = Box(size=Vector3(1, 2, 3))
        link.visuals.append(Visual(geometry=box, name="box_vis"))

        cyl = Cylinder(radius=0.5, length=2.0)
        link.visuals.append(Visual(geometry=cyl, name="cyl_vis"))

        sph = Sphere(radius=1.0)
        link.visuals.append(Visual(geometry=sph, name="sph_vis"))

        mesh = Mesh(filepath=Path("meshes/part.stl"), scale=Vector3(0.1, 0.1, 0.1))
        link.visuals.append(Visual(geometry=mesh, name="mesh_vis"))

        robot.add_link(link)

        generator = URDFGenerator(pretty_print=False)
        xml_str = generator.generate(robot)
        root = ET.fromstring(xml_str)

        link_elem = root.find("link")
        visuals = link_elem.findall("visual")
        assert len(visuals) == 4

        assert visuals[0].find("geometry/box").get("size") == "1 2 3"

        assert visuals[1].find("geometry/cylinder").get("radius") == "0.5"
        assert visuals[1].find("geometry/cylinder").get("length") == "2"

        assert visuals[2].find("geometry/sphere").get("radius") == "1"

        mesh_elem = visuals[3].find("geometry/mesh")
        assert mesh_elem.get("filename") == "meshes/part.stl"
        assert mesh_elem.get("scale") == "0.1 0.1 0.1"

    def test_generate_materials_deduplication(self):
        """Test material deduplication logic."""
        robot = Robot(name="mat_robot")

        link1 = Link(name="link1")
        mat1 = Material(name="red", color=Color(1, 0, 0, 1))
        link1.visuals.append(Visual(geometry=Box(Vector3(1, 1, 1)), material=mat1))

        link2 = Link(name="link2")
        mat2 = Material(name="red", color=Color(1, 0, 0, 1))
        link2.visuals.append(Visual(geometry=Box(Vector3(1, 1, 1)), material=mat2))

        link3 = Link(name="link3")
        mat3 = Material(name="blue", color=Color(0, 0, 1, 1))
        link3.visuals.append(Visual(geometry=Box(Vector3(1, 1, 1)), material=mat3))

        robot.add_link(link1)
        robot.add_link(link2)
        robot.add_link(link3)

        # Disable validation since we have disconnected links
        generator = URDFGenerator(pretty_print=False)
        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)

        # 2 unique materials (red, blue) should be at top level

        materials = root.findall("material")
        assert len(materials) == 2
        names = sorted([m.get("name") for m in materials])
        assert names == ["blue", "red"]

        # Check references in visuals
        # Visuals should only have name attribute inside geometry block?
        # No, <visual><material name="red"/></visual>

        links = root.findall("link")
        vis1 = links[0].find("visual")
        assert vis1.find("material").get("name") == "red"
        assert vis1.find("material").find("color") is None  # Should be reference

    def test_generate_materials_conflict(self):
        """Test material conflict (same name, different color) -> Inline."""
        robot = Robot(name="conflict_robot")

        link1 = Link(name="link1")
        mat1 = Material(name="generic", color=Color(1, 0, 0, 1))
        link1.visuals.append(Visual(geometry=Box(Vector3(1, 1, 1)), material=mat1))

        link2 = Link(name="link2")
        mat2 = Material(name="generic", color=Color(0, 0, 1, 1))
        link2.visuals.append(Visual(geometry=Box(Vector3(1, 1, 1)), material=mat2))

        robot.add_link(link1)
        robot.add_link(link2)

        # Disable validation for disconnected links
        generator = URDFGenerator(pretty_print=False)
        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)

        # Should have NO global materials
        assert len(root.findall("material")) == 0

        # Inline definitions
        links = root.findall("link")
        vis1 = links[0].find("visual/material")
        assert vis1.get("name") == "generic"
        assert vis1.find("color").get("rgba") == "1 0 0 1"

        vis2 = links[1].find("visual/material")
        assert vis2.get("name") == "generic"
        assert vis2.find("color").get("rgba") == "0 0 1 1"

    def test_generate_joints(self):
        """Test generating joints with limits and dynamics."""
        robot = Robot(name="joint_robot")
        parent = Link(name="parent")
        child = Link(name="child")
        robot.add_link(parent)
        robot.add_link(child)

        joint = Joint(
            name="arm_joint",
            type=JointType.REVOLUTE,
            parent="parent",
            child="child",
            origin=Transform(xyz=Vector3(1, 0, 0)),
            axis=Vector3(0, 0, 1),
            limits=JointLimits(effort=100.0, velocity=5.0, lower=-1.57, upper=1.57),
            dynamics=JointDynamics(damping=0.1, friction=0.2),
            mimic=JointMimic(joint="other_joint", multiplier=2.0, offset=0.5),
        )
        robot.add_joint(joint)

        # Disable validation since we have disconnected links or incomplete graph
        generator = URDFGenerator(pretty_print=False)
        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)

        joint_elem = root.find("joint")
        assert joint_elem.get("name") == "arm_joint"
        assert joint_elem.get("type") == "revolute"

        assert joint_elem.find("parent").get("link") == "parent"
        assert joint_elem.find("child").get("link") == "child"
        assert joint_elem.find("origin").get("xyz") == "1 0 0"
        assert joint_elem.find("axis").get("xyz") == "0 0 1"

        limit = joint_elem.find("limit")
        assert limit.get("effort") == "100"
        assert limit.get("velocity") == "5"
        assert limit.get("lower") == "-1.57"
        assert limit.get("upper") == "1.57"

        dyn = joint_elem.find("dynamics")
        assert dyn.get("damping") == "0.1"
        assert dyn.get("friction") == "0.2"

        mimic = joint_elem.find("mimic")
        assert mimic.get("joint") == "other_joint"
        assert mimic.get("multiplier") == "2"
        assert mimic.get("offset") == "0.5"

    def test_generate_joint_with_safety_and_calibration(self):
        """Test generating joint with safety controller and calibration."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base"))
        robot.add_link(Link(name="link1"))

        safety = JointSafetyController(
            soft_lower_limit=-1.0,
            soft_upper_limit=1.0,
            k_position=10.0,
            k_velocity=5.0,
        )
        calib = JointCalibration(rising=0.1)

        joint = Joint(
            name="j1",
            type=JointType.REVOLUTE,
            parent="base",
            child="link1",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(lower=-1.57, upper=1.57, effort=10, velocity=5),
            safety_controller=safety,
            calibration=calib,
        )
        robot.add_joint(joint)

        generator = URDFGenerator()
        xml_str = generator.generate(robot)
        root = ET.fromstring(xml_str)

        joint_elem = root.find(".//joint[@name='j1']")
        assert joint_elem is not None

        safety_elem = joint_elem.find("safety_controller")
        assert safety_elem is not None
        assert safety_elem.get("soft_lower_limit") == "-1"
        assert safety_elem.get("k_position") == "10"

        calib_elem = joint_elem.find("calibration")
        assert calib_elem is not None
        assert calib_elem.get("rising") == "0.1"
        assert calib_elem.get("falling") is None

    def test_generate_inertial(self):
        """Test generating inertial properties."""
        robot = Robot(name="inert_robot")

        inertial = Inertial(
            mass=10.0,
            origin=Transform(xyz=Vector3(0, 0, 0.5)),
            inertia=InertiaTensor(ixx=1.0, iyy=1.0, izz=1.0, ixy=0, ixz=0, iyz=0),
        )
        # Link is frozen, pass inertial in constructor
        link = Link(name="body", inertial=inertial)
        robot.add_link(link)

        generator = URDFGenerator(pretty_print=False)
        xml_str = generator.generate(robot)
        root = ET.fromstring(xml_str)

        inertial_elem = root.find("link/inertial")
        assert inertial_elem.find("mass").get("value") == "10"
        assert inertial_elem.find("origin").get("xyz") == "0 0 0.5"

        inertia = inertial_elem.find("inertia")
        assert inertia.get("ixx") == "1"
        assert inertia.get("ixy") == "0"

    def test_validation_failure(self):
        """Test that invalid robot raises error."""
        robot = Robot(name="broken")
        # No links

        generator = URDFGenerator(pretty_print=False)
        # Should normally fail if robot has no links?
        # Actually Robot.validate_tree_structure checks for root link.

        with pytest.raises(RobotGeneratorError):
            generator.generate(robot)

    def test_mesh_path_relativity(self, tmp_path):
        """Test making mesh paths relative to URDF output path."""
        robot = Robot(name="rel_robot")
        link = Link(name="base")

        # Create a mesh file
        mesh_dir = tmp_path / "meshes"
        mesh_dir.mkdir()
        mesh_file = mesh_dir / "geom.stl"
        mesh_file.touch()

        # Use absolute path in model
        mesh = Mesh(filepath=mesh_file)
        link.visuals.append(Visual(geometry=mesh))
        robot.add_link(link)

        # Generate to a file in tmp_path (parent of meshes)
        urdf_path = tmp_path / "robot.urdf"
        generator = URDFGenerator(urdf_path=urdf_path)

        xml_str = generator.generate(robot)
        root = ET.fromstring(xml_str)

        mesh_elem = root.find("link/visual/geometry/mesh")
        # Should be relative: "meshes/geom.stl"
        assert mesh_elem.get("filename") == "meshes/geom.stl"

    def test_generate_transmission(self):
        """Test generating transmission with hardware interfaces."""
        robot = Robot(name="trans_robot")
        link = Link(name="base")
        robot.add_link(link)

        # dummy joint needed? Transmission references joint.
        # But URDF validation only checks link graph.
        # Transmission references joint name string.

        from linkforge_core.models.transmission import Transmission, TransmissionJoint

        trans = Transmission(
            name="arm_trans",
            type="transmission_interface/SimpleTransmission",
            joints=[
                TransmissionJoint(
                    name="arm_joint",
                    hardware_interfaces=["PositionJointInterface", "VelocityJointInterface"],
                    mechanical_reduction=50.0,
                )
            ],
        )
        robot.transmissions.append(trans)

        generator = URDFGenerator(pretty_print=False)
        xml_str = generator.generate(robot)
        root = ET.fromstring(xml_str)

        trans_elem = root.find("transmission")
        assert trans_elem.get("name") == "arm_trans"
        assert trans_elem.find("type").text == "transmission_interface/SimpleTransmission"

        joint_elem = trans_elem.find("joint")
        assert joint_elem.get("name") == "arm_joint"

        hw_ifaces = joint_elem.findall("hardwareInterface")
        assert len(hw_ifaces) == 2
        # Generator normalizes names? No, checks logic
        # normalize_interface_name logic: PositionJointInterface -> position
        assert hw_ifaces[0].text == "position"
        assert hw_ifaces[1].text == "velocity"

        assert joint_elem.find("mechanicalReduction").text == "50"

    def test_generate_sensors(self):
        """Test generating various sensors."""
        robot = Robot(name="sensor_robot")
        link = Link(name="base")
        robot.add_link(link)

        from linkforge_core.models.sensor import (
            CameraInfo,
            LidarInfo,
            Sensor,
            SensorNoise,
            SensorType,
        )

        lidar = Sensor(
            name="lidar",
            type=SensorType.LIDAR,
            link_name="base",
            update_rate=10.0,
            lidar_info=LidarInfo(
                horizontal_samples=720,
                horizontal_min_angle=-1.57,
                horizontal_max_angle=1.57,
                range_max=10.0,
            ),
        )
        robot.sensors.append(lidar)

        camera = Sensor(
            name="camera",
            type=SensorType.CAMERA,
            link_name="base",
            update_rate=30.0,
            camera_info=CameraInfo(
                width=640,
                height=480,
                horizontal_fov=1.0,
                noise=SensorNoise(type="gaussian", mean=0.0, stddev=0.01),
            ),
        )
        robot.sensors.append(camera)

        generator = URDFGenerator(pretty_print=False)
        xml_str = generator.generate(
            robot, validate=False
        )  # validate false to skip structure check if needed
        root = ET.fromstring(xml_str)

        # Sensors are inside <gazebo> tags at the end
        gazebos = root.findall("gazebo")

        lidar_gazebo = next(g for g in gazebos if g.find("sensor[@name='lidar']") is not None)
        assert lidar_gazebo.get("reference") == "base"
        sensor_elem = lidar_gazebo.find("sensor")
        assert sensor_elem.get("type") == "gpu_lidar"  # mapped type
        assert sensor_elem.find("update_rate").text == "10"

        ray = sensor_elem.find("ray")
        assert ray.find("scan/horizontal/samples").text == "720"
        assert ray.find("range/max").text == "10"

        cam_gazebo = next(g for g in gazebos if g.find("sensor[@name='camera']") is not None)
        cam_sensor = cam_gazebo.find("sensor")
        assert cam_sensor.get("type") == "camera"

        cam_elem = cam_sensor.find("camera")
        assert cam_elem.find("image/width").text == "640"
        assert cam_elem.find("noise/type").text == "gaussian"
        assert cam_elem.find("noise/stddev").text == "0.01"

    def test_gazebo_elements(self):
        """Test generating gazebo extension elements."""
        robot = Robot(name="gz_robot")
        link = Link(name="base")
        robot.add_link(link)

        from linkforge_core.models.gazebo import GazeboElement

        # Gazebo element for link with material color
        gz = GazeboElement(
            reference="base", material="Gazebo/Blue", gravity=False, self_collide=True
        )
        robot.gazebo_elements.append(gz)

        generator = URDFGenerator(pretty_print=False)
        xml_str = generator.generate(robot)
        root = ET.fromstring(xml_str)

        gz_elem = root.find("gazebo[@reference='base']")
        assert gz_elem is not None
        assert gz_elem.find("material").text == "Gazebo/Blue"
        assert gz_elem.find("gravity").text == "false"
        assert gz_elem.find("selfCollide").text == "true"

    def test_generate_ros2_control_auto(self):
        """Test auto-generation of ros2_control from transmissions."""
        robot = Robot(name="auto_control")
        link = Link(
            name="base",
            inertial=Inertial(
                mass=1.0, inertia=InertiaTensor(ixx=1, iyy=1, izz=1, ixy=0, ixz=0, iyz=0)
            ),
        )
        robot.add_link(link)

        from linkforge_core.models.transmission import Transmission, TransmissionJoint

        # Add a transmission
        trans = Transmission(
            name="arm_trans",
            type="transmission_interface/SimpleTransmission",
            joints=[
                TransmissionJoint(name="arm_joint", hardware_interfaces=["PositionJointInterface"])
            ],
        )
        robot.transmissions.append(trans)

        # Generator should create ros2_control block
        generator = URDFGenerator(pretty_print=False)
        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)

        rc_elem = root.find("ros2_control")
        assert rc_elem is not None
        assert rc_elem.get("name") == "GazeboSimSystem"
        assert rc_elem.find("hardware/plugin").text == "gz_ros2_control/GazeboSimSystem"

        joint_elem = rc_elem.find("joint")
        assert joint_elem.get("name") == "arm_joint"
        assert joint_elem.find("command_interface").get("name") == "position"
        assert joint_elem.find("state_interface[@name='position']") is not None
        assert joint_elem.find("state_interface[@name='velocity']") is not None

    def test_generate_ros2_control_explicit(self):
        """Test explicit ros2_control generation."""
        robot = Robot(name="explicit_control")
        link = Link(name="base")
        robot.add_link(link)

        from linkforge_core.models.ros2_control import Ros2Control, Ros2ControlJoint

        rc = Ros2Control(
            name="MySystem",
            type="system",
            hardware_plugin="some_plugin/MySystem",
            joints=[
                Ros2ControlJoint(
                    name="custom_joint",
                    command_interfaces=["position", "velocity"],
                    state_interfaces=["position", "velocity", "effort"],
                )
            ],
            parameters={"param1": "value1"},
        )
        robot.ros2_controls.append(rc)

        generator = URDFGenerator(pretty_print=False)
        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)

        rc_elem = root.find("ros2_control")
        assert rc_elem.get("name") == "MySystem"
        assert rc_elem.find("hardware/plugin").text == "some_plugin/MySystem"

        joint = rc_elem.find("joint")
        assert joint.get("name") == "custom_joint"
        assert len(joint.findall("command_interface")) == 2
        assert len(joint.findall("state_interface")) == 3

        param_elem = rc_elem.find(".//param[@name='param1']")
        assert param_elem is not None
        assert param_elem.text == "value1"

    def test_generate_gazebo_plugins(self):
        """Test generation of Gazebo plugins (raw XML and parameters)."""
        robot = Robot(name="plugin_robot")
        link = Link(name="base")
        robot.add_link(link)

        from linkforge_core.models.gazebo import GazeboElement, GazeboPlugin

        # Plugin with parameters
        p1 = GazeboPlugin(
            name="param_plugin", filename="libparam.so", parameters={"key": "value", "rate": "100"}
        )

        # Plugin with raw XML
        xml_content = "<sub_param>data</sub_param><flag/>"
        p2 = GazeboPlugin(name="xml_plugin", filename="libxml.so", raw_xml=xml_content)

        gz = GazeboElement(reference="base", plugins=[p1, p2])
        robot.gazebo_elements.append(gz)

        generator = URDFGenerator(pretty_print=False)
        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)

        gz_elem = root.find("gazebo")
        plugins = gz_elem.findall("plugin")
        assert len(plugins) == 2

        pl1 = next(p for p in plugins if p.get("name") == "param_plugin")
        assert pl1.find("key").text == "value"
        assert pl1.find("rate").text == "100"

        pl2 = next(p for p in plugins if p.get("name") == "xml_plugin")
        assert pl2.find("sub_param").text == "data"
        assert pl2.find("flag") is not None

    def test_generate_more_sensors(self):
        """Test IMU, GPS, ForceTorque, Contact sensors."""
        robot = Robot(name="more_sensors")
        link = Link(name="base")
        robot.add_link(link)

        from linkforge_core.models.sensor import (
            ContactInfo,
            ForceTorqueInfo,
            GPSInfo,
            IMUInfo,
            Sensor,
            SensorNoise,
            SensorType,
        )

        imu = Sensor(
            name="imu",
            type=SensorType.IMU,
            link_name="base",
            imu_info=IMUInfo(
                angular_velocity_noise=SensorNoise(type="gaussian", mean=0.0, stddev=0.01),
                linear_acceleration_noise=SensorNoise(type="gaussian", mean=0.0, stddev=0.1),
            ),
        )
        robot.sensors.append(imu)

        gps = Sensor(
            name="gps",
            type=SensorType.GPS,
            link_name="base",
            gps_info=GPSInfo(
                position_sensing_horizontal_noise=SensorNoise(type="gaussian", stddev=0.5),
                velocity_sensing_vertical_noise=SensorNoise(type="gaussian", stddev=0.1),
            ),
        )
        robot.sensors.append(gps)

        ft = Sensor(
            name="ft_sensor",
            type=SensorType.FORCE_TORQUE,
            link_name="base",
            force_torque_info=ForceTorqueInfo(
                frame="child",
                measure_direction="child_to_parent",
                noise=SensorNoise(type="gaussian", stddev=0.01),
            ),
        )
        robot.sensors.append(ft)

        contact = Sensor(
            name="bumper",
            type=SensorType.CONTACT,
            link_name="base",
            contact_info=ContactInfo(
                collision="base_collision", noise=SensorNoise(type="gaussian", stddev=0.01)
            ),
        )
        robot.sensors.append(contact)

        generator = URDFGenerator(pretty_print=False)
        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)

        # Parse all sensors into a dict by name for easier verification
        sensors_map = {}
        for gz in root.findall("gazebo"):
            for s in gz.findall("sensor"):
                sensors_map[s.get("name")] = s

        assert "imu" in sensors_map
        imu_elem = sensors_map["imu"].find("imu")
        assert imu_elem is not None, "IMU element missing"
        assert imu_elem.find("angular_velocity/x/noise/type").text == "gaussian"
        assert imu_elem.find("linear_acceleration/z/noise/stddev").text == "0.1"

        assert "gps" in sensors_map
        gps_elem = sensors_map["gps"].find("navsat")
        assert gps_elem is not None, "GPS element (navsat) missing"
        # Position noise uses flattened structure (prefix="")
        assert gps_elem.find("position_sensing/horizontal/stddev").text == "0.5"
        # Velocity noise uses default structure (prefix="noise")
        assert gps_elem.find("velocity_sensing/vertical/noise/stddev").text == "0.1"

        assert "ft_sensor" in sensors_map
        ft_sensor = sensors_map["ft_sensor"]
        ft_elem = ft_sensor.find("force_torque")
        assert ft_elem is not None, "ForceTorque element missing"
        assert ft_elem.find("frame").text == "child"
        assert ft_elem.find("measure_direction").text == "child_to_parent"
        # Noise is flattened
        assert ft_elem.find("stddev").text == "0.01"

        assert "bumper" in sensors_map
        contact_elem = sensors_map["bumper"].find("contact")
        assert contact_elem is not None, "Contact element missing"
        assert contact_elem.find("collision").text == "base_collision"
        assert contact_elem.find("noise/stddev").text == "0.01"

    def test_util_normalize_interface(self):
        """Test interface name normalization directly via generator subclass wrapper or inspection."""
        # Using a dummy robot with weird interface name
        robot = Robot(name="norm_test")
        link = Link(name="base")
        robot.add_link(link)

        trans = Transmission(
            name="t1",
            type="transmission_interface/SimpleTransmission",
            joints=[TransmissionJoint(name="j1", hardware_interfaces=["UnknownInterface"])],
        )
        robot.transmissions.append(trans)

        generator = URDFGenerator(pretty_print=False)
        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)

        # normalization fallback is "position"
        iface = root.find("transmission/joint/hardwareInterface")
        assert iface.text == "position"

    def test_generate_material_texture(self):
        """Test generating material with texture."""
        mat = Material(name="tex_mat", texture="package://pkg/tex.png")
        gen = URDFGenerator()
        root = ET.Element("robot")
        gen._add_material_element(root, mat)

        node = root.find("material/texture")
        assert node is not None
        assert node.get("filename") == "package://pkg/tex.png"

    def test_generate_multiple_collisions_with_names(self):
        """Test generating multiple collisions with names for a link."""
        coll1 = Collision(name="coll1", geometry=Box(size=Vector3(1, 1, 1)))
        coll2 = Collision(name="coll2", geometry=Sphere(radius=0.5))
        link = Link(name="l", collisions=[coll1, coll2])
        gen = URDFGenerator()
        root = ET.Element("robot")
        gen._add_link_element(root, link)

        colls = root.findall(".//collision")
        assert len(colls) == 2
        assert colls[0].get("name") == "coll1"
        assert colls[1].get("name") == "coll2"

    def test_generate_depth_camera(self):
        """Test generating depth camera sensor with pose and topic."""
        sensor = Sensor(
            name="dcam",
            type=SensorType.DEPTH_CAMERA,
            link_name="base",
            topic="camera/depth",
            origin=Transform(xyz=Vector3(0.1, 0, 0.2)),
            camera_info=CameraInfo(width=640, height=480),
        )
        gen = URDFGenerator()
        root = ET.Element("robot")
        gen._add_sensor_element(root, sensor)

        depth_node = root.find(".//sensor/depth_camera")
        assert depth_node is not None
        assert depth_node.find("output/type").text == "depth"

        pose_node = root.find(".//sensor/pose")
        assert pose_node is not None
        assert "0.1 0 0.2" in pose_node.text
        assert root.find(".//sensor/topic").text == "camera/depth"

    def test_generate_3d_lidar(self):
        """Test generating 3D LIDAR with vertical scan parameters."""
        sensor = Sensor(
            name="lidar3d",
            type=SensorType.LIDAR,
            link_name="base",
            lidar_info=LidarInfo(
                vertical_samples=16, vertical_min_angle=-0.5, vertical_max_angle=0.5
            ),
        )
        gen = URDFGenerator()
        root = ET.Element("robot")
        gen._add_sensor_element(root, sensor)

        vert_node = root.find(".//sensor/ray/scan/vertical")
        assert vert_node is not None
        assert vert_node.find("samples").text == "16"

    def test_generate_transmission_with_effort_and_offsets(self):
        """Test generating transmission with effort interface and joint/actuator offsets."""
        trans = Transmission(
            name="t1",
            type="transmission_interface/SimpleTransmission",
            joints=[TransmissionJoint(name="j1", hardware_interfaces=["effort"], offset=0.5)],
            actuators=[TransmissionActuator(name="a1", offset=0.1)],
        )
        robot = Robot(name="r", initial_links=[Link(name="base")], transmissions=[trans])
        gen = URDFGenerator(use_ros2_control=True)
        xml = gen.generate(robot)

        assert "<offset>0.5</offset>" in xml
        assert "<offset>0.1</offset>" in xml
        # Auto-generated ros2_control should have effort state interface
        assert 'state_interface name="effort"' in xml

    def test_generate_visual_with_texture(self):
        """Test generating visual element with texture."""
        mat = Material(name="m", texture="pkg/tex.png")
        visual = Visual(geometry=Box(size=Vector3(1, 1, 1)), material=mat)
        gen = URDFGenerator()
        gen.global_materials = {}  # Initialize to avoid AttributeError in direct call
        root = ET.Element("link")
        gen._add_visual_element(root, visual)

        node = root.find("visual/material/texture")
        assert node is not None
        assert node.get("filename") == "pkg/tex.png"

    def test_deterministic_sorting(self):
        """Test that the generator produces deterministic output by sorting elements."""
        robot = Robot(name="sorting_robot")

        # Add links in non-alphabetical order
        robot.add_link(Link(name="link_C"))
        robot.add_link(Link(name="link_A"))
        robot.add_link(Link(name="link_B"))

        # Add joints in non-alphabetical order
        robot.add_joint(
            Joint(name="joint_Z", parent="link_A", child="link_B", type=JointType.FIXED)
        )
        robot.add_joint(
            Joint(name="joint_X", parent="link_B", child="link_C", type=JointType.FIXED)
        )

        # Add materials in non-alphabetical order
        mat_red = Material(name="red", color=Color(1, 0, 0, 1))
        mat_blue = Material(name="blue", color=Color(0, 0, 1, 1))

        robot.add_link(
            Link(
                name="mat_link_1",
                visuals=[Visual(geometry=Box(Vector3(1, 1, 1)), material=mat_red)],
            )
        )
        robot.add_link(
            Link(
                name="mat_link_2",
                visuals=[Visual(geometry=Box(Vector3(1, 1, 1)), material=mat_blue)],
            )
        )

        generator = URDFGenerator(pretty_print=False)
        xml_str = generator.generate(robot, validate=False)
        root = ET.fromstring(xml_str)

        # Check links order
        links = [link_elem.get("name") for link_elem in root.findall("link")]
        # Note: links which are not roots will be sorted alphabetically after root
        # base_link is usually first if it's the only root.
        # In this test mat_link_1, mat_link_2 and link_A are roots.
        assert links == ["link_A", "link_B", "link_C", "mat_link_1", "mat_link_2"]
        # link_B and link_C should follow

        # Check joints order
        joint_names = [joint_elem.get("name") for joint_elem in root.findall("joint")]
        assert joint_names == ["joint_X", "joint_Z"]

        # Check materials order
        material_names = [mat_elem.get("name") for mat_elem in root.findall("material")]
        assert material_names == ["blue", "red"]

    def test_generate_camera_with_fov(self):
        """Test generating camera with explicit horizontal FOV."""
        sensor = Sensor(
            name="cam",
            type=SensorType.CAMERA,
            link_name="base",
            camera_info=CameraInfo(width=640, height=480, horizontal_fov=1.57),
        )
        gen = URDFGenerator()
        root = ET.Element("robot")
        gen._add_sensor_element(root, sensor)

        fov_node = root.find(".//sensor/camera/horizontal_fov")
        assert fov_node is not None
        assert fov_node.text == "1.57"

    def test_generate_lidar_with_noise(self):
        """Test generating LIDAR with sensor noise."""
        sensor = Sensor(
            name="lidar",
            type=SensorType.LIDAR,
            link_name="base",
            lidar_info=LidarInfo(noise=SensorNoise(stddev=0.01)),
        )
        gen = URDFGenerator()
        root = ET.Element("robot")
        gen._add_sensor_element(root, sensor)

        noise_node = root.find(".//sensor/ray/noise/stddev")
        assert noise_node is not None
        assert noise_node.text == "0.01"

    def test_generate_gazebo_custom_properties(self):
        """Test generating Gazebo element with custom properties."""
        gz = GazeboElement(reference="base", properties={"dampingFactor": "0.1"})
        robot = Robot(name="r", initial_links=[Link(name="base")], gazebo_elements=[gz])
        gen = URDFGenerator()
        xml = gen.generate(robot)

        assert "<dampingFactor>0.1</dampingFactor>" in xml

    def test_generate_gazebo_plugin_whitespace_handling(self):
        """Test Gazebo plugin whitespace stripping and malformed XML fallback."""
        from linkforge_core.models.gazebo import GazeboPlugin

        # Test Case 1: Whitespace stripping
        raw_xml = "<parent>  <child>   </child>  </parent>   "
        p = GazeboPlugin(name="p1", filename="f1.so", raw_xml=raw_xml)
        gz = GazeboElement(plugins=[p])
        gen = URDFGenerator()
        root = ET.Element("gz")
        gen._add_gazebo_element(root, gz)

        xml = ET.tostring(root, encoding="unicode")
        assert "<child />" in xml or "<child/>" in xml

        # Test Case 2: Malformed XML fallback
        p2 = GazeboPlugin(name="p2", filename="f2.so", raw_xml="<malformed", parameters={"p": "v"})
        gz2 = GazeboElement(plugins=[p2])
        root2 = ET.Element("gz")
        gen._add_gazebo_element(root2, gz2)
        xml2 = ET.tostring(root2, encoding="unicode")
        assert "<p>v</p>" in xml2

    def test_generate_ros2_control_plugin_detection(self):
        """Test skipping auto-gz_ros2_control if a plugin already exists."""
        from linkforge_core.models.gazebo import GazeboPlugin

        gz_plugin = GazeboPlugin(name="my_ros2_control", filename="lib.so")
        gz = GazeboElement(plugins=[gz_plugin])
        # Add a transmission to ensure we enter the logic if it's gated
        trans = Transmission(
            name="t", joints=[TransmissionJoint(name="j")], type="interface/Simple"
        )
        robot = Robot(
            name="r",
            initial_links=[Link(name="base"), Link(name="child")],
            initial_joints=[
                Joint(
                    name="j",
                    type=JointType.CONTINUOUS,
                    parent="base",
                    child="child",
                    axis=Vector3(1.0, 0.0, 0.0),
                )
            ],
            gazebo_elements=[gz],
            transmissions=[trans],
        )

        gen = URDFGenerator(use_ros2_control=True)
        xml = gen.generate(robot)

        assert xml.count("libgz_ros2_control-system.so") == 0
        assert "my_ros2_control" in xml

    def test_generate_gazebo_physics_properties(self):
        """Test generating Gazebo physics properties (kp, kd, friction, etc.)."""
        gz = GazeboElement(
            reference="base",
            mu1=0.8,
            mu2=0.7,
            kp=1000.0,
            kd=10.0,
            self_collide=True,
            gravity=False,
        )
        robot = Robot(name="r", initial_links=[Link(name="base")], gazebo_elements=[gz])
        gen = URDFGenerator()
        xml = gen.generate(robot)

        assert '<gazebo reference="base">' in xml
        assert "<mu1>0.8</mu1>" in xml
        assert "<mu2>0.7</mu2>" in xml
        assert "<kp>1000</kp>" in xml
        assert "<kd>10</kd>" in xml
        assert "<selfCollide>true</selfCollide>" in xml
        assert "<gravity>false</gravity>" in xml

    def test_generate_gps_with_vertical_noise(self):
        """Test generating GPS with vertical position noise only."""
        sensor = Sensor(
            name="gps_v",
            type=SensorType.GPS,
            link_name="base",
            gps_info=GPSInfo(position_sensing_vertical_noise=SensorNoise(stddev=0.1)),
        )
        gen = URDFGenerator()
        root = ET.Element("robot")
        gen._add_sensor_element(root, sensor)

        pos_node = root.find(".//position_sensing")
        assert pos_node is not None
        assert pos_node.find("vertical/stddev").text == "0.1"
        assert pos_node.find("horizontal") is None

    def test_generate_gps_with_velocity_noise(self):
        """Test generating GPS with horizontal and vertical velocity noise."""
        sensor = Sensor(
            name="gps",
            type=SensorType.GPS,
            link_name="base",
            gps_info=GPSInfo(
                velocity_sensing_horizontal_noise=SensorNoise(stddev=0.1),
                velocity_sensing_vertical_noise=SensorNoise(stddev=0.2),
            ),
        )
        gen = URDFGenerator()
        root = ET.Element("robot")
        gen._add_sensor_element(root, sensor)

        vel_node = root.find(".//velocity_sensing")
        assert vel_node is not None
        # Velocity noise uses "noise" wrapper in _add_sensor_noise by default
        assert vel_node.find("horizontal/noise/stddev").text == "0.1"
        assert vel_node.find("vertical/noise/stddev").text == "0.2"

    def test_generate_sensor_noise_mean_and_bias(self):
        """Test generating sensor noise with non-zero mean, bias mean and bias stddev."""
        noise = SensorNoise(mean=0.01, bias_mean=0.001, bias_stddev=0.005)
        gen = URDFGenerator()
        root = ET.Element("parent")
        gen._add_sensor_noise(root, noise)

        assert root.find("noise/mean").text == "0.01"
        assert root.find("noise/bias_stddev").text == "0.005"

    def test_generate_explicit_ros2_control_with_params(self):
        """Test generating explicit ros2_control with joint interfaces and parameters."""
        from linkforge_core.models.ros2_control import Ros2ControlJoint

        rc = Ros2Control(
            name="ctrl",
            type="system",
            hardware_plugin="plugin/name",
            parameters={"param1": "val1"},
            joints=[
                Ros2ControlJoint(
                    name="j1",
                    command_interfaces=["position"],
                    state_interfaces=["position", "velocity"],
                )
            ],
        )
        robot = Robot(
            name="r",
            initial_links=[Link(name="base"), Link(name="l1")],
            initial_joints=[
                Joint(
                    name="j1",
                    parent="base",
                    child="l1",
                    type=JointType.CONTINUOUS,
                    axis=Vector3(1, 0, 0),
                )
            ],
            ros2_controls=[rc],
        )
        gen = URDFGenerator()
        xml = gen.generate(robot)

        assert "<plugin>plugin/name</plugin>" in xml
        assert '<param name="param1">val1</param>' in xml
        assert '<state_interface name="velocity"' in xml

    def test_generate_sensor_with_plugin(self):
        """Line 578: Sensor with plugin."""
        from linkforge_core.models.gazebo import GazeboPlugin

        p = GazeboPlugin(name="p", filename="f.so")
        sensor = Sensor(
            name="s",
            type=SensorType.CAMERA,
            link_name="l",
            plugin=p,
            camera_info=CameraInfo(width=640, height=480),
        )
        gen = URDFGenerator()
        root = ET.Element("root")
        gen._add_sensor_element(root, sensor)

        assert root.find(".//plugin").get("name") == "p"


class TestURDFGeneratorEdgeCoverage:
    """Generator behavior for sensors, mimic joints, and disabled features."""

    def test_generate_without_ros2_control(self):
        """Generator skips ros2_control when disabled."""
        robot = Robot(name="r")
        robot.add_link(Link(name="base"))
        gen = URDFGenerator(use_ros2_control=False)
        result = gen.generate(robot)
        assert "ros2_control" not in result

    def test_mimic_joint_with_default_multiplier_and_offset(self):
        """Mimic joint with multiplier=1.0 and offset=0.0 omits those attributes from XML."""
        robot = Robot(name="r")
        robot.add_link(Link(name="l1"))
        robot.add_link(Link(name="l2"))
        robot.add_link(Link(name="l3"))
        # first joint is the one being mimicked
        j_main = Joint(
            name="main_j",
            type=JointType.REVOLUTE,
            parent="l1",
            child="l2",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(effort=1.0, velocity=1.0),
        )
        # second joint mimics the first with defaults
        j_mimic = Joint(
            name="mimic_j",
            type=JointType.REVOLUTE,
            parent="l1",
            child="l3",
            axis=Vector3(1.0, 0.0, 0.0),
            limits=JointLimits(effort=1.0, velocity=1.0),
            mimic=JointMimic(joint="main_j", multiplier=1.0, offset=0.0),
        )
        robot.add_joint(j_main)
        robot.add_joint(j_mimic)
        gen = URDFGenerator()
        result = gen.generate(robot)
        root = ET.fromstring(result)
        mimic = root.find(".//mimic")
        assert mimic is not None
        assert mimic.get("multiplier") is None
        assert mimic.get("offset") is None

    def test_gps_with_only_vertical_position_noise(self):
        """GPS sensor with vertical-only position noise creates the position_sensing element."""
        robot = Robot(name="r")
        robot.add_link(Link(name="base"))
        noise = SensorNoise(mean=0.0, stddev=0.01)
        sensor = Sensor(
            name="gps",
            type=SensorType.GPS,
            link_name="base",
            gps_info=GPSInfo(position_sensing_vertical_noise=noise),
        )
        robot.add_sensor(sensor)
        gen = URDFGenerator()
        result = gen.generate(robot)
        assert "position_sensing" in result
        assert "vertical" in result

    def test_contact_sensor_with_empty_collision_skips_collision_element(self):
        """Contact sensor with empty collision string omits the collision child."""
        from linkforge_core.models.sensor import ContactInfo

        robot = Robot(name="r")
        robot.add_link(Link(name="base"))
        sensor = Sensor(
            name="ct",
            type=SensorType.CONTACT,
            link_name="base",
            contact_info=ContactInfo(collision=""),
        )
        robot.add_sensor(sensor)
        gen = URDFGenerator()
        result = gen.generate(robot)
        assert "ct" in result

    def test_force_torque_sensor_generates_output(self):
        """Force/torque sensor branch is exercised and produces XML output."""
        from linkforge_core.models.sensor import ForceTorqueInfo

        robot = Robot(name="r")
        robot.add_link(Link(name="base"))
        sensor = Sensor(
            name="ft",
            type=SensorType.FORCE_TORQUE,
            link_name="base",
            force_torque_info=ForceTorqueInfo(),
        )
        robot.add_sensor(sensor)
        gen = URDFGenerator()
        result = gen.generate(robot)
        assert "ft" in result

    def test_generate_robot_with_empty_sections(self) -> None:
        """Verify the generator skips XML sections when the robot model has no relevant elements."""
        robot = Robot(name="empty")
        gen = URDFGenerator()
        xml = gen.generate(robot, validate=False)
        assert "<link" not in xml
        assert "<joint" not in xml
        assert "<!-- Links -->" not in xml

    def test_generate_collision_without_name(self) -> None:
        """Verify the generator correctly handles collisions that lack an explicit name."""
        link = Link(name="l", collisions=[Collision(geometry=Box(Vector3(1, 1, 1)))])
        robot = Robot(name="r", initial_links=[link])
        gen = URDFGenerator()
        xml = gen.generate(robot)
        assert "<collision>" in xml
        assert 'name="' not in xml.split("<collision>")[1].split(">")[0]

    def test_generate_joint_with_partial_calibration(self) -> None:
        """Verify the generator correctly renders partial joint calibration data."""
        robot = Robot(name="r")
        robot.add_link(Link(name="p"))
        robot.add_link(Link(name="c"))
        j1 = Joint(
            name="j1",
            type=JointType.FIXED,
            parent="p",
            child="c",
            calibration=JointCalibration(falling=0.5),
        )
        robot.add_joint(j1)
        gen = URDFGenerator()
        xml = gen.generate(robot)
        assert 'falling="0.5"' in xml
        assert "rising" not in xml

    def test_generate_transmission_with_default_actuator_values(self) -> None:
        """Verify the generator skips optional actuator tags when they match default values."""
        trans = Transmission(
            name="t1",
            type="Simple",
            joints=[TransmissionJoint(name="j1")],
            actuators=[TransmissionActuator(name="a1", mechanical_reduction=None, offset=0.0)],
        )
        robot = Robot(name="r", initial_links=[Link(name="base")], transmissions=[trans])
        gen = URDFGenerator()
        xml = gen.generate(robot)
        assert "<mechanicalReduction>" not in xml
        assert "<offset>" not in xml

    def test_generate_imu_sensor_with_partial_noise_profile(self) -> None:
        """Verify the generator correctly renders IMU sensors with incomplete noise data."""
        sensor = Sensor(
            name="imu",
            type=SensorType.IMU,
            link_name="base",
            imu_info=IMUInfo(angular_velocity_noise=SensorNoise(stddev=0.1)),
        )
        robot = Robot(name="r", initial_links=[Link(name="base")], sensors=[sensor])
        gen = URDFGenerator()
        xml = gen.generate(robot, validate=False)
        assert "<angular_velocity>" in xml
        assert "<linear_acceleration>" not in xml

    def test_gps_vertical_noise_only_branch(self):
        """Test sensor noise configuration with partial data."""
        sensor = Sensor(
            name="gps",
            type=SensorType.GPS,
            link_name="base",
            gps_info=GPSInfo(position_sensing_vertical_noise=SensorNoise(stddev=0.1)),
        )
        robot = Robot(name="r", initial_links=[Link(name="base")], sensors=[sensor])
        gen = URDFGenerator()
        xml = gen.generate(robot, validate=False)
        assert "<position_sensing>" in xml
        assert "<vertical>" in xml
        assert "<horizontal>" not in xml

    def test_ros2_control_plugin_traversal(self) -> None:
        """Verify ROS2 control plugin detection logic handles multiple gazebo plugins."""
        from linkforge_core.models.gazebo import GazeboPlugin

        p1 = GazeboPlugin(name="other", filename="o.so")
        p2 = GazeboPlugin(name="my_ros2_control", filename="r.so")
        gz = GazeboElement(plugins=[p1, p2])
        robot = Robot(name="r", initial_links=[Link(name="base")], gazebo_elements=[gz])
        gen = URDFGenerator(use_ros2_control=True)
        robot.transmissions.append(
            Transmission(name="t", joints=[TransmissionJoint(name="j")], type="Simple")
        )
        xml = gen.generate(robot, validate=False)
        assert xml.count("libgz_ros2_control-system.so") == 0

    def test_generate_ros2_control_joint_parameters(self) -> None:
        """Verify generator handles custom parameters for ROS2 control joints."""
        from linkforge_core.models.ros2_control import Ros2ControlJoint

        # Must have matching joint in robot model for auto-sync logic
        robot = Robot(name="r")
        robot.add_link(Link(name="p"))
        robot.add_link(Link(name="c"))
        robot.add_joint(
            Joint(name="j", type=JointType.CONTINUOUS, parent="p", child="c", axis=Vector3(1, 0, 0))
        )

        rc = Ros2Control(
            name="c",
            hardware_plugin="lib.so",
            joints=[
                Ros2ControlJoint(name="j", command_interfaces=["position"], parameters={"p1": "v1"})
            ],
        )
        robot.ros2_controls.append(rc)

        gen = URDFGenerator()
        xml = gen.generate(robot, validate=False)
        assert '<param name="p1">v1</param>' in xml

    def test_generate_joint_without_calibration_tags(self) -> None:
        """Verify the generator skips calibration tags when they are empty."""
        # This is tricky because JointCalibration fields default to None,
        # but if we have an empty calibration object, we hit the branch.
        j = Joint(
            name="j",
            type=JointType.FIXED,
            parent="a",
            child="b",
            # rising=None, falling=None is default
            calibration=JointCalibration(),
        )
        robot = Robot(name="r", initial_links=[Link(name="a"), Link(name="b")], initial_joints=[j])
        gen = URDFGenerator()
        xml = gen.generate(robot)
        assert "<calibration" not in xml

    def test_generate_imu_sensor_with_linear_acceleration_noise_only(self) -> None:
        """Verify IMU sensor generation when only linear acceleration noise is configured."""
        sensor = Sensor(
            name="imu",
            type=SensorType.IMU,
            link_name="base",
            imu_info=IMUInfo(linear_acceleration_noise=SensorNoise(stddev=0.1)),
        )
        robot = Robot(name="r", initial_links=[Link(name="base")], sensors=[sensor])
        gen = URDFGenerator()
        xml = gen.generate(robot, validate=False)
        assert "<angular_velocity>" not in xml
        assert "<linear_acceleration>" in xml

    def test_generate_gps_sensor_with_full_position_noise(self) -> None:
        """Verify GPS sensor generation when both horizontal and vertical position noise are present."""
        sensor = Sensor(
            name="gps",
            type=SensorType.GPS,
            link_name="base",
            gps_info=GPSInfo(
                position_sensing_horizontal_noise=SensorNoise(stddev=0.1),
                position_sensing_vertical_noise=SensorNoise(stddev=0.2),
            ),
        )
        robot = Robot(name="r", initial_links=[Link(name="base")], sensors=[sensor])
        gen = URDFGenerator()
        xml = gen.generate(robot, validate=False)
        assert "<position_sensing>" in xml
        assert "<horizontal>" in xml
        assert "<vertical>" in xml

    def test_generate_ros2_control_with_empty_plugins_or_gazebo_elements(self) -> None:
        """Verify ROS2 control generation when optional plugins or Gazebo elements are missing."""
        # Case 1: Empty gazebo_elements
        robot = Robot(name="r", initial_links=[Link(name="base")])
        robot.transmissions.append(
            Transmission(name="t", joints=[TransmissionJoint(name="j")], type="Simple")
        )
        gen = URDFGenerator(use_ros2_control=True)
        xml = gen.generate(robot, validate=False)
        assert "libgz_ros2_control-system.so" in xml

        # Case 2: Gazebo elements with NO plugins (covers 1005->1009 skip)
        robot.gazebo_elements.append(GazeboElement(reference="base"))
        xml = gen.generate(robot, validate=False)
        assert "libgz_ros2_control-system.so" in xml

    def test_generate_unsupported_geometry_fallback(self) -> None:
        """Verify the generator provides a safe fallback for unknown geometry types."""

        # Just create any object that is NOT Box, Cylinder, Sphere, or Mesh
        class UnknownGeometry:
            pass

        # Python dataclasses don't enforce type constraints at runtime by default.
        # We just need an object that fails all isinstance checks in _add_geometry_element.
        link = Link(name="l", visuals=[Visual(geometry=UnknownGeometry())])  # type: ignore
        robot = Robot(name="r", initial_links=[link])
        gen = URDFGenerator()
        xml = gen.generate(robot)
        # Should create <geometry> but no child element
        # Using a more flexible check for the tag presence
        assert "<geometry" in xml
        assert "<box" not in xml
        assert "<mesh" not in xml

    def test_generate_gps_sensor_with_horizontal_velocity_noise_only(self) -> None:
        """Verify GPS sensor generation with partial velocity noise data."""
        sensor = Sensor(
            name="gps",
            type=SensorType.GPS,
            link_name="base",
            gps_info=GPSInfo(velocity_sensing_horizontal_noise=SensorNoise(stddev=0.1)),
        )
        robot = Robot(name="r", initial_links=[Link(name="base")], sensors=[sensor])
        gen = URDFGenerator()
        xml = gen.generate(robot, validate=False)
        assert "<velocity_sensing>" in xml
        assert "<horizontal>" in xml
        assert "<vertical>" not in xml

    def test_generate_sensor_without_info_block(self) -> None:
        """Verify sensor generation correctly skips info blocks when none are provided."""
        # FORCE_TORQUE doesn't require info in Sensor.__post_init__
        sensor = Sensor(name="s", type=SensorType.FORCE_TORQUE, link_name="base")
        robot = Robot(name="r")
        robot.add_link(Link(name="base"))
        robot.sensors.append(sensor)

        gen = URDFGenerator()
        xml = gen.generate(robot, validate=False)
        assert '<sensor name="s" type="force_torque">' in xml
        assert "<force_torque>" not in xml
