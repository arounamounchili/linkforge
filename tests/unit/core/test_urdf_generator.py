"""Unit tests for URDF generator."""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from linkforge_core.base import RobotGeneratorError
from linkforge_core.generators.urdf_generator import URDFGenerator
from linkforge_core.models import (
    Box,
    Color,
    Cylinder,
    Inertial,
    InertiaTensor,
    Joint,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointType,
    Link,
    Material,
    Mesh,
    Robot,
    Sphere,
    Transform,
    Vector3,
    Visual,
)


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
        # Generator normalizes names? No, checks logic (lines 471)
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

        assert rc_elem.find("param1").text == "value1"

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

        from linkforge_core.models.transmission import Transmission, TransmissionJoint

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
