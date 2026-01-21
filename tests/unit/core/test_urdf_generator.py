"""Tests for URDF generator."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import pytest

from linkforge.core import URDFGenerator
from linkforge.core.models import (
    Box,
    CameraInfo,
    Collision,
    Color,
    Cylinder,
    Inertial,
    InertiaTensor,
    Joint,
    JointLimits,
    JointType,
    Link,
    Material,
    Robot,
    Sensor,
    SensorType,
    Sphere,
    Transform,
    Vector3,
    Visual,
)


class TestURDFGenerator:
    """Tests for URDF generator."""

    def test_simple_robot(self):
        """Test generating URDF for a simple robot."""
        robot = Robot(name="simple_robot")

        # Create a simple link
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        visual = Visual(geometry=box)
        link = Link(name="base_link", visuals=[visual])
        robot.add_link(link)

        # Generate URDF
        generator = URDFGenerator()
        urdf = generator.generate(robot)

        # Parse and validate
        root = ET.fromstring(urdf)
        assert root.tag == "robot"
        assert root.get("name") == "simple_robot"

        # Check link exists
        links = root.findall("link")
        assert len(links) == 1
        assert links[0].get("name") == "base_link"

    def test_robot_with_joints(self, simple_robot: Robot):
        """Test generating URDF for robot with joints."""
        generator = URDFGenerator()
        urdf = generator.generate(simple_robot)

        root = ET.fromstring(urdf)

        # Check joints
        joints = root.findall("joint")
        assert len(joints) == 1
        assert joints[0].get("name") == "joint1"
        assert joints[0].get("type") == "revolute"

        # Check parent/child
        parent = joints[0].find("parent")
        child = joints[0].find("child")
        assert parent.get("link") == "link1"
        assert child.get("link") == "link2"

    def test_link_with_visual(self):
        """Test generating URDF for link with visual."""
        robot = Robot(name="test_robot")

        box = Box(size=Vector3(1.0, 2.0, 3.0))
        material = Material(name="red", color=Color(1.0, 0.0, 0.0, 1.0))
        visual = Visual(geometry=box, material=material)
        link = Link(name="link1", visuals=[visual])

        robot.add_link(link)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        # Check visual element
        visual_elem = root.find(".//link[@name='link1']/visual")
        assert visual_elem is not None

        # Check geometry
        geometry = visual_elem.find("geometry/box")
        assert geometry is not None
        assert geometry.get("size") == "1 2 3"

        # Check material
        material_elem = visual_elem.find("material")
        assert material_elem is not None

    def test_link_with_collision(self):
        """Test generating URDF for link with collision."""
        robot = Robot(name="test_robot")

        cylinder = Cylinder(radius=0.5, length=1.0)
        collision = Collision(geometry=cylinder)
        link = Link(name="link1", collisions=[collision])

        robot.add_link(link)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        # Check collision element
        collision_elem = root.find(".//link[@name='link1']/collision")
        assert collision_elem is not None

        # Check geometry
        geometry = collision_elem.find("geometry/cylinder")
        assert geometry is not None
        assert geometry.get("radius") == "0.5"
        assert geometry.get("length") == "1"

    def test_link_with_inertial(self):
        """Test generating URDF for link with inertial properties."""
        robot = Robot(name="test_robot")

        inertia = InertiaTensor(ixx=1.0, ixy=0.0, ixz=0.0, iyy=1.0, iyz=0.0, izz=1.0)
        inertial = Inertial(mass=5.0, inertia=inertia)
        link = Link(name="link1", inertial=inertial)

        robot.add_link(link)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        # Check inertial element
        inertial_elem = root.find(".//link[@name='link1']/inertial")
        assert inertial_elem is not None

        # Check mass
        mass_elem = inertial_elem.find("mass")
        assert mass_elem is not None
        assert mass_elem.get("value") == "5"

        # Check inertia
        inertia_elem = inertial_elem.find("inertia")
        assert inertia_elem is not None
        assert inertia_elem.get("ixx") == "1"
        assert inertia_elem.get("iyy") == "1"
        assert inertia_elem.get("izz") == "1"

    def test_joint_types(self):
        """Test generating URDF for different joint types."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        # Test revolute joint
        joint = Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="link1",
            child="link2",
            limits=JointLimits(lower=-math.pi, upper=math.pi, effort=10.0, velocity=1.0),
        )
        robot.add_joint(joint)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        joint_elem = root.find(".//joint[@name='joint1']")
        assert joint_elem.get("type") == "revolute"

        # Check limits
        limits = joint_elem.find("limit")
        assert limits is not None
        assert float(limits.get("lower")) == pytest.approx(-math.pi)
        assert float(limits.get("upper")) == pytest.approx(math.pi)

        # Check axis
        axis = joint_elem.find("axis")
        assert axis is not None

    def test_joint_with_origin(self):
        """Test generating URDF for joint with origin transform."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        origin = Transform(xyz=Vector3(1.0, 0.0, 0.5), rpy=Vector3(0.0, 0.0, 1.57))
        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
            origin=origin,
        )
        robot.add_joint(joint)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        # Check origin
        origin_elem = root.find(".//joint[@name='joint1']/origin")
        assert origin_elem is not None
        assert origin_elem.get("xyz") == "1 0 0.5"
        assert "1.57" in origin_elem.get("rpy")

    def test_invalid_robot_raises_error(self):
        """Test that invalid robot raises error."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        # Add joint with missing parent (bypass validation)
        robot.joints.append(
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="nonexistent",
                child="link2",
            )
        )

        generator = URDFGenerator()
        with pytest.raises(ValueError, match="validation failed"):
            generator.generate(robot)

    def test_geometry_types(self):
        """Test all geometry types are correctly generated."""
        robot = Robot(name="test_robot")

        # Box
        robot.add_link(Link(name="box_link", visuals=[Visual(geometry=Box(Vector3(1, 2, 3)))]))

        # Cylinder
        robot.add_link(Link(name="cyl_link", visuals=[Visual(geometry=Cylinder(0.5, 1.0))]))

        # Sphere
        robot.add_link(Link(name="sphere_link", visuals=[Visual(geometry=Sphere(0.3))]))

        # Connect links to form valid tree structure
        robot.add_joint(
            Joint(name="joint1", type=JointType.FIXED, parent="box_link", child="cyl_link")
        )
        robot.add_joint(
            Joint(name="joint2", type=JointType.FIXED, parent="cyl_link", child="sphere_link")
        )

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        # Check all geometry types exist
        assert root.find(".//link[@name='box_link']/visual/geometry/box") is not None
        assert root.find(".//link[@name='cyl_link']/visual/geometry/cylinder") is not None
        assert root.find(".//link[@name='sphere_link']/visual/geometry/sphere") is not None

    def test_sensor_export_camera(self):
        """Test that camera sensors are exported to URDF."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        # Add camera sensor
        camera_info = CameraInfo(
            horizontal_fov=1.57,
            width=640,
            height=480,
            format="R8G8B8",
        )
        sensor = Sensor(
            name="front_camera",
            type=SensorType.CAMERA,
            link_name="base_link",
            update_rate=30.0,
            camera_info=camera_info,
        )
        robot.add_sensor(sensor)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        # Check sensor is in gazebo element
        gazebo_elem = root.find(".//gazebo[@reference='base_link']")
        assert gazebo_elem is not None, "Sensor should be in <gazebo reference='base_link'>"

        # Check sensor element
        sensor_elem = gazebo_elem.find("sensor[@name='front_camera']")
        assert sensor_elem is not None, "Sensor element should exist"
        assert sensor_elem.get("type") == "camera"

        # Check update rate
        update_rate_elem = sensor_elem.find("update_rate")
        assert update_rate_elem is not None
        assert float(update_rate_elem.text) == 30.0

        # Check camera info
        camera_elem = sensor_elem.find("camera")
        assert camera_elem is not None

        hfov_elem = camera_elem.find("horizontal_fov")
        assert hfov_elem is not None
        assert float(hfov_elem.text) == pytest.approx(1.57)

        image_elem = camera_elem.find("image")
        assert image_elem is not None
        assert image_elem.find("width").text == "640"
        assert image_elem.find("height").text == "480"
        assert image_elem.find("format").text == "R8G8B8"

    def test_transmission_export_simple(self):
        """Test that simple transmissions are exported to URDF."""
        from linkforge.core.models import Transmission

        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                limits=JointLimits(lower=-math.pi, upper=math.pi),
            )
        )

        # Add simple transmission
        trans = Transmission.create_simple(name="trans1", joint_name="joint1")
        robot.add_transmission(trans)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        # Check transmission element
        trans_elem = root.find(".//transmission[@name='trans1']")
        assert trans_elem is not None

        # Type is a child element, not attribute
        type_elem = trans_elem.find("type")
        assert type_elem is not None
        assert "SimpleTransmission" in type_elem.text

    def test_gazebo_element_robot_level(self):
        """Test that robot-level Gazebo elements are exported."""
        from linkforge.core.models import GazeboElement

        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        # Add robot-level Gazebo element
        gazebo_elem = GazeboElement(reference=None, static=True, gravity=False)
        robot.add_gazebo_element(gazebo_elem)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        # Check robot-level gazebo element
        gazebo = root.find("./gazebo")
        assert gazebo is not None
        assert gazebo.get("reference") is None  # Robot-level has no reference

        static_elem = gazebo.find("static")
        assert static_elem is not None
        assert static_elem.text == "true"

    def test_gazebo_element_link_level(self):
        """Test that link-level Gazebo elements are exported."""
        from linkforge.core.models import GazeboElement

        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        # Add link-level Gazebo element
        gazebo_elem = GazeboElement(reference="base_link", material="Gazebo/Red", mu1=0.8, mu2=0.5)
        robot.add_gazebo_element(gazebo_elem)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        # Check link-level gazebo element
        gazebo = root.find(".//gazebo[@reference='base_link']")
        assert gazebo is not None

        material_elem = gazebo.find("material")
        assert material_elem is not None
        assert material_elem.text == "Gazebo/Red"

    def test_gazebo_plugin(self):
        """Test that Gazebo plugins are exported."""
        from linkforge.core.models import GazeboElement, GazeboPlugin

        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        # Add plugin
        plugin = GazeboPlugin(
            name="test_plugin",
            filename="libtest.so",
            parameters={"param1": "value1"},
        )

        gazebo_elem = GazeboElement(reference=None, plugins=[plugin])
        robot.add_gazebo_element(gazebo_elem)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        # Check plugin
        plugin_elem = root.find(".//gazebo/plugin[@name='test_plugin']")
        assert plugin_elem is not None
        assert plugin_elem.get("filename") == "libtest.so"

        param_elem = plugin_elem.find("param1")
        assert param_elem is not None
        assert param_elem.text == "value1"

    def test_mesh_geometry(self):
        """Test mesh geometry export."""
        from pathlib import Path

        from linkforge.core.models import Mesh

        robot = Robot(name="test_robot")
        # Use non-default scale to test scale export
        mesh = Mesh(filepath=Path("meshes/model.stl"), scale=Vector3(2.0, 2.0, 2.0))
        visual = Visual(geometry=mesh)
        link = Link(name="link1", visuals=[visual])
        robot.add_link(link)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        mesh_elem = root.find(".//visual/geometry/mesh")
        assert mesh_elem is not None
        assert "meshes/model.stl" in mesh_elem.get("filename")
        assert mesh_elem.get("scale") == "2 2 2"

    def test_joint_dynamics(self):
        """Test joint dynamics export."""
        from linkforge.core.models import JointDynamics

        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        dynamics = JointDynamics(damping=0.5, friction=0.3)
        joint = Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="link1",
            child="link2",
            limits=JointLimits(lower=-math.pi, upper=math.pi),
            dynamics=dynamics,
        )
        robot.add_joint(joint)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        dynamics_elem = root.find(".//joint[@name='joint1']/dynamics")
        assert dynamics_elem is not None
        assert dynamics_elem.get("damping") == "0.5"
        assert dynamics_elem.get("friction") == "0.3"

    def test_joint_mimic(self):
        """Test joint mimic export."""
        from linkforge.core.models import JointMimic

        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_link(Link(name="link3"))

        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                limits=JointLimits(lower=-math.pi, upper=math.pi),
            )
        )

        mimic = JointMimic(joint="joint1", multiplier=2.0, offset=0.1)
        robot.add_joint(
            Joint(
                name="joint2",
                type=JointType.REVOLUTE,
                parent="link2",
                child="link3",
                limits=JointLimits(lower=-math.pi, upper=math.pi),
                mimic=mimic,
            )
        )

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        mimic_elem = root.find(".//joint[@name='joint2']/mimic")
        assert mimic_elem is not None
        assert mimic_elem.get("joint") == "joint1"
        assert mimic_elem.get("multiplier") == "2"
        assert mimic_elem.get("offset") == "0.1"

    def test_lidar_sensor_export(self):
        """Test LIDAR sensor export."""
        from linkforge.core.models import LidarInfo

        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        lidar_info = LidarInfo(
            horizontal_samples=1024,
            horizontal_resolution=1.0,
            horizontal_min_angle=-math.pi,
            horizontal_max_angle=math.pi,
            vertical_samples=16,
            vertical_resolution=1.0,
            vertical_min_angle=-0.26,
            vertical_max_angle=0.26,
            range_min=0.1,
            range_max=30.0,
            range_resolution=0.01,
        )
        sensor = Sensor(
            name="lidar",
            type=SensorType.LIDAR,
            link_name="base_link",
            update_rate=10.0,
            lidar_info=lidar_info,
        )
        robot.add_sensor(sensor)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        sensor_elem = root.find(".//gazebo[@reference='base_link']/sensor[@name='lidar']")
        assert sensor_elem is not None
        assert sensor_elem.get("type") == "gpu_lidar"

    def test_imu_sensor_export(self):
        """Test IMU sensor export."""
        from linkforge.core.models import IMUInfo

        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        imu_info = IMUInfo()
        sensor = Sensor(
            name="imu",
            type=SensorType.IMU,
            link_name="base_link",
            update_rate=100.0,
            imu_info=imu_info,
        )
        robot.add_sensor(sensor)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        sensor_elem = root.find(".//gazebo[@reference='base_link']/sensor[@name='imu']")
        assert sensor_elem is not None
        assert sensor_elem.get("type") == "imu"

    def test_gps_sensor_export(self):
        """Test GPS sensor export."""
        from linkforge.core.models import GPSInfo

        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        gps_info = GPSInfo()
        sensor = Sensor(
            name="gps",
            type=SensorType.GPS,
            link_name="base_link",
            update_rate=1.0,
            gps_info=gps_info,
        )
        robot.add_sensor(sensor)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        sensor_elem = root.find(".//gazebo[@reference='base_link']/sensor[@name='gps']")
        assert sensor_elem is not None
        assert sensor_elem.get("type") == "navsat"

    def test_pretty_print_enabled(self):
        """Test pretty print formatting."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        generator = URDFGenerator(pretty_print=True)
        urdf = generator.generate(robot)

        # Pretty printed XML should have newlines and indentation
        assert "\n" in urdf
        assert "  " in urdf  # Should have indentation

    def test_pretty_print_disabled(self):
        """Test compact formatting."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        generator = URDFGenerator(pretty_print=False)
        urdf = generator.generate(robot)

        # Compact XML should be on fewer lines
        lines = urdf.split("\n")
        assert len(lines) < 10  # Should be very compact

    def test_format_float_trailing_zeros(self):
        """Test format_float removes trailing zeros."""
        from linkforge.core import format_float

        assert format_float(1.0) == "1"
        assert format_float(1.5) == "1.5"
        assert format_float(1.50000) == "1.5"
        assert format_float(0.0) == "0"
        assert format_float(-0.0) == "0"  # Special case

    def test_format_vector(self):
        """Test format_vector."""
        from linkforge.core import format_vector

        assert format_vector(1.0, 2.0, 3.0) == "1 2 3"
        assert format_vector(1.5, 2.5, 3.5) == "1.5 2.5 3.5"
        assert format_vector(0.0, 0.0, 0.0) == "0 0 0"

    def test_write_to_file(self, tmp_path):
        """Test writing URDF to file."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        output_file = tmp_path / "robot.urdf"

        generator = URDFGenerator()
        generator.write(robot, output_file)

        # Check file exists and contains valid XML
        assert output_file.exists()
        content = output_file.read_text()
        root = ET.fromstring(content)
        assert root.tag == "robot"
        assert root.get("name") == "test_robot"

    def test_visual_with_name(self):
        """Test visual element with name attribute."""
        robot = Robot(name="test_robot")
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        visual = Visual(geometry=box, name="my_visual")
        link = Link(name="link1", visuals=[visual])
        robot.add_link(link)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        visual_elem = root.find(".//visual[@name='my_visual']")
        assert visual_elem is not None

    def test_collision_with_name(self):
        """Test collision element with name attribute."""
        robot = Robot(name="test_robot")
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        collision = Collision(geometry=box, name="my_collision")
        link = Link(name="link1", collisions=[collision])
        robot.add_link(link)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        collision_elem = root.find(".//collision[@name='my_collision']")
        assert collision_elem is not None

    def test_material_with_texture(self):
        """Test material with texture instead of color."""
        robot = Robot(name="test_robot")
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        material = Material(name="textured", texture="textures/metal.png")
        visual = Visual(geometry=box, material=material)
        link = Link(name="link1", visuals=[visual])
        robot.add_link(link)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        # Check material definition
        mat_elem = root.find(".//material[@name='textured']")
        assert mat_elem is not None

        texture_elem = mat_elem.find("texture")
        assert texture_elem is not None
        assert texture_elem.get("filename") == "textures/metal.png"
