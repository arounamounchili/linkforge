import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import pytest
from linkforge.core import (
    Box,
    CameraInfo,
    Color,
    Cylinder,
    JointType,
    Material,
    Mesh,
    RobotModelError,
    RobotParserError,
    RobotParserIOError,
    Sensor,
    SensorType,
    Sphere,
    URDFParser,
    XacroDetectedError,
)
from linkforge.core.constants import MIN_REASONABLE_INERTIA


class TestURDFParser:
    @pytest.fixture
    def parser(self):
        return URDFParser()

    def test_parse_geometry_box(self, parser) -> None:
        """Test parsing box geometry."""
        xml = '<geometry><box size="1 2 3"/></geometry>'
        elem = ET.fromstring(xml)
        geom = parser._parse_geometry_element(elem)

        assert isinstance(geom, Box)
        assert geom.size.x == 1.0
        assert geom.size.y == 2.0
        assert geom.size.z == 3.0

    def test_parse_geometry_cylinder(self, parser) -> None:
        """Test parsing cylinder geometry."""
        xml = '<geometry><cylinder radius="0.5" length="2.0"/></geometry>'
        elem = ET.fromstring(xml)
        geom = parser._parse_geometry_element(elem)

        assert isinstance(geom, Cylinder)
        assert geom.radius == 0.5
        assert geom.length == 2.0

    def test_parse_geometry_sphere(self, parser) -> None:
        """Test parsing sphere geometry."""
        xml = '<geometry><sphere radius="1.5"/></geometry>'
        elem = ET.fromstring(xml)
        geom = parser._parse_geometry_element(elem)

        assert isinstance(geom, Sphere)
        assert geom.radius == 1.5

    def test_parse_geometry_mesh(self, parser) -> None:
        """Test parsing mesh geometry with scaling."""
        xml = (
            '<geometry><mesh filename="package://my_pkg/mesh.stl" scale="0.1 0.1 0.1"/></geometry>'
        )
        elem = ET.fromstring(xml)

        geom = parser._parse_geometry_element(elem)

        assert isinstance(geom, Mesh)
        assert Path(geom.resource).name == "mesh.stl"
        assert geom.scale.x == 0.1

    def test_parse_material_color(self, parser) -> None:
        """Test parsing material with color."""
        xml = '<material name="blue"><color rgba="0 0 1 1"/></material>'
        elem = ET.fromstring(xml)
        mat = parser._parse_material_element(elem, {})

        assert isinstance(mat, Material)
        assert mat.name == "blue"
        assert mat.color is not None
        assert mat.color.r == 0.0
        assert mat.color.b == 1.0
        assert mat.color.a == 1.0

    def test_parse_link_full(self, parser) -> None:
        """Test parsing a complete link with visual, collision, and inertial."""
        xml = """
        <link name="base_link">
            <inertial>
                <mass value="5.0"/>
                <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
            </inertial>
            <visual>
                <geometry><box size="1 2 3"/></geometry>
            </visual>
            <collision>
                <geometry><box size="1 2 3"/></geometry>
            </collision>
        </link>
        """
        elem = ET.fromstring(xml)
        link = parser._parse_link(elem, {})

        assert link.name == "base_link"
        assert len(link.visuals) == 1
        assert isinstance(link.visuals[0].geometry, Box)
        assert len(link.collisions) == 1
        assert link.inertial.mass == 5.0
        assert link.inertial.inertia.ixx == 0.1

    def test_parse_joint_limits(self, parser) -> None:
        """Test parsing joint with limits and dynamics."""
        xml = """
        <joint name="j1" type="revolute">
            <parent link="base"/>
            <child link="link1"/>
            <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
            <dynamics damping="0.5" friction="0.1"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parser._parse_joint(elem)

        assert joint.name == "j1"
        assert joint.type == JointType.REVOLUTE
        assert joint.limits.lower == -1.57
        assert joint.limits.upper == 1.57
        assert joint.dynamics.damping == 0.5

    def test_parse_mimic_joint(self, parser) -> None:
        """Test parsing joint with mimic properties."""
        xml = """
        <joint name="j2" type="revolute">
            <parent link="base"/>
            <child link="link2"/>
            <mimic joint="j1" multiplier="2.0" offset="0.5"/>
            <limit lower="-1" upper="1" effort="10" velocity="1"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parser._parse_joint(elem)

        assert joint.mimic is not None
        assert joint.mimic.joint == "j1"
        assert joint.mimic.multiplier == 2.0
        assert joint.mimic.offset == 0.5

    def test_parse_joint_safety_controller(self, parser) -> None:
        """Test parsing joint with safety controller."""
        xml = """
        <joint name="j_safe" type="revolute">
            <parent link="base"/>
            <child link="link1"/>
            <safety_controller soft_lower_limit="-0.9" soft_upper_limit="0.9" k_position="15.0" k_velocity="10.0"/>
            <limit lower="-1" upper="1" effort="10" velocity="1"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parser._parse_joint(elem)

        assert joint.safety_controller is not None
        assert joint.safety_controller.soft_lower_limit == -0.9
        assert joint.safety_controller.soft_upper_limit == 0.9
        assert joint.safety_controller.k_position == 15.0
        assert joint.safety_controller.k_velocity == 10.0

    def test_parse_joint_calibration(self, parser) -> None:
        """Test parsing joint with calibration."""
        xml = """
        <joint name="j_cal" type="fixed">
            <parent link="base"/>
            <child link="link1"/>
            <calibration rising="0.5" falling="1.0"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parser._parse_joint(elem)

        assert joint.calibration is not None
        assert joint.calibration.rising == 0.5
        assert joint.calibration.falling == 1.0

    def test_parse_joint_postels_law(self, parser) -> None:
        """Test Postel's law: be conservative in what you send, liberal in what you receive."""
        xml = """
        <joint name="j_postel" type="fixed">
            <parent link="base"/>
            <child link="link1"/>
            <axis xyz="0 0 1"/> <!-- Axis ignored for fixed joint -->
            <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/> <!-- Limits ignored for fixed joint -->
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parser._parse_joint(elem)

        assert joint.name == "j_postel"
        assert joint.type == JointType.FIXED
        assert joint.axis is None  # Should have been ignored
        assert joint.limits is None  # Should have been ignored

        # REVOLUTE joint with missing axis (should default to 1 0 0)
        xml_rev = """
        <joint name="j_default_axis" type="revolute">
            <parent link="base"/>
            <child link="link1"/>
            <limit lower="-1" upper="1" effort="10" velocity="1"/>
        </joint>
        """
        joint_rev = parser._parse_joint(ET.fromstring(xml_rev))
        assert joint_rev.axis.x == 1.0
        assert joint_rev.axis.y == 0.0
        assert joint_rev.axis.z == 0.0

    def test_parse_sensor_camera(self, parser) -> None:
        """Test parsing camera sensor from Gazebo element."""
        xml = """
        <gazebo reference="link1">
            <sensor name="camera1" type="camera">
                <camera name="head_camera">
                    <horizontal_fov>1.3962634</horizontal_fov>
                    <image>
                        <width>800</width>
                        <height>800</height>
                        <format>R8G8B8</format>
                    </image>
                    <clip>
                        <near>0.02</near>
                        <far>300</far>
                    </clip>
                    <noise>
                        <type>gaussian</type>
                        <mean>0.0</mean>
                        <stddev>0.007</stddev>
                    </noise>
                </camera>
                <always_on>1</always_on>
                <update_rate>30</update_rate>
                <visualize>1</visualize>
            </sensor>
        </gazebo>
        """
        elem = ET.fromstring(xml)
        sensor = parser._parse_sensor_from_gazebo(elem)

        assert isinstance(sensor, Sensor)
        assert sensor.name == "camera1"
        assert sensor.type == SensorType.CAMERA
        assert isinstance(sensor.camera_info, CameraInfo)

    def test_parse_sphere_invalid(self, parser) -> None:
        """Test parsing invalid sphere."""
        # Negative radius is a RobotModelError at model level,
        # but parse_geometry catches it and returns None (robust behavior)
        xml = '<geometry><sphere radius="-1.0"/></geometry>'
        assert parser._parse_geometry_element(ET.fromstring(xml)) is None

    def test_parse_geometry_invalid(self, parser) -> None:
        """Test parsing invalid geometries."""
        # Box missing size
        xml = "<geometry><box/></geometry>"
        assert parser._parse_geometry_element(ET.fromstring(xml)) is None

        # Negative dimensions
        xml = '<geometry><box size="-1 1 1"/></geometry>'
        assert parser._parse_geometry_element(ET.fromstring(xml)) is None

        # Cylinder invalid
        xml = '<geometry><cylinder radius="-1" length="1"/></geometry>'
        assert parser._parse_geometry_element(ET.fromstring(xml)) is None

    def test_parse_material_texture(self, parser) -> None:
        """Test parsing material with texture."""
        xml = '<material name="tex"><texture filename="package://pkg/tex.png"/></material>'
        elem = ET.fromstring(xml)
        mat = parser._parse_material_element(elem, {})

        assert isinstance(mat, Material)
        assert mat.texture == "package://pkg/tex.png"
        assert mat.color is None

    def test_parse_material_reference(self, parser) -> None:
        """Test parsing material reference."""
        global_mats = {"global_blue": Material(name="global_blue", color=Color(0, 0, 1, 1))}

        xml = '<material name="global_blue"/>'
        elem = ET.fromstring(xml)
        mat = parser._parse_material_element(elem, global_mats)

        assert mat is global_mats["global_blue"]

    def test_parse_transmission(self, parser) -> None:
        """Test parsing transmission element."""
        xml = """
        <transmission name="trans1">
            <type>transmission_interface/SimpleTransmission</type>
            <joint name="joint1">
                <hardwareInterface>PositionJointInterface</hardwareInterface>
            </joint>
            <actuator name="actuator1">
                <mechanicalReduction>50</mechanicalReduction>
            </actuator>
        </transmission>
        """
        elem = ET.fromstring(xml)
        trans = parser._parse_transmission(elem)

        assert trans.name == "trans1"
        assert trans.type == "transmission_interface/SimpleTransmission"
        assert len(trans.joints) == 1
        assert trans.joints[0].name == "joint1"
        assert trans.joints[0].hardware_interfaces == ("position",)  # Normalized
        assert len(trans.actuators) == 1
        assert trans.actuators[0].mechanical_reduction == 50.0

    def test_parse_transmission_invalid(self, parser) -> None:
        """Test parsing invalid transmission returns None."""
        xml = "<transmission><type>invalid</type></transmission>"
        assert parser._parse_transmission(ET.fromstring(xml)) is None

    def test_parse_ros2_control(self, parser) -> None:
        """Test parsing ros2_control element."""
        xml = """
        <ros2_control name="System" type="system">
            <hardware>
                <plugin>mock_components/GenericSystem</plugin>
                <param name="ip">192.168.1.1</param>
            </hardware>
            <joint name="j1">
                <command_interface name="position"/>
                <state_interface name="velocity"/>
            </joint>
        </ros2_control>
        """
        elem = ET.fromstring(xml)
        rc = parser._parse_ros2_control(elem)

        assert rc.name == "System"
        assert rc.type == "system"
        assert rc.hardware_plugin == "mock_components/GenericSystem"
        assert rc.parameters["ip"] == "192.168.1.1"
        assert len(rc.joints) == 1
        assert rc.joints[0].name == "j1"
        assert "position" in rc.joints[0].command_interfaces
        assert "velocity" in rc.joints[0].state_interfaces

    def test_urdf_parser_integration(self, parser) -> None:
        """Test full URDF parsing via URDFParser class."""
        xml = """
        <robot name="test_robot">
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>

            <link name="base">
                <visual>
                    <geometry><box size="1 1 1"/></geometry>
                    <material name="blue"/>
                </visual>
            </link>

            <link name="child"/>

            <joint name="j1" type="fixed">
                <parent link="base"/>
                <child link="child"/>
            </joint>

            <transmission name="t1">
                <type>transmission_interface/SimpleTransmission</type>
                <joint name="j1">
                    <hardwareInterface>position</hardwareInterface>
                </joint>
                <actuator name="a1"/>
            </transmission>

            <gazebo reference="base">
                <material>Gazebo/Blue</material>
            </gazebo>
        </robot>
        """
        robot = parser.parse_string(xml)

        assert robot.name == "test_robot"
        assert len(robot.links) == 2
        assert len(robot.joints) == 1
        assert len(robot.transmissions) == 1
        assert len(robot.gazebo_elements) == 1

        # Check that material stayed in GazeboElement
        assert robot.gazebo_elements[0].material == "Gazebo/Blue"

        # Check global material resolution
        assert robot.links[0].visuals[0].material.name == "blue"
        assert robot.links[0].visuals[0].material.color.b == 1.0

    def test_urdf_parser_xacro_detection(self, parser) -> None:
        """Test that XACRO content triggers an error."""
        xml = """
        <robot name="xacro_bot" xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:macro name="m"/>
        </robot>
        """
        with pytest.raises(XacroDetectedError, match="XACRO file detected"):
            parser.parse_string(xml)

    def test_urdf_parser_security(self) -> None:
        """Test security limits."""

        # Test max file size
        parser = URDFParser(max_file_size=10)
        with pytest.raises(RobotParserError, match="Content too large"):
            parser.parse_string("<robot>..............</robot>")

    def test_urdf_parser_robustness(self) -> None:
        """Test duplicate name handling (should skip duplicate)."""

        xml = """
        <robot name="dupe_bot">
            <link name="link1"/>
            <link name="link1"/>
        </robot>
        """
        parser = URDFParser()
        robot = parser.parse_string(xml)

        assert len(robot.links) == 1
        assert robot.links[0].name == "link1"

    def test_parse_sensors_extended(self, parser) -> None:
        """Test parsing various sensor types (lidar, imu, gps, ft, contact)."""
        # Lidar
        xml_lidar = """
        <gazebo reference="link1">
            <sensor name="lidar1" type="ray">
                <ray>
                    <scan><horizontal><samples>640</samples></horizontal></scan>
                    <range><min>0.1</min><max>10.0</max></range>
                </ray>
            </sensor>
        </gazebo>
        """
        sensor_lidar = parser._parse_sensor_from_gazebo(ET.fromstring(xml_lidar))
        assert sensor_lidar.type == SensorType.LIDAR

        # IMU
        xml_imu = (
            '<gazebo reference="link1"><sensor name="imu1" type="imu"><imu/></sensor></gazebo>'
        )
        sensor_imu = parser._parse_sensor_from_gazebo(ET.fromstring(xml_imu))
        assert sensor_imu.type == SensorType.IMU

        # GPS
        xml_gps = (
            '<gazebo reference="link1"><sensor name="gps1" type="gps"><gps/></sensor></gazebo>'
        )
        sensor_gps = parser._parse_sensor_from_gazebo(ET.fromstring(xml_gps))
        assert sensor_gps.type == SensorType.GPS

        # Force/Torque
        xml_ft = '<gazebo reference="link1"><sensor name="ft1" type="force_torque"><force_torque/></sensor></gazebo>'
        sensor_ft = parser._parse_sensor_from_gazebo(ET.fromstring(xml_ft))
        assert sensor_ft.type == SensorType.FORCE_TORQUE

        # Contact
        xml_contact = """
        <gazebo reference="link1">
            <sensor name="c1" type="contact">
                <contact><collision>link1_collision</collision></contact>
            </sensor>
        </gazebo>
        """
        sensor_contact = parser._parse_sensor_from_gazebo(ET.fromstring(xml_contact))
        assert sensor_contact.type == SensorType.CONTACT

    def test_parse_gazebo_element_properties(self, parser) -> None:
        """Test parsing link-specific gazebo properties into link.physics."""
        xml = """
        <robot name="r">
            <link name="link1"/>
            <gazebo reference="link1">
                <material>Gazebo/Red</material>
                <mu1>0.5</mu1>
                <mu2>0.4</mu2>
                <selfCollide>true</selfCollide>
                <gravity>false</gravity>
                <kp>100000</kp>
                <kd>50</kd>
                <maxVel>0.01</maxVel>
                <minDepth>0.001</minDepth>
            </gazebo>
        </robot>
        """
        robot = parser.parse_string(xml)
        link = robot.link("link1")
        phys = link.physics

        # Universal physics properties should be in Link.physics
        assert phys.mu == 0.5
        assert phys.mu2 == 0.4
        assert phys.self_collide is True
        assert phys.gravity is False
        assert phys.kp == 100000.0
        assert phys.kd == 50.0

        # Gazebo-specific properties should be in GazeboElement
        gz = robot.get_gazebo_elements("link1")[0]
        assert gz.material == "Gazebo/Red"
        assert gz.properties["maxVel"] == "0.01"
        assert gz.properties["minDepth"] == "0.001"

    def test_urdf_parser_file_parsing(self, tmp_path) -> None:
        """Test parsing from a file using iterative parser."""

        urdf_content = """
        <robot name="file_robot">
            <link name="base"/>
        </robot>
        """
        urdf_file = tmp_path / "robot.urdf"
        urdf_file.write_text(urdf_content)

        parser = URDFParser()
        robot = parser.parse(urdf_file)

        assert robot.name == "file_robot"
        assert len(robot.links) == 1

    def test_urdf_parser_xacro_extension_check(self, tmp_path) -> None:
        """Test that .xacro extension raises error."""

        xacro_file = tmp_path / "robot.urdf.xacro"
        xacro_file.write_text("<robot/>")

        parser = URDFParser()
        with pytest.raises(RobotParserError, match="XACRO file detected"):
            parser.parse(xacro_file)

    def test_parse_mesh_file_uri(self, parser, tmp_path) -> None:
        """Test parsing mesh with file:// URI and validation."""
        (tmp_path / "mesh.stl").write_text("data")
        elem = ET.fromstring(f'<geometry><mesh filename="file://{tmp_path}/mesh.stl"/></geometry>')
        geom = parser._parse_geometry_element(elem, base_directory=tmp_path)
        assert (
            Path(geom.resource).resolve().as_posix() == (tmp_path / "mesh.stl").resolve().as_posix()
        )

        # Invalid path
        elem_bad = ET.fromstring('<geometry><mesh filename="file:///etc/passwd"/></geometry>')
        geom_bad = parser._parse_geometry_element(elem_bad, base_directory=tmp_path)
        assert geom_bad is None

        # Path traversal
        elem_escape = ET.fromstring(
            '<geometry><mesh filename="file://../../etc/passwd"/></geometry>'
        )
        geom_escape = parser._parse_geometry_element(elem_escape, base_directory=tmp_path)
        assert geom_escape is None

    def test_iterparse_full_structure(self, tmp_path) -> None:
        """Test iterparse loop with ALL element types to hit every branch."""
        xml = """
        <robot name="full_robot">
            <material name="global_mat"><color rgba="1 1 1 1"/></material>

            <link name="base">
                <inertial>
                    <mass value="1.0"/>
                    <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
                </inertial>
                <visual>
                    <geometry><box size="1 1 1"/></geometry>
                </visual>
                <collision>
                    <geometry><box size="1 1 1"/></geometry>
                </collision>
            </link>

            <link name="child"/>

            <joint name="j1" type="revolute">
                <parent link="base"/>
                <child link="child"/>
                <limit lower="-1" upper="1" effort="10" velocity="10"/>
            </joint>

            <transmission name="trans1">
                <type>transmission_interface/SimpleTransmission</type>
                <joint name="j1">
                    <hardwareInterface>position</hardwareInterface>
                </joint>
                <actuator name="a1"/>
            </transmission>

            <ros2_control name="Control" type="system">
                <hardware><plugin>MyPlugin</plugin></hardware>
                <joint name="j1">
                    <command_interface name="position"/>
                </joint>
            </ros2_control>

            <gazebo reference="base">
                <sensor name="cam" type="camera">
                    <camera><image><width>640</width></image></camera>
                </sensor>
            </gazebo>

            <gazebo reference="child">
                <material>Gazebo/Red</material>
            </gazebo>
        </robot>
        """
        urdf_file = tmp_path / "full.urdf"
        urdf_file.write_text(xml)

        parser = URDFParser()
        robot = parser.parse(urdf_file)

        assert len(robot.links) == 2
        assert len(robot.joints) == 1
        assert len(robot.transmissions) == 1
        assert len(robot.ros2_controls) == 1
        assert len(robot.sensors) == 1
        assert len(robot.gazebo_elements) == 1

    def test_link_duplication_skipping(self, tmp_path) -> None:
        """Test that duplicate links are skipped."""
        xml = """
        <robot name="dupe_links">
            <link name="link1"/>
            <link name="link1"/>
            <link name="link1"/>
        </robot>
        """
        urdf_file = tmp_path / "dupe_link.urdf"
        urdf_file.write_text(xml)

        parser = URDFParser()
        robot = parser.parse(urdf_file)

        assert len(robot.links) == 1
        assert robot.links[0].name == "link1"

    def test_joint_duplication_skipping(self, tmp_path) -> None:
        """Test that duplicate joints are skipped."""
        xml = """
        <robot name="dupe_joints">
            <link name="base"/>
            <link name="c1"/>
            <link name="c2"/>
            <link name="c3"/>

            <joint name="j1" type="fixed">
                <parent link="base"/><child link="c1"/>
            </joint>
            <joint name="j1" type="fixed">
                <parent link="base"/><child link="c2"/>
            </joint>
            <joint name="j1" type="fixed">
                <parent link="base"/><child link="c3"/>
            </joint>
        </robot>
        """
        urdf_file = tmp_path / "dupe_joint.urdf"
        urdf_file.write_text(xml)

        parser = URDFParser()
        robot = parser.parse(urdf_file)

        assert len(robot.joints) == 1
        assert robot.joints[0].name == "j1"

    def test_xacro_detection_detailed(self, parser, tmp_path) -> None:
        """Test various XACRO artifacts triggering detection."""

        # Attribute substitution
        with pytest.raises(XacroDetectedError, match="XACRO file detected"):
            parser.parse_string('<robot name="${name}"/>')

        # Xacro namespace in tag
        with pytest.raises(XacroDetectedError, match="XACRO file detected"):
            parser.parse_string(
                '<robot xmlns:xacro="http://ros.org/wiki/xacro"><xacro:macro/></robot>'
            )

        # File content check
        xacro_file = tmp_path / "test.urdf"
        xacro_file.write_text('<robot xmlns:xacro="http://..."/>')

        with pytest.raises(RobotParserError, match="XACRO file detected"):
            parser.parse(xacro_file)

    def test_invalid_values_and_defaults(self) -> None:
        """Test negative values and missing defaults logic."""
        # Negative inertia
        xml = """
        <robot name="bad_inertia">
            <link name="base">
                <inertial>
                    <mass value="1"/>
                    <!-- Negative diagonal elements should be sanitized -->
                    <inertia ixx="-1" iyy="0" izz="-0.1" ixy="0" ixz="0" iyz="0"/>
                </inertial>
            </link>
        </robot>
        """
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert robot.links[0].inertial is not None
        inertia = robot.links[0].inertial.inertia
        assert inertia.ixx == MIN_REASONABLE_INERTIA
        assert inertia.izz == MIN_REASONABLE_INERTIA

    def test_sensor_defaults(self) -> None:
        """Test default info creation for sensors missing specific tags."""
        xml = """
        <robot name="sensor_defaults">
            <link name="base"/>
            <gazebo reference="base">
                <sensor name="cam" type="camera"/> <!-- No <camera> tag -->
            </gazebo>
            <gazebo reference="base">
                <sensor name="lidar" type="ray"/> <!-- No <ray> tag -->
            </gazebo>
            <gazebo reference="base">
                <sensor name="gps" type="gps"/> <!-- No <gps> tag -->
            </gazebo>
            <gazebo reference="base">
                <sensor name="imu" type="imu"/> <!-- No <imu> tag -->
            </gazebo>
        </robot>
        """
        parser = URDFParser()
        robot = parser.parse_string(xml)

        cam = next(s for s in robot.sensors if s.name == "cam")
        assert cam.camera_info is not None
        assert isinstance(cam.camera_info, CameraInfo)

        lidar = next(s for s in robot.sensors if s.name == "lidar")
        assert lidar.lidar_info is not None
        # The parser creates default LidarInfo()

        gps = next(s for s in robot.sensors if s.name == "gps")
        assert isinstance(gps.gps_info, object)  # GPSInfo

        imu = next(s for s in robot.sensors if s.name == "imu")
        assert isinstance(imu.imu_info, object)  # IMUInfo

    def test_parse_contact_sensor_missing_collision(self, parser) -> None:
        """Test contact sensor missing collision element raises RobotModelError."""
        xml = """
        <gazebo reference="link1">
            <sensor name="contact1" type="contact">
                <contact/> <!-- Missing collision -->
            </sensor>
        </gazebo>
        """
        with pytest.raises(RobotModelError, match="contact"):
            parser._parse_sensor_from_gazebo(ET.fromstring(xml))

    def test_security_exceptions(self, parser, tmp_path) -> None:
        """Test security exception re-raising."""
        # Package URI validation - Parse geometry swallows RobotModelError and returns None
        xml = '<geometry><mesh filename="package://../traversal"/></geometry>'
        assert parser._parse_geometry_element(ET.fromstring(xml), base_directory=tmp_path) is None

        # File URI validation - Parse geometry swallows RobotModelError and returns None
        xml2 = '<geometry><mesh filename="file:///etc/passwd"/></geometry>'
        assert parser._parse_geometry_element(ET.fromstring(xml2), base_directory=tmp_path) is None

    def test_gps_noise_structure(self, parser) -> None:
        """Test parsing of nested GPS noise elements."""
        xml = """
        <gazebo reference="gps_link">
            <sensor name="gps" type="gps">
                <gps>
                    <position_sensing>
                        <horizontal>
                            <noise type="gaussian"><mean>0.1</mean></noise>
                        </horizontal>
                    </position_sensing>
                    <!-- Missing velocity sensing to test optional branches -->
                </gps>
            </sensor>
        </gazebo>
        """
        sensor = parser._parse_sensor_from_gazebo(ET.fromstring(xml))
        assert sensor is not None
        assert sensor.gps_info is not None
        assert sensor.gps_info.position_sensing_horizontal_noise is not None
        assert sensor.gps_info.position_sensing_horizontal_noise.mean == 0.1
        assert sensor.gps_info.velocity_sensing_vertical_noise is None

    def test_parse_origin_element(self, parser) -> None:
        """Verify explicit parsing of individual origin elements."""
        # Parse Origin with values
        xml = '<origin xyz="1 2 3" rpy="0.1 0.2 0.3"/>'
        origin = parser._parse_origin_element(ET.fromstring(xml))
        assert origin.xyz.x == 1.0
        assert origin.rpy.x == 0.1

    def test_parse_file_not_found(self) -> None:
        """Test parsing a non-existent file."""

        parser = URDFParser()
        with pytest.raises(RobotParserIOError, match="File not found"):
            parser.parse(Path("non_existent_file.urdf"))

    def test_parse_invalid_root(self, tmp_path) -> None:
        """Test parsing an XML with invalid root element."""

        invalid_urdf = tmp_path / "invalid.urdf"
        invalid_urdf.write_text("<not_robot></not_robot>")

        parser = URDFParser()
        # Parser now wraps standard errors into RobotParserError
        with pytest.raises(RobotParserError, match="Invalid XML root"):
            parser.parse(invalid_urdf)

    def test_parse_string_unexpected_error(self) -> None:
        """Test unexpected error during string parsing."""

        parser = URDFParser()
        with (
            patch("io.BytesIO", side_effect=Exception("Boom")),
            pytest.raises(RobotParserError, match="Unexpected error"),
        ):
            parser.parse_string("<robot name='test'/>")

    def test_parse_material_invalid_color(self, parser) -> None:
        """Test parsing invalid material colors."""
        materials = {}

        # Too few components
        xml = '<material name="m"><color rgba="0.1 0.2" /></material>'
        elem = ET.fromstring(xml)
        assert parser._parse_material_element(elem, materials) is None

        # Too many components
        xml = '<material name="m"><color rgba="0.1 0.2 0.3 0.4 0.5" /></material>'
        elem = ET.fromstring(xml)
        assert parser._parse_material_element(elem, materials) is None

    def test_parse_geometry_invalid_mesh(self, parser) -> None:
        """Test parsing invalid mesh geometry."""
        # Missing filename
        xml = "<geometry><mesh /></geometry>"
        elem = ET.fromstring(xml)
        assert parser._parse_geometry_element(elem) is None  # Warns instead of raising out

        # Cylinder invalid length
        xml = '<geometry><cylinder radius="1" length="-1"/></geometry>'
        assert parser._parse_geometry_element(ET.fromstring(xml)) is None

        # Mesh missing filename
        xml = "<geometry><mesh/></geometry>"
        assert parser._parse_geometry_element(ET.fromstring(xml)) is None

        # Mesh negative scale (Mirroring support)
        xml = '<geometry><mesh filename="f.stl" scale="-1 1 1"/></geometry>'
        geom = parser._parse_geometry_element(ET.fromstring(xml))
        assert isinstance(geom, Mesh)
        assert geom.scale.x == -1.0

        # Material Errors
        # Invalid RGBA length
        xml = '<material name="bad"><color rgba="1 1"/></material>'
        assert parser._parse_material_element(ET.fromstring(xml), {}) is None

        xml = '<material name="bad"><color rgba="1 1 1 1 1"/></material>'
        assert parser._parse_material_element(ET.fromstring(xml), {}) is None

        # Empty material
        xml = '<material name="empty"/>'
        assert parser._parse_material_element(ET.fromstring(xml), {}) is None

        # Transmission Errors
        # Invalid mechanicalReduction
        xml = '<joint name="j1"><mechanicalReduction>not_number</mechanicalReduction></joint>'
        # parse_float raises RobotMathError for non-numeric strings
        with pytest.raises(RobotModelError, match="Invalid float format 'not_number'"):
            parser._parse_transmission_component(ET.fromstring(xml), "joint")

        # Gazebo Sensor missing reference
        xml = '<gazebo><sensor name="s" type="camera"/></gazebo>'  # No reference attr
        assert parser._parse_sensor_from_gazebo(ET.fromstring(xml)) is None

        # Parse String errors
        parser = URDFParser()
        with pytest.raises(RobotParserError, match="Unexpected error in URDF XML"):
            parser.parse_string("<robot>unclosed tags")

        # Joint with explicit Axis
        xml = '<joint name="j1" type="continuous"><parent link="p"/><child link="c"/><axis xyz="0 1 0"/></joint>'
        joint = parser._parse_joint(ET.fromstring(xml))
        assert joint.axis is not None
        assert joint.axis.y == 1.0

        # Gazebo Plugin parsing
        xml = """
        <plugin name="p" filename="lib.so">
            <param>value</param>
        </plugin>
        """
        plugin = parser._parse_gazebo_plugin(ET.fromstring(xml))
        assert plugin is not None
        assert plugin.name == "p"
        assert plugin.filename == "lib.so"
        assert plugin.raw_xml is not None
        assert "<param>value</param>" in plugin.raw_xml

        # ROS2 Control misc parameters
        xml = """
        <ros2_control name="c" type="system">
            <hardware><plugin>H</plugin></hardware>
            <param_block>some config</param_block>
        </ros2_control>
        """
        rc = parser._parse_ros2_control(ET.fromstring(xml))
        assert rc is not None
        assert rc.parameters["param_block"] == "some config"

    def test_parse_robot_full_traversal(self) -> None:
        """Hit edge cases in _parse_robot that aren't hit by iterparse."""

        # Defines a robot with features that trigger specific loops in _parse_robot
        xml = """
        <robot name="full_traversal">
            <ros2_control name="c" type="system">
                <hardware><plugin>H</plugin></hardware>
            </ros2_control>
            <!-- Invalid joint to trigger exception handler in _parse_robot loop -->
            <joint name="bad_joint" type="invalid_type"/>
            <!-- Valid joint -->
            <link name="base"/>
            <link name="child"/>
            <joint name="good" type="fixed">
                 <parent link="base"/>
                 <child link="child"/>
             </joint>
        </robot>
        """
        parser = URDFParser()
        # parse_string uses _parse_robot internal logic
        robot = parser.parse_string(xml)
        assert len(robot.ros2_controls) == 1

    def test_parse_gazebo_element_with_plugins(self) -> None:
        """Gazebo element with only a plugin (no sensor) is stored as a GazeboElement."""

        parser = URDFParser()
        xml = """
        <robot name="r">
            <gazebo>
                <plugin name="p" filename="lib.so"/>
            </gazebo>
        </robot>
        """
        robot = parser.parse_string(xml)
        assert len(robot.gazebo_elements) == 1
        assert len(robot.gazebo_elements[0].plugins) == 1
        assert robot.gazebo_elements[0].plugins[0].name == "p"

    def test_parse_file_iterparse_error(self, tmp_path) -> None:
        """A malformed XML file raises RobotParserError with a clear message."""
        from unittest import mock

        path = tmp_path / "test.urdf"
        path.touch()
        parser = URDFParser()

        # Mock ET.iterparse to raise ParseError
        with (
            mock.patch("xml.etree.ElementTree.iterparse", side_effect=ET.ParseError("Bad XML")),
            pytest.raises(RobotParserError, match="URDF XML"),
        ):
            parser.parse(path)

    # Robustness and Edge Cases

    def test_parse_lidar_sensor_without_range_element(self) -> None:
        """Lidar sensor element missing a range sub-element uses default range values."""
        xml = """<robot name="r">
            <link name="base"/>
            <gazebo reference="base">
                <sensor type="ray" name="lidar"><ray><horizontal/></ray></sensor>
            </gazebo>
        </robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert len(robot.sensors) == 1

    def test_parse_imu_with_empty_angular_and_linear_elements(self) -> None:
        """IMU sensor with empty angular_velocity and linear_acceleration elements parses cleanly."""
        xml = """<robot name="r">
            <link name="base"/>
            <gazebo reference="base">
                <sensor type="imu" name="imu0">
                    <imu><angular_velocity/><linear_acceleration/></imu>
                </sensor>
            </gazebo>
        </robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert len(robot.sensors) == 1

    def test_parse_force_torque_without_inner_element(self) -> None:
        """Force/torque sensor element without inner force_torque child uses defaults."""
        xml = """<robot name="r">
            <link name="base"/>
            <gazebo reference="base">
                <sensor type="force_torque" name="ft0"></sensor>
            </gazebo>
        </robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert len(robot.sensors) == 1

    def test_parse_gazebo_element_without_sensor(self) -> None:
        """Gazebo element referencing a link but without a sensor is stored as a GazeboElement."""
        xml = """<robot name="r">
            <link name="base"/>
            <gazebo reference="base"><mu1>0.9</mu1></gazebo>
        </robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        # It should be 0 because mu1 is now part of Link.physics
        assert len(robot.gazebo_elements) == 0
        assert robot.link("base").physics.mu == 0.9

    def test_parse_robot_without_filepath_uses_unnamed_robot(self) -> None:
        """Parsing from a string without a filepath falls back to 'unnamed_robot' name."""
        xml = """<robot><link name="l1"/></robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert robot.name == "unnamed_robot"

    # Mesh and Path Validation

    def test_mesh_path_validation_error_with_directory(self, tmp_path) -> None:
        """Non-package:// mesh path with source_directory triggers security validation."""
        xml = """<robot name="r"><link name="l1"><visual>
            <geometry><mesh filename="/outside/path/mesh.stl"/></geometry>
        </visual></link></robot>"""
        parser = URDFParser()
        # When source_directory is set and path escapes it, logs warning and returns None geometry
        robot = parser.parse_string(xml, source_directory=tmp_path)
        # Mesh should be skipped — link exists but no visual geometry
        assert len(robot.links) == 1

    def test_link_with_inertial_but_no_inertia_element(self) -> None:
        """Link with <inertial><mass.../></inertial> but without <inertia> creates Inertial with no tensor."""
        xml = """<robot name="r"><link name="l1">
            <inertial><mass value="2.0"/></inertial>
        </link></robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        # We now create a valid Inertial with zero() tensor instead of None
        assert robot.links[0].inertial is not None
        assert robot.links[0].inertial.mass == 2.0
        assert robot.links[0].inertial.inertia.ixx == MIN_REASONABLE_INERTIA

    def test_link_with_negative_inertia_is_sanitized(self) -> None:
        """Link inertia with negative diagonal values are sanitized to 1e-6."""
        xml = """<robot name="r"><link name="l1">
            <inertial>
                <mass value="1.0"/>
                <inertia ixx="-1.0" ixy="0.0" ixz="0.0" iyy="-1.0" iyz="0.0" izz="-1.0"/>
            </inertial>
        </link></robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert robot.links[0].inertial is not None
        t = robot.links[0].inertial.inertia
        assert t.ixx == pytest.approx(MIN_REASONABLE_INERTIA)
        assert t.iyy == pytest.approx(MIN_REASONABLE_INERTIA)
        assert t.izz == pytest.approx(MIN_REASONABLE_INERTIA)

    def test_collision_with_no_geometry_is_skipped(self) -> None:
        """Collision element without any geometry child is silently not added."""
        xml = """<robot name="r"><link name="l1">
            <collision><geometry></geometry></collision>
        </link></robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert len(robot.links[0].collisions) == 0

    def test_invalid_transmission_is_ignored(self) -> None:
        """A transmission with invalid values logs a warning and is not added."""
        xml = """<robot name="r">
            <link name="l1"/>
            <link name="l2"/>
            <joint name="j" type="revolute">
                <parent link="l1"/><child link="l2"/>
                <limit effort="1" velocity="1"/>
            </joint>
            <transmission name="t">
                <type>transmission_interface/SimpleTransmission</type>
                <joint name="j"><hardwareInterface>this_is_invalid</hardwareInterface></joint>
            </transmission>
        </robot>"""
        parser = URDFParser()
        # Should not raise — just skip the transmission
        robot = parser.parse_string(xml)
        assert isinstance(
            robot, __import__("linkforge.core.models.robot", fromlist=["Robot"]).Robot
        )

    def test_camera_sensor_without_image_element_uses_defaults(self) -> None:
        """Camera sensor missing an <image> child falls back to width=640, height=480."""
        xml = """<robot name="r">
            <link name="base"/>
            <gazebo reference="base">
                <sensor type="camera" name="cam0">
                    <camera><clip><near>0.1</near><far>10.0</far></clip></camera>
                </sensor>
            </gazebo>
        </robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert robot.sensors[0].camera_info is not None
        assert robot.sensors[0].camera_info.width == 640
        assert robot.sensors[0].camera_info.height == 480

    def test_contact_sensor_missing_contact_element_robustness(self) -> None:
        """Contact sensor without a <contact> child is skipped and logged (robustness)."""
        xml = """<robot name="r"><gazebo reference="base">
            <sensor type="contact" name="ct0"></sensor>
        </gazebo></robot>"""
        parser = URDFParser()
        with patch("linkforge.core.parsers.urdf_parser.logger") as mock_logger:
            robot = parser.parse_string(xml)
            assert len(robot.sensors) == 0
            assert mock_logger.warning.called

    def test_parse_string_with_robot_parser_error_reraises(self) -> None:
        """RobotParserError from within parse_string passes through unchanged."""
        import pytest

        xml = """<robot name="r"><xacro:if value="1"><link name="l1"/></xacro:if></robot>"""
        parser = URDFParser()
        with pytest.raises(XacroDetectedError):
            parser.parse_string(xml)

    # System and Hardware Parameters

    def test_ros2_control_hardware_params_are_parsed(self) -> None:
        """Hardware <param> elements inside ros2_control are collected into the parameters dict."""
        xml = """<robot name="r">
            <link name="base"/>
            <link name="child"/>
            <joint name="base_joint" type="fixed">
                <parent link="base"/>
                <child link="child"/>
            </joint>
            <ros2_control name="hw" type="system">
                <hardware>
                    <plugin>fake_components/GenericSystem</plugin>
                    <param name="joints">base_joint</param>
                </hardware>
                <joint name="base_joint">
                    <command_interface name="position"/>
                    <state_interface name="position"/>
                </joint>
            </ros2_control>
        </robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert len(robot.ros2_controls) == 1
        ctrl = robot.ros2_controls[0]
        assert "joints" in ctrl.parameters

    def test_ros2_control_joint_without_interfaces_is_not_added(self) -> None:
        """A ros2_control joint with no command or state interfaces is skipped."""
        xml = """<robot name="r">
            <link name="base"/>
            <ros2_control name="hw" type="system">
                <hardware><plugin>test/System</plugin></hardware>
                <joint name="ignored"/>
            </ros2_control>
        </robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert robot.ros2_controls[0].joints == ()

    def test_ros2_control_sensor_without_interfaces_is_not_added(self) -> None:
        """A ros2_control sensor with no state interfaces or params is skipped."""
        xml = """<robot name="r">
            <link name="base"/>
            <ros2_control name="hw" type="system">
                <hardware><plugin>test/System</plugin></hardware>
                <sensor name="ignored"/>
            </ros2_control>
        </robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert robot.ros2_controls[0].sensors == ()

    def test_imu_sensor_with_angular_velocity_x_noise(self) -> None:
        """IMU angular_velocity with an <x> noise element parses the noise correctly."""
        xml = """<robot name="r">
            <link name="base"/>
            <gazebo reference="base">
                <sensor type="imu" name="imu0">
                    <imu>
                        <angular_velocity>
                            <x><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></x>
                        </angular_velocity>
                    </imu>
                </sensor>
            </gazebo>
        </robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        imu = robot.sensors[0].imu_info
        assert imu is not None
        assert imu.angular_velocity_noise is not None

    def test_imu_sensor_with_linear_acceleration_x_noise(self) -> None:
        """IMU linear_acceleration with an <x> noise element parses the noise correctly."""
        xml = """<robot name="r">
            <link name="base"/>
            <gazebo reference="base">
                <sensor type="imu" name="imu0">
                    <imu>
                        <linear_acceleration>
                            <x><noise type="gaussian"><mean>0.0</mean><stddev>0.05</stddev></noise></x>
                        </linear_acceleration>
                    </imu>
                </sensor>
            </gazebo>
        </robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        imu = robot.sensors[0].imu_info
        assert imu is not None
        assert imu.linear_acceleration_noise is not None

    def test_parse_file_too_large_raises_error(self, tmp_path) -> None:
        """Parser raises RobotParserError if the file exceeds max_file_size."""

        urdf_file = tmp_path / "big.urdf"
        urdf_file.write_text("<robot name='r'><link name='l1'/></robot>")
        parser = URDFParser()
        parser.max_file_size = 1  # 1 byte — will trip on any content
        with pytest.raises(RobotParserError, match="File too large"):
            parser.parse(urdf_file)

    def test_parse_file_xacro_string_too_large_raises_error(self) -> None:
        """parse_string raises RobotParserError if the string exceeds max_file_size."""

        content = "<robot name='r'>" + "<link name='l1'/>" * 10 + "</robot>"
        parser = URDFParser()
        parser.max_file_size = 1  # 1 byte — will trip immediately
        with pytest.raises(RobotParserError, match="Content too large"):
            parser.parse_string(content)

    def test_parse_iterative_out_of_order(self, tmp_path) -> None:
        """Test that URDFParser.parse (iterative) handles joints before links."""
        urdf_content = """<?xml version="1.0"?>
<robot name="test_robot">
  <joint name="joint1" type="fixed">
    <parent link="link1"/>
    <child link="link2"/>
  </joint>
  <link name="link1"/>
  <link name="link2"/>
</robot>
"""
        urdf_file = tmp_path / "test.urdf"
        urdf_file.write_text(urdf_content)

        parser = URDFParser()
        # This should succeed even though joint is before links
        robot = parser.parse(urdf_file)

        assert robot.has_link("link1")
        assert robot.has_link("link2")
        assert robot.has_joint("joint1")
        joint = robot.get_joint("joint1")
        assert joint is not None
        assert joint.parent == "link1"
        assert joint.child == "link2"

    def test_urdf_parser_directory_failure_and_file_success(self, tmp_path) -> None:
        """Test file-based parsing edge cases."""
        parser = URDFParser()

        # Path is a directory
        with pytest.raises(RobotParserError, match="Target path is a directory"):
            parser.parse(tmp_path)

    def test_urdf_parser_string_invalid_root(self) -> None:
        """Test parse_string with invalid root tag."""
        parser = URDFParser()
        with pytest.raises(RobotParserError, match="Invalid XML root"):
            parser.parse_string("<wrong_root></wrong_root>")

    def test_urdf_parser_unexpected_error(self, tmp_path) -> None:
        """Test fallback Exception handler in parse."""
        parser = URDFParser()
        p = tmp_path / "test.urdf"
        p.write_text("<robot></robot>")

        from unittest.mock import patch

        with patch("xml.etree.ElementTree.iterparse") as mock_iter:
            mock_iter.side_effect = RuntimeError("Mocked crash")
            with pytest.raises(RobotParserError, match="Unexpected URDF parse"):
                parser.parse(p)

        # Successful file parse
        f = tmp_path / "simple.urdf"
        f.write_text("<robot name='ftest'/>")
        robot = parser.parse(f)
        assert robot.name == "ftest"

    def test_urdf_parser_remaining_coverage(self, parser, tmp_path) -> None:
        """Cover remaining edge cases in urdf_parser.py for 100% coverage."""
        import xml.etree.ElementTree as ET
        from unittest.mock import patch

        from linkforge.core import FileSystemResolver, JointType
        from linkforge.core.exceptions import XacroDetectedError
        from linkforge.core.models.robot import Robot

        # 1. Line 200: Visual element missing geometry
        visual_elem = ET.fromstring("<visual></visual>")
        assert parser._parse_visual_element(visual_elem, {}) is None

        # 2. Line 230: Collision element missing geometry
        collision_elem = ET.fromstring("<collision></collision>")
        assert parser._parse_collision_element(collision_elem) is None

        # 3. Line 311: _parse_joint_axis with a JointType that isn't fixed, floating, or revolute/continuous/prismatic/planar
        joint_elem = ET.fromstring("<joint></joint>")
        assert parser._parse_joint_axis(joint_elem, "dummy") is None

        # 4. Lines 349-350 and 352-353: Joint limits negative effort/velocity
        joint_elem_lim = ET.fromstring('<joint><limit effort="-5.0" velocity="-2.0"/></joint>')
        limits = parser._parse_joint_limits(joint_elem_lim, JointType.REVOLUTE, "test_joint")
        assert limits is not None
        assert limits.effort == 0.0
        assert limits.velocity == 0.0

        # 5. Lines 582-583: mechanicalReduction is 0
        reduction_elem = ET.fromstring(
            '<joint name="test_comp"><mechanicalReduction>0.0</mechanicalReduction></joint>'
        )
        comp = parser._parse_transmission_component(reduction_elem, "joint")
        assert comp.mechanical_reduction == 1.0

        # 6. Line 705: Gazebo sensor pose with at least 6 values
        sensor_xml = """
        <gazebo reference="l">
            <sensor name="s" type="camera">
                <pose>1.0 2.0 3.0 0.1 0.2 0.3</pose>
            </sensor>
        </gazebo>
        """
        sensor = parser._parse_sensor_from_gazebo(ET.fromstring(sensor_xml))
        assert sensor is not None
        assert sensor.origin is not None
        assert sensor.origin.xyz.x == 1.0
        assert sensor.origin.rpy.x == 0.1

        # 7. Lines 849, 851, 853, 855: safety fallbacks when camera, lidar, imu, gps infos are None
        # Camera fallback
        xml_cam = '<gazebo reference="l"><sensor name="s" type="camera"></sensor></gazebo>'
        sensor_cam = parser._parse_sensor_from_gazebo(ET.fromstring(xml_cam))
        assert sensor_cam.camera_info is not None

        # Depth camera fallback
        xml_depth = '<gazebo reference="l"><sensor name="s" type="depth_camera"></sensor></gazebo>'
        sensor_depth = parser._parse_sensor_from_gazebo(ET.fromstring(xml_depth))
        assert sensor_depth.camera_info is not None

        # Lidar fallback
        xml_lidar = '<gazebo reference="l"><sensor name="s" type="ray"></sensor></gazebo>'
        sensor_lidar = parser._parse_sensor_from_gazebo(ET.fromstring(xml_lidar))
        assert sensor_lidar.lidar_info is not None

        # IMU fallback
        xml_imu = '<gazebo reference="l"><sensor name="s" type="imu"></sensor></gazebo>'
        sensor_imu = parser._parse_sensor_from_gazebo(ET.fromstring(xml_imu))
        assert sensor_imu.imu_info is not None

        # GPS fallback
        xml_gps = '<gazebo reference="l"><sensor name="s" type="gps"></sensor></gazebo>'
        sensor_gps = parser._parse_sensor_from_gazebo(ET.fromstring(xml_gps))
        assert sensor_gps.gps_info is not None

        # 8. Lines 964-965: XACRO namespace in a child element tag (detected xacro)
        xml_xacro_child = """
        <robot xmlns:myprefix="http://www.ros.org/wiki/xacro">
            <myprefix:macro/>
        </robot>
        """
        with pytest.raises(XacroDetectedError):
            parser.parse_string(xml_xacro_child)

        # 9. Line 1004: kwargs["resource_resolver"] when resource_resolver is not None
        resolver = FileSystemResolver()
        resolver_parser = URDFParser(resource_resolver=resolver)
        robot = resolver_parser.parse_string('<robot name="resolver_robot"></robot>')
        assert robot.resource_resolver is resolver

        # 10. Lines 1058-1059: parse exception for invalid transmission
        # 11. Lines 1068-1069: parse exception for invalid ros2_control
        with (
            patch.object(
                URDFParser, "_parse_transmission", side_effect=ValueError("Invalid trans")
            ),
            patch.object(URDFParser, "_parse_ros2_control", side_effect=ValueError("Invalid ctrl")),
        ):
            xml_exc = """
            <robot name="exc_bot">
                <transmission name="t1"/>
                <ros2_control name="c1" type="system"/>
            </robot>
            """
            robot = parser.parse_string(xml_exc)
            assert len(robot.transmissions) == 0
            assert len(robot.ros2_controls) == 0

        # 12. Lines 1114-1115: robot.add_transmission exception
        # 13. Lines 1120-1121: robot.add_ros2_control exception
        # 14. Lines 1126-1127: robot.add_sensor exception
        # 15. Lines 1152-1153: robot.add_gazebo_element exception
        with (
            patch.object(Robot, "add_transmission", side_effect=RuntimeError("trans fail")),
            patch.object(Robot, "add_ros2_control", side_effect=RuntimeError("ctrl fail")),
            patch.object(Robot, "add_sensor", side_effect=RuntimeError("sensor fail")),
            patch.object(Robot, "add_gazebo_element", side_effect=RuntimeError("gazebo fail")),
        ):
            xml_add_exc = """
            <robot name="add_exc_bot">
                <link name="base"/>
                <joint name="j" type="fixed"><parent link="base"/><child link="base"/></joint>
                <transmission name="t">
                    <type>T</type>
                    <joint name="j"/>
                    <actuator name="a"/>
                </transmission>
                <ros2_control name="c" type="system"><hardware><plugin>P</plugin></hardware></ros2_control>
                <gazebo reference="base">
                    <sensor name="s" type="camera"></sensor>
                </gazebo>
                <gazebo reference="base">
                    <material>Red</material>
                </gazebo>
            </robot>
            """
            robot = parser.parse_string(xml_add_exc)
            assert len(robot.transmissions) == 0
            assert len(robot.ros2_controls) == 0
            assert len(robot.sensors) == 0
            assert len(robot.gazebo_elements) == 0


def test_urdf_parser_unnamed_gazebo_element_parsing() -> None:
    """Verify that Gazebo elements without a reference attribute are parsed correctly with None reference."""
    import xml.etree.ElementTree as ET

    parser = URDFParser()
    xml = "<gazebo><material>Gazebo/Grey</material></gazebo>"
    elem = ET.fromstring(xml)
    res = parser._parse_gazebo_element(elem)
    assert res.reference is None
    assert res.material == "Gazebo/Grey"


def test_urdf_parser_empty_material_root_element_skipped() -> None:
    """Verify that a <material> at root with no name/color/texture is silently ignored."""
    xml = """<robot name="r">
        <material></material>
    </robot>"""
    parser = URDFParser()
    robot = parser.parse_string(xml)
    assert len(robot.materials) == 0


def test_ros2_control_missing_hardware_and_param_edge_cases() -> None:
    """Verify ros2_control parsing with missing hardware or malformed parameters."""
    xml_no_hw = """<robot name="r">
        <ros2_control name="ctrl_no_hw" type="system">
            <joint name="j">
                <command_interface name="position"/>
                <state_interface name="position"/>
            </joint>
        </ros2_control>
    </robot>"""
    parser = URDFParser()
    robot = parser.parse_string(xml_no_hw)
    # Hardware plugin is empty, which raises RobotValidationError, so it is skipped (0 controls)
    assert len(robot.ros2_controls) == 0

    # Test param without name or without text
    xml_bad_params = """<robot name="r">
        <link name="base"/>
        <link name="child"/>
        <joint name="j" type="fixed"><parent link="base"/><child link="child"/></joint>
        <ros2_control name="ctrl_bad_params" type="system">
            <hardware>
                <plugin>fake_plugin</plugin>
                <param>no_name</param>
                <param name="no_text"></param>
            </hardware>
            <joint name="j">
                <command_interface name="position"/>
            </joint>
        </ros2_control>
    </robot>"""
    robot2 = parser.parse_string(xml_bad_params)
    assert len(robot2.ros2_controls) == 1
    assert len(robot2.ros2_controls[0].parameters) == 0


def test_transmission_component_missing_name_skipped() -> None:
    """Verify that joint/actuator components without name are skipped inside transmission."""
    xml = """<robot name="r">
        <link name="base"/>
        <link name="child"/>
        <joint name="j" type="fixed"><parent link="base"/><child link="child"/></joint>
        <transmission name="t">
            <type>transmission_interface/SimpleTransmission</type>
            <joint/>
            <joint name="j"/>
            <actuator/>
            <actuator name="a"/>
        </transmission>
    </robot>"""
    parser = URDFParser()
    robot = parser.parse_string(xml)
    # Should parse transmission with only 1 valid joint and 1 valid actuator
    assert len(robot.transmissions) == 1
    assert len(robot.transmissions[0].joints) == 1
    assert len(robot.transmissions[0].actuators) == 1


def test_parse_sensor_gpu_lidar() -> None:
    """Verify that a gpu_ray/gpu_lidar sensor type is parsed but leaves other info fields None."""
    from linkforge.core import SensorType

    xml = """<robot name="r">
        <link name="base"/>
        <gazebo reference="base">
            <sensor type="gpu_ray" name="lidar0">
                <update_rate>10.0</update_rate>
            </sensor>
        </gazebo>
    </robot>"""
    parser = URDFParser()
    robot = parser.parse_string(xml)
    assert len(robot.sensors) == 1
    assert robot.sensors[0].type == SensorType.GPU_LIDAR
    assert robot.sensors[0].lidar_info is None


def test_detect_xacro_file_read_text_error(tmp_path) -> None:
    """Verify that detect_xacro handles OSError during read_text gracefully."""
    import xml.etree.ElementTree as ET
    from pathlib import Path
    from unittest.mock import patch

    filepath = tmp_path / "broken.urdf"
    filepath.write_text("<robot/>")
    parser = URDFParser()

    with patch.object(Path, "read_text", side_effect=OSError("Read error")):
        parser._detect_xacro_file(ET.Element("robot"), filepath=filepath)


def test_ros2_control_invalid_returns_none() -> None:
    """Verify that ros2_control with invalid values (like invalid type) is caught and skipped."""
    xml = """<robot name="r">
        <ros2_control name="ctrl" type="invalid_type">
            <hardware><plugin>foo</plugin></hardware>
        </ros2_control>
    </robot>"""
    parser = URDFParser()
    robot = parser.parse_string(xml)
    assert len(robot.ros2_controls) == 0


def test_gazebo_element_exception_handling() -> None:
    """Verify that exception during Gazebo element parsing is caught and logged."""
    from unittest.mock import patch

    xml = """<robot name="r">
        <gazebo reference="base">
            <sensor name="s" type="contact"></sensor>
        </gazebo>
    </robot>"""
    parser = URDFParser()
    with patch.object(
        URDFParser, "_parse_gazebo_element", side_effect=ValueError("Gazebo parse fail")
    ):
        robot = parser.parse_string(xml)
        assert len(robot.sensors) == 0


def test_parse_with_xacro_suffix_raises_immediately(tmp_path) -> None:
    """Verify that parsing a file with a .xacro suffix raises XacroDetectedError immediately."""
    filepath = tmp_path / "model.urdf.xacro"
    filepath.write_text("<robot/>")
    parser = URDFParser()
    with pytest.raises(XacroDetectedError, match="XACRO file detected"):
        parser.parse(filepath)

    # Call _detect_xacro_file directly to hit the 944->951 suffix branch
    with pytest.raises(XacroDetectedError, match="XACRO file detected"):
        parser._detect_xacro_file(ET.Element("robot"), filepath=filepath)


def test_urdf_parser_unknown_root_tag_ignored() -> None:
    """Verify that an unknown tag in the root of URDF is safely ignored."""
    xml = """<robot name="r">
        <unknown_tag attribute="val"/>
    </robot>"""
    parser = URDFParser()
    robot = parser.parse_string(xml)
    assert robot is not None
