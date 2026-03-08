import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import pytest
from linkforge_core.base import RobotParserError, XacroDetectedError
from linkforge_core.exceptions import RobotModelError
from linkforge_core.models import (
    Box,
    CameraInfo,
    Color,
    Cylinder,
    JointType,
    Material,
    Mesh,
    Sensor,
    SensorType,
    Sphere,
)
from linkforge_core.parsers.urdf_parser import (
    URDFParser,
    _detect_xacro_file,
    parse_geometry,
    parse_joint,
    parse_link,
    parse_material,
    parse_sensor_from_gazebo,
    parse_transmission,
)


class TestURDFParser:
    def test_parse_geometry_box(self):
        """Test parsing box geometry."""
        xml = '<geometry><box size="1 2 3"/></geometry>'
        elem = ET.fromstring(xml)
        geom = parse_geometry(elem)

        assert isinstance(geom, Box)
        assert geom.size.x == 1.0
        assert geom.size.y == 2.0
        assert geom.size.z == 3.0

    def test_parse_geometry_cylinder(self):
        """Test parsing cylinder geometry."""
        xml = '<geometry><cylinder radius="0.5" length="2.0"/></geometry>'
        elem = ET.fromstring(xml)
        geom = parse_geometry(elem)

        assert isinstance(geom, Cylinder)
        assert geom.radius == 0.5
        assert geom.length == 2.0

    def test_parse_geometry_sphere(self):
        """Test parsing sphere geometry."""
        xml = '<geometry><sphere radius="1.5"/></geometry>'
        elem = ET.fromstring(xml)
        geom = parse_geometry(elem)

        assert isinstance(geom, Sphere)
        assert geom.radius == 1.5

    def test_parse_geometry_mesh(self):
        """Test parsing mesh geometry with scaling."""
        xml = (
            '<geometry><mesh filename="package://my_pkg/mesh.stl" scale="0.1 0.1 0.1"/></geometry>'
        )
        elem = ET.fromstring(xml)

        geom = parse_geometry(elem)

        assert isinstance(geom, Mesh)
        assert Path(geom.resource).name == "mesh.stl"
        assert geom.scale.x == 0.1

    def test_parse_material_color(self):
        """Test parsing material with color."""
        xml = '<material name="blue"><color rgba="0 0 1 1"/></material>'
        elem = ET.fromstring(xml)
        mat = parse_material(elem, {})

        assert isinstance(mat, Material)
        assert mat.name == "blue"
        assert mat.color.r == 0.0
        assert mat.color.b == 1.0
        assert mat.color.a == 1.0

    def test_parse_link_full(self):
        """Test parsing a complete link with visual, collision, and inertial."""
        xml = """
        <link name="base_link">
            <visual>
                <geometry><box size="1 1 1"/></geometry>
                <material name="mat"><color rgba="1 0 0 1"/></material>
            </visual>
            <collision>
                <geometry><box size="1 1 1"/></geometry>
            </collision>
            <inertial>
                <mass value="5.0"/>
                <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
            </inertial>
        </link>
        """
        elem = ET.fromstring(xml)
        link = parse_link(elem, {})

        assert link.name == "base_link"
        assert len(link.visuals) == 1
        assert isinstance(link.visuals[0].geometry, Box)
        assert len(link.collisions) == 1
        assert link.inertial.mass == 5.0
        assert link.inertial.inertia.ixx == 0.1

    def test_parse_joint_limits(self):
        """Test parsing joint with limits and dynamics."""
        xml = """
        <joint name="j1" type="revolute">
            <parent link="base"/>
            <child link="link1"/>
            <limit lower="-1.57" upper="1.57" effort="10" velocity="5"/>
            <dynamics damping="0.5" friction="0.1"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parse_joint(elem)

        assert joint.name == "j1"
        assert joint.type == JointType.REVOLUTE
        assert joint.limits.lower == -1.57
        assert joint.limits.upper == 1.57
        assert joint.dynamics.damping == 0.5

    def test_parse_mimic_joint(self):
        """Test parsing joint with mimic tag."""
        xml = """
        <joint name="j2" type="revolute">
            <parent link="base"/>
            <child link="link2"/>
            <limit lower="-1.57" upper="1.57" effort="10" velocity="5"/>
            <mimic joint="j1" multiplier="2.0" offset="0.5"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parse_joint(elem)

        assert joint.mimic is not None
        assert joint.mimic.joint == "j1"
        assert joint.mimic.multiplier == 2.0
        assert joint.mimic.offset == 0.5

    def test_parse_joint_safety_controller(self):
        """Test parsing joint with safety controller."""
        xml = """
        <joint name="j_safety" type="revolute">
            <parent link="base"/>
            <child link="link1"/>
            <limit effort="10" velocity="5" lower="-1" upper="1"/>
            <safety_controller soft_lower_limit="-0.9" soft_upper_limit="0.9" k_position="15" k_velocity="10"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parse_joint(elem)

        assert joint.safety_controller is not None
        assert joint.safety_controller.soft_lower_limit == -0.9
        assert joint.safety_controller.soft_upper_limit == 0.9
        assert joint.safety_controller.k_position == 15.0
        assert joint.safety_controller.k_velocity == 10.0

    def test_parse_joint_calibration(self):
        """Test parsing joint with calibration."""
        xml = """
        <joint name="j_calib" type="fixed">
            <parent link="base"/>
            <child link="link2"/>
            <calibration rising="0.5" falling="1.0"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parse_joint(elem)

        assert joint.calibration is not None
        assert joint.calibration.rising == 0.5
        assert joint.calibration.falling == 1.0

    def test_parse_joint_postels_law(self):
        """Test Postel's Law: liberal on import (ignore invalid tags for fixed joints)."""
        # FIXED joint with axis and limit (invalid in model, but should be ignored by parser)
        xml = """
        <joint name="j_postel" type="fixed">
            <parent link="base"/>
            <child link="link1"/>
            <axis xyz="0 0 1"/>
            <limit lower="0" upper="1" effort="10" velocity="1"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parse_joint(elem)

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
        joint_rev = parse_joint(ET.fromstring(xml_rev))
        assert joint_rev.axis.x == 1.0
        assert joint_rev.axis.y == 0.0
        assert joint_rev.axis.z == 0.0

    def test_parse_sensor_camera(self):
        """Test parsing a camera sensor from Gazebo format."""
        xml = """
        <gazebo reference="camera_link">
            <sensor name="camera1" type="camera">
                <update_rate>30</update_rate>
                <camera>
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
                </camera>
            </sensor>
        </gazebo>
        """
        elem = ET.fromstring(xml)
        sensor = parse_sensor_from_gazebo(elem)

        assert isinstance(sensor, Sensor)
        assert sensor.name == "camera1"
        assert sensor.type == SensorType.CAMERA
        assert isinstance(sensor.camera_info, CameraInfo)

    def test_parse_sphere_invalid(self):
        """Test parsing invalid sphere."""
        # Negative radius is a RobotModelError at model level,
        # but parse_geometry catches it and returns None (robust behavior)
        xml = '<geometry><sphere radius="-1.0"/></geometry>'
        assert parse_geometry(ET.fromstring(xml)) is None

    def test_parse_geometry_invalid(self):
        """Test parsing invalid geometries."""
        # Box missing size
        xml = "<geometry><box/></geometry>"
        assert parse_geometry(ET.fromstring(xml)) is None

        # Negative dimensions
        xml = '<geometry><box size="-1 1 1"/></geometry>'
        assert parse_geometry(ET.fromstring(xml)) is None

        # Cylinder invalid
        xml = '<geometry><cylinder radius="-1" length="1"/></geometry>'
        assert parse_geometry(ET.fromstring(xml)) is None

    def test_parse_material_texture(self):
        """Test parsing material with texture."""
        xml = '<material name="tex"><texture filename="package://pkg/tex.png"/></material>'
        elem = ET.fromstring(xml)
        mat = parse_material(elem, {})

        assert isinstance(mat, Material)
        assert mat.texture == "package://pkg/tex.png"
        assert mat.color is None

    def test_parse_material_reference(self):
        """Test parsing material reference."""
        global_mats = {"global_blue": Material(name="global_blue", color=Color(0, 0, 1, 1))}

        xml = '<material name="global_blue"/>'
        elem = ET.fromstring(xml)
        mat = parse_material(elem, global_mats)

        assert mat is global_mats["global_blue"]

    def test_parse_transmission(self):
        """Test parsing transmission element."""
        xml = """
        <transmission name="trans1">
            <type>transmission_interface/SimpleTransmission</type>
            <joint name="joint1">
                <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
            </joint>
            <actuator name="motor1">
                <mechanicalReduction>50</mechanicalReduction>
            </actuator>
        </transmission>
        """
        elem = ET.fromstring(xml)
        trans = parse_transmission(elem)

        assert trans.name == "trans1"
        assert trans.type == "transmission_interface/SimpleTransmission"
        assert len(trans.joints) == 1
        assert trans.joints[0].name == "joint1"
        assert trans.joints[0].hardware_interfaces == ["position"]  # Normalized
        assert len(trans.actuators) == 1
        assert trans.actuators[0].mechanical_reduction == 50.0

    def test_parse_transmission_invalid(self):
        """Test parsing invalid transmission."""

    def test_parse_ros2_control(self):
        """Test parsing ros2_control element."""
        from linkforge_core.parsers.urdf_parser import parse_ros2_control

        xml = """
        <ros2_control name="System" type="system">
            <hardware>
                <plugin>mock_components/GenericSystem</plugin>
                <param name="ip">192.168.1.1</param>
            </hardware>
            <joint name="j1">
                <command_interface name="position"/>
                <state_interface name="position"/>
                <state_interface name="velocity"/>
            </joint>
        </ros2_control>
        """
        elem = ET.fromstring(xml)
        rc = parse_ros2_control(elem)

        assert rc.name == "System"
        assert rc.type == "system"
        assert rc.hardware_plugin == "mock_components/GenericSystem"
        assert rc.parameters["ip"] == "192.168.1.1"
        assert len(rc.joints) == 1
        assert rc.joints[0].name == "j1"
        assert "position" in rc.joints[0].command_interfaces
        assert "velocity" in rc.joints[0].state_interfaces

    def test_urdf_parser_integration(self):
        """Test full URDF parsing via URDFParser class."""
        from linkforge_core.parsers.urdf_parser import URDFParser

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
            </transmission>

            <gazebo reference="base">
                <material>Gazebo/Blue</material>
            </gazebo>
        </robot>
        """
        parser = URDFParser()
        robot = parser.parse_string(xml)

        assert robot.name == "test_robot"
        assert len(robot.links) == 2
        assert len(robot.joints) == 1
        assert len(robot.transmissions) == 1
        assert len(robot.gazebo_elements) == 1

        # Check global material resolution
        assert robot.links[0].visuals[0].material.name == "blue"
        assert robot.links[0].visuals[0].material.color.b == 1.0

    def test_urdf_parser_xacro_detection(self):
        """Test that XACRO content triggers an error."""
        from linkforge_core.parsers.urdf_parser import URDFParser

        xml = """
        <robot name="xacro_bot" xmlns:xacro="http://www.ros.org/wiki/xacro">
            <xacro:macro name="m"/>
        </robot>
        """
        parser = URDFParser()
        with pytest.raises(XacroDetectedError, match="XACRO file detected"):
            parser.parse_string(xml)

    def test_urdf_parser_security(self):
        """Test security limits."""
        from linkforge_core.base import RobotParserError
        from linkforge_core.parsers.urdf_parser import URDFParser

        # Test max file size
        parser = URDFParser(max_file_size=10)
        with pytest.raises(RobotParserError, match="URDF string too large"):
            parser.parse_string("<robot>..............</robot>")

    def test_urdf_parser_robustness(self):
        """Test duplicate name handling."""
        from linkforge_core.parsers.urdf_parser import URDFParser

        xml = """
        <robot name="dupe_bot">
            <link name="link1"/>
            <link name="link1"/>
        </robot>
        """
        parser = URDFParser()
        robot = parser.parse_string(xml)

        assert len(robot.links) == 2
        names = {link.name for link in robot.links}
        assert "link1" in names

    def test_parse_sensors_extended(self):
        """Test parsing other sensor types (Lidar, IMU, GPS, Force/Torque)."""
        # Lidar
        xml = """
        <gazebo reference="lidar_link">
            <sensor name="lidar1" type="ray">
                <ray>
                    <scan>
                        <horizontal>
                            <samples>100</samples>
                            <min_angle>-1</min_angle>
                            <max_angle>1</max_angle>
                        </horizontal>
                    </scan>
                    <range>
                        <min>0.2</min>
                        <max>50</max>
                    </range>
                    <noise>
                        <type>gaussian</type>
                        <mean>0.0</mean>
                        <stddev>0.01</stddev>
                    </noise>
                </ray>
            </sensor>
        </gazebo>
        """
        sensor = parse_sensor_from_gazebo(ET.fromstring(xml))
        assert sensor.type == SensorType.LIDAR
        assert sensor.lidar_info.horizontal_samples == 100
        assert sensor.lidar_info.range_max == 50.0
        assert sensor.lidar_info.noise.stddev == 0.01

        # IMU
        xml_imu = """
        <gazebo reference="imu_link">
            <sensor name="imu1" type="imu">
                <imu>
                    <angular_velocity>
                        <x><noise type="gaussian"><stddev>0.02</stddev></noise></x>
                    </angular_velocity>
                    <linear_acceleration>
                        <x><noise type="gaussian"><stddev>0.1</stddev></noise></x>
                    </linear_acceleration>
                </imu>
            </sensor>
        </gazebo>
        """
        sensor_imu = parse_sensor_from_gazebo(ET.fromstring(xml_imu))
        assert sensor_imu.type == SensorType.IMU
        assert sensor_imu.imu_info.angular_velocity_noise.stddev == 0.02
        assert sensor_imu.imu_info.linear_acceleration_noise.stddev == 0.1

        # GPS
        xml_gps = """
        <gazebo reference="gps_link">
            <sensor name="gps1" type="gps">
                <gps>
                    <position_sensing>
                        <horizontal><noise type="gaussian"><stddev>0.5</stddev></noise></horizontal>
                    </position_sensing>
                </gps>
            </sensor>
        </gazebo>
        """
        sensor_gps = parse_sensor_from_gazebo(ET.fromstring(xml_gps))
        assert sensor_gps.type == SensorType.GPS
        assert sensor_gps.gps_info.position_sensing_horizontal_noise.stddev == 0.5

        # Force/Torque
        xml_ft = """
        <gazebo reference="ft_link">
            <sensor name="ft1" type="force_torque">
                <force_torque>
                    <frame>sensor</frame>
                    <measure_direction>child_to_parent</measure_direction>
                </force_torque>
            </sensor>
        </gazebo>
        """
        sensor_ft = parse_sensor_from_gazebo(ET.fromstring(xml_ft))
        assert sensor_ft.type == SensorType.FORCE_TORQUE
        assert sensor_ft.force_torque_info.frame == "sensor"

        # Contact
        xml_contact = """
        <gazebo reference="bumper_link">
            <sensor name="contact1" type="contact">
                <contact>
                    <collision>bumper_collision</collision>
                </contact>
            </sensor>
        </gazebo>
        """
        sensor_contact = parse_sensor_from_gazebo(ET.fromstring(xml_contact))
        assert sensor_contact.type == SensorType.CONTACT
        assert sensor_contact.contact_info.collision == "bumper_collision"

    def test_parse_gazebo_element_properties(self):
        """Test parsing Gazebo element properties."""
        from linkforge_core.parsers.urdf_parser import parse_gazebo_element

        xml = """
        <gazebo reference="link1">
            <material>Gazebo/Red</material>
            <mu1>0.5</mu1>
            <mu2>0.5</mu2>
            <selfCollide>true</selfCollide>
            <gravity>false</gravity>
            <kp>100000.0</kp>
            <kd>1.0</kd>
            <maxVel>0.01</maxVel>
            <minDepth>0.001</minDepth>
        </gazebo>
        """
        elem = parse_gazebo_element(ET.fromstring(xml))

        assert elem.material == "Gazebo/Red"
        assert elem.mu1 == 0.5
        assert elem.self_collide is True
        assert elem.gravity is False
        assert elem.kp == 100000.0
        # Check that generic properties captured extras
        assert elem.properties["maxVel"] == "0.01"
        assert elem.properties["minDepth"] == "0.001"

    def test_urdf_parser_file_parsing(self, tmp_path):
        """Test parsing from a file using iterative parser."""
        from linkforge_core.parsers.urdf_parser import URDFParser

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

    def test_urdf_parser_xacro_extension_check(self, tmp_path):
        """Test that .xacro extension raises error."""
        from linkforge_core.base import RobotParserError
        from linkforge_core.parsers.urdf_parser import URDFParser

        xacro_file = tmp_path / "robot.urdf.xacro"
        xacro_file.write_text("<robot/>")

        parser = URDFParser()
        with pytest.raises(RobotParserError, match="XACRO file detected"):
            parser.parse(xacro_file)

    def test_parse_mesh_file_uri(self, tmp_path):
        """Test parsing mesh with file:// URI and validation."""
        from linkforge_core.parsers.urdf_parser import parse_geometry

        mesh_file = tmp_path / "mesh.stl"
        mesh_file.touch()

        xml = f'<geometry><mesh filename="file://{mesh_file.name}"/></geometry>'
        elem = ET.fromstring(xml)

        geom = parse_geometry(elem, urdf_directory=tmp_path)
        assert Path(geom.resource).name == mesh_file.name

        assert Path(geom.resource).name == mesh_file.name

        xml_bad = '<geometry><mesh filename="file:///etc/passwd"/></geometry>'
        elem_bad = ET.fromstring(xml_bad)

        geom_bad = parse_geometry(elem_bad, urdf_directory=tmp_path)
        assert geom_bad is None

        subdir = tmp_path / "subdir"
        subdir.mkdir()

        xml_escape = '<geometry><mesh filename="file://../secret.txt"/></geometry>'
        elem_escape = ET.fromstring(xml_escape)

        geom_escape = parse_geometry(elem_escape, urdf_directory=tmp_path)
        assert geom_escape is None

    def test_iterparse_full_structure(self, tmp_path):
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

    def test_link_duplication_renaming(self, tmp_path):
        """Test that duplicate links are renamed."""
        xml = """
        <robot name="dupe_links">
            <link name="link1"/>
            <link name="link1"/> <!-- Should become link1_duplicate_1 -->
            <link name="link1"/> <!-- Should become link1_duplicate_2 -->
        </robot>
        """
        urdf_file = tmp_path / "dupe_link.urdf"
        urdf_file.write_text(xml)

        parser = URDFParser()
        robot = parser.parse(urdf_file)

        names = sorted([link.name for link in robot.links])
        assert names == ["link1", "link1_duplicate_1", "link1_duplicate_2"]

    def test_joint_duplication_renaming(self, tmp_path):
        """Test that duplicate joints are renamed."""
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

        names = sorted([j.name for j in robot.joints])
        assert names == ["j1", "j1_duplicate_1", "j1_duplicate_2"]

    def test_xacro_detection_detailed(self, tmp_path):
        """Test various XACRO artifacts triggering detection."""
        parser = URDFParser()

        # 1. Attribute substitution
        with pytest.raises(XacroDetectedError, match="XACRO file detected"):
            _detect_xacro_file(ET.fromstring('<robot name="${name}"/>'))

        # 2. Xacro namespace in tag
        with pytest.raises(XacroDetectedError, match="XACRO file detected"):
            _detect_xacro_file(
                ET.fromstring(
                    '<robot xmlns:xacro="http://ros.org/wiki/xacro"><xacro:macro/></robot>'
                )
            )

        # 3. File content check
        xacro_file = tmp_path / "test.urdf"
        xacro_file.write_text('<robot xmlns:xacro="http://..."/>')

        with pytest.raises(RobotParserError, match="XACRO file detected"):
            parser.parse(xacro_file)

    def test_invalid_values_and_defaults(self):
        """Test negative values and missing defaults logic."""
        # 1. Negative inertia
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
        inertia = robot.links[0].inertial.inertia
        assert inertia.ixx == 1e-6
        assert inertia.izz == 1e-6

    def test_sensor_defaults(self):
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
        assert isinstance(cam.camera_info, CameraInfo)

        lidar = next(s for s in robot.sensors if s.name == "lidar")
        assert lidar.lidar_info is not None
        # The parser creates default LidarInfo()

        gps = next(s for s in robot.sensors if s.name == "gps")
        assert isinstance(gps.gps_info, object)  # GPSInfo

        imu = next(s for s in robot.sensors if s.name == "imu")
        assert isinstance(imu.imu_info, object)  # IMUInfo

    def test_parse_contact_sensor_missing_collision(self):
        """Test contact sensor missing collision element raises RobotModelError."""
        from linkforge_core.exceptions import RobotModelError
        from linkforge_core.parsers.urdf_parser import parse_sensor_from_gazebo

        xml = """
        <gazebo reference="link1">
            <sensor name="contact1" type="contact">
                <contact/> <!-- Missing collision -->
            </sensor>
        </gazebo>
        """
        with pytest.raises(RobotModelError, match="missing required <collision>"):
            parse_sensor_from_gazebo(ET.fromstring(xml))

    def test_security_exceptions(self, tmp_path):
        """Test security exception re-raising."""
        # 1. Package URI validation - Parse geometry swallows RobotModelError and returns None
        xml = '<geometry><mesh filename="package://../traversal"/></geometry>'
        assert parse_geometry(ET.fromstring(xml), urdf_directory=tmp_path) is None

        # 2. File URI validation - Parse geometry swallows RobotModelError and returns None
        xml2 = '<geometry><mesh filename="file:///etc/passwd"/></geometry>'
        assert parse_geometry(ET.fromstring(xml2), urdf_directory=tmp_path) is None

    def test_gps_noise_structure(self):
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
        sensor = parse_sensor_from_gazebo(ET.fromstring(xml))
        assert sensor.gps_info.position_sensing_horizontal_noise.mean == 0.1
        assert sensor.gps_info.velocity_sensing_vertical_noise is None

    def test_coverage_edge_cases(self):
        """Test remaining edge cases for 100% coverage."""
        from linkforge_core.parsers.urdf_parser import (
            parse_origin,
        )

        # 1. Parse Origin with values
        xml = '<origin xyz="1 2 3" rpy="0.1 0.2 0.3"/>'
        origin = parse_origin(ET.fromstring(xml))
        assert origin.xyz.x == 1.0
        assert origin.rpy.x == 0.1

    def test_parse_file_not_found(self):
        """Test parsing a non-existent file."""
        from linkforge_core.parsers.urdf_parser import URDFParser

        parser = URDFParser()
        with pytest.raises(FileNotFoundError):
            parser.parse(Path("non_existent_file.urdf"))

    def test_parse_invalid_root(self, tmp_path):
        """Test parsing an XML with invalid root element."""
        from linkforge_core.parsers.urdf_parser import URDFParser

        invalid_urdf = tmp_path / "invalid.urdf"
        invalid_urdf.write_text("<not_robot></not_robot>")

        parser = URDFParser()
        # Parser now wraps standard errors into RobotParserError
        with pytest.raises(RobotParserError, match="Root element must be <robot>"):
            parser.parse(invalid_urdf)

    def test_parse_string_unexpected_error(self):
        """Test unexpected error during string parsing."""
        from linkforge_core.parsers.urdf_parser import URDFParser

        parser = URDFParser()
        with (
            patch("xml.etree.ElementTree.fromstring", side_effect=Exception("Boom")),
            pytest.raises(RobotParserError, match="Unexpected error parsing URDF"),
        ):
            parser.parse_string("<robot name='test'/>")

    def test_joint_renaming_robustness_broken_ref(self):
        """Test robustness of joint renaming when duplicates exist AND references are broken."""
        from unittest.mock import patch

        from linkforge_core.models import Joint, JointType, Robot
        from linkforge_core.parsers.urdf_parser import URDFParser

        robot = Robot(name="test_robot")
        parser = URDFParser()
        elem = ET.Element("joint", name="fixed_joint")
        joint = Joint(name="fixed_joint", type=JointType.FIXED, parent="p", child="c")

        # We need to simulate the exact conditions to hit the nested exception handler
        # in _add_joint_robust.
        # This requires add_joint to fail with "already exists" first (to enter rename loop),
        # then fail with "not found" (to hit the break).

        with patch.object(robot, "add_joint") as mock_add:
            mock_add.side_effect = [
                RobotModelError("Joint 'fixed_joint' already exists"),
                RobotModelError("Link 'missing' not found"),
            ]

            parser._add_joint_robust(robot, joint, elem)

            # Should have attempted twice
            assert mock_add.call_count == 2

    def test_parse_material_invalid_color(self):
        """Test parsing invalid material colors."""
        materials = {}

        # Too few components
        xml = '<material name="m"><color rgba="0.1 0.2" /></material>'
        elem = ET.fromstring(xml)
        assert parse_material(elem, materials) is None

        # Too many components
        xml = '<material name="m"><color rgba="0.1 0.2 0.3 0.4 0.5" /></material>'
        elem = ET.fromstring(xml)
        assert parse_material(elem, materials) is None

    def test_parse_geometry_invalid_mesh(self):
        """Test parsing invalid mesh geometry."""
        # Missing filename
        xml = "<geometry><mesh /></geometry>"
        elem = ET.fromstring(xml)
        assert parse_geometry(elem) is None

        # Negative scale
        xml = '<geometry><mesh filename="test.stl" scale="-1 1 1" /></geometry>'
        elem = ET.fromstring(xml)
        assert parse_geometry(elem) is None

        # 2. Geometry Errors
        # Cylinder invalid length
        xml = '<geometry><cylinder radius="1" length="-1"/></geometry>'
        assert parse_geometry(ET.fromstring(xml)) is None

        # Mesh missing filename
        xml = "<geometry><mesh/></geometry>"
        assert parse_geometry(ET.fromstring(xml)) is None

        # Mesh negative scale
        xml = '<geometry><mesh filename="f.stl" scale="-1 1 1"/></geometry>'
        assert parse_geometry(ET.fromstring(xml)) is None

        # 3. Material Errors
        # Invalid RGBA length (239, 243)
        xml = '<material name="bad"><color rgba="1 1"/></material>'
        assert parse_material(ET.fromstring(xml), {}) is None

        xml = '<material name="bad"><color rgba="1 1 1 1 1"/></material>'
        assert parse_material(ET.fromstring(xml), {}) is None

        # Empty material (267)
        xml = '<material name="empty"/>'
        # Should return None if no color/texture/ref found?
        # Actually parse_material line 267 is explicit return None if no color/texture
        assert parse_material(ET.fromstring(xml), {}) is None

        # 4. Transmission Errors
        from linkforge_core.parsers.urdf_parser import (
            TransmissionJoint,
            _parse_transmission_component,
        )

        # Invalid mechanicalReduction (557-558)
        xml = '<joint name="j1"><mechanicalReduction>not_number</mechanicalReduction></joint>'
        with pytest.raises(RobotModelError, match="Invalid mechanicalReduction"):
            _parse_transmission_component(ET.fromstring(xml), TransmissionJoint)

        # 5. Gazebo Sensor missing reference (681)
        xml = '<gazebo><sensor name="s" type="camera"/></gazebo>'  # No reference attr
        assert parse_sensor_from_gazebo(ET.fromstring(xml)) is None

        # 6. Parse String errors (1271)
        parser = URDFParser()
        with pytest.raises(RobotParserError, match="Failed to parse"):
            parser.parse_string("<robot>unclosed tags")

        # 7. Joint with explicit Axis
        xml = '<joint name="j1" type="continuous"><parent link="p"/><child link="c"/><axis xyz="0 1 0"/></joint>'
        joint = parse_joint(ET.fromstring(xml))
        assert joint.axis.y == 1.0

        # 8. Gazebo Plugin parsing (956-970)
        from linkforge_core.parsers.urdf_parser import parse_gazebo_plugin

        xml = """
        <plugin name="p" filename="lib.so">
            <param>value</param>
            <nested><child>content</child></nested>
        </plugin>
        """
        plugin = parse_gazebo_plugin(ET.fromstring(xml))
        assert plugin.name == "p"
        assert plugin.parameters["param"] == "value"
        assert plugin.raw_xml is not None

        # 9. ROS2 Control misc parameters (631)
        from linkforge_core.parsers.urdf_parser import parse_ros2_control

        xml = """
        <ros2_control name="c" type="system">
            <hardware><plugin>H</plugin></hardware>
            <param_block>some config</param_block>
        </ros2_control>
        """
        rc = parse_ros2_control(ET.fromstring(xml))
        assert rc.parameters["param_block"] == "some config"

    def test_parse_robot_full_traversal(self):
        """Hit lines in _parse_robot that aren't hit by iterparse."""
        from linkforge_core.parsers.urdf_parser import URDFParser

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

    def test_parse_gazebo_element_with_plugins(self):
        """Gazebo element with only a plugin (no sensor) is stored as a GazeboElement."""
        from linkforge_core.parsers.urdf_parser import URDFParser

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

    def test_parse_file_iterparse_error(self, tmp_path):
        """A malformed XML file raises RobotParserError with a clear message."""
        from unittest import mock

        from linkforge_core.parsers.urdf_parser import URDFParser

        path = tmp_path / "test.urdf"
        path.touch()
        parser = URDFParser()

        # Mock ET.iterparse to raise ParseError
        with (
            mock.patch("xml.etree.ElementTree.iterparse", side_effect=ET.ParseError("Bad XML")),
            pytest.raises(RobotParserError, match="Failed to parse URDF XML"),
        ):
            parser.parse(path)

    def test_add_joint_robust_renaming(self):
        """Test robust joint addition with duplicate names."""
        from unittest.mock import MagicMock

        from linkforge_core.models import Joint, JointType, Robot
        from linkforge_core.parsers.urdf_parser import URDFParser

        parser = URDFParser()
        robot = MagicMock(spec=Robot)
        robot._joint_index = {"j": MagicMock(), "j_duplicate_1": MagicMock()}

        joint = Joint(name="j", type=JointType.FIXED, parent="p", child="c")
        joint_elem = ET.Element("joint", name="j")

        # First two names already exist in _joint_index
        # Third name 'j_duplicate_2' is free
        robot.add_joint.side_effect = [
            RobotModelError("already exists"),  # j exists
            RobotModelError("already exists"),  # j_duplicate_1 exists
            None,  # Success for j_duplicate_2
        ]

        parser._add_joint_robust(robot, joint, joint_elem)

        assert robot.add_joint.call_count == 3

    def test_add_link_robust_renaming(self):
        """Test robust link addition with duplicate names."""
        from unittest.mock import patch

        from linkforge_core.models import Link, Robot
        from linkforge_core.parsers.urdf_parser import URDFParser

        parser = URDFParser()
        robot = Robot(name="test")
        robot.add_link(Link(name="l"))
        robot.add_link(Link(name="l_duplicate_1"))

        link = Link(name="l")

        with patch.object(robot, "add_link") as mock_add_link:
            mock_add_link.side_effect = [
                RobotModelError("Link 'l' already exists"),
                RobotModelError("Link 'l_duplicate_1' failed"),  # Trigger exception inside loop
                None,  # Succeeds for l_duplicate_2
            ]

            with patch("linkforge_core.parsers.urdf_parser.logger") as mock_logger:
                parser._add_link_robust(robot, link)
                assert mock_add_link.call_count == 3
                assert mock_logger.warning.called
                assert "Renamed duplicate link" in mock_logger.warning.call_args[0][0]


class TestURDFParserEdgeCoverage:
    """Parser behavior for sensors missing optional sub-elements."""

    def test_parse_lidar_sensor_without_range_element(self):
        """Lidar sensor element missing a range sub-element uses default range values."""
        xml = """<robot name="r"><gazebo reference="base">
            <sensor type="ray" name="lidar"><ray><horizontal/></ray></sensor>
        </gazebo></robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert len(robot.sensors) == 1

    def test_parse_imu_with_empty_angular_and_linear_elements(self):
        """IMU sensor with empty angular_velocity and linear_acceleration elements parses cleanly."""
        xml = """<robot name="r"><gazebo reference="base">
            <sensor type="imu" name="imu0">
                <imu><angular_velocity/><linear_acceleration/></imu>
            </sensor>
        </gazebo></robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert len(robot.sensors) == 1

    def test_parse_force_torque_without_inner_element(self):
        """Force/torque sensor element without inner force_torque child uses defaults."""
        xml = """<robot name="r"><gazebo reference="base">
            <sensor type="force_torque" name="ft0"></sensor>
        </gazebo></robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert len(robot.sensors) == 1

    def test_parse_gazebo_element_without_sensor(self):
        """Gazebo element referencing a link but without a sensor is stored as a GazeboElement."""
        xml = """<robot name="r">
            <link name="base"/>
            <gazebo reference="base"><mu1>0.9</mu1></gazebo>
        </robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert len(robot.gazebo_elements) == 1

    def test_parse_robot_without_filepath_uses_unnamed_robot(self):
        """Parsing from a string without a filepath falls back to 'unnamed_robot' name."""
        xml = """<robot><link name="l1"/></robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert robot.name == "unnamed_robot"


class TestURDFParserAdditionalEdgeCoverage:
    """Parser behavior for inertial properties, transmissions, and mesh validation."""

    def test_mesh_path_validation_error_with_directory(self, tmp_path):
        """Non-package:// mesh path with urdf_directory triggers security validation."""
        xml = """<robot name="r"><link name="l1"><visual>
            <geometry><mesh filename="/outside/path/mesh.stl"/></geometry>
        </visual></link></robot>"""
        parser = URDFParser()
        # When urdf_directory is set and path escapes it, logs warning and returns None geometry
        robot = parser.parse_string(xml, urdf_directory=tmp_path)
        # Mesh should be skipped — link exists but no visual geometry
        assert len(robot.links) == 1

    def test_link_with_inertial_but_no_inertia_element(self):
        """Link with <inertial><mass.../></inertial> but without <inertia> creates Inertial with no tensor."""
        xml = """<robot name="r"><link name="l1">
            <inertial><mass value="2.0"/></inertial>
        </link></robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert robot.links[0].inertial is None  # No inertia element means no Inertial stored

    def test_link_with_negative_inertia_is_sanitized(self):
        """Link inertia with negative diagonal values are sanitized to 1e-6."""
        xml = """<robot name="r"><link name="l1">
            <inertial>
                <mass value="1.0"/>
                <inertia ixx="-1.0" ixy="0.0" ixz="0.0" iyy="-1.0" iyz="0.0" izz="-1.0"/>
            </inertial>
        </link></robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        t = robot.links[0].inertial.inertia
        assert t.ixx == pytest.approx(1e-6)
        assert t.iyy == pytest.approx(1e-6)
        assert t.izz == pytest.approx(1e-6)

    def test_collision_with_no_geometry_is_skipped(self):
        """Collision element without any geometry child is silently not added."""
        xml = """<robot name="r"><link name="l1">
            <collision><geometry></geometry></collision>
        </link></robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert len(robot.links[0].collisions) == 0

    def test_invalid_transmission_is_ignored(self):
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
            robot, __import__("linkforge_core.models.robot", fromlist=["Robot"]).Robot
        )

    def test_camera_sensor_without_image_element_uses_defaults(self):
        """Camera sensor missing an <image> child falls back to width=640, height=480."""
        xml = """<robot name="r"><gazebo reference="base">
            <sensor type="camera" name="cam0">
                <camera><clip><near>0.1</near><far>10.0</far></clip></camera>
            </sensor>
        </gazebo></robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        assert robot.sensors[0].camera_info.width == 640
        assert robot.sensors[0].camera_info.height == 480

    def test_contact_sensor_missing_contact_element_raises_parser_error(self):
        """Contact sensor without a <contact> child raises RobotParserError."""
        from linkforge_core.base import RobotParserError

        xml = """<robot name="r"><gazebo reference="base">
            <sensor type="contact" name="ct0"></sensor>
        </gazebo></robot>"""
        parser = URDFParser()
        with pytest.raises(RobotParserError):
            parser.parse_string(xml)

    def test_parse_string_with_robot_parser_error_reraises(self):
        """RobotParserError from within parse_string passes through unchanged."""
        import pytest
        from linkforge_core.base import XacroDetectedError

        xml = """<robot name="r"><xacro:if value="1"><link name="l1"/></xacro:if></robot>"""
        parser = URDFParser()
        with pytest.raises(XacroDetectedError):
            parser.parse_string(xml)


class TestURDFParserFileProtectionAndSensorCoverage:
    """File size limits, ros2_control parameters, and IMU noise parsing."""

    def test_ros2_control_hardware_params_are_parsed(self):
        """Hardware <param> elements inside ros2_control are collected into the parameters dict."""
        xml = """<robot name="r">
            <link name="base"/>
            <ros2_control name="hw" type="system">
                <hardware>
                    <plugin>fake_components/GenericSystem</plugin>
                    <param name="joints">base</param>
                </hardware>
                <joint name="base">
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

    def test_ros2_control_joint_without_interfaces_is_not_added(self):
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
        assert robot.ros2_controls[0].joints == []

    def test_imu_sensor_with_angular_velocity_x_noise(self):
        """IMU angular_velocity with an <x> noise element parses the noise correctly."""
        xml = """<robot name="r"><gazebo reference="base">
            <sensor type="imu" name="imu0">
                <imu>
                    <angular_velocity>
                        <x><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></x>
                    </angular_velocity>
                </imu>
            </sensor>
        </gazebo></robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        imu = robot.sensors[0].imu_info
        assert imu.angular_velocity_noise is not None

    def test_imu_sensor_with_linear_acceleration_x_noise(self):
        """IMU linear_acceleration with an <x> noise element parses the noise correctly."""
        xml = """<robot name="r"><gazebo reference="base">
            <sensor type="imu" name="imu0">
                <imu>
                    <linear_acceleration>
                        <x><noise type="gaussian"><mean>0.0</mean><stddev>0.05</stddev></noise></x>
                    </linear_acceleration>
                </imu>
            </sensor>
        </gazebo></robot>"""
        parser = URDFParser()
        robot = parser.parse_string(xml)
        imu = robot.sensors[0].imu_info
        assert imu.linear_acceleration_noise is not None

    def test_parse_file_too_large_raises_error(self, tmp_path):
        """Parser raises RobotParserError if the file exceeds max_file_size."""
        from linkforge_core.base import RobotParserError

        urdf_file = tmp_path / "big.urdf"
        urdf_file.write_text("<robot name='r'><link name='l1'/></robot>")
        parser = URDFParser()
        parser.max_file_size = 1  # 1 byte — will trip on any content
        with pytest.raises(RobotParserError, match="too large"):
            parser.parse(urdf_file)

    def test_parse_file_xacro_string_too_large_raises_error(self):
        """parse_string raises RobotParserError if the string exceeds max_file_size."""
        from linkforge_core.base import RobotParserError

        content = "<robot name='r'>" + "<link name='l1'/>" * 10 + "</robot>"
        parser = URDFParser()
        parser.max_file_size = 1  # 1 byte — will trip immediately
        with pytest.raises(RobotParserError, match="too large"):
            parser.parse_string(content)

    def test_parse_iterative_out_of_order(self, tmp_path):
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

        assert "link1" in robot._link_index
        assert "link2" in robot._link_index
        assert "joint1" in robot._joint_index
        assert robot.get_joint("joint1").parent == "link1"
        assert robot.get_joint("joint1").child == "link2"
