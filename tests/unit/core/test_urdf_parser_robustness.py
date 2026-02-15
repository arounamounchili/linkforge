"""Tests for URDFParser robustness, error handling, and security."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from linkforge_core.base import RobotParserError, XacroDetectedError
from linkforge_core.models import SensorType
from linkforge_core.parsers.urdf_parser import (
    URDFParser,
    parse_geometry,
    parse_material,
    parse_sensor_from_gazebo,
    parse_transmission,
)
from linkforge_core.validation import validate_mesh_path
from linkforge_core.validation.security import validate_package_uri


def test_geometry_invalid_inputs():
    """Test geometry parsing with various invalid inputs."""
    # Box validation
    assert parse_geometry(ET.fromstring("<geometry><box/></geometry>")) is None
    assert parse_geometry(ET.fromstring('<geometry><box size="0 1 1"/></geometry>')) is None

    # Cylinder validation
    assert (
        parse_geometry(ET.fromstring('<geometry><cylinder radius="0" length="1"/></geometry>'))
        is None
    )
    assert (
        parse_geometry(ET.fromstring('<geometry><cylinder radius="1" length="0"/></geometry>'))
        is None
    )

    # Sphere validation
    assert parse_geometry(ET.fromstring('<geometry><sphere radius="0"/></geometry>')) is None

    # Mesh validation
    assert parse_geometry(ET.fromstring("<geometry><mesh/></geometry>")) is None
    assert (
        parse_geometry(
            ET.fromstring('<geometry><mesh filename="cube.stl" scale="0 1 1"/></geometry>')
        )
        is None
    )

    # Unknown geometry type
    assert parse_geometry(ET.fromstring("<geometry><unknown/></geometry>")) is None


def test_joint_axis_default_coverage():
    """Test default axis assignment for different joint types."""
    from linkforge_core.models import Vector3
    from linkforge_core.parsers.urdf_parser import parse_joint

    # Revolute defaults to 1 0 0
    j_rev = parse_joint(
        ET.fromstring(
            '<joint name="j" type="revolute"><parent link="a"/><child link="b"/><limit effort="1" velocity="1" lower="-1" upper="1"/></joint>'
        )
    )
    assert j_rev.axis == Vector3(1, 0, 0)

    # Fixed has no axis
    j_fixed = parse_joint(
        ET.fromstring('<joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint>')
    )
    assert j_fixed.axis is None


def test_inertia_sanitization_coverage():
    """Test sanitization of zero/negative inertia moments."""
    from linkforge_core.parsers.urdf_parser import parse_link

    # We use values that will be sanitized to 1e-6 but won't trigger
    # the physical property validation error (triangle inequality: Ixx + Iyy >= Izz etc)
    # 1e-6 + 1e-6 >= 1e-6 (holds)
    # Mass must be positive too.
    xml = """
    <link name="l">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/>
        </inertial>
    </link>
    """
    link = parse_link(ET.fromstring(xml), {})
    assert link.inertial.inertia.ixx == 1e-6
    assert link.inertial.inertia.iyy == 1e-6
    assert link.inertial.inertia.izz == 1e-6


def test_geometry_security_hardening(tmp_path):
    """Test security constraints in geometry parsing."""
    urdf_dir = tmp_path / "urdf"
    urdf_dir.mkdir()

    # Path traversal
    assert (
        parse_geometry(
            ET.fromstring('<geometry><mesh filename="../evil.stl"/></geometry>'), urdf_dir
        )
        is None
    )
    assert (
        parse_geometry(
            ET.fromstring('<geometry><mesh filename="file:///etc/passwd"/></geometry>'), urdf_dir
        )
        is None
    )

    # Package URI traversal
    assert (
        parse_geometry(
            ET.fromstring('<geometry><mesh filename="package://pkg/../../etc/passwd"/></geometry>'),
            urdf_dir,
        )
        is None
    )


def test_material_parsing_robustness():
    """Test material color and texture robustness."""
    mats = {}

    # Invalid RGBA text formats
    assert parse_material(ET.fromstring('<material><color rgba="1 1 "/></material>'), mats) is None
    assert (
        parse_material(ET.fromstring('<material><color rgba="not a color"/></material>'), mats)
        is None
    )
    assert (
        parse_material(ET.fromstring('<material><color rgba="1 1 1 1 1"/></material>'), mats)
        is None
    )

    # Color out of bounds (Color validates 0.0-1.0)
    assert (
        parse_material(ET.fromstring('<material><color rgba="2.0 0 0 1"/></material>'), mats)
        is None
    )

    # Texture path support
    mat = parse_material(
        ET.fromstring('<material name="m"><texture filename="texture.jpg"/></material>'), mats
    )
    assert mat.texture == "texture.jpg"


def test_transmission_parsing_comprehensive():
    """Test transmission parsing with optional mechanical reduction and offsets."""
    urdf = """
    <transmission name="t1">
        <type>Simple</type>
        <joint name="j1">
            <mechanicalReduction>100.0</mechanicalReduction>
            <offset>3.14</offset>
        </joint>
        <actuator name="a1">
            <mechanicalReduction>50.0</mechanicalReduction>
        </actuator>
    </transmission>
    """
    trans = parse_transmission(ET.fromstring(urdf))
    assert trans.joints[0].mechanical_reduction == 100.0
    assert trans.joints[0].offset == 3.14
    assert trans.actuators[0].mechanical_reduction == 50.0
    assert "position" in trans.joints[0].hardware_interfaces


def test_gazebo_sensor_parsing_robustness():
    """Test Gazebo sensor parsing edge cases and variety."""
    # Missing reference link
    assert (
        parse_sensor_from_gazebo(ET.fromstring('<gazebo><sensor name="s" type="camera"/></gazebo>'))
        is None
    )

    # GPS variety (navsat)
    gps_xml = """
    <gazebo reference="gps_link">
        <sensor name="my_gps" type="navsat"/>
    </gazebo>
    """
    sensor = parse_sensor_from_gazebo(ET.fromstring(gps_xml))
    assert sensor.type == SensorType.GPS

    # Contact sensor requirement
    contact_xml = """
    <gazebo reference="foot">
        <sensor name="foot_contact" type="contact"/>
    </gazebo>
    """
    with pytest.raises(ValueError, match="missing required <contact> element"):
        parse_sensor_from_gazebo(ET.fromstring(contact_xml))


def test_sensor_parsing_coverage():
    """Test various sensor types and missing attributes for coverage."""
    # IMU with noise
    imu_xml = """
    <gazebo reference="imu_link">
        <sensor name="my_imu" type="imu">
            <imu>
                <angular_velocity><x><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></x></angular_velocity>
            </imu>
        </sensor>
    </gazebo>
    """
    sensor = parse_sensor_from_gazebo(ET.fromstring(imu_xml))
    assert sensor.type == SensorType.IMU

    # Ray/Lidar variety
    lidar_xml = '<gazebo reference="l"><sensor name="lidar" type="ray"/></gazebo>'
    assert parse_sensor_from_gazebo(ET.fromstring(lidar_xml)).type == SensorType.LIDAR

    # Missing reference again (line 684)
    assert (
        parse_sensor_from_gazebo(ET.fromstring('<gazebo><sensor name="s" type="camera"/></gazebo>'))
        is None
    )


def test_gazebo_element_parsing_coverage():
    """Test Gazebo element parsing edge cases."""
    from linkforge_core.parsers.urdf_parser import parse_gazebo_element

    # Simple element (line 651)
    xml = '<gazebo reference="base_link"><mu1>0.2</mu1></gazebo>'
    elem = parse_gazebo_element(ET.fromstring(xml))
    assert elem.reference == "base_link"

    # Missing reference is fine for robot-level (line 651)
    xml_robot = "<gazebo><static>true</static></gazebo>"
    elem_robot = parse_gazebo_element(ET.fromstring(xml_robot))
    assert elem_robot.reference is None


def test_transmission_malformed_xml():
    """Test transmission parsing with malformed XML."""
    # Transmission missing name (line 543 in urdf_parser.py)
    # The parser should log a warning and return None
    assert (
        parse_transmission(ET.fromstring("<transmission><type>Simple</type></transmission>"))
        is None
    )

    # Joint missing name (line 546) - will result in empty joints list,
    # causing Transmission model to raise ValueError, caught and returned None.
    xml = '<transmission name="t1"><joint><reduction>10</reduction></joint></transmission>'
    assert parse_transmission(ET.fromstring(xml)) is None

    # Actuator missing name (line 562) - should skip actuator
    xml_act = """
    <transmission name="t1">
        <type>Simple</type>
        <joint name="j1"><hardwareInterface>position</hardwareInterface></joint>
        <actuator><reduction>10</reduction></actuator>
    </transmission>
    """
    trans_act = parse_transmission(ET.fromstring(xml_act))
    assert trans_act.name == "t1"
    assert len(trans_act.actuators) == 0


def test_validate_package_uri_errors():
    """Test various validation errors in package URIs."""
    import pytest
    from linkforge_core.validation.security import validate_package_uri

    # Missing package name (line 200)
    with pytest.raises(ValueError, match="missing package name"):
        validate_package_uri("package://")

    # Suspicious segments (line 205)
    with pytest.raises(ValueError, match="contains suspicious path components"):
        validate_package_uri("package://robot/./meshes/arm.stl")
    with pytest.raises(ValueError, match="contains suspicious path components"):
        validate_package_uri("package://robot//meshes/arm.stl")  # Empty segment


def test_validate_mesh_path_complex(tmp_path):
    """Test validate_mesh_path with complex scenarios."""
    from linkforge_core.validation.security import validate_mesh_path

    urdf_dir = tmp_path / "robot"
    urdf_dir.mkdir()

    # Absolute path disallowed (line 69-74)
    with pytest.raises(ValueError, match="Absolute path .* not allowed"):
        validate_mesh_path(Path("/tmp/mesh.stl"), urdf_dir)

    # Path escapes directory (line 101-104)
    with pytest.raises(ValueError, match="attempts to escape the sandbox root"):
        validate_mesh_path(Path("../other/mesh.stl"), urdf_dir)

    # URL encoding decoding coverage (line 64-65)
    with pytest.raises(ValueError, match="attempts to escape the sandbox root"):
        validate_mesh_path(Path("meshes/%2e%2e/%2e%2e/evil.stl"), urdf_dir)

    # Safe path with explicit urdf_directory (hits line 106)
    mesh_path = Path("mesh.stl")
    (urdf_dir / mesh_path).touch()
    resolved = validate_mesh_path(mesh_path, urdf_dir)
    assert resolved.name == "mesh.stl"


def test_mesh_scale_coverage():
    """Test mesh scale parsing with 0 or negative values."""
    # Scale 0 (line 206)
    assert (
        parse_geometry(ET.fromstring('<geometry><mesh filename="m.stl" scale="0 0 0"/></geometry>'))
        is None
    )
    # Invalid scale format (line 206)
    assert (
        parse_geometry(ET.fromstring('<geometry><mesh filename="m.stl" scale="1 1 "/></geometry>'))
        is None
    )


def test_parsing_exception_wrapping(tmp_path, monkeypatch):
    """Verify that unexpected exceptions are wrapped in RobotParserError."""
    parser = URDFParser()

    # RuntimeError wrapping in parse()
    def mock_fail(*args, **kwargs):
        raise RuntimeError("Underlying cause")

    monkeypatch.setattr(ET, "iterparse", mock_fail)
    f = tmp_path / "test.urdf"
    f.write_text('<robot name="t"/>')

    # We match only the start to be robust against message variations
    with pytest.raises(RobotParserError, match="Unexpected error parsing URDF"):
        parser.parse(f)


def test_security_utilities_exhaustive():
    """Directly verify security validation utilities."""
    with pytest.raises(ValueError, match="must start with 'package://'"):
        validate_package_uri("not-package://test")

    with pytest.raises(ValueError, match="Path traversal detected"):
        validate_package_uri("package://pkg/%2e%2e/%2e%2e/etc/passwd")

    with pytest.raises(ValueError, match="restricted system location"):
        validate_mesh_path(Path("/etc/passwd"), Path("/tmp"), allow_absolute=True)


def test_parser_root_validation():
    """Test strict root element validation."""
    parser = URDFParser()
    with pytest.raises(ValueError, match="Root element must be <robot>"):
        parser._parse_robot(ET.fromstring("<not_robot/>"))


def test_joint_parsing_uncovered_paths():
    """Test invalid joint configurations (for line coverage)."""
    parser = URDFParser()
    # Missing child link
    urdf = """
    <robot name="r">
        <link name="l1"/>
        <joint name="j1" type="fixed"><parent link="l1"/><child link="MISSING"/></joint>
    </robot>
    """
    # This should trigger robot.add_joint(joint) -> ValueError -> logger.warning
    robot = parser.parse_string(urdf)
    assert len(robot.joints) == 0


def test_xacro_detection_full(tmp_path):
    """Test full XACRO detection logic including namespace and substitutions."""
    from linkforge_core.parsers.urdf_parser import URDFParser

    parser = URDFParser()

    # 1. Detect by namespace in content (even if filename is .urdf)
    f_ns = tmp_path / "detect_ns.urdf"
    f_ns.write_text('<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="t"/>')
    with pytest.raises(XacroDetectedError, match="XACRO file detected"):
        parser.parse(f_ns)

    # 2. Detect by xacro elements in parse_string (quick check)
    xml_elem = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="t"><xacro:macro name="m"/></robot>'
    with pytest.raises(XacroDetectedError, match="XACRO file detected"):
        parser.parse_string(xml_elem)

    # 3. Detect by ${} substitutions (triggers _detect_xacro_file via parse_string)
    xml_sub = '<robot name="t"><link name="${link_name}"/></robot>'
    with pytest.raises(XacroDetectedError, match="XACRO file detected"):
        parser.parse_string(xml_sub)

    # 4. Detect by xacro extension (triggers proactive check in parse)
    f_ext = tmp_path / "test.xacro"
    f_ext.write_text('<robot name="t"/>')
    with pytest.raises(XacroDetectedError, match="XACRO file detected"):
        parser.parse(f_ext)

    # 5. Detect by {namespace} tags during iterative parsing (line 1089)
    # We use a path to trigger iterative parsing instead of fromstring check
    f_ns_tag = tmp_path / "detect_ns_tag.urdf"
    f_ns_tag.write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="t"><xacro:macro name="m"/></robot>'
    )
    with pytest.raises(XacroDetectedError, match="Detected XACRO features: macro"):
        parser.parse(f_ns_tag)

    # 6. Simulate file read error in XACRO detection (lines 1079-1082)
    def mock_read_fail(*args, **kwargs):
        raise OSError("Permission denied")

    import unittest.mock as mock

    with mock.patch("pathlib.Path.read_text", mock_read_fail):
        # Should not raise error on read failure, proceed to element check
        parser.parse_string('<robot name="t"/>')


def test_sensor_parsing_detailed_coverage():
    """Test detailed sensor parsing branches for coverage."""
    # 1. Camera without <camera> element (default info)
    xml_cam = '<gazebo reference="l"><sensor name="s" type="camera"/></gazebo>'
    sensor_cam = parse_sensor_from_gazebo(ET.fromstring(xml_cam))
    assert sensor_cam.camera_info is not None
    assert sensor_cam.camera_info.width == 640

    # 2. LIDAR without <ray> element
    xml_lidar = '<gazebo reference="l"><sensor name="s" type="lidar"/></gazebo>'
    sensor_lidar = parse_sensor_from_gazebo(ET.fromstring(xml_lidar))
    assert sensor_lidar.lidar_info is not None
    assert sensor_lidar.lidar_info.horizontal_samples == 640

    # 3. GPS with detailed noise
    xml_gps = """
    <gazebo reference="l">
        <sensor name="s" type="gps">
            <gps>
                <position_sensing>
                    <horizontal><noise type="gaussian"><stddev>0.1</stddev></noise></horizontal>
                </position_sensing>
            </gps>
        </sensor>
    </gazebo>
    """
    sensor_gps = parse_sensor_from_gazebo(ET.fromstring(xml_gps))
    assert sensor_gps.gps_info.position_sensing_horizontal_noise.stddev == 0.1

    # 4. Sensor with <pose> element (Transform parsing)
    xml_pose = """
    <gazebo reference="l">
        <sensor name="s" type="imu">
            <pose>1 2 3 0 0 0</pose>
        </sensor>
    </gazebo>
    """
    sensor_pose = parse_sensor_from_gazebo(ET.fromstring(xml_pose))
    assert sensor_pose.origin.xyz.x == 1.0

    # 5. Contact sensor success path
    xml_contact = """
    <gazebo reference="foot">
        <sensor name="foot_contact" type="contact">
            <contact><collision>foot_collision</collision></contact>
        </sensor>
    </gazebo>
    """
    sensor_contact = parse_sensor_from_gazebo(ET.fromstring(xml_contact))
    assert sensor_contact.contact_info.collision == "foot_collision"

    # 6. Contact sensor failure path (missing collision)
    xml_no_coll = """
    <gazebo reference="foot">
        <sensor name="foot_contact" type="contact">
            <contact></contact>
        </sensor>
    </gazebo>
    """
    with pytest.raises(ValueError, match="missing required <collision> element"):
        parse_sensor_from_gazebo(ET.fromstring(xml_no_coll))

    # 7. Sensor with plugin (line 920)
    xml_plugin = """
    <gazebo reference="l">
        <sensor name="s" type="imu">
            <plugin name="p" filename="f.so"/>
        </sensor>
    </gazebo>
    """
    sensor_plugin = parse_sensor_from_gazebo(ET.fromstring(xml_plugin))
    assert sensor_plugin.plugin.name == "p"


def test_ros2_control_detailed_coverage():
    """Test ros2_control hardware parameters and other details."""
    from linkforge_core.parsers.urdf_parser import parse_ros2_control

    xml = """
    <ros2_control name="S" type="system">
        <hardware>
            <plugin>H</plugin>
            <param name="p1">v1</param>
        </hardware>
        <top_param>v2</top_param>
    </ros2_control>
    """
    rc = parse_ros2_control(ET.fromstring(xml))
    assert rc.parameters["hardware.p1"] == "v1"
    assert rc.parameters["top_param"] == "v2"


def test_iterative_parse_skip_invalid_joint(tmp_path):
    """Test skipping invalid joints during iterative parsing (line 1202)."""
    from linkforge_core.parsers.urdf_parser import URDFParser

    parser = URDFParser()
    f = tmp_path / "invalid_joint.urdf"
    # Joint missing child link is invalid
    f.write_text(
        '<robot name="r"><link name="l1"/><joint name="j" type="fixed"><parent link="l1"/></joint></robot>'
    )

    robot = parser.parse(f)
    assert len(robot.joints) == 0


def test_gazebo_element_unknown_properties():
    """Test storing unknown gazebo elements as properties."""
    from linkforge_core.parsers.urdf_parser import parse_gazebo_element

    xml = '<gazebo reference="l"><unknown_tag>some_value</unknown_tag></gazebo>'
    elem = parse_gazebo_element(ET.fromstring(xml))
    assert elem.properties["unknown_tag"] == "some_value"


def test_ros2_control_parameters_coverage():
    """Test ros2_control top-level parameters (line 629)."""
    from linkforge_core.parsers.urdf_parser import parse_ros2_control

    xml = """
    <ros2_control name="S" type="system">
        <hardware><plugin>H</plugin></hardware>
        <some_param>123</some_param>
    </ros2_control>
    """
    rc = parse_ros2_control(ET.fromstring(xml))
    assert rc.parameters["some_param"] == "123"


def test_sensor_noise_parsing_handles_none():
    """Test that sensor noise parsing returns None when input is empty."""
    from linkforge_core.parsers.urdf_parser import parse_sensor_noise

    assert parse_sensor_noise(None) is None


def test_geometry_parsing_resolves_package_uris():
    """Test that geometry elements correctly preserve package:// URIs."""
    xml = '<geometry><mesh filename="package://my_pkg/meshes/base.stl"/></geometry>'
    geom = parse_geometry(ET.fromstring(xml))
    assert geom.filepath == Path("package://my_pkg/meshes/base.stl")


def test_iterative_parsing_detects_xacro_namespace(tmp_path):
    """Test that iterative parsing detects XACRO files via namespace attributes."""
    from linkforge_core.parsers.urdf_parser import URDFParser

    parser = URDFParser()
    f_ns_tag = tmp_path / "detect_ns_tag.urdf"
    f_ns_tag.write_text(
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="t"><xacro:macro name="m"/></robot>'
    )
    with pytest.raises(XacroDetectedError, match="XACRO file detected"):
        parser.parse(f_ns_tag)


def test_joint_renaming_handles_malformed_references():
    """Test that renaming duplicate joints works even if they have broken link references."""
    from linkforge_core.parsers.urdf_parser import URDFParser

    parser = URDFParser()
    urdf = """
    <robot name="r">
        <link name="l1"/><link name="l2"/>
        <joint name="j1" type="fixed"><parent link="l1"/><child link="l2"/></joint>
        <joint name="j1" type="fixed"><parent link="l1"/><child link="MISSING"/></joint>
    </robot>
    """
    robot = parser.parse_string(urdf)
    # First joint added, second duplicate tried rename, but rename add_joint failed -> skipped
    assert len(robot.joints) == 1
    assert robot.joints[0].name == "j1"


def test_xacro_detection_resilient_to_read_failures(tmp_path):
    """Test that XACRO detection falls back gracefully if file reading fails."""
    from unittest.mock import patch

    from linkforge_core.parsers.urdf_parser import URDFParser

    parser = URDFParser()
    f = tmp_path / "test.urdf"
    f.write_text("<robot/>")

    with patch("pathlib.Path.read_text", side_effect=OSError("Read failed")):
        # Should catch OSError and proceed
        robot = parser.parse(f)
        assert robot is not None


def test_iterative_parsing_handles_transmissions(tmp_path):
    """Test that transmission elements are correctly captured during iterative parsing."""
    from linkforge_core.parsers.urdf_parser import URDFParser

    parser = URDFParser()
    urdf = """
    <robot name="r">
        <link name="l1"/>
        <transmission name="t1">
            <type>transmission_interface/SimpleTransmission</type>
            <joint name="j1"><hardwareInterface>PositionJointInterface</hardwareInterface></joint>
            <actuator name="a1"><hardwareInterface>PositionJointInterface</hardwareInterface></actuator>
        </transmission>
    </robot>
    """
    f = tmp_path / "trans_iter.urdf"
    f.write_text(urdf)
    robot = parser.parse(f)
    assert len(robot.transmissions) == 1
    assert robot.transmissions[0].name == "t1"


def test_parser_renames_duplicate_elements_programmatically():
    """Test automatic renaming of duplicate links and joints during string parsing."""
    from linkforge_core.parsers.urdf_parser import URDFParser

    parser = URDFParser()
    # Duplicate links
    urdf_links = '<robot name="r"><link name="l1"/><link name="l1"/></robot>'
    robot = parser.parse_string(urdf_links)
    assert len(robot.links) == 2
    assert robot.links[1].name == "l1_duplicate_1"

    # Duplicate joints
    urdf_joints = """
    <robot name="r">
        <link name="l1"/><link name="l2"/>
        <joint name="j1" type="fixed"><parent link="l1"/><child link="l2"/></joint>
        <joint name="j1" type="fixed"><parent link="l1"/><child link="l2"/></joint>
    </robot>
    """
    robot = parser.parse_string(urdf_joints)
    assert len(robot.joints) == 2
    assert robot.joints[1].name == "j1_duplicate_1"


def test_iterative_parsing_handles_ros2_control(tmp_path):
    """Test that ros2_control blocks are correctly captured during iterative parsing."""
    from linkforge_core.parsers.urdf_parser import URDFParser

    parser = URDFParser()
    urdf = """
    <robot name="r">
        <link name="l1"/>
        <ros2_control name="rc" type="system">
            <hardware><plugin>p</plugin></hardware>
        </ros2_control>
    </robot>
    """
    f = tmp_path / "rc_iter.urdf"
    f.write_text(urdf)
    robot = parser.parse(f)
    assert len(robot.ros2_controls) == 1
    assert robot.ros2_controls[0].name == "rc"


def test_iterative_parsing_renames_duplicate_links(tmp_path):
    """Test that the iterative parsing path handles duplicate link names via renaming."""
    from linkforge_core.parsers.urdf_parser import URDFParser

    parser = URDFParser()
    f = tmp_path / "dup_link.urdf"
    f.write_text('<robot name="r"><link name="l"/><link name="l"/></robot>')
    robot = parser.parse(f)
    assert len(robot.links) == 2
    assert robot.links[1].name == "l_duplicate_1"


def test_iterative_parsing_renames_duplicate_joints(tmp_path):
    """Test that the iterative parsing path handles duplicate joint names via renaming."""
    from linkforge_core.parsers.urdf_parser import URDFParser

    parser = URDFParser()
    f = tmp_path / "dup_joint.urdf"
    f.write_text("""
    <robot name="r">
        <link name="l1"/><link name="l2"/>
        <joint name="j" type="fixed"><parent link="l1"/><child link="l2"/></joint>
        <joint name="j" type="fixed"><parent link="l1"/><child link="l2"/></joint>
    </robot>
    """)
    robot = parser.parse(f)
    assert len(robot.joints) == 2
    assert robot.joints[1].name == "j_duplicate_1"


def test_parser_handles_sequential_duplicate_renames():
    """Test that the parser can handle multiple sequential renames (l_duplicate_1, l_duplicate_2)."""
    from linkforge_core.parsers.urdf_parser import URDFParser

    parser = URDFParser()
    # Need 3 links with same name to trigger counter > 1
    urdf = '<robot name="r"><link name="l"/><link name="l"/><link name="l"/></robot>'
    robot = parser.parse_string(urdf)
    assert robot.links[1].name == "l_duplicate_1"
    assert robot.links[2].name == "l_duplicate_2"

    # Same for joints
    urdf_j = """
    <robot name="r">
        <link name="l1"/><link name="l2"/>
        <joint name="j" type="fixed"><parent link="l1"/><child link="l2"/></joint>
        <joint name="j" type="fixed"><parent link="l1"/><child link="l2"/></joint>
        <joint name="j" type="fixed"><parent link="l1"/><child link="l2"/></joint>
    </robot>
    """
    robot = parser.parse_string(urdf_j)
    assert robot.joints[1].name == "j_duplicate_1"
    assert robot.joints[2].name == "j_duplicate_2"


def test_string_parsing_supports_custom_sandbox_root():
    """Test that parse_string correctly respects an override sandbox_root."""
    from pathlib import Path

    from linkforge_core.parsers.urdf_parser import URDFParser

    parser = URDFParser()
    robot = parser.parse_string('<robot name="r"/>', sandbox_root=Path("/tmp/sandbox"))
    assert robot.name == "r"


def test_parsing_handles_unknown_gazebo_tags_gracefully():
    """Test that generic gazebo elements are safely captured without specialized parsers."""
    from linkforge_core.parsers.urdf_parser import URDFParser

    parser = URDFParser()
    urdf = (
        '<robot name="r"><gazebo reference="l1"><self_collide>true</self_collide></gazebo></robot>'
    )
    robot = parser.parse_string(urdf)
    assert len(robot.gazebo_elements) == 1
    assert robot.gazebo_elements[0].reference == "l1"
    assert robot.gazebo_elements[0].properties["self_collide"] == "true"


def test_parse_string_raises_error_on_truncated_xml():
    """Test that malformed/truncated XML raises a RobotParserError."""
    from linkforge_core.base import RobotParserError
    from linkforge_core.parsers.urdf_parser import URDFParser

    parser = URDFParser()
    with pytest.raises(RobotParserError, match="Failed to parse URDF XML"):
        parser.parse_string("<robot><link")


def test_parse_string_rejects_excessive_content_size():
    """Test that strings exceeding max_file_size are rejected for security."""
    from linkforge_core.base import RobotParserError
    from linkforge_core.parsers.urdf_parser import URDFParser

    parser = URDFParser()
    parser.max_file_size = 10  # 10 bytes
    with pytest.raises(RobotParserError, match="URDF string too large"):
        parser.parse_string("<robot name='too_large'/>")


def test_parse_rejects_excessive_file_size(tmp_path):
    """Test that files exceeding max_file_size are rejected during iterative parsing."""
    from linkforge_core.parsers.urdf_parser import URDFParser

    parser = URDFParser()
    parser.max_file_size = 5  # 5 bytes
    f = tmp_path / "large.urdf"
    f.write_text('<robot name="too_large_file"/>')

    with pytest.raises(RobotParserError, match="URDF file too large"):
        parser.parse(f)


def test_parse_rejects_excessive_xml_depth(tmp_path):
    """Test that deeply nested XML is rejected during iterative parsing."""
    from linkforge_core.parsers.urdf_parser import URDFParser
    from linkforge_core.utils.xml_utils import MAX_XML_DEPTH

    parser = URDFParser()
    f = tmp_path / "deep.urdf"

    # Build deeply nested XML string
    xml_parts = ['<robot name="deep">']
    for i in range(MAX_XML_DEPTH + 1):
        xml_parts.append(f'<link name="l{i}">')
    for _ in range(MAX_XML_DEPTH + 1):
        xml_parts.append("</link>")
    xml_parts.append("</robot>")

    f.write_text("".join(xml_parts))

    with pytest.raises(RobotParserError, match="XML nesting too deep"):
        parser.parse(f)
