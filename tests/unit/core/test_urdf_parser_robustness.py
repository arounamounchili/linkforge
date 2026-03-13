import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest
from linkforge_core.exceptions import RobotModelError, RobotParserError, XacroDetectedError
from linkforge_core.models import Joint, Link, Robot
from linkforge_core.parsers.urdf_parser import URDFParser
from linkforge_core.parsers.xml_base import RobotXMLParser


class MockXMLParser(RobotXMLParser):
    """Minimal implementation of RobotXMLParser for testing base functionality."""

    def parse(self, filepath: Path, **kwargs):
        pass


# --- Base Parser Robustness (RobotXMLParser) ---


def test_xml_base_material_parsing_robustness() -> None:
    """Verify robust handling of invalid material attributes."""
    parser = MockXMLParser()

    # Handle invalid color formats gracefully
    elem = ET.fromstring('<material name="m"><color rgba="1.0 not_a_float 0.0"/></material>')
    assert parser._parse_material_element(elem, {}) is None

    # Support 3-component RGB by defaulting alpha to 1.0
    elem = ET.fromstring('<material name="m"><color rgba="1.0 0.5 0.0"/></material>')
    mat = parser._parse_material_element(elem, {})
    assert mat.color.r == 1.0
    assert mat.color.a == 1.0


def test_xml_base_inertia_sanitization() -> None:
    """Ensure inertia tensor violations fall back to safe minimal values."""
    parser = MockXMLParser()

    # Violating triangle inequality should fallback to minimal valid tensor
    elem = ET.fromstring(
        '<inertial><mass value="1"/><inertia ixx="10" iyy="1" izz="1"/></inertial>'
    )
    inertial = parser._parse_inertial_element(elem)
    # Minimal stable diagonal is 1e-6
    assert inertial.inertia.ixx == 1e-6


def test_xml_base_geometry_error_handling() -> None:
    """Verify geometry parsing handles malformed elements or security violations."""
    parser = MockXMLParser()

    # Malformed box geometry
    elem = ET.fromstring("<geometry><box/></geometry>")
    assert parser._parse_geometry_element(elem) is None

    # Path traversal attempt should trigger security check and return None
    elem = ET.fromstring('<geometry><mesh filename="/etc/passwd"/></geometry>')
    with patch("linkforge_core.parsers.xml_base.logger") as mock_logger:
        assert parser._parse_geometry_element(elem, base_directory=Path("/tmp")) is None
        assert "mesh geometry" in mock_logger.warning.call_args[0][0]

    # Handle unknown geometry types during exception fallback
    elem = ET.fromstring('<geometry><box size="invalid"/></geometry>')
    with patch("linkforge_core.parsers.xml_base.logger") as mock_logger:
        assert parser._parse_geometry_element(elem) is None
        assert "box geometry" in mock_logger.warning.call_args[0][0]


def test_xml_base_robust_joint_addition_errors() -> None:
    """Verify logging of non-duplicate joint model errors."""
    parser = MockXMLParser()
    robot = Robot(name="test")
    joint = MagicMock(spec=Joint)
    joint.name = "unstable_joint"

    with (
        patch("linkforge_core.parsers.xml_base.logger") as mock_logger,
        patch.object(robot, "add_joint", side_effect=RobotModelError("invalid parent link")),
    ):
        parser._add_joint_robust(robot, joint, ET.fromstring('<joint name="unstable_joint"/>'))
        assert mock_logger.warning.called
        assert "invalid parent link" in mock_logger.warning.call_args[0][0]


# --- URDF Parser Feature Robustness ---


def test_urdf_parser_axis_normalization() -> None:
    """Verify normalization of zero-magnitude joint axes."""
    parser = URDFParser()
    xml = """
    <robot name="axis_test">
        <link name="base"/><link name="top"/>
        <joint name="j" type="revolute">
            <parent link="base"/><child link="top"/>
            <axis xyz="0 0 0"/>
            <limit effort="1" velocity="1"/>
        </joint>
    </robot>
    """
    robot = parser.parse_string(xml)
    # Zero axis should normalize to default [1, 0, 0]
    assert robot.joints[0].axis.x == 1.0


def test_urdf_parser_revolute_joint_without_limits() -> None:
    """Verify revolute joints without limits are handled by defaulting to zero range."""
    parser = URDFParser()
    xml = """
    <robot name="limit_test">
        <link name="base"/><link name="l1"/>
        <joint name="j1" type="revolute">
            <parent link="base"/><child link="l1"/>
        </joint>
    </robot>
    """
    robot = parser.parse_string(xml)
    assert robot.joints[0].limits.lower == 0.0
    assert robot.joints[0].limits.upper == 0.0


def test_urdf_parser_gazebo_sensor_parsing_robustness() -> None:
    """Verify parsing of various Gazebo sensor types and error handling."""
    parser = URDFParser()

    # Camera with invalid pose (trigger warning)
    xml = """
    <gazebo reference="link_a">
        <sensor name="cam" type="camera">
            <pose>0 0 1 invalid_rpy 0 0</pose>
            <camera><horizontal_fov>1.0</horizontal_fov></camera>
        </sensor>
    </gazebo>
    """
    with patch("linkforge_core.parsers.urdf_parser.logger") as mock_logger:
        sensor = parser._parse_sensor_from_gazebo(ET.fromstring(xml))
        assert sensor is not None
        assert mock_logger.warning.called

    # Contact sensor missing collision (trigger exception)
    xml = """
    <gazebo reference="l"><sensor name="s" type="contact"><contact></contact></sensor></gazebo>
    """
    with pytest.raises(RobotModelError, match="missing <collision>"):
        parser._parse_sensor_from_gazebo(ET.fromstring(xml))


def test_urdf_parser_ros2_control_robustness() -> None:
    """Verify robust handling of invalid ros2_control configurations."""
    parser = URDFParser()

    # Invalid type skip
    elem = ET.fromstring(
        '<ros2_control name="ctrl" type="invalid"><hardware><plugin>p</plugin></hardware></ros2_control>'
    )
    with patch("linkforge_core.parsers.urdf_parser.logger") as mock_logger:
        assert parser._parse_ros2_control(elem) is None
        assert mock_logger.warning.called


def test_urdf_parser_transmission_parsing_robustness() -> None:
    """Verify robust parsing of transmission components."""
    parser = URDFParser()

    # Missing name
    elem = ET.fromstring("<transmission/>")
    assert parser._parse_transmission(elem) is None

    # Transmission component missing name
    elem = ET.fromstring("<actuator/>")
    assert parser._parse_transmission_component(elem, "actuator") is None


def test_urdf_parser_xacro_detection_robustness(tmp_path) -> None:
    """Verify all layers of XACRO detection."""
    parser = URDFParser()

    # Extension check
    p = tmp_path / "test.xacro"
    p.write_text("<robot/>")
    with pytest.raises(XacroDetectedError):
        parser.parse(p)

    # Tag check
    xml = '<robot><xacro:macro name="m"/></robot>'
    with pytest.raises(XacroDetectedError):
        parser.parse_string(xml)


def test_urdf_parser_iterative_parsing_robustness(tmp_path) -> None:
    """Verify robust handling of structural errors in iterative parsing."""
    parser = URDFParser()

    # XML nesting too deep
    p = tmp_path / "deep.urdf"
    nested_xml = "<robot>" + "<a>" * 101 + "</a>" * 101 + "</robot>"
    p.write_text(nested_xml)
    with pytest.raises(RobotParserError, match="nesting too deep"):
        parser.parse(p)

    # Missing root <robot>
    p.write_text("<not_robot/>")
    with pytest.raises(RobotParserError, match="Root element must be <robot>"):
        parser.parse(p)

    # Invalid Link (trigger skip warning)
    xml = '<robot name="r"><link name="l"><visual><origin xyz="invalid"/></visual></link></robot>'
    import linkforge_core.parsers.urdf_parser

    with patch.object(linkforge_core.parsers.urdf_parser, "logger") as mock_logger:
        parser.parse_string(xml)
        assert mock_logger.warning.called


def test_urdf_parser_link_renaming_recursive() -> None:
    """Verify iterative renaming handles multiple sequential collisions."""
    parser = URDFParser()
    robot = Robot(name="test")
    robot.add_link(Link(name="l"))
    robot.add_link(Link(name="l_duplicate_1"))

    # Adding 'l' again should result in 'l_duplicate_2'
    parser._add_link_robust(robot, Link(name="l"))
    assert "l_duplicate_2" in robot._link_index


def test_urdf_parser_material_naming() -> None:
    """Verify material without name uses 'default'."""
    parser = URDFParser()
    xml = '<robot><material><color rgba="1 1 1 1"/></material></robot>'
    robot = parser.parse_string(xml)
    assert "default" in robot.materials


def test_urdf_parser_joint_invalid_origin() -> None:
    """Verify joint with invalid origin is skipped."""
    parser = URDFParser()
    xml = """
    <robot name="r">
        <link name="l1"/><link name="l2"/>
        <joint name="j" type="fixed">
            <parent link="l1"/><child link="l2"/>
            <origin xyz="invalid"/>
        </joint>
    </robot>
    """
    import linkforge_core.parsers.urdf_parser

    with patch.object(linkforge_core.parsers.urdf_parser, "logger") as mock_logger:
        parser.parse_string(xml)
        assert mock_logger.warning.called


def test_urdf_parser_xml_error_in_parse(tmp_path) -> None:
    """Verify XML ParseError is wrapped in RobotParserError."""
    parser = URDFParser()
    p = tmp_path / "bad.urdf"
    p.write_text("<robot><")  # Malformed XML
    # Standardizing on 'Failed to parse URDF XML' for ParseError
    with pytest.raises(RobotParserError, match="Failed to parse URDF XML"):
        parser.parse(p)
