from pathlib import Path

import pytest
from linkforge_core.exceptions import (
    RobotParserIOError,
    RobotParserUnexpectedError,
    RobotParserXMLRootError,
)
from linkforge_core.models.robot import Robot
from linkforge_core.parsers.srdf_parser import SRDFParser

BASIC_SRDF = """<?xml version="1.0"?>
<robot name="test_robot">
    <virtual_joint name="world_joint" type="fixed" parent_frame="world" child_link="base_link"/>
    <group name="arm">
        <chain base_link="base_link" tip_link="tool0"/>
        <joint name="joint1"/>
        <link name="link1"/>
    </group>
    <group_state name="home" group="arm">
        <joint name="joint1" value="0.0"/>
    </group_state>
    <end_effector name="hand" group="hand_group" parent_link="link4"/>
    <passive_joint name="passive_1"/>
    <disable_collisions link1="link1" link2="link2" reason="Adjacent"/>
</robot>
"""

XACRO_SRDF = """<?xml version="1.0"?>
<robot name="xacro_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:property name="joint_val" value="1.57"/>
    <group_state name="pose" group="arm">
        <joint name="joint1" value="${joint_val}"/>
    </group_state>
</robot>
"""


def test_srdf_parser_basic_string():
    """Test parsing a basic SRDF string."""
    parser = SRDFParser()
    robot = parser.parse_string(BASIC_SRDF)

    assert robot.name == "test_robot"
    assert robot.semantic is not None
    assert len(robot.semantic.virtual_joints) == 1
    assert robot.semantic.virtual_joints[0].name == "world_joint"

    assert len(robot.semantic.groups) == 1
    assert robot.semantic.groups[0].name == "arm"
    assert ("base_link", "tool0") in robot.semantic.groups[0].chains

    assert len(robot.semantic.group_states) == 1
    assert robot.semantic.group_states[0].name == "home"
    assert robot.semantic.group_states[0].joint_values["joint1"] == 0.0

    assert len(robot.semantic.end_effectors) == 1
    assert robot.semantic.end_effectors[0].name == "hand"

    assert len(robot.semantic.passive_joints) == 1
    assert robot.semantic.passive_joints[0].name == "passive_1"

    assert len(robot.semantic.disabled_collisions) == 1
    assert robot.semantic.disabled_collisions[0].link1 == "link1"
    assert robot.semantic.disabled_collisions[0].reason == "Adjacent"


def test_srdf_parser_xacro_resolution():
    """Test that SRDFParser resolves XACRO macros."""
    parser = SRDFParser()
    robot = parser.parse_string(XACRO_SRDF)

    assert robot.semantic is not None
    assert robot.semantic.group_states[0].joint_values["joint1"] == 1.57


def test_srdf_parser_robot_integration():
    """Test parsing SRDF and attaching it to an existing Robot."""
    existing_robot = Robot(name="my_robot")
    parser = SRDFParser()
    parser.parse_string(BASIC_SRDF, robot=existing_robot)

    # Existing robot should be updated
    assert existing_robot.semantic is not None
    assert len(existing_robot.semantic.groups) == 1


def test_srdf_parser_invalid_xml():
    """Test that malformed XML raises RobotParserError."""
    parser = SRDFParser()
    with pytest.raises(RobotParserUnexpectedError, match="SRDF parse"):
        parser.parse_string("<robot><unclosed_tag></robot>")


def test_srdf_parser_wrong_root():
    """Test that non-<robot> root raises RobotParserError."""
    parser = SRDFParser()
    with pytest.raises(RobotParserXMLRootError, match="Invalid XML root: <not_a_robot>"):
        parser.parse_string("<not_a_robot></not_a_robot>")


def test_srdf_parser_file_parsing(tmp_path):
    """Test parsing from a file."""
    srdf_file = tmp_path / "test.srdf"
    srdf_file.write_text(BASIC_SRDF)

    parser = SRDFParser()
    robot = parser.parse(srdf_file)
    assert robot.semantic is not None
    assert robot.name == "test_robot"


def test_srdf_parser_xacro_file_parsing(tmp_path):
    """Test parsing from a .xacro file."""
    xacro_file = tmp_path / "test.srdf.xacro"
    xacro_file.write_text(XACRO_SRDF)

    parser = SRDFParser()
    robot = parser.parse(xacro_file)
    assert robot.semantic is not None
    assert robot.semantic.group_states[0].joint_values["joint1"] == 1.57


def test_srdf_parser_file_not_found():
    """Test error when SRDF file does not exist."""
    parser = SRDFParser()
    with pytest.raises(RobotParserIOError, match="Missing file"):
        parser.parse(Path("non_existent.srdf"))


def test_srdf_parser_file_too_large(tmp_path):
    """Test safety check for large SRDF files."""
    srdf_file = tmp_path / "large.srdf"
    srdf_file.write_text("a" * 100)

    parser = SRDFParser(max_file_size=10)
    with pytest.raises(RobotParserIOError, match="File too large"):
        parser.parse(srdf_file)


def test_srdf_parser_malformed_xml():
    """Test error when SRDF XML is malformed."""
    parser = SRDFParser()
    with pytest.raises(RobotParserUnexpectedError, match="Unexpected error in SRDF parse"):
        parser.parse_string("<robot><unclosed>")


def test_srdf_parser_unexpected_exception(monkeypatch):
    """Test generic exception handling during parsing."""
    parser = SRDFParser()

    def mock_fromstring(*args, **kwargs):
        raise ValueError("Unexpected error")

    import xml.etree.ElementTree as ET

    monkeypatch.setattr(ET, "fromstring", mock_fromstring)

    with pytest.raises(
        RobotParserUnexpectedError, match="Unexpected error in Unexpected SRDF parse"
    ):
        parser.parse_string("<robot/>")


def test_srdf_parser_optional_attributes():
    """Test parsing elements with optional attributes missing/present."""
    xml = """<?xml version="1.0"?>
    <robot name="opts">
      <end_effector name="ee" group="g" parent_link="l"/>
      <disable_collisions link1="l1" link2="l2"/>
      <group name="empty_tags">
        <link/>
        <joint/>
        <chain/>
        <group/>
      </group>
    </robot>
    """
    parser = SRDFParser()
    robot = parser.parse_string(xml)
    ee = robot.semantic.end_effectors[0]
    assert ee.parent_group is None

    dc = robot.semantic.disabled_collisions[0]
    assert dc.reason is None

    group = robot.semantic.groups[0]
    assert len(group.links) == 0
    assert len(group.joints) == 0
    assert len(group.chains) == 0
    assert len(group.subgroups) == 0


def test_srdf_parser_subgroups_and_collisions():
    """Test parsing of subgroups and disabled collisions."""
    parser = SRDFParser()
    xml = """
    <robot name="test">
        <group name="arm">
            <group name="hand"/>
        </group>
        <disable_collisions link1="l1" link2="l2" reason="adjacent"/>
    </robot>
    """
    robot = parser.parse_string(xml)
    assert len(robot.semantic.groups) == 1
    assert robot.semantic.groups[0].subgroups == ["hand"]
    assert len(robot.semantic.disabled_collisions) == 1
    assert robot.semantic.disabled_collisions[0].reason == "adjacent"


def test_srdf_parser_kwargs_and_malformed_joints():
    """Test kwargs passing and malformed joint names."""
    parser = SRDFParser()
    xml = """
    <robot name="test">
        <group_state name="s1" group="g1">
            <joint value="1.0"/> <!-- Missing name -->
        </group_state>
    </robot>
    """
    # Test kwargs logging
    robot = parser.parse_string(xml, debug=True)
    assert len(robot.semantic.group_states) == 1
    assert len(robot.semantic.group_states[0].joint_values) == 0


def test_srdf_parser_generic_exception_in_parse(tmp_path, monkeypatch):
    """Test generic exception catching in parse method."""
    parser = SRDFParser()
    srdf_file = tmp_path / "test.srdf"
    srdf_file.write_text("<robot name='test'/>")

    def mock_read(*args, **kwargs):
        raise RuntimeError("Disk failure")

    monkeypatch.setattr("pathlib.Path.read_text", mock_read)
    with pytest.raises(RobotParserIOError, match="Parser IO error: Disk failure"):
        parser.parse(srdf_file)


def test_srdf_parser_rethrown_exceptions(tmp_path):
    """Test that RobotParserError is re-thrown in parse method."""
    parser = SRDFParser()
    srdf_file = tmp_path / "bad_root.srdf"
    srdf_file.write_text("<not_robot/>")
    with pytest.raises(RobotParserXMLRootError, match="Invalid XML root: <not_robot>"):
        parser.parse(srdf_file)


def test_srdf_parser_missing_names_in_elements():
    """Test elements missing name attributes to cover all branches."""
    parser = SRDFParser()
    xml = """
    <robot name="test">
        <group name="g1">
            <group/> <!-- Missing name -->
        </group>
        <disable_collisions link1="l1" link2="l2"/>
    </robot>
    """
    robot = parser.parse_string(xml)
    assert len(robot.semantic.groups[0].subgroups) == 0
    assert len(robot.semantic.disabled_collisions) == 1


def test_srdf_parser_unrecognized_tags():
    """Test unrecognized tags are ignored (covers loop branches)."""
    parser = SRDFParser()
    xml = """
    <robot name="test">
        <unknown_robot_tag/>
        <group name="g1">
            <unknown_group_tag/>
        </group>
    </robot>
    """
    robot = parser.parse_string(xml)
    assert len(robot.semantic.groups) == 1
