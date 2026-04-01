import pytest
from linkforge_core.base import RobotParserError
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
    with pytest.raises(RobotParserError):
        parser.parse_string("<robot><unclosed_tag></robot>")


def test_srdf_parser_wrong_root():
    """Test that non-<robot> root raises RobotParserError."""
    parser = SRDFParser()
    with pytest.raises(RobotParserError, match="Root element must be <robot>"):
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
