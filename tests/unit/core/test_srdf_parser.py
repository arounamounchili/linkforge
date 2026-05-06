from pathlib import Path

import pytest
from linkforge_core.exceptions import (
    RobotParserIOError,
    RobotParserUnexpectedError,
    RobotParserXMLRootError,
)
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
        <joint name="multi_joint" value="0.0 1.0"/>
    </group_state>
    <end_effector name="hand" group="hand_group" parent_link="link4"/>
    <passive_joint name="passive_1"/>
    <disable_collisions link1="link1" link2="link2" reason="Adjacent"/>
    <enable_collisions link1="link1" link2="link3" reason="Testing"/>
    <disable_default_collisions link="link1"/>
    <link_sphere_approximation link="link1">
        <sphere center="0.1 0.2 0.3" radius="0.5"/>
    </link_sphere_approximation>
    <joint_property joint_name="joint1" property_name="prop1" value="val1"/>
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
    semantic = parser.parse_string(BASIC_SRDF)
    assert semantic.robot_name == "test_robot"

    assert len(semantic.virtual_joints) == 1
    assert semantic.virtual_joints[0].name == "world_joint"

    assert len(semantic.groups) == 1
    assert semantic.groups[0].name == "arm"
    assert semantic.groups[0].chains[0].base_link == "base_link"

    assert len(semantic.group_states) == 1
    assert semantic.group_states[0].name == "home"
    assert semantic.group_states[0].joint_values["joint1"] == (0.0,)
    assert semantic.group_states[0].joint_values["multi_joint"] == (0.0, 1.0)

    assert len(semantic.end_effectors) == 1
    assert semantic.end_effectors[0].name == "hand"

    assert len(semantic.passive_joints) == 1
    assert semantic.passive_joints[0].name == "passive_1"

    assert len(semantic.disabled_collisions) == 1
    assert semantic.disabled_collisions[0].link1 == "link1"
    assert semantic.disabled_collisions[0].reason == "Adjacent"

    assert len(semantic.enabled_collisions) == 1
    assert semantic.enabled_collisions[0].link1 == "link1"

    assert len(semantic.no_default_collision_links) == 1
    assert semantic.no_default_collision_links[0] == "link1"

    assert len(semantic.link_sphere_approximations) == 1
    assert semantic.link_sphere_approximations[0].link == "link1"
    assert len(semantic.link_sphere_approximations[0].spheres) == 1
    assert semantic.link_sphere_approximations[0].spheres[0].center_x == 0.1

    assert len(semantic.joint_properties) == 1
    assert semantic.joint_properties[0].joint_name == "joint1"


def test_srdf_parser_xacro_resolution():
    """Test the two-step XACRO to SRDF resolution workflow."""
    from linkforge_core.parsers.xacro_parser import XacroResolver

    xml_string = XacroResolver().resolve_string(XACRO_SRDF)

    parser = SRDFParser()
    semantic = parser.parse_string(xml_string)

    assert semantic.group_states[0].joint_values["joint1"] == (1.57,)


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
    semantic = parser.parse(srdf_file)


def test_srdf_parser_xacro_file_parsing(tmp_path):
    """Test parsing from a .xacro file using parse_xacro()."""
    xacro_file = tmp_path / "test.srdf.xacro"
    xacro_file.write_text(XACRO_SRDF)

    parser = SRDFParser()
    semantic = parser.parse_xacro(xacro_file)
    assert semantic.group_states[0].joint_values["joint1"] == (1.57,)


def test_srdf_parser_file_not_found():
    """Test error when SRDF file does not exist."""
    parser = SRDFParser()
    with pytest.raises(RobotParserIOError, match="File not found"):
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

    def mock_iterparse(*args, **kwargs):
        raise ValueError("Unexpected error")

    import xml.etree.ElementTree as ET

    monkeypatch.setattr(ET, "iterparse", mock_iterparse)

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
        <link name="l1"/>
        <joint name="j1"/>
        <chain base_link="base" tip_link="tip"/>
        <group name="sub"/>
      </group>
    </robot>
    """
    parser = SRDFParser()
    semantic = parser.parse_string(xml)
    ee = semantic.end_effectors[0]
    assert ee.parent_group is None

    dc = semantic.disabled_collisions[0]
    assert dc.reason is None

    group = semantic.groups[0]
    assert len(group.links) == 1
    assert len(group.joints) == 1
    assert len(group.chains) == 1
    assert len(group.subgroups) == 1


def test_srdf_parser_subgroups_and_collisions():
    """Test parsing of subgroups and disabled collisions."""
    parser = SRDFParser()
    xml = """
    <robot name="test">
        <group name="arm">
            <joint name="j1"/>
            <group name="hand"/>
        </group>
        <disable_collisions link1="l1" link2="l2" reason="adjacent"/>
    </robot>
    """
    semantic = parser.parse_string(xml)
    assert len(semantic.groups) == 1
    assert semantic.groups[0].subgroups == ("hand",)
    assert len(semantic.disabled_collisions) == 1
    assert semantic.disabled_collisions[0].reason == "adjacent"


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
    semantic = parser.parse_string(xml, debug=True)
    assert len(semantic.group_states) == 1
    assert len(semantic.group_states[0].joint_values) == 0


def test_srdf_parser_generic_exception_in_parse(tmp_path, monkeypatch):
    """Test generic exception catching in parse method."""
    parser = SRDFParser()
    srdf_file = tmp_path / "test.srdf"
    srdf_file.write_text("<robot name='test'/>")

    def mock_iterparse(*args, **kwargs):
        raise RuntimeError("Disk failure")

    import xml.etree.ElementTree as ET

    monkeypatch.setattr(ET, "iterparse", mock_iterparse)
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
            <joint name="j1"/>
            <group/> <!-- Missing name -->
        </group>
        <disable_collisions link1="l1" link2="l2"/>
    </robot>
    """
    semantic = parser.parse_string(xml)
    assert len(semantic.groups[0].subgroups) == 0
    assert len(semantic.disabled_collisions) == 1


def test_srdf_parser_unrecognized_tags():
    """Test unrecognized tags are ignored (covers loop branches)."""
    parser = SRDFParser()
    xml = """
    <robot name="test">
        <unknown_robot_tag/>
    <group name="g1">
        <joint name="j1"/>
        <unknown_subtag/>
    </group>
    </robot>
    """
    semantic = parser.parse_string(xml)
    assert len(semantic.groups) == 1


def test_srdf_parser_malformed_sphere():
    """Test sphere approximation missing attributes."""
    parser = SRDFParser()
    xml = """
    <robot name="test">
        <link_sphere_approximation link="link1">
            <sphere center="0 0 0"/> <!-- Missing radius -->
            <sphere radius="0.1"/>    <!-- Missing center -->
            <sphere center="0 0 invalid" radius="0.1"/> <!-- Invalid float -->
        </link_sphere_approximation>
    </robot>
    """
    semantic = parser.parse_string(xml)
    assert len(semantic.link_sphere_approximations) == 1
    assert len(semantic.link_sphere_approximations[0].spheres) == 0


def test_srdf_parser_malformed_joint_property():
    """Test joint property missing attributes."""
    parser = SRDFParser()
    xml = """
    <robot name="test">
        <joint_property joint_name="j1" property_name="p1"/> <!-- Missing value -->
        <joint_property joint_name="j1" value="v1"/>        <!-- Missing property_name -->
        <joint_property property_name="p1" value="v1"/>     <!-- Missing joint_name -->
    </robot>
    """
    semantic = parser.parse_string(xml)
    assert len(semantic.joint_properties) == 0


def test_srdf_parser_missing_root():
    """Test missing root element."""
    parser = SRDFParser()
    with pytest.raises(RobotParserXMLRootError):
        parser.parse_string("<not_robot/>")


def test_srdf_parser_empty_string():
    """Test empty string parsing."""
    parser = SRDFParser()
    with pytest.raises(RobotParserUnexpectedError):
        parser.parse_string("")


def test_srdf_parser_all_missing_attributes():
    """Test all possible missing required attributes in various tags."""
    parser = SRDFParser()
    xml = """
    <robot name="test">
        <group/> <!-- Missing name -->
        <group name="g1">
            <link name="valid_link"/> <!-- Valid child -->
            <chain/> <!-- Missing links -->
            <group/> <!-- Missing subgroup name -->
        </group>
        <end_effector group="g1" parent_link="l1"/> <!-- Missing name -->
        <passive_joint/> <!-- Missing name -->
        <disable_default_collisions/> <!-- Missing link -->
        <link_sphere_approximation/> <!-- Missing link -->
    </robot>
    """
    semantic = parser.parse_string(xml)
    assert len(semantic.groups) == 1  # 'g1' is validly named
    assert len(semantic.groups[0].chains) == 0
    assert len(semantic.groups[0].subgroups) == 0
    assert len(semantic.end_effectors) == 0
    assert len(semantic.passive_joints) == 0
    assert len(semantic.no_default_collision_links) == 0
    assert len(semantic.link_sphere_approximations) == 0
