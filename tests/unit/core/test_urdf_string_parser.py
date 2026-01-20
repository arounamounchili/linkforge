"""Tests for parsing URDF from string (used by XACRO import)."""

import pytest

from linkforge.core.models.robot import Robot
from linkforge.core.parsers.urdf_parser import parse_urdf_string


class TestParseURDFString:
    """Tests for parse_urdf_string function."""

    def test_parse_simple_urdf_string(self):
        """Test parsing a simple URDF from string."""
        urdf_xml = """<?xml version="1.0"?>
<robot name="test_robot">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="1 1 1"/>
            </geometry>
        </visual>
    </link>
</robot>
"""
        robot = parse_urdf_string(urdf_xml)

        assert isinstance(robot, Robot)
        assert robot.name == "test_robot"
        assert len(robot.links) == 1
        assert robot.links[0].name == "base_link"

    def test_parse_urdf_string_with_materials(self):
        """Test parsing URDF string with materials."""
        urdf_xml = """<?xml version="1.0"?>
<robot name="test_robot">
    <material name="blue">
        <color rgba="0 0 1 1"/>
    </material>

    <link name="base_link">
        <visual>
            <geometry>
                <box size="1 1 1"/>
            </geometry>
            <material name="blue"/>
        </visual>
    </link>
</robot>
"""
        robot = parse_urdf_string(urdf_xml)

        assert robot.name == "test_robot"
        assert len(robot.links) == 1
        link = robot.links[0]
        assert link.visuals[0] is not None
        assert link.visuals[0].material is not None
        assert link.visuals[0].material.name == "blue"

    def test_parse_urdf_string_with_joints(self):
        """Test parsing URDF string with joints."""
        urdf_xml = """<?xml version="1.0"?>
<robot name="test_robot">
    <link name="base_link"/>
    <link name="child_link"/>

    <joint name="test_joint" type="revolute">
        <parent link="base_link"/>
        <child link="child_link"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
    </joint>
</robot>
"""
        robot = parse_urdf_string(urdf_xml)

        assert robot.name == "test_robot"
        assert len(robot.links) == 2
        assert len(robot.joints) == 1

        joint = robot.joints[0]
        assert joint.name == "test_joint"
        assert joint.parent == "base_link"
        assert joint.child == "child_link"
        assert joint.limits is not None
        assert joint.limits.lower == pytest.approx(-1.57)
        assert joint.limits.upper == pytest.approx(1.57)

    def test_parse_urdf_string_invalid_xml(self):
        """Test that invalid XML raises ParseError."""
        import xml.etree.ElementTree as ET

        invalid_xml = "<robot name='test'><link></robot>"  # Unclosed link tag

        with pytest.raises(ET.ParseError):
            parse_urdf_string(invalid_xml)

    def test_parse_urdf_string_non_robot_root(self):
        """Test that non-robot root element raises ValueError."""
        urdf_xml = """<?xml version="1.0"?>
<model name="test">
    <link name="base_link"/>
</model>
"""
        with pytest.raises(ValueError, match="Root element must be <robot>"):
            parse_urdf_string(urdf_xml)

    def test_parse_urdf_string_with_inertial(self):
        """Test parsing URDF string with inertial properties."""
        urdf_xml = """<?xml version="1.0"?>
<robot name="test_robot">
    <link name="base_link">
        <inertial>
            <mass value="1.5"/>
            <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                     iyy="0.1" iyz="0.0" izz="0.1"/>
        </inertial>
    </link>
</robot>
"""
        robot = parse_urdf_string(urdf_xml)

        link = robot.links[0]
        assert link.inertial is not None
        assert link.inertial.mass == pytest.approx(1.5)
        assert link.inertial.inertia.ixx == pytest.approx(0.1)

    def test_parse_urdf_string_empty_robot(self):
        """Test parsing URDF string with no links or joints."""
        urdf_xml = """<?xml version="1.0"?>
<robot name="empty_robot">
</robot>
"""
        robot = parse_urdf_string(urdf_xml)

        assert robot.name == "empty_robot"
        assert len(robot.links) == 0
        assert len(robot.joints) == 0

    def test_parse_urdf_string_complex_robot(self):
        """Test parsing a more complex URDF string."""
        urdf_xml = """<?xml version="1.0"?>
<robot name="complex_robot">
    <material name="red">
        <color rgba="1 0 0 1"/>
    </material>

    <link name="base_link">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.5"/>
            </geometry>
            <material name="red"/>
        </visual>
        <inertial>
            <mass value="2.0"/>
            <inertia ixx="0.1" ixy="0" ixz="0"
                     iyy="0.1" iyz="0" izz="0.1"/>
        </inertial>
    </link>

    <link name="wheel_left"/>
    <link name="wheel_right"/>

    <joint name="left_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="wheel_left"/>
        <origin xyz="-0.2 0 -0.1" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
    </joint>

    <joint name="right_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="wheel_right"/>
        <origin xyz="0.2 0 -0.1" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
    </joint>
</robot>
"""
        robot = parse_urdf_string(urdf_xml)

        assert robot.name == "complex_robot"
        assert len(robot.links) == 3
        assert len(robot.joints) == 2

        # Check base link
        base_link = next(link for link in robot.links if link.name == "base_link")
        assert base_link.visuals[0] is not None
        assert base_link.inertial is not None
        assert base_link.inertial.mass == pytest.approx(2.0)

        # Check joints
        left_joint = next(joint for joint in robot.joints if joint.name == "left_wheel_joint")
        assert left_joint.parent == "base_link"
        assert left_joint.child == "wheel_left"
