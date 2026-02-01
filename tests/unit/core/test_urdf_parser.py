"""Tests for the URDFParser class and core integration logic."""

from __future__ import annotations

import pytest
from linkforge_core.base import RobotParserError
from linkforge_core.models import Robot
from linkforge_core.parsers.urdf_parser import URDFParser


def test_parser_init():
    """Test URDFParser initialization with custom settings."""
    parser = URDFParser(max_file_size=5000)
    assert parser.max_file_size == 5000


def test_parse_string_simple():
    """Test parsing a basic URDF from string."""
    urdf = """<?xml version="1.0"?>
    <robot name="test_robot">
        <link name="base_link">
            <visual>
                <geometry><box size="1 1 1"/></geometry>
            </visual>
        </link>
    </robot>
    """
    robot = URDFParser().parse_string(urdf)
    assert isinstance(robot, Robot)
    assert robot.name == "test_robot"
    assert len(robot.links) == 1
    assert robot.links[0].name == "base_link"


def test_parse_string_with_materials():
    """Test parsing URDF string with global materials."""
    urdf = """
    <robot name="mat_bot">
        <material name="blue"><color rgba="0 0 1 1"/></material>
        <link name="base_link">
            <visual>
                <geometry><sphere radius="0.5"/></geometry>
                <material name="blue"/>
            </visual>
        </link>
    </robot>
    """
    robot = URDFParser().parse_string(urdf)
    assert len(robot.links) == 1
    link = robot.links[0]
    assert link.visuals[0].material.name == "blue"


def test_parse_string_with_joints():
    """Test parsing URDF string with joints and references."""
    urdf = """
    <robot name="joint_bot">
        <link name="l1"/><link name="l2"/>
        <joint name="j1" type="revolute">
            <parent link="l1"/><child link="l2"/>
            <axis xyz="0 0 1"/>
            <limit lower="-1.5" upper="1.5" effort="10" velocity="1"/>
        </joint>
    </robot>
    """
    robot = URDFParser().parse_string(urdf)
    assert len(robot.joints) == 1
    joint = robot.joints[0]
    assert joint.parent == "l1"
    assert joint.child == "l2"
    assert joint.limits.lower == pytest.approx(-1.5)


def test_parse_from_file(tmp_path):
    """Test parsing URDF from a physical file."""
    urdf_content = """
    <robot name="file_bot">
        <link name="base_link"/>
    </robot>
    """
    urdf_file = tmp_path / "test.urdf"
    urdf_file.write_text(urdf_content)

    parser = URDFParser()
    robot = parser.parse(urdf_file)
    assert robot.name == "file_bot"
    assert len(robot.links) == 1


def test_parse_complex_robot_integration():
    """Test parsing a comprehensive robot description."""
    urdf = """
    <robot name="full_robot">
        <link name="base_link">
            <inertial>
                <mass value="2.0"/>
                <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
            </inertial>
        </link>
        <link name="link1"/>
        <joint name="joint1" type="fixed">
            <parent link="base_link"/><child link="link1"/>
        </joint>
        <transmission name="trans1">
            <type>transmission_interface/SimpleTransmission</type>
            <joint name="joint1"><hardwareInterface>PositionJointInterface</hardwareInterface></joint>
        </transmission>
        <gazebo><plugin name="p1" filename="f1"/></gazebo>
    </robot>
    """
    robot = URDFParser().parse_string(urdf)
    assert len(robot.links) == 2
    assert len(robot.joints) == 1
    assert len(robot.transmissions) == 1
    assert len(robot.gazebo_elements) == 1


def test_large_file_rejection():
    """Test rejection of oversized URDF input."""
    parser = URDFParser(max_file_size=100)
    with pytest.raises(RobotParserError, match="URDF string too large"):
        parser.parse_string(" " * 200)


def test_xacro_detection_rejection():
    """Test that XACRO files are rejected by the standard parser."""
    urdf = '<robot xmlns:xacro="x"><link name="l"/><xacro:macro name="m"/></robot>'
    with pytest.raises(RobotParserError, match="XACRO features detected"):
        URDFParser().parse_string(urdf)
