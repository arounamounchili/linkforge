"""Tests for URDFParser class and integration logic."""

from __future__ import annotations

import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from linkforge_core.base import RobotParserError
from linkforge_core.parsers.urdf_parser import (
    URDFParser,
    _detect_xacro_file,
)
from linkforge_core.validation.security import is_suspicious_location


def test_urdf_parser_duplicate_links():
    """Test renaming logic for duplicate link names."""
    urdf = """
    <robot name="dup_bot">
        <link name="base_link"></link>
        <link name="base_link"></link>
    </robot>
    """
    robot = URDFParser().parse_string(urdf)
    assert "base_link" in robot._link_index
    assert "base_link_duplicate_1" in robot._link_index


def test_urdf_parser_duplicate_joints():
    """Test renaming logic for duplicate joint names."""
    urdf = """
    <robot name="dup_bot">
        <link name="l1"></link>
        <link name="l2"></link>
        <joint name="j1" type="fixed">
            <parent link="l1"/><child link="l2"/>
        </joint>
        <joint name="j1" type="fixed">
            <parent link="l1"/><child link="l2"/>
        </joint>
    </robot>
    """
    robot = URDFParser().parse_string(urdf)
    assert "j1" in robot._joint_index
    assert "j1_duplicate_1" in robot._joint_index


def test_urdf_parser_large_file_rejection():
    """Test rejection of oversized URDF files."""
    # Use configurable max size instead of patching
    parser = URDFParser(max_file_size=10)

    with pytest.raises(RobotParserError, match="URDF string too large"):
        parser.parse_string("a" * 100)


def test_is_suspicious_location_direct():
    """Test suspicious location detection directly."""
    # Test valid relative paths
    assert not is_suspicious_location(Path("meshes/box.stl"))
    assert not is_suspicious_location(Path("mesh.stl"))

    # Absolute paths to system directories are suspicious
    assert is_suspicious_location(Path("/etc/passwd"))
    assert is_suspicious_location(Path("/root/secret"))


def test_urdf_parser_xacro_unicode_error(tmp_path):
    """Test _detect_xacro_file handling UnicodeDecodeError using a real file."""
    bad_file = tmp_path / "test.urdf"
    # Write invalid UTF-8 bytes
    bad_file.write_bytes(b"\x80\x81\xff")

    # Should not raise ValueError (swallows UnicodeDecodeError and assumes not XACRO namespace)
    _detect_xacro_file(ET.Element("robot"), bad_file)


def test_urdf_parser_full_robot():
    """Test parsing a complete robot description using the main parser class."""
    xml_content = """<?xml version="1.0"?>
    <robot name="full_robot">
        <link name="base_link">
            <visual>
                <geometry><box size="1 1 1"/></geometry>
            </visual>
        </link>

        <link name="link1">
            <visual>
                <geometry><sphere radius="0.5"/></geometry>
            </visual>
        </link>

        <joint name="joint1" type="revolute">
            <parent link="base_link"/>
            <child link="link1"/>
            <axis xyz="0 0 1"/>
            <limit lower="-1" upper="1" effort="10" velocity="10"/>
        </joint>

        <transmission name="trans1">
            <type>transmission_interface/SimpleTransmission</type>
            <joint name="joint1">
                <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
            </joint>
        </transmission>

        <gazebo>
            <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
                <robotNamespace>/</robotNamespace>
            </plugin>
        </gazebo>
    </robot>
    """

    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as tmp:
        tmp.write(xml_content)
        tmp_path = Path(tmp.name)

    try:
        parser = URDFParser()
        robot = parser.parse(tmp_path)

        assert robot.name == "full_robot"
        assert len(robot.links) == 2
        assert len(robot.joints) == 1
        assert len(robot.transmissions) == 1
        assert len(robot.gazebo_elements) == 1

        # Verify parsed content
        assert robot.links[0].name == "base_link"
        assert robot.joints[0].name == "joint1"
        assert robot.transmissions[0].name == "trans1"
        assert robot.gazebo_elements[0].plugins[0].name == "gazebo_ros_control"

    finally:
        # Cleanup
        if tmp_path.exists():
            tmp_path.unlink()


def test_detect_xacro_rejection():
    """Test such that xacro files are rejected even if using URDFParser directly."""
    xml_content = """<?xml version="1.0"?>
    <robot xmlns:xacro="http://ros.org/wiki/xacro" name="xacro_bot">
        <xacro:property name="width" value="1.0"/>
        <link name="base"/>
    </robot>
    """

    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as tmp:
        tmp.write(xml_content)
        tmp_path = Path(tmp.name)

    try:
        parser = URDFParser()
        # Should raise ValueError or RobotParserError because XACRO namespace is present
        with pytest.raises((ValueError, RobotParserError), match="XACRO file detected"):
            parser.parse(tmp_path)

    finally:
        if tmp_path.exists():
            tmp_path.unlink()
