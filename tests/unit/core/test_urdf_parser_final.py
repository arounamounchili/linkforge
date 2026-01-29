"""Final coverage sweep for URDF parser, XACRO, and Security."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from linkforge_core.base import RobotParserError
from linkforge_core.parsers.urdf_parser import (
    URDFParser,
    parse_gazebo_element,
    parse_sensor_from_gazebo,
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


def test_parse_sensor_missing_inner_elements():
    """Test parsing sensors with missing type-specific elements."""
    # GPS missing <gps>
    xml = '<gazebo reference="l1"><sensor name="s1" type="navsat"></sensor></gazebo>'
    sensor = parse_sensor_from_gazebo(ET.fromstring(xml))
    assert sensor.gps_info is not None  # Returns default GPSInfo

    # IMU missing <imu>
    xml = '<gazebo reference="l1"><sensor name="s1" type="imu"></sensor></gazebo>'
    sensor = parse_sensor_from_gazebo(ET.fromstring(xml))
    assert sensor.imu_info is not None  # Returns default IMUInfo

    # Contact missing <contact> -> Should raise ValueError
    xml = '<gazebo reference="l1"><sensor name="s1" type="contact"></sensor></gazebo>'
    with pytest.raises(ValueError, match="missing required <contact> element"):
        parse_sensor_from_gazebo(ET.fromstring(xml))


def test_parse_gazebo_element_optional_float_empty():
    """Test _parse_optional_float with empty string."""
    xml = "<gazebo><mu1></mu1></gazebo>"
    ge = parse_gazebo_element(ET.fromstring(xml))
    assert ge.mu1 == 0.0  # Default if empty


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

    from linkforge_core.parsers.urdf_parser import _detect_xacro_file

    # Should not raise ValueError (swallows UnicodeDecodeError and assumes not XACRO namespace)
    _detect_xacro_file(ET.Element("robot"), bad_file)
