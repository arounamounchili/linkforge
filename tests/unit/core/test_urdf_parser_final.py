"""Final coverage sweep for URDF parser, XACRO, and Security."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from linkforge.core.parsers.urdf_parser import (
    parse_gazebo_element,
    parse_sensor_from_gazebo,
    parse_urdf_string,
)
from linkforge.core.validation.security import validate_mesh_path


def test_urdf_parser_duplicate_links():
    """Test renaming logic for duplicate link names."""
    urdf = """
    <robot name="dup_bot">
        <link name="base_link"></link>
        <link name="base_link"></link>
    </robot>
    """
    robot = parse_urdf_string(urdf)
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
    robot = parse_urdf_string(urdf)
    assert "j1" in robot._joint_index
    assert "j1_duplicate_1" in robot._joint_index


def test_urdf_parser_large_file_rejection():
    """Test rejection of oversized URDF files."""
    with (
        patch("linkforge.core.parsers.urdf_parser.MAX_FILE_SIZE", 10),
        pytest.raises(ValueError, match="URDF string too large"),
    ):
        parse_urdf_string("a" * 100)


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


def test_security_validate_mesh_path_suspicious_success():
    """Test is_suspicious=True branch in validate_mesh_path."""
    # We need to make relative_to(suspicious) succeed.
    # We mock it to return a dummy path (which counts as success)
    with (
        patch("pathlib.Path.relative_to", return_value=Path("foo")),
        pytest.raises(ValueError, match="restricted system location"),
    ):
        validate_mesh_path(Path("meshes/box.stl"), Path("/tmp"), allow_absolute=True)


def test_urdf_parser_xacro_unicode_error():
    """Test _detect_xacro_file handling UnicodeDecodeError."""
    mock_path = MagicMock(spec=Path)
    mock_path.suffix = ".urdf"
    mock_path.name = "test.urdf"
    mock_path.read_text.side_effect = UnicodeDecodeError("utf-8", b"", 0, 1, "error")

    from linkforge.core.parsers.urdf_parser import _detect_xacro_file

    # Should not raise ValueError (swallows UnicodeDecodeError and assumes not XACRO namespace)
    _detect_xacro_file(ET.Element("robot"), mock_path)
