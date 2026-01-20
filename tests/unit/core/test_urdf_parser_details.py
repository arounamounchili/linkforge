"""Tests for detailed URDF parser features (sensors, meshes, transmissions)."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

from linkforge.core.parsers.urdf_parser import (
    parse_geometry,
    parse_link,
    parse_ros2_control,
    parse_sensor_noise,
    parse_transmission,
)


def test_parse_geometry_mesh_uris():
    """Test parsing meshes with package:// and file:// URIs."""
    geom_elem = ET.Element("geometry")
    mesh = ET.SubElement(geom_elem, "mesh")

    # package:// URI
    mesh.set("filename", "package://my_robot/meshes/base.stl")
    geom = parse_geometry(geom_elem)
    assert geom.filepath == Path("package://my_robot/meshes/base.stl")

    # file:// URI
    mesh.set("filename", "file:///abs/path/mesh.stl")
    geom = parse_geometry(geom_elem)
    assert geom.filepath == Path("/abs/path/mesh.stl")


def test_parse_geometry_validation_calls():
    """Test that validation functions are called for mesh paths."""
    geom_elem = ET.Element("geometry")
    mesh = ET.SubElement(geom_elem, "mesh")
    mesh.set("filename", "relative/path.stl")

    with patch("linkforge.core.parsers.urdf_parser.validate_mesh_path") as mock_val:
        parse_geometry(geom_elem, urdf_directory=Path("/tmp"))
        mock_val.assert_called_once()


def test_parse_transmission_defaults():
    """Test transmission parsing with default interfaces."""
    trans_xml = """
    <transmission name="trans1">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="joint1">
            <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        </joint>
        <actuator name="actuator1" />
    </transmission>
    """
    elem = ET.fromstring(trans_xml)
    trans = parse_transmission(elem)

    assert trans.joints[0].hardware_interfaces == ["position"]
    assert trans.actuators[0].hardware_interfaces == ["position"]


def test_parse_ros2_control_parameters():
    """Test parsing extra parameters in ros2_control."""
    rc_xml = """
    <ros2_control name="system" type="system">
        <hardware><plugin>mock_hw</plugin></hardware>
        <joint name="j1"><command_interface name="pos"/><state_interface name="pos"/></joint>
        <extra_param>42</extra_param>
    </ros2_control>
    """
    elem = ET.fromstring(rc_xml)
    rc = parse_ros2_control(elem)
    assert rc.parameters["extra_param"] == "42"


def test_parse_sensor_noise_none():
    """Test sensor noise parsing edge cases."""
    assert parse_sensor_noise(None) is None

    elem = ET.Element("sensor")
    assert parse_sensor_noise(elem) is None


def test_parse_link_multi_visual():
    """Test parsing links with multiple visual elements."""
    link_xml = """
    <link name="link1">
        <visual name="v1"><geometry><box size="1 1 1"/></geometry></visual>
        <visual name="v2"><geometry><sphere radius="0.5"/></geometry></visual>
    </link>
    """
    elem = ET.fromstring(link_xml)
    link = parse_link(elem, {})
    assert len(link.visuals) == 2
    assert link.visuals[0].name == "v1"
    assert link.visuals[1].name == "v2"
