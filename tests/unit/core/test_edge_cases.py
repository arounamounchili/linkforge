"""Validation of complex edge cases and stress tests for LinkForge Core.

This suite verifies the behavior of generators and parsers when encountering
non-standard inputs, deep naming collisions, and specialized sensor configurations.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from linkforge.core import URDFGenerator
from linkforge.core.models import (
    Box,
    Color,
    GazeboElement,
    GazeboPlugin,
    GPSInfo,
    Joint,
    JointType,
    Link,
    Material,
    Mesh,
    Robot,
    Sensor,
    SensorNoise,
    SensorType,
    Transform,
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    Vector3,
    Visual,
)
from linkforge.core.parsers.urdf_parser import (
    parse_sensor_from_gazebo,
    parse_urdf_string,
)
from linkforge.core.validation.security import validate_package_uri


def test_security_uri_validation():
    """Verify robust handling of malformed or malicious package URIs."""
    # Ensure non-package schemes are rejected
    with pytest.raises(ValueError, match="must start with 'package://'"):
        validate_package_uri("http://bogus")

    # Verify path traversal detection in package strings
    with pytest.raises(ValueError, match="Path traversal detected"):
        validate_package_uri("package://pkg/../etc")

    # Validation of package name presence
    with pytest.raises(ValueError, match="missing package name"):
        validate_package_uri("package://")

    # Detection of suspicious relative components
    with pytest.raises(ValueError, match="suspicious path components"):
        validate_package_uri("package://pkg/./test")


def test_urdf_generator_complex_scenarios():
    """Validate URDF generation for advanced sensor and material configurations."""
    robot = Robot(name="stress_test")
    robot.add_link(Link(name="base"))
    robot.add_link(Link(name="l1"))
    robot.add_joint(Joint(name="j1", parent="base", child="l1", type=JointType.FIXED))

    # Verification of inline material serialization for unnamed materials
    mat_inline = Material(name=None, color=Color(1, 0, 0, 1), texture="metal.png")
    robot.add_link(
        Link(name="l2", visuals=[Visual(geometry=Box(Vector3(1, 1, 1)), material=mat_inline)])
    )
    robot.add_joint(Joint(name="j2", parent="base", child="l2", type=JointType.FIXED))

    # Fallback to absolute paths when mesh relativization is cross-volume or invalid
    abs_mesh = Path("/different/volume/mesh.stl")
    robot.add_link(Link(name="l3", visuals=[Visual(geometry=Mesh(filepath=abs_mesh))]))
    robot.add_joint(Joint(name="j3", parent="base", child="l3", type=JointType.FIXED))

    # Serialization of actuator parameters and offsets
    trans = Transmission(
        name="t",
        type="simple",
        joints=[TransmissionJoint(name="j1")],
        actuators=[TransmissionActuator(name="a", offset=0.1)],
    )
    robot.add_transmission(trans)

    # Validation of asymmetric sensor noise profiles
    gps = GPSInfo(position_sensing_vertical_noise=SensorNoise(stddev=0.1, bias_mean=0.01))
    s_gps = Sensor(
        name="gps",
        type=SensorType.GPS,
        link_name="l1",
        gps_info=gps,
        origin=Transform(xyz=Vector3(1, 1, 1)),
    )
    robot.add_sensor(s_gps)

    # Stress test: Gazebo plugins with direct XML payload injection
    p = GazeboPlugin(name="p", filename="f", raw_xml="<data>123</data>")
    robot.gazebo_elements.append(GazeboElement(plugins=[p]))

    gen = URDFGenerator(urdf_path=Path("/tmp/robot.urdf"))
    xml = gen.generate(robot)
    assert 'filename="metal.png"' in xml
    assert "<bias_mean>0.01</bias_mean>" in xml


def test_parser_collision_resolution():
    """Verify that name collisions during parsing are resolved via iterative renaming."""
    # Stress test: Multiple identical names requiring incremental suffixes
    xml = """
    <robot name="r">
        <link name="l1"/><link name="l1_duplicate_1"/><link name="l1_duplicate_2"/><link name="l1"/>
        <joint name="j1" type="fixed"><parent link="l1"/><child link="l1_duplicate_1"/></joint>
        <joint name="j1_duplicate_1" type="fixed"><parent link="l1"/><child link="l1_duplicate_1"/></joint>
        <joint name="j1" type="fixed"><parent link="l1"/><child link="l1_duplicate_1"/></joint>
    </robot>
    """
    r = parse_urdf_string(xml)
    # Ensure the 4th 'l1' gets correctly suffixed past existing duplicates
    assert "l1_duplicate_3" in r._link_index
    assert "j1_duplicate_2" in r._joint_index


def test_specialized_sensor_parsing():
    """Verify parsing of specialized Force/Torque and Gazebo-specific sensor extensions."""
    xml_ft = """
    <gazebo reference="l1">
        <sensor name="ft" type="force_torque">
            <force_torque>
                <frame>child</frame>
                <measure_direction>child_to_parent</measure_direction>
                <noise type="gaussian"><stddev>0.1</stddev></noise>
            </force_torque>
        </sensor>
    </gazebo>
    """
    s_ft = parse_sensor_from_gazebo(ET.fromstring(xml_ft))
    assert s_ft.force_torque_info.frame == "child"
    assert s_ft.force_torque_info.noise.stddev == 0.1
