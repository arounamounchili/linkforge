"""Tests for detailed URDF parser features (sensors, meshes, transmissions)."""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from pathlib import Path

from linkforge_core.parsers.urdf_parser import (
    parse_gazebo_element,
    parse_geometry,
    parse_joint,
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

    # If we provide a directory, the parser calls validate_mesh_path
    # which resolves it to an absolute path.
    urdf_dir = Path("/tmp")
    geom = parse_geometry(geom_elem, urdf_directory=urdf_dir)

    expected = (urdf_dir / "relative/path.stl").resolve()
    assert geom.filepath == expected


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
    elem = ET.fromstring(trans_xml.strip())
    trans = parse_transmission(elem)

    assert trans.joints[0].hardware_interfaces == ["position"]
    assert trans.actuators[0].hardware_interfaces == ["position"]


def test_parse_transmission_complex():
    """Test parsing detailed transmission element."""
    xml = """
    <transmission name="trans1">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="joint1">
            <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        </joint>
        <actuator name="motor1">
            <mechanicalReduction>50</mechanicalReduction>
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </actuator>
    </transmission>
    """
    elem = ET.fromstring(xml.strip())
    trans = parse_transmission(elem)

    assert trans.name == "trans1"
    assert len(trans.joints) == 1
    assert len(trans.actuators) == 1
    # Check normalized interfaces
    assert "position" in trans.joints[0].hardware_interfaces
    assert "effort" in trans.actuators[0].hardware_interfaces


def test_parse_ros2_control_parameters():
    """Test parsing extra parameters in ros2_control."""
    rc_xml = """
    <ros2_control name="system" type="system">
        <hardware><plugin>mock_hw</plugin></hardware>
        <joint name="j1"><command_interface name="pos"/><state_interface name="pos"/></joint>
        <extra_param>42</extra_param>
    </ros2_control>
    """
    elem = ET.fromstring(rc_xml.strip())
    rc = parse_ros2_control(elem)
    assert rc.parameters["extra_param"] == "42"


def test_parse_ros2_control_hardware_params():
    """Test parsing ros2_control with hardware parameters."""
    xml = """
    <ros2_control name="TestSystem" type="system">
        <hardware>
            <plugin>test_driver/TestHardware</plugin>
            <param name="usb_port">/dev/ttyUSB0</param>
            <param name="baud_rate">115200</param>
        </hardware>
        <joint name="joint1">
            <command_interface name="position"/>
            <state_interface name="position"/>
        </joint>
    </ros2_control>
    """
    elem = ET.fromstring(xml.strip())
    rc = parse_ros2_control(elem)

    assert rc.name == "TestSystem"
    assert rc.hardware_plugin == "test_driver/TestHardware"
    assert rc.parameters["hardware.usb_port"] == "/dev/ttyUSB0"
    assert rc.parameters["hardware.baud_rate"] == "115200"
    assert len(rc.joints) == 1
    assert "position" in rc.joints[0].command_interfaces


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
    elem = ET.fromstring(link_xml.strip())
    link = parse_link(elem, {})
    assert len(link.visuals) == 2
    assert link.visuals[0].name == "v1"
    assert link.visuals[1].name == "v2"


def test_parse_link_full():
    """Test parsing a complete link with visual, collision, and inertial."""
    xml = """
    <link name="base_link">
        <inertial>
            <mass value="5.0"/>
            <origin xyz="0 0 0.1" rpy="0 0 0"/>
            <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
        </inertial>
        <visual name="base_visual">
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <box size="1 1 1"/>
            </geometry>
            <material name="Blue">
                <color rgba="0 0 1 1"/>
            </material>
        </visual>
        <collision name="base_collision">
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <cylinder length="1.0" radius="0.5"/>
            </geometry>
        </collision>
    </link>
    """
    elem = ET.fromstring(xml.strip())
    # Pass empty materials dict and dummy path
    link = parse_link(elem, {}, Path("."))

    assert link.name == "base_link"
    assert link.inertial is not None
    assert link.inertial.mass == 5.0

    assert len(link.visuals) == 1
    assert link.visuals[0].name == "base_visual"
    assert link.visuals[0].material is not None
    assert link.visuals[0].material.name == "Blue"

    assert len(link.collisions) == 1
    assert link.collisions[0].name == "base_collision"


def test_parse_inertia_sanitization(caplog):
    """Test that invalid inertia moments are sanitized and warned."""
    xml = """
    <link name="unstable_link">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0" ixy="0" ixz="0" iyy="-0.1" iyz="0" izz="0.0"/>
        </inertial>
    </link>
    """
    elem = ET.fromstring(xml.strip())

    with caplog.at_level(logging.WARNING):
        link = parse_link(elem, {}, Path("."))

    assert link.inertial is not None
    assert link.inertial.inertia.ixx == 1e-6
    assert link.inertial.inertia.iyy == 1e-6
    assert "Sanitizing invalid inertia" in caplog.text


def test_parse_gazebo_reference():
    """Test parsing <gazebo reference='link'>."""
    xml = """
    <robot name="test">
        <gazebo reference="base_link">
            <material>Gazebo/Red</material>
            <mu1>0.5</mu1>
        </gazebo>
    </robot>
    """
    elem = ET.fromstring(xml.strip()).find("gazebo")
    element = parse_gazebo_element(elem)

    assert element.reference == "base_link"
    assert element.material == "Gazebo/Red"
    assert element.mu1 == 0.5


def test_parse_joint_dynamics_mimic():
    """Test parsing joint dynamics and mimic."""
    xml = """
    <joint name="mimic_joint" type="revolute">
        <parent link="p"/>
        <child link="c"/>
        <limit effort="10" velocity="10" lower="0" upper="1"/>
        <dynamics damping="0.5" friction="0.1"/>
        <mimic joint="driver_joint" multiplier="2.0" offset="0.5"/>
    </joint>
    """
    elem = ET.fromstring(xml.strip())
    joint = parse_joint(elem)

    assert joint.dynamics is not None
    assert joint.dynamics.damping == 0.5
    assert joint.dynamics.friction == 0.1

    assert joint.mimic is not None
    assert joint.mimic.joint == "driver_joint"
    assert joint.mimic.multiplier == 2.0
    assert joint.mimic.offset == 0.5
