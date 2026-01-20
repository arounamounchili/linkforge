"""Tests for URDF parser sensor and Gazebo features."""

from __future__ import annotations

import xml.etree.ElementTree as ET

from linkforge.core.models import SensorType
from linkforge.core.parsers.urdf_parser import (
    parse_sensor_from_gazebo,
)


def test_parse_all_sensor_types_from_gazebo():
    """Test parsing every sensor type from Gazebo XML."""
    sensor_types = [
        ("gpu_lidar", SensorType.LIDAR, ""),
        ("navsat", SensorType.GPS, "<gps/>"),
        ("camera", SensorType.CAMERA, ""),
        ("depth_camera", SensorType.DEPTH_CAMERA, ""),
        ("imu", SensorType.IMU, "<imu/>"),
        ("contact", SensorType.CONTACT, "<contact><collision>c1</collision></contact>"),
        ("force_torque", SensorType.FORCE_TORQUE, "<force_torque/>"),
    ]

    for sim_type, internal_type, extra_xml in sensor_types:
        xml = f"""
        <gazebo reference="link1">
            <sensor name="my_sensor" type="{sim_type}">
                <always_on>true</always_on>
                <update_rate>30</update_rate>
                <visualize>true</visualize>
                <topic>/test_topic</topic>
                <pose>1 2 3 0 0 0</pose>
                {extra_xml}
            </sensor>
        </gazebo>
        """
        elem = ET.fromstring(xml)
        sensor = parse_sensor_from_gazebo(elem)
        assert sensor is not None
        assert sensor.type == internal_type
        assert sensor.topic == "/test_topic"
        assert sensor.origin.xyz.x == 1.0


def test_parse_sensor_noise_details():
    """Test parsing detailed sensor noise parameters."""
    from linkforge.core.parsers.urdf_parser import parse_sensor_noise

    xml = """
    <noise type="gaussian">
        <mean>0.1</mean>
        <stddev>0.05</stddev>
    </noise>
    """
    elem = ET.fromstring(xml)
    noise = parse_sensor_noise(elem)
    assert noise.mean == 0.1
    assert noise.stddev == 0.05


def test_parse_ros2_control_joint_interfaces():
    """Test parsing ros2_control with various joint interfaces."""
    from linkforge.core.parsers.urdf_parser import parse_ros2_control

    xml = """
    <ros2_control name="test" type="system">
        <hardware><plugin>mock</plugin></hardware>
        <joint name="j1">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>
    </ros2_control>
    """
    elem = ET.fromstring(xml)
    rc = parse_ros2_control(elem)
    assert len(rc.joints) == 1
    assert "velocity" in rc.joints[0].command_interfaces
    assert "effort" in rc.joints[0].state_interfaces
