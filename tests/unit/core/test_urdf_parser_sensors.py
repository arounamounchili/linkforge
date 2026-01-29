"""Tests for URDF parser sensor and Gazebo features."""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest
from linkforge_core.models import (
    SensorType,
)
from linkforge_core.parsers.urdf_parser import (
    parse_sensor_from_gazebo,
    parse_sensor_noise,
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
        elem = ET.fromstring(xml.strip())
        sensor = parse_sensor_from_gazebo(elem)
        assert sensor is not None
        assert sensor.type == internal_type
        assert sensor.topic == "/test_topic"
        assert sensor.origin.xyz.x == 1.0


def test_parse_sensor_noise_details():
    """Test parsing detailed sensor noise parameters."""
    xml = """
    <noise type="gaussian">
        <mean>0.1</mean>
        <stddev>0.05</stddev>
    </noise>
    """
    elem = ET.fromstring(xml.strip())
    noise = parse_sensor_noise(elem)
    assert noise is not None
    assert noise.mean == 0.1
    assert noise.stddev == 0.05

    """Test parsing Lidar sensor with minimal tags (defaults)."""
    xml = """
    <gazebo reference="link1">
        <sensor type="ray" name="lidar_sensor">
            <ray>
                <scan>
                    <horizontal>
                        <samples>100</samples>
                    </horizontal>
                </scan>
            </ray>
        </sensor>
    </gazebo>
    """
    elem = ET.fromstring(xml.strip())
    sensor = parse_sensor_from_gazebo(elem)

    assert sensor is not None
    assert sensor.type == SensorType.LIDAR
    assert sensor.lidar_info is not None
    assert sensor.lidar_info.horizontal_min_angle == -1.570796
    assert sensor.lidar_info.range_min == 0.1


def test_parse_lidar_full():
    """Test parsing fully specified Lidar sensor."""
    xml = """
    <gazebo reference="link1">
        <sensor type="ray" name="lidar_full">
            <ray>
                <scan>
                    <horizontal>
                        <samples>720</samples>
                        <min_angle>-3.14</min_angle>
                        <max_angle>3.14</max_angle>
                    </horizontal>
                </scan>
                <range>
                    <min>0.5</min>
                    <max>50.0</max>
                </range>
            </ray>
        </sensor>
    </gazebo>
    """
    elem = ET.fromstring(xml.strip())
    sensor = parse_sensor_from_gazebo(elem)

    assert sensor is not None
    assert sensor.lidar_info is not None
    assert sensor.lidar_info.horizontal_samples == 720
    assert sensor.lidar_info.range_max == 50.0


def test_parse_camera_full():
    """Test parsing Camera sensor."""
    xml = """
    <gazebo reference="link1">
        <sensor type="camera" name="cam1">
            <camera>
                <horizontal_fov>1.5</horizontal_fov>
                <image>
                    <width>1920</width>
                    <height>1080</height>
                    <format>R8G8B8</format>
                </image>
                <clip>
                    <near>0.5</near>
                    <far>500.0</far>
                </clip>
            </camera>
        </sensor>
    </gazebo>
    """
    elem = ET.fromstring(xml.strip())
    sensor = parse_sensor_from_gazebo(elem)

    assert sensor is not None
    assert sensor.type == SensorType.CAMERA
    assert sensor.camera_info is not None
    assert sensor.camera_info.width == 1920
    assert sensor.camera_info.height == 1080
    assert sensor.camera_info.near_clip == 0.5
    assert sensor.camera_info.far_clip == 500.0


def test_parse_imu():
    """Test parsing IMU sensor."""
    xml = """
    <gazebo reference="link1">
        <sensor type="imu" name="imu1">
            <imu>
                <angular_velocity>
                    <x>
                        <noise type="gaussian">
                            <mean>0.0</mean>
                            <stddev>0.01</stddev>
                        </noise>
                    </x>
                </angular_velocity>
                <linear_acceleration>
                    <x>
                        <noise type="gaussian">
                            <mean>0.0</mean>
                            <stddev>0.1</stddev>
                        </noise>
                    </x>
                </linear_acceleration>
            </imu>
        </sensor>
    </gazebo>
    """
    elem = ET.fromstring(xml.strip())
    sensor = parse_sensor_from_gazebo(elem)

    assert sensor is not None
    assert sensor.type == SensorType.IMU
    assert sensor.imu_info is not None
    assert sensor.imu_info.angular_velocity_noise is not None
    assert sensor.imu_info.angular_velocity_noise.mean == 0.0
    assert sensor.imu_info.linear_acceleration_noise.stddev == 0.1


def test_parse_gps():
    """Test parsing GPS sensor."""
    xml = """
    <gazebo reference="link1">
        <sensor type="gps" name="gps1">
            <gps>
                <position_sensing>
                    <horizontal>
                        <noise type="gaussian_quantized">
                            <mean>0.0</mean>
                            <stddev>2.0</stddev>
                        </noise>
                    </horizontal>
                    <vertical>
                        <noise type="gaussian">
                            <mean>0.0</mean>
                            <stddev>4.0</stddev>
                        </noise>
                    </vertical>
                </position_sensing>
            </gps>
        </sensor>
    </gazebo>
    """
    elem = ET.fromstring(xml.strip())
    sensor = parse_sensor_from_gazebo(elem)

    assert sensor is not None
    assert sensor.type == SensorType.GPS
    assert sensor.gps_info is not None
    assert sensor.gps_info.position_sensing_horizontal_noise.stddev == 2.0
    assert sensor.gps_info.position_sensing_vertical_noise.stddev == 4.0


def test_parse_contact():
    """Test parsing Contact sensor."""
    xml = """
    <gazebo reference="link1">
        <sensor type="contact" name="bumper">
            <contact>
                <collision>bumper_collision</collision>
            </contact>
        </sensor>
    </gazebo>
    """
    elem = ET.fromstring(xml.strip())
    sensor = parse_sensor_from_gazebo(elem)

    assert sensor is not None
    assert sensor.type == SensorType.CONTACT
    assert sensor.contact_info is not None
    assert sensor.contact_info.collision == "bumper_collision"


def test_parse_contact_missing_collision():
    """Test parsing Contact sensor with missing collision tag."""
    xml = """
    <gazebo reference="link1">
        <sensor type="contact" name="bumper_bad">
            <contact/>
        </sensor>
    </gazebo>
    """
    elem = ET.fromstring(xml.strip())
    with pytest.raises(ValueError, match="missing required <collision>"):
        parse_sensor_from_gazebo(elem)


def test_parse_force_torque():
    """Test parsing Force/Torque sensor."""
    xml = """
    <gazebo reference="link1">
        <sensor type="force_torque" name="ft_sensor">
            <force_torque>
                <frame>sensor</frame>
                <measure_direction>child_to_parent</measure_direction>
            </force_torque>
        </sensor>
    </gazebo>
    """
    elem = ET.fromstring(xml.strip())
    sensor = parse_sensor_from_gazebo(elem)

    assert sensor is not None
    assert sensor.type == SensorType.FORCE_TORQUE
    assert sensor.force_torque_info is not None
    assert sensor.force_torque_info.frame == "sensor"


def test_parse_sensor_missing_inner_elements():
    """Test parsing sensors with missing type-specific elements."""
    # GPS missing <gps>
    xml = '<gazebo reference="l1"><sensor name="s1" type="navsat"></sensor></gazebo>'
    elem = ET.fromstring(xml.strip())
    sensor = parse_sensor_from_gazebo(elem)
    assert sensor.gps_info is not None  # Returns default GPSInfo

    # IMU missing <imu>
    xml = '<gazebo reference="l1"><sensor name="s1" type="imu"></sensor></gazebo>'
    elem = ET.fromstring(xml.strip())
    sensor = parse_sensor_from_gazebo(elem)
    assert sensor.imu_info is not None  # Returns default IMUInfo
