"""Test XACRO export completeness.

Verify that XACRO generator exports all URDF elements:
- Transmissions
- Sensors
- ROS2 Control
- Gazebo elements
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from linkforge_core import URDFGenerator, XACROGenerator
from linkforge_core.models import (
    Box,
    CameraInfo,
    Collision,
    Inertial,
    InertiaTensor,
    Joint,
    JointLimits,
    JointType,
    Link,
    Robot,
    Ros2Control,
    Sensor,
    SensorType,
    Transform,
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    Vector3,
    Visual,
)


@pytest.fixture
def simple_robot() -> Robot:
    """Create a simple robot with 2 links and 1 joint."""
    # Create links
    link1 = Link(
        name="base_link",
        visuals=[
            Visual(
                geometry=Box(size=Vector3(1.0, 1.0, 0.1)),
                origin=Transform.identity(),
            )
        ],
        collisions=[
            Collision(
                geometry=Box(size=Vector3(1.0, 1.0, 0.1)),
                origin=Transform.identity(),
            )
        ],
        inertial=Inertial(
            mass=1.0,
            origin=Transform.identity(),
            inertia=InertiaTensor(ixx=0.1, iyy=0.1, izz=0.1, ixy=0.0, ixz=0.0, iyz=0.0),
        ),
    )

    link2 = Link(
        name="link1",
        visuals=[
            Visual(
                geometry=Box(size=Vector3(0.5, 0.5, 0.5)),
                origin=Transform.identity(),
            )
        ],
        collisions=[
            Collision(
                geometry=Box(size=Vector3(0.5, 0.5, 0.5)),
                origin=Transform.identity(),
            )
        ],
        inertial=Inertial(
            mass=0.5,
            origin=Transform.identity(),
            inertia=InertiaTensor(ixx=0.05, iyy=0.05, izz=0.05, ixy=0.0, ixz=0.0, iyz=0.0),
        ),
    )

    # Create joint
    joint1 = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="link1",
        origin=Transform(xyz=Vector3(0.0, 0.0, 0.5), rpy=Vector3(0.0, 0.0, 0.0)),
        axis=Vector3(0.0, 0.0, 1.0),
        limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=1.0),
    )

    # Create robot
    robot = Robot(name="test_robot")
    robot.add_link(link1)
    robot.add_link(link2)
    robot.add_joint(joint1)

    return robot


def test_xacro_exports_transmissions(simple_robot: Robot, tmp_path: Path) -> None:
    """Test that XACRO generator exports transmissions."""
    # Add transmission to robot
    transmission = Transmission(
        name="trans1",
        type="transmission_interface/SimpleTransmission",
        joints=[
            TransmissionJoint(
                name="joint1",
                hardware_interfaces=["position"],
                mechanical_reduction=1.0,
            )
        ],
        actuators=[
            TransmissionActuator(
                name="motor1",
                hardware_interfaces=["position"],
                mechanical_reduction=1.0,
            )
        ],
    )
    simple_robot.add_transmission(transmission)

    # Generate XACRO
    generator = XACROGenerator(advanced_mode=False)
    xacro_str = generator.generate(simple_robot)

    # Parse XML
    root = ET.fromstring(xacro_str)

    # Find transmission element
    transmissions = root.findall("transmission")
    assert len(transmissions) == 1, "XACRO should export transmission"
    assert transmissions[0].get("name") == "trans1"

    # Verify transmission content
    trans_type = transmissions[0].find("type")
    assert trans_type is not None
    assert trans_type.text == "transmission_interface/SimpleTransmission"

    # Verify joint
    trans_joint = transmissions[0].find("joint")
    assert trans_joint is not None
    assert trans_joint.get("name") == "joint1"


def test_xacro_exports_sensors(simple_robot: Robot, tmp_path: Path) -> None:
    """Test that XACRO generator exports sensors."""
    # Add camera sensor to robot
    sensor = Sensor(
        name="camera1",
        type=SensorType.CAMERA,
        link_name="link1",
        update_rate=30.0,
        camera_info=CameraInfo(
            horizontal_fov=1.047,
            width=640,
            height=480,
            format="R8G8B8",
        ),
    )
    simple_robot.add_sensor(sensor)

    # Generate XACRO
    generator = XACROGenerator(advanced_mode=False)
    xacro_str = generator.generate(simple_robot)

    # Parse XML
    root = ET.fromstring(xacro_str)

    # Find gazebo element with sensor
    gazebo_elems = root.findall("gazebo")
    sensor_gazebo = None
    for gz in gazebo_elems:
        if gz.get("reference") == "link1":
            sensor_elem = gz.find("sensor")
            if sensor_elem is not None:
                sensor_gazebo = sensor_elem
                break

    assert sensor_gazebo is not None, "XACRO should export sensor in gazebo tag"
    assert sensor_gazebo.get("name") == "camera1"
    assert sensor_gazebo.get("type") == "camera"

    # Verify camera info
    camera_elem = sensor_gazebo.find("camera")
    assert camera_elem is not None
    hfov = camera_elem.find("horizontal_fov")
    assert hfov is not None
    assert float(hfov.text) == pytest.approx(1.047)


def test_xacro_exports_ros2_control_from_transmissions(simple_robot: Robot, tmp_path: Path) -> None:
    """Test that XACRO generator exports ros2_control from transmissions."""
    # Add transmission to robot
    transmission = Transmission(
        name="trans1",
        type="transmission_interface/SimpleTransmission",
        joints=[
            TransmissionJoint(
                name="joint1",
                hardware_interfaces=["position"],
                mechanical_reduction=1.0,
            )
        ],
        actuators=[
            TransmissionActuator(
                name="motor1",
                hardware_interfaces=["position"],
                mechanical_reduction=1.0,
            )
        ],
    )
    simple_robot.add_transmission(transmission)

    # Generate XACRO with ros2_control enabled
    generator = XACROGenerator(advanced_mode=False, use_ros2_control=True)
    xacro_str = generator.generate(simple_robot)

    # Parse XML
    root = ET.fromstring(xacro_str)

    # Find ros2_control element
    ros2_control_elems = root.findall("ros2_control")
    assert len(ros2_control_elems) == 1, "XACRO should export ros2_control"

    rc = ros2_control_elems[0]
    assert rc.get("name") == "GazeboSimSystem"
    assert rc.get("type") == "system"

    # Verify hardware
    hardware = rc.find("hardware")
    assert hardware is not None

    # Verify joint
    joints = rc.findall("joint")
    assert len(joints) == 1
    assert joints[0].get("name") == "joint1"


def test_xacro_urdf_parity(simple_robot: Robot, tmp_path: Path) -> None:
    """Test that XACRO and URDF exports have the same elements."""
    # Add transmission, sensor to robot
    transmission = Transmission(
        name="trans1",
        type="transmission_interface/SimpleTransmission",
        joints=[
            TransmissionJoint(
                name="joint1",
                hardware_interfaces=["position"],
                mechanical_reduction=1.0,
            )
        ],
        actuators=[
            TransmissionActuator(
                name="motor1",
                hardware_interfaces=["position"],
                mechanical_reduction=1.0,
            )
        ],
    )
    sensor = Sensor(
        name="camera1",
        type=SensorType.CAMERA,
        link_name="link1",
        update_rate=30.0,
        camera_info=CameraInfo(
            horizontal_fov=1.047,
            width=640,
            height=480,
        ),
    )
    simple_robot.add_transmission(transmission)
    simple_robot.add_sensor(sensor)

    # Generate URDF
    urdf_gen = URDFGenerator(use_ros2_control=True)
    urdf_str = urdf_gen.generate(simple_robot)
    urdf_root = ET.fromstring(urdf_str)

    # Generate XACRO (without advanced features for fair comparison)
    xacro_gen = XACROGenerator(advanced_mode=False, use_ros2_control=True)
    xacro_str = xacro_gen.generate(simple_robot)
    xacro_root = ET.fromstring(xacro_str)

    # Compare element counts (ignoring xacro namespace attributes)
    urdf_links = urdf_root.findall("link")
    xacro_links = xacro_root.findall("link")
    assert len(urdf_links) == len(xacro_links), "Same number of links"

    urdf_joints = urdf_root.findall("joint")
    xacro_joints = xacro_root.findall("joint")
    assert len(urdf_joints) == len(xacro_joints), "Same number of joints"

    urdf_transmissions = urdf_root.findall("transmission")
    xacro_transmissions = xacro_root.findall("transmission")
    assert len(urdf_transmissions) == len(xacro_transmissions), "Same number of transmissions"

    urdf_ros2_control = urdf_root.findall("ros2_control")
    xacro_ros2_control = xacro_root.findall("ros2_control")
    assert len(urdf_ros2_control) == len(xacro_ros2_control), "Same number of ros2_control blocks"

    # Count sensors (in gazebo tags)
    urdf_sensors = 0
    for gz in urdf_root.findall("gazebo"):
        if gz.find("sensor") is not None:
            urdf_sensors += 1

    xacro_sensors = 0
    for gz in xacro_root.findall("gazebo"):
        if gz.find("sensor") is not None:
            xacro_sensors += 1

    assert urdf_sensors == xacro_sensors, "Same number of sensors"


def test_xacro_without_ros2_control(simple_robot: Robot, tmp_path: Path) -> None:
    """Test that XACRO generator respects use_ros2_control=False."""
    # Add transmission to robot
    transmission = Transmission(
        name="trans1",
        type="transmission_interface/SimpleTransmission",
        joints=[
            TransmissionJoint(
                name="joint1",
                hardware_interfaces=["position"],
                mechanical_reduction=1.0,
            )
        ],
        actuators=[
            TransmissionActuator(
                name="motor1",
                hardware_interfaces=["position"],
                mechanical_reduction=1.0,
            )
        ],
    )
    simple_robot.add_transmission(transmission)

    # Generate XACRO with ros2_control disabled
    generator = XACROGenerator(advanced_mode=False, use_ros2_control=False)
    xacro_str = generator.generate(simple_robot)

    # Parse XML
    root = ET.fromstring(xacro_str)

    # Verify transmission is exported
    transmissions = root.findall("transmission")
    assert len(transmissions) == 1

    # Verify ros2_control is NOT exported
    ros2_control_elems = root.findall("ros2_control")
    assert len(ros2_control_elems) == 0, "ros2_control should not be exported when disabled"


def test_xacro_suppresses_existing_ros2_control(simple_robot: Robot, tmp_path: Path) -> None:
    """Test that XACRO generator suppresses EXISTING ros2_control when disabled."""
    # Add ros2_control to robot manually
    rc = Ros2Control(
        name="existing_control",
        type="system",
        hardware_plugin="some_plugin",
        joints=[],
    )
    simple_robot.add_ros2_control(rc)

    # Generate XACRO with ros2_control disabled
    generator = XACROGenerator(advanced_mode=False, use_ros2_control=False)
    xacro_str = generator.generate(simple_robot)

    # Parse XML
    root = ET.fromstring(xacro_str)

    # Verify ros2_control is NOT exported
    ros2_control_elems = root.findall("ros2_control")
    assert len(ros2_control_elems) == 0, (
        "Existing ros2_control should be suppressed when use_ros2_control=False"
    )
