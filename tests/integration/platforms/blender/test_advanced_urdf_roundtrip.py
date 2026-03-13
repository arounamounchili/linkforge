"""Test round-trip for advanced URDF elements (sensors, transmissions, Gazebo)."""

from __future__ import annotations

import pytest
from linkforge_core import URDFGenerator
from linkforge_core.models import (
    Box,
    GazeboElement,
    GazeboPlugin,
    Inertial,
    InertiaTensor,
    Joint,
    JointLimits,
    JointType,
    Link,
    Robot,
    Transform,
    Transmission,
    Vector3,
    Visual,
)
from linkforge_core.parsers.urdf_parser import URDFParser


class TestTransmissionRoundtrip:
    """Test round-trip for transmission elements."""

    def test_simple_transmission_roundtrip(self):
        """Test that simple transmission survives round-trip."""
        # Create robot with transmission
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))
        robot.add_link(Link(name="arm_link"))
        robot.add_joint(
            Joint(
                name="arm_joint",
                type=JointType.REVOLUTE,
                parent="base_link",
                child="arm_link",
                axis=Vector3(0, 0, 1),
                limits=JointLimits(lower=-1.57, upper=1.57, velocity=2.0, effort=100.0),
            )
        )

        # Add simple transmission
        trans = Transmission.create_simple(
            name="arm_trans",
            joint_name="arm_joint",
            mechanical_reduction=50.0,
            hardware_interface="effort",
        )
        robot.add_transmission(trans)

        # Generate URDF
        generator = URDFGenerator()
        urdf_string = generator.generate(robot)

        # Parse back
        parsed_robot = URDFParser().parse_string(urdf_string)

        # Verify transmission
        assert len(parsed_robot.transmissions) == 1
        parsed_trans = parsed_robot.transmissions[0]
        assert parsed_trans.name == "arm_trans"
        assert len(parsed_trans.joints) == 1
        assert parsed_trans.joints[0].name == "arm_joint"
        assert parsed_trans.joints[0].mechanical_reduction == 50.0
        assert "effort" in parsed_trans.joints[0].hardware_interfaces

    def test_differential_transmission_roundtrip(self):
        """Test that differential transmission survives round-trip."""
        # Create robot with differential transmission
        robot = Robot(name="diff_robot")
        robot.add_link(Link(name="base_link"))
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="base_link",
                child="link1",
                axis=Vector3(0, 0, 1),
                limits=JointLimits(lower=-3.14, upper=3.14),
            )
        )
        robot.add_joint(
            Joint(
                name="joint2",
                type=JointType.REVOLUTE,
                parent="base_link",
                child="link2",
                axis=Vector3(0, 0, 1),
                limits=JointLimits(lower=-3.14, upper=3.14),
            )
        )

        # Add differential transmission
        trans = Transmission.create_differential(
            name="diff_trans",
            joint1_name="joint1",
            joint2_name="joint2",
            mechanical_reduction=100.0,
        )
        robot.add_transmission(trans)

        # Round-trip
        generator = URDFGenerator()
        urdf_string = generator.generate(robot)
        parsed_robot = URDFParser().parse_string(urdf_string)

        # Verify
        assert len(parsed_robot.transmissions) == 1
        parsed_trans = parsed_robot.transmissions[0]
        assert parsed_trans.name == "diff_trans"
        assert len(parsed_trans.joints) == 2
        assert {j.name for j in parsed_trans.joints} == {"joint1", "joint2"}


class TestGazeboRoundtrip:
    """Test round-trip for Gazebo elements."""

    def test_robot_level_gazebo_roundtrip(self):
        """Test that robot-level Gazebo element survives round-trip."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        # Add robot-level Gazebo element with plugin
        plugin = GazeboPlugin(
            name="joint_state_publisher",
            filename="libgazebo_ros_joint_state_publisher.so",
            parameters={"update_rate": "50"},
        )
        element = GazeboElement(reference=None, static=True, plugins=[plugin])
        robot.add_gazebo_element(element)

        # Round-trip
        generator = URDFGenerator()
        urdf_string = generator.generate(robot)
        parsed_robot = URDFParser().parse_string(urdf_string)

        # Verify
        assert len(parsed_robot.gazebo_elements) == 1
        parsed_elem = parsed_robot.gazebo_elements[0]
        assert parsed_elem.reference is None
        assert parsed_elem.static is True
        assert len(parsed_elem.plugins) == 1
        assert parsed_elem.plugins[0].name == "joint_state_publisher"

    def test_link_level_gazebo_roundtrip(self):
        """Test that link-level Gazebo element survives round-trip."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        # Add link-level Gazebo element
        element = GazeboElement(
            reference="base_link",
            material="Gazebo/Red",
            mu1=0.8,
            mu2=0.8,
            kp=1000.0,
            kd=100.0,
        )
        robot.add_gazebo_element(element)

        # Round-trip
        generator = URDFGenerator()
        urdf_string = generator.generate(robot)
        parsed_robot = URDFParser().parse_string(urdf_string)

        # Verify
        assert len(parsed_robot.gazebo_elements) == 1
        parsed_elem = parsed_robot.gazebo_elements[0]
        assert parsed_elem.reference == "base_link"
        assert parsed_elem.material == "Gazebo/Red"
        assert parsed_elem.mu1 == pytest.approx(0.8)
        assert parsed_elem.mu2 == pytest.approx(0.8)
        assert parsed_elem.kp == pytest.approx(1000.0)
        assert parsed_elem.kd == pytest.approx(100.0)

    def test_joint_level_gazebo_roundtrip(self):
        """Test that joint-level Gazebo element survives round-trip."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                axis=Vector3(0, 0, 1),
                limits=JointLimits(lower=-1.57, upper=1.57),
            )
        )

        # Add joint-level Gazebo element
        element = GazeboElement(
            reference="joint1",
            provide_feedback=True,
            implicit_spring_damper=True,
        )
        robot.add_gazebo_element(element)

        # Round-trip
        generator = URDFGenerator()
        urdf_string = generator.generate(robot)
        parsed_robot = URDFParser().parse_string(urdf_string)

        # Verify
        assert len(parsed_robot.gazebo_elements) == 1
        parsed_elem = parsed_robot.gazebo_elements[0]
        assert parsed_elem.reference == "joint1"
        assert parsed_elem.provide_feedback is True
        assert parsed_elem.implicit_spring_damper is True


class TestComplexRobotRoundtrip:
    """Test round-trip for complex robot with multiple advanced elements."""

    def test_complete_robot_with_all_elements(self):
        """Test robot with sensors, transmissions, and Gazebo elements."""
        # Create a mobile robot with camera and IMU
        robot = Robot(name="mobile_robot")

        # Links
        robot.add_link(
            Link(
                name="base_link",
                initial_visuals=[Visual(geometry=Box(size=Vector3(0.5, 0.3, 0.2)))],
                inertial=Inertial(
                    mass=10.0,
                    inertia=InertiaTensor(ixx=0.1, ixy=0.0, ixz=0.0, iyy=0.1, iyz=0.0, izz=0.1),
                ),
            )
        )
        robot.add_link(Link(name="left_wheel"))
        robot.add_link(Link(name="right_wheel"))
        robot.add_link(Link(name="camera_link"))
        robot.add_link(Link(name="imu_link"))

        # Joints
        robot.add_joint(
            Joint(
                name="left_wheel_joint",
                type=JointType.CONTINUOUS,
                parent="base_link",
                child="left_wheel",
                axis=Vector3(0.0, 1.0, 0.0),
                origin=Transform(xyz=Vector3(0.0, 0.2, 0.0)),
            )
        )
        robot.add_joint(
            Joint(
                name="right_wheel_joint",
                type=JointType.CONTINUOUS,
                parent="base_link",
                child="right_wheel",
                axis=Vector3(0.0, 1.0, 0.0),
                origin=Transform(xyz=Vector3(0.0, -0.2, 0.0)),
            )
        )
        robot.add_joint(
            Joint(
                name="camera_joint",
                type=JointType.FIXED,
                parent="base_link",
                child="camera_link",
                origin=Transform(xyz=Vector3(0.25, 0.0, 0.1)),
            )
        )
        robot.add_joint(
            Joint(
                name="imu_joint",
                type=JointType.FIXED,
                parent="base_link",
                child="imu_link",
            )
        )

        # Transmissions (currently not validated against joints in direct append)
        left_trans = Transmission.create_simple(
            name="left_wheel_trans", joint_name="left_wheel_joint", hardware_interface="velocity"
        )
        right_trans = Transmission.create_simple(
            name="right_wheel_trans", joint_name="right_wheel_joint", hardware_interface="velocity"
        )
        robot.add_transmission(left_trans)
        robot.add_transmission(right_trans)

        # Gazebo elements
        # Differential drive plugin
        diff_drive_plugin = GazeboPlugin(
            name="diff_drive",
            filename="libgazebo_ros_diff_drive.so",
            parameters={
                "left_joint": "left_wheel_joint",
                "right_joint": "right_wheel_joint",
                "wheel_separation": "0.4",
                "wheel_diameter": "0.1",
                "command_topic": "cmd_vel",
                "odometry_topic": "odom",
            },
        )
        robot.add_gazebo_element(GazeboElement(reference=None, plugins=[diff_drive_plugin]))

        # Link friction
        robot.add_gazebo_element(
            GazeboElement(reference="base_link", mu1=0.5, mu2=0.5, material="Gazebo/Grey")
        )

        # Round-trip
        generator = URDFGenerator()
        urdf_string = generator.generate(robot)
        parsed_robot = URDFParser().parse_string(urdf_string)

        # Verify structure
        assert parsed_robot.name == "mobile_robot"
        assert len(parsed_robot.links) == 5
        assert len(parsed_robot.joints) == 4
        assert len(parsed_robot.transmissions) == 2
        # Expects 3: gazebo_ros2_control plugin + diff_drive plugin + base_link friction
        assert len(parsed_robot.gazebo_elements) == 3

        # Verify transmissions
        trans_names = {t.name for t in parsed_robot.transmissions}
        assert trans_names == {"left_wheel_trans", "right_wheel_trans"}

        # Verify Gazebo elements
        robot_level_gz = [e for e in parsed_robot.gazebo_elements if e.reference is None]
        link_level_gz = [e for e in parsed_robot.gazebo_elements if e.reference == "base_link"]

        # Should have 2 robot-level elements: gazebo_ros2_control + diff_drive
        assert len(robot_level_gz) == 2

        # Find the diff_drive plugin
        diff_drive_elem = next(
            (e for e in robot_level_gz if any(p.name == "diff_drive" for p in e.plugins)), None
        )
        assert diff_drive_elem is not None
        assert len(diff_drive_elem.plugins) == 1
        assert diff_drive_elem.plugins[0].name == "diff_drive"

        # Find the gazebo_ros2_control plugin (modern name)
        ros2_control_elem = next(
            (e for e in robot_level_gz if any("ros2_control" in p.name for p in e.plugins)),
            None,
        )
        assert ros2_control_elem is not None
        assert len(ros2_control_elem.plugins) == 1
        assert "ros2_control" in ros2_control_elem.plugins[0].name

        assert len(link_level_gz) == 1
        assert link_level_gz[0].mu1 == pytest.approx(0.5)
        assert link_level_gz[0].material == "Gazebo/Grey"
