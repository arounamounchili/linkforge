"""Test round-trip for advanced URDF elements (sensors, transmissions, Gazebo)."""

from __future__ import annotations

import pytest
from linkforge_core.models import (
    GazeboElement,
    GazeboPlugin,
    Joint,
    JointLimits,
    JointType,
    Link,
    Robot,
    Transmission,
    Vector3,
)

from tests.core_test_utils import assert_robots_equal, perform_urdf_roundtrip


class TestTransmissionRoundtrip:
    """Test round-trip for transmission elements."""

    def test_simple_transmission_roundtrip(self) -> None:
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

        # Roundtrip and compare
        parsed_robot = perform_urdf_roundtrip(robot)
        assert_robots_equal(robot, parsed_robot)

        # Extra verification for transmission specifics
        assert len(parsed_robot.transmissions) == 1
        parsed_trans = parsed_robot.transmissions[0]
        assert parsed_trans.name == "arm_trans"
        assert parsed_trans.joints[0].mechanical_reduction == 50.0

    def test_differential_transmission_roundtrip(self) -> None:
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

        # Round-trip and compare
        parsed_robot = perform_urdf_roundtrip(robot)
        assert_robots_equal(robot, parsed_robot)

        # Verify
        assert len(parsed_robot.transmissions) == 1
        parsed_trans = parsed_robot.transmissions[0]
        assert parsed_trans.name == "diff_trans"
        assert {j.name for j in parsed_trans.joints} == {"joint1", "joint2"}


class TestGazeboRoundtrip:
    """Test round-trip for Gazebo elements."""

    def test_robot_level_gazebo_roundtrip(self) -> None:
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

        # Round-trip and compare
        parsed_robot = perform_urdf_roundtrip(robot)
        assert_robots_equal(robot, parsed_robot)

        # Verify
        assert len(parsed_robot.gazebo_elements) == 1
        parsed_elem = parsed_robot.gazebo_elements[0]
        assert parsed_elem.static is True
        assert parsed_elem.plugins[0].name == "joint_state_publisher"

    def test_link_level_gazebo_roundtrip(self) -> None:
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

        # Round-trip and compare
        parsed_robot = perform_urdf_roundtrip(robot)
        assert_robots_equal(robot, parsed_robot)

        # Verify
        assert len(parsed_robot.gazebo_elements) == 1
        parsed_elem = parsed_robot.gazebo_elements[0]
        assert parsed_elem.reference == "base_link"
        assert parsed_elem.mu1 == pytest.approx(0.8)

    def test_joint_level_gazebo_roundtrip(self) -> None:
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

        # Round-trip and compare
        parsed_robot = perform_urdf_roundtrip(robot)
        assert_robots_equal(robot, parsed_robot)

        # Verify
        assert len(parsed_robot.gazebo_elements) == 1
        parsed_elem = parsed_robot.gazebo_elements[0]
        assert parsed_elem.reference == "joint1"
        assert parsed_elem.provide_feedback is True
