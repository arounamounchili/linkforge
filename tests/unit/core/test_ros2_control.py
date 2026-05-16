"""Unit tests for ROS2 Control model and parser."""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest
from linkforge.core import Ros2Control, Ros2ControlJoint, URDFParser


@pytest.fixture
def parser() -> URDFParser:
    return URDFParser()


# ROS 2 Control Model Tests


class TestRos2ControlModels:
    def test_ros2_control_joint_creation(self) -> None:
        """Test creating a ROS 2 control joint."""
        joint = Ros2ControlJoint(
            name="j1",
            command_interfaces=["position", "velocity"],
            state_interfaces=["position", "velocity"],
        )
        assert joint.name == "j1"
        assert "position" in joint.command_interfaces

    def test_ros2_control_system_creation(self) -> None:
        """Test creating a ROS 2 control system block."""
        ctrl = Ros2Control(
            name="TestSystem", type="system", hardware_plugin="fake_hardware/GenericSystem"
        )
        assert ctrl.name == "TestSystem"
        assert ctrl.type == "system"


# ROS 2 Control Parsing Tests


class TestRos2ControlParsing:
    def test_parse_ros2_control_block(self, parser) -> None:
        """Test parsing a ros2_control block from URDF."""
        xml = """
        <ros2_control name="RealRobot" type="system">
            <hardware>
                <plugin>my_robot_hardware/MyRobotSystem</plugin>
                <param name="port">/dev/ttyUSB0</param>
            </hardware>
            <joint name="joint1">
                <command_interface name="position">
                    <param name="min">-1.57</param>
                </command_interface>
                <state_interface name="position"/>
            </joint>
        </ros2_control>
        """
        elem = ET.fromstring(xml)
        rc = parser._parse_ros2_control(elem)

        assert rc.name == "RealRobot"
        assert rc.hardware_plugin == "my_robot_hardware/MyRobotSystem"
        assert rc.parameters["port"] == "/dev/ttyUSB0"
        assert len(rc.joints) == 1
        assert rc.joints[0].name == "joint1"
        assert "position" in rc.joints[0].command_interfaces
