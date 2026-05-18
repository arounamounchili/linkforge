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

    def test_ros2_control_joint_validation_errors(self) -> None:
        """Test validation error branches in Ros2ControlJoint."""
        from linkforge.core.exceptions import RobotValidationError, ValidationErrorCode

        # Empty name
        with pytest.raises(RobotValidationError) as exc:
            Ros2ControlJoint(name="")
        assert exc.value.code == ValidationErrorCode.NAME_EMPTY

        # Empty interfaces
        with pytest.raises(RobotValidationError) as exc:
            Ros2ControlJoint(name="j1")
        assert exc.value.code == ValidationErrorCode.VALUE_EMPTY

    def test_ros2_control_joint_prefix_and_normalization(self) -> None:
        """Test with_prefix and normalization for joint."""
        joint = Ros2ControlJoint(
            name="j1",
            command_interfaces=["velocity", "position"],
            state_interfaces=["velocity", "position"],
        )
        prefixed = joint.with_prefix("pre_")
        assert prefixed.name == "pre_j1"

        normalized = joint.normalized()
        assert normalized.command_interfaces == ("position", "velocity")
        assert normalized.state_interfaces == ("position", "velocity")

    def test_ros2_control_sensor_model(self) -> None:
        """Test Ros2ControlSensor prefix, validation and normalization."""
        from linkforge.core.exceptions import RobotValidationError, ValidationErrorCode
        from linkforge.core.models.ros2_control import Ros2ControlSensor

        # Empty name validation
        with pytest.raises(RobotValidationError) as exc:
            Ros2ControlSensor(name="")
        assert exc.value.code == ValidationErrorCode.NAME_EMPTY

        sensor = Ros2ControlSensor(name="s1", state_interfaces=["velocity", "position"])
        assert sensor.name == "s1"

        prefixed = sensor.with_prefix("pre_")
        assert prefixed.name == "pre_s1"

        normalized = sensor.normalized()
        assert normalized.state_interfaces == ("position", "velocity")

    def test_ros2_control_validation_errors(self) -> None:
        """Test all validation error boundaries in Ros2Control."""
        from linkforge.core.exceptions import RobotValidationError, ValidationErrorCode
        from linkforge.core.models.ros2_control import Ros2ControlSensor

        # 1. Empty name
        with pytest.raises(RobotValidationError) as exc:
            Ros2Control(name="", hardware_plugin="fake")
        assert exc.value.code == ValidationErrorCode.NAME_EMPTY

        # 2. Invalid control type
        with pytest.raises(RobotValidationError) as exc:
            Ros2Control(name="c1", type="invalid", hardware_plugin="fake")
        assert exc.value.code == ValidationErrorCode.INVALID_VALUE

        # 3. Empty plugin
        with pytest.raises(RobotValidationError) as exc:
            Ros2Control(name="c1", hardware_plugin="")
        assert exc.value.code == ValidationErrorCode.VALUE_EMPTY

        # 4. Duplicate joints
        j1 = Ros2ControlJoint(name="j1", command_interfaces=["position"])
        with pytest.raises(RobotValidationError) as exc:
            Ros2Control(name="c1", hardware_plugin="fake", joints=[j1, j1])
        assert exc.value.code == ValidationErrorCode.DUPLICATE_NAME

        # 5. Duplicate sensors
        s1 = Ros2ControlSensor(name="s1")
        with pytest.raises(RobotValidationError) as exc:
            Ros2Control(name="c1", hardware_plugin="fake", sensors=[s1, s1])
        assert exc.value.code == ValidationErrorCode.DUPLICATE_NAME

        # 6. Sensor type with command interfaces in joint
        with pytest.raises(RobotValidationError) as exc:
            Ros2Control(name="c1", type="sensor", hardware_plugin="fake", joints=[j1])
        assert exc.value.code == ValidationErrorCode.INVALID_VALUE

        # 7. Actuator type with != 1 joint
        with pytest.raises(RobotValidationError) as exc:
            Ros2Control(name="c1", type="actuator", hardware_plugin="fake", joints=[])
        assert exc.value.code == ValidationErrorCode.INVALID_VALUE

        j2 = Ros2ControlJoint(name="j2", command_interfaces=["position"])
        with pytest.raises(RobotValidationError) as exc:
            Ros2Control(name="c1", type="actuator", hardware_plugin="fake", joints=[j1, j2])
        assert exc.value.code == ValidationErrorCode.INVALID_VALUE

    def test_ros2_control_prefix_and_normalization(self) -> None:
        """Test with_prefix and normalized methods of Ros2Control."""
        from linkforge.core.models.ros2_control import Ros2ControlSensor

        j1 = Ros2ControlJoint(name="j2", command_interfaces=["velocity", "position"])
        j2 = Ros2ControlJoint(name="j1", command_interfaces=["position"])
        s1 = Ros2ControlSensor(name="s2", state_interfaces=["velocity", "position"])
        s2 = Ros2ControlSensor(name="s1", state_interfaces=["position"])

        ctrl = Ros2Control(name="ctrl", hardware_plugin="fake", joints=[j1, j2], sensors=[s1, s2])

        prefixed = ctrl.with_prefix("pre_")
        assert prefixed.name == "pre_ctrl"
        assert prefixed.joints[0].name == "pre_j2"
        assert prefixed.sensors[0].name == "pre_s2"

        normalized = ctrl.normalized()
        # Should sort alphabetically by name: j1 then j2
        assert normalized.joints[0].name == "j1"
        assert normalized.joints[1].name == "j2"
        # Should sort joint's interfaces too
        assert normalized.joints[1].command_interfaces == ("position", "velocity")
        # Should sort sensors: s1 then s2
        assert normalized.sensors[0].name == "s1"
        assert normalized.sensors[1].name == "s2"

    def test_ros2_control_sensor_valid_joints(self) -> None:
        """Test valid joint configurations for sensor control type."""
        # Joint with only state interfaces is valid for sensor type
        j_sensor = Ros2ControlJoint(name="j_sensor", state_interfaces=["position"])
        ctrl_ok = Ros2Control(
            name="sensor_ctrl",
            type="sensor",
            hardware_plugin="fake",
            joints=[j_sensor],
        )
        assert ctrl_ok.name == "sensor_ctrl"

        # Empty joints is also valid for sensor type
        ctrl_empty = Ros2Control(
            name="sensor_empty",
            type="sensor",
            hardware_plugin="fake",
            joints=[],
        )
        assert ctrl_empty.name == "sensor_empty"


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
