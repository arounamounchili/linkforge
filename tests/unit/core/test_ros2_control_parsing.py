"""Test ros2_control parsing and round-trip fidelity."""

import pytest
from linkforge_core import URDFGenerator
from linkforge_core.exceptions import RobotModelError
from linkforge_core.models.ros2_control import Ros2Control, Ros2ControlJoint
from linkforge_core.parsers.urdf_parser import URDFParser


def test_parse_ros2_control_basic() -> None:
    """Test parsing basic ros2_control element."""
    urdf_string = """<?xml version="1.0"?>
    <robot name="test">
      <link name="base_link"/>
      <ros2_control name="MySystem" type="system">
        <hardware>
          <plugin>my_custom_plugin</plugin>
        </hardware>
        <joint name="joint1">
          <command_interface name="position"/>
          <state_interface name="position"/>
          <state_interface name="velocity"/>
        </joint>
      </ros2_control>
    </robot>
    """
    robot = URDFParser().parse_string(urdf_string)

    assert len(robot.ros2_controls) == 1
    rc = robot.ros2_controls[0]
    assert rc.name == "MySystem"
    assert rc.type == "system"
    assert rc.hardware_plugin == "my_custom_plugin"
    assert len(rc.joints) == 1
    assert rc.joints[0].name == "joint1"
    assert rc.joints[0].command_interfaces == ["position"]
    assert rc.joints[0].state_interfaces == ["position", "velocity"]


def test_parse_ros2_control_multiple_interfaces() -> None:
    """Test parsing ros2_control with multiple command interfaces."""
    urdf_string = """<?xml version="1.0"?>
    <robot name="test">
      <link name="base_link"/>
      <ros2_control name="TestSystem" type="system">
        <hardware>
          <plugin>test_plugin</plugin>
        </hardware>
        <joint name="joint1">
          <command_interface name="position"/>
          <command_interface name="velocity"/>
          <state_interface name="position"/>
          <state_interface name="velocity"/>
          <state_interface name="effort"/>
        </joint>
      </ros2_control>
    </robot>
    """
    robot = URDFParser().parse_string(urdf_string)

    rc = robot.ros2_controls[0]
    assert rc.joints[0].command_interfaces == ["position", "velocity"]
    assert rc.joints[0].state_interfaces == ["position", "velocity", "effort"]


def test_ros2_control_roundtrip() -> None:
    """Test that ros2_control is preserved during import/export round-trip."""
    original_urdf = """<?xml version="1.0"?>
    <robot name="test_robot">
      <link name="base_link"/>
      <link name="link1"/>
      <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
      </joint>
      <ros2_control name="CustomSystem" type="system">
        <hardware>
          <plugin>my_custom_hardware/MyHardware</plugin>
        </hardware>
        <joint name="joint1">
          <command_interface name="effort"/>
          <state_interface name="position"/>
          <state_interface name="velocity"/>
          <state_interface name="effort"/>
        </joint>
      </ros2_control>
    </robot>
    """

    # Import
    robot1 = URDFParser().parse_string(original_urdf)

    # Verify parsed correctly
    assert len(robot1.ros2_controls) == 1
    assert robot1.ros2_controls[0].name == "CustomSystem"
    assert robot1.ros2_controls[0].hardware_plugin == "my_custom_hardware/MyHardware"
    assert robot1.ros2_controls[0].joints[0].command_interfaces == ["effort"]
    assert robot1.ros2_controls[0].joints[0].state_interfaces == ["position", "velocity", "effort"]

    # Export
    generator = URDFGenerator()
    exported_urdf = generator.generate(robot1)

    # Re-import
    robot2 = URDFParser().parse_string(exported_urdf)

    # Verify ros2_control preserved
    assert len(robot2.ros2_controls) == 1
    rc2 = robot2.ros2_controls[0]
    assert rc2.name == "CustomSystem"
    assert rc2.type == "system"
    assert rc2.hardware_plugin == "my_custom_hardware/MyHardware"
    assert len(rc2.joints) == 1
    assert rc2.joints[0].name == "joint1"
    assert rc2.joints[0].command_interfaces == ["effort"]
    assert rc2.joints[0].state_interfaces == ["position", "velocity", "effort"]


def test_ros2_control_multiple_joints() -> None:
    """Test ros2_control with multiple joints."""
    urdf_string = """<?xml version="1.0"?>
    <robot name="test">
      <link name="base_link"/>
      <ros2_control name="MultiJointSystem" type="system">
        <hardware>
          <plugin>test_plugin</plugin>
        </hardware>
        <joint name="joint1">
          <command_interface name="position"/>
          <state_interface name="position"/>
          <state_interface name="velocity"/>
        </joint>
        <joint name="joint2">
          <command_interface name="velocity"/>
          <state_interface name="position"/>
          <state_interface name="velocity"/>
        </joint>
      </ros2_control>
    </robot>
    """

    robot = URDFParser().parse_string(urdf_string)

    rc = robot.ros2_controls[0]
    assert len(rc.joints) == 2
    assert rc.joints[0].name == "joint1"
    assert rc.joints[0].command_interfaces == ["position"]
    assert rc.joints[1].name == "joint2"
    assert rc.joints[1].command_interfaces == ["velocity"]


class TestRos2ControlJointValidation:
    """Tests for Ros2ControlJoint validation."""

    def test_empty_joint_name(self) -> None:
        """Test that empty joint name raises error."""
        with pytest.raises(RobotModelError):
            Ros2ControlJoint(
                name="",
                command_interfaces=["position"],
                state_interfaces=["position"],
            )

    def test_missing_all_interfaces(self) -> None:
        """Test that missing ALL interfaces raises error."""
        with pytest.raises(RobotModelError):
            Ros2ControlJoint(
                name="joint1",
                command_interfaces=[],
                state_interfaces=[],
            )


def test_ros2_control_normalization() -> None:
    """Test that interface names are normalized (e.g. full ROS paths to short names)."""
    urdf_string = """<?xml version="1.0"?>
    <robot name="test">
      <link name="base_link"/>
      <ros2_control name="TestSystem" type="system">
        <hardware>
          <plugin>test_plugin</plugin>
        </hardware>
        <joint name="joint1">
          <command_interface name="hardware_interface/PositionJointInterface"/>
          <state_interface name="hardware_interface/VelocityJointInterface"/>
        </joint>
      </ros2_control>
    </robot>
    """
    robot = URDFParser().parse_string(urdf_string)

    rc = robot.ros2_controls[0]
    assert rc.joints[0].command_interfaces == ["position"]
    assert rc.joints[0].state_interfaces == ["velocity"]


class TestRos2ControlValidation:
    """Tests for Ros2Control validation."""

    def test_empty_name(self) -> None:
        """Test that empty ros2_control name raises error."""
        with pytest.raises(RobotModelError):
            Ros2Control(
                name="",
                hardware_plugin="test_plugin",
            )

    def test_invalid_type(self) -> None:
        """Test that invalid ros2_control type raises error."""
        with pytest.raises(RobotModelError):
            Ros2Control(
                name="TestSystem",
                type="invalid_type",
                hardware_plugin="test_plugin",
            )

    def test_empty_hardware_plugin(self) -> None:
        """Test that empty hardware plugin raises error."""
        with pytest.raises(RobotModelError):
            Ros2Control(
                name="TestSystem",
                type="system",
                hardware_plugin="",
            )


def test_parse_ros2_control_readonly_joint() -> None:
    """Test parsing a joint with ONLY state interfaces (should be valid)."""
    xml = """
    <robot name="test">
        <ros2_control name="SensorSystem" type="system">
            <hardware><plugin>mock</plugin></hardware>
            <joint name="sensor_joint">
                <state_interface name="position"/>
            </joint>
        </ros2_control>
    </robot>
    """
    robot = URDFParser().parse_string(xml)
    assert len(robot.ros2_controls) == 1
    rc = robot.ros2_controls[0]
    assert len(rc.joints) == 1
    assert rc.joints[0].name == "sensor_joint"
    assert rc.joints[0].state_interfaces == ["position"]


def test_parse_ros2_control_writeonly_joint() -> None:
    """Test parsing a joint with ONLY command interfaces (should be valid)."""
    xml = """
    <robot name="test">
        <ros2_control name="ActuatorSystem" type="system">
            <hardware><plugin>mock</plugin></hardware>
            <joint name="actuator_joint">
                <command_interface name="velocity"/>
            </joint>
        </ros2_control>
    </robot>
    """
    robot = URDFParser().parse_string(xml)
    assert len(robot.ros2_controls) == 1
    rc = robot.ros2_controls[0]
    assert len(rc.joints) == 1
    assert rc.joints[0].name == "actuator_joint"
    assert rc.joints[0].command_interfaces == ["velocity"]
