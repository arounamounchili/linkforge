"""Test ros2_control parsing and round-trip fidelity."""

from linkforge.core.generators.urdf import URDFGenerator
from linkforge.core.parsers.urdf_parser import parse_urdf_string


def test_parse_ros2_control_basic():
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
    robot = parse_urdf_string(urdf_string)

    assert len(robot.ros2_controls) == 1
    rc = robot.ros2_controls[0]
    assert rc.name == "MySystem"
    assert rc.type == "system"
    assert rc.hardware_plugin == "my_custom_plugin"
    assert len(rc.joints) == 1
    assert rc.joints[0].name == "joint1"
    assert rc.joints[0].command_interfaces == ["position"]
    assert rc.joints[0].state_interfaces == ["position", "velocity"]


def test_parse_ros2_control_multiple_interfaces():
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
    robot = parse_urdf_string(urdf_string)

    rc = robot.ros2_controls[0]
    assert rc.joints[0].command_interfaces == ["position", "velocity"]
    assert rc.joints[0].state_interfaces == ["position", "velocity", "effort"]


def test_ros2_control_roundtrip():
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
    robot1 = parse_urdf_string(original_urdf)

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
    robot2 = parse_urdf_string(exported_urdf)

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


def test_ros2_control_multiple_joints():
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

    robot = parse_urdf_string(urdf_string)

    rc = robot.ros2_controls[0]
    assert len(rc.joints) == 2
    assert rc.joints[0].name == "joint1"
    assert rc.joints[0].command_interfaces == ["position"]
    assert rc.joints[1].name == "joint2"
    assert rc.joints[1].command_interfaces == ["velocity"]
