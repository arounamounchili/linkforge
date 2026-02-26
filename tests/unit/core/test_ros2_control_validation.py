"""Unit tests for Ros2Control validation logic."""

import pytest
from linkforge_core.models.ros2_control import Ros2Control, Ros2ControlJoint


def test_ros2_control_sensor_read_only_validation():
    """Test that hardware type 'sensor' cannot have command interfaces."""
    joint = Ros2ControlJoint(
        name="sensor_joint",
        command_interfaces=["position"],  # Should cause error
        state_interfaces=["position"],
    )

    with pytest.raises(ValueError, match="Hardware type 'sensor' cannot have command interfaces"):
        Ros2Control(
            name="MySensor",
            type="sensor",
            hardware_plugin="mock_sensor",
            joints=[joint],
        )


def test_ros2_control_system_with_commands_valid():
    """Test that hardware type 'system' CAN have command interfaces."""
    joint = Ros2ControlJoint(
        name="actuator_joint",
        command_interfaces=["position"],
        state_interfaces=["position"],
    )

    # Should NOT raise error
    rc = Ros2Control(
        name="MyRobot",
        type="system",
        hardware_plugin="mock_actuator",
        joints=[joint],
    )
    assert rc.type == "system"


def test_ros2_control_joint_parameters_export_fidelity():
    """Test that parameters are correctly stored in the models."""
    joint = Ros2ControlJoint(
        name="joint1",
        command_interfaces=["position"],
        parameters={"can_id": "0x10", "gear_ratio": "50.0"},
    )

    assert joint.parameters["can_id"] == "0x10"
    assert joint.parameters["gear_ratio"] == "50.0"


def test_ros2_control_global_parameters_fidelity():
    """Test that global hardware parameters are correctly stored."""
    rc = Ros2Control(
        name="MySystem",
        hardware_plugin="plugin",
        parameters={"port": "/dev/ttyUSB0", "baud": "115200"},
    )

    assert rc.parameters["port"] == "/dev/ttyUSB0"
    assert rc.parameters["baud"] == "115200"


def test_validator_detects_non_existent_ros2_control_joint():
    """Test that RobotValidator catches joints that don't exist in the kinematic tree."""
    from linkforge_core.models.robot import Link, Robot
    from linkforge_core.validation.validator import RobotValidator

    robot = Robot(name="test_robot")
    robot.add_link(Link(name="base_link"))
    # No joint 'joint1' in the robot

    rc = Ros2Control(
        name="ctrl",
        hardware_plugin="fake_plugin",
        joints=[Ros2ControlJoint(name="joint1", command_interfaces=["position"])],
    )
    robot.ros2_controls.append(rc)

    validator = RobotValidator(robot)
    result = validator.validate()

    assert not result.is_valid
    errors = [e for e in result.issues if e.severity.value == "error"]
    assert any("joint 'joint1' does not exist" in e.message.lower() for e in errors)


def test_ros2_control_actuator_joint_limit_validation():
    """Test that hardware type 'actuator' must have exactly one joint."""
    j1 = Ros2ControlJoint(name="joint1", command_interfaces=["position"])
    j2 = Ros2ControlJoint(name="joint2", command_interfaces=["position"])

    # 0 joints - should fail
    with pytest.raises(ValueError, match="must have exactly one joint"):
        Ros2Control(
            name="MyActuator",
            type="actuator",
            hardware_plugin="mock",
            joints=[],
        )

    # 2 joints - should fail
    with pytest.raises(ValueError, match="must have exactly one joint"):
        Ros2Control(
            name="MyActuator",
            type="actuator",
            hardware_plugin="mock",
            joints=[j1, j2],
        )

    # 1 joint - should pass
    rc = Ros2Control(
        name="MyActuator",
        type="actuator",
        hardware_plugin="mock",
        joints=[j1],
    )
    assert len(rc.joints) == 1
