"""Unit tests for Ros2Control validation logic."""

import pytest
from linkforge_core.exceptions import RobotModelError
from linkforge_core.models.ros2_control import Ros2Control, Ros2ControlJoint


def test_ros2_control_sensor_read_only_validation() -> None:
    """Test that hardware type 'sensor' cannot have command interfaces."""
    joint = Ros2ControlJoint(
        name="sensor_joint",
        command_interfaces=["position"],  # Should cause error
        state_interfaces=["position"],
    )

    with pytest.raises(
        RobotModelError, match="Hardware type 'sensor' cannot have command interfaces"
    ):
        Ros2Control(
            name="MySensor",
            type="sensor",
            hardware_plugin="mock_sensor",
            joints=[joint],
        )


def test_ros2_control_system_with_commands_valid() -> None:
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


def test_ros2_control_joint_parameters_export_fidelity() -> None:
    """Test that parameters are correctly stored in the models."""
    joint = Ros2ControlJoint(
        name="joint1",
        command_interfaces=["position"],
        parameters={"can_id": "0x10", "gear_ratio": "50.0"},
    )

    assert joint.parameters["can_id"] == "0x10"
    assert joint.parameters["gear_ratio"] == "50.0"


def test_ros2_control_global_parameters_fidelity() -> None:
    """Test that global hardware parameters are correctly stored."""
    rc = Ros2Control(
        name="MySystem",
        hardware_plugin="plugin",
        parameters={"port": "/dev/ttyUSB0", "baud": "115200"},
    )

    assert rc.parameters["port"] == "/dev/ttyUSB0"
    assert rc.parameters["baud"] == "115200"


def test_validator_detects_non_existent_ros2_control_joint() -> None:
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
    robot.add_ros2_control(rc)

    validator = RobotValidator()
    result = validator.validate(robot)

    assert not result.is_valid
    errors = [e for e in result.issues if e.severity.value == "error"]
    assert any("joint 'joint1' does not exist" in e.message.lower() for e in errors)


def test_ros2_control_actuator_joint_limit_validation() -> None:
    """Test that hardware type 'actuator' must have exactly one joint."""
    j1 = Ros2ControlJoint(name="joint1", command_interfaces=["position"])
    j2 = Ros2ControlJoint(name="joint2", command_interfaces=["position"])

    # 0 joints - should fail
    with pytest.raises(RobotModelError, match="must have exactly one joint"):
        Ros2Control(
            name="MyActuator",
            type="actuator",
            hardware_plugin="mock",
            joints=[],
        )

    # 2 joints - should fail
    with pytest.raises(RobotModelError, match="must have exactly one joint"):
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


def test_ros2_control_empty_name_validation() -> None:
    """Test that Ros2Control must have a non-empty name."""
    with pytest.raises(RobotModelError, match="ros2_control name cannot be empty"):
        Ros2Control(name="", hardware_plugin="mock")


def test_ros2_control_invalid_type_validation() -> None:
    """Test that Ros2Control must have a valid type."""
    with pytest.raises(RobotModelError, match="Invalid ros2_control type"):
        Ros2Control(name="ctrl", type="invalid", hardware_plugin="mock")


def test_ros2_control_joint_empty_name_validation() -> None:
    """Test that Ros2ControlJoint must have a non-empty name."""
    with pytest.raises(RobotModelError, match="Joint name cannot be empty"):
        Ros2ControlJoint(name="")


def test_ros2_control_joint_empty_interfaces_validation() -> None:
    """Test that Ros2ControlJoint must have at least one interface."""
    with pytest.raises(RobotModelError, match="must have at least one command OR state interface"):
        Ros2ControlJoint(name="joint1", command_interfaces=[], state_interfaces=[])


def test_ros2_control_empty_hardware_plugin_validation() -> None:
    """Test that Ros2Control must have a non-empty hardware plugin."""
    with pytest.raises(RobotModelError, match="Hardware plugin cannot be empty"):
        Ros2Control(name="ctrl", hardware_plugin="")


def test_ros2_control_sensor_no_command_interfaces() -> None:
    """Test that a sensor without command interfaces passes validation."""
    j1 = Ros2ControlJoint(name="sensor_joint", state_interfaces=["position"])
    rc = Ros2Control(name="sens", type="sensor", hardware_plugin="mock", joints=[j1])
    assert len(rc.joints) == 1
    assert rc.joints[0].name == "sensor_joint"


def test_ros2_control_sensor_empty_joints() -> None:
    """Test that a sensor with no joints passes validation (e.g., IMU on a link)."""
    rc = Ros2Control(name="sens", type="sensor", hardware_plugin="mock", joints=[])
    assert len(rc.joints) == 0
