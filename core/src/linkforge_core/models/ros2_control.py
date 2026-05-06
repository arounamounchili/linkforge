"""ros2_control data models for ROS 2 control configuration."""

from __future__ import annotations

from dataclasses import dataclass, field, replace

from ..exceptions import RobotValidationError, ValidationErrorCode


@dataclass
class Ros2ControlJoint:
    """Joint configuration in ros2_control block.

    Represents a joint's control interfaces in a ros2_control system.
    """

    name: str
    command_interfaces: list[str] = field(default_factory=list)
    state_interfaces: list[str] = field(default_factory=list)
    parameters: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Validate joint configuration."""
        if not self.name:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY,
                "ROS2 control joint name cannot be empty",
                target="JointName",
                value=self.name,
            )
        # At least one interface is required to ensure the control block is functional.
        # This prevents defining "empty" joints that ROS 2 control would reject.
        if not self.command_interfaces and not self.state_interfaces:
            raise RobotValidationError(
                ValidationErrorCode.VALUE_EMPTY,
                f"ROS2 control joint '{self.name}' must have at least one command or state interface",
                target="Ros2ControlInterfaces",
                value=self.name,
            )

    def with_prefix(self, prefix: str) -> Ros2ControlJoint:
        """Create a new control joint with a prefixed name."""
        return replace(self, name=f"{prefix}{self.name}")


@dataclass
class Ros2Control:
    """ros2_control configuration block.

    Represents a complete ros2_control system configuration including
    hardware plugin and joint interfaces.
    """

    name: str
    type: str = "system"  # "system", "actuator", or "sensor"
    hardware_plugin: str = ""
    joints: list[Ros2ControlJoint] = field(default_factory=list)
    parameters: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Validate ros2_control configuration."""
        if not self.name:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY,
                "ROS2 control name cannot be empty",
                target="Ros2ControlName",
                value=self.name,
            )
        if self.type not in ("system", "actuator", "sensor"):
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                f"Invalid ROS2 control type '{self.type}' (must be system, actuator, or sensor)",
                target="Ros2ControlType",
                value=self.type,
            )
        if not self.hardware_plugin:
            raise RobotValidationError(
                ValidationErrorCode.VALUE_EMPTY,
                "Hardware plugin cannot be empty",
                target="HardwarePlugin",
                value=self.hardware_plugin,
            )

        # Ensure all joints have unique names within this system
        joint_names = [j.name for j in self.joints]
        if len(joint_names) != len(set(joint_names)):
            raise RobotValidationError(
                ValidationErrorCode.DUPLICATE_NAME,
                f"Duplicate joint names found in ROS2 control system '{self.name}'",
                target="Ros2ControlJoints",
            )

        # Hardware sensors are read-only and do not accept command interfaces
        if self.type == "sensor":
            for joint in self.joints:
                if joint.command_interfaces:
                    raise RobotValidationError(
                        ValidationErrorCode.INVALID_VALUE,
                        "Hardware sensors cannot have command interfaces",
                        target="Ros2ControlMode",
                        value=self.type,
                    )

        # Hardware actuators are designed for exactly one joint
        if self.type == "actuator" and len(self.joints) != 1:
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                "Actuator type must have exactly one joint",
                target="Ros2ControlJoints",
                value=len(self.joints),
            )

    def with_prefix(self, prefix: str) -> Ros2Control:
        """Create a new control block with prefixed name and joints."""
        return replace(
            self,
            name=f"{prefix}{self.name}",
            joints=[j.with_prefix(prefix) for j in self.joints],
        )
