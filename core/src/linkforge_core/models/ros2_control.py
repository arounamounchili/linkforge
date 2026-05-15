"""ros2_control data models for hardware interface configuration.

This module provides data structures to define how robot joints and sensors
interface with the ROS 2 Control framework. It manages the metadata required
to generate <ros2_control> blocks in URDF.

Control Block Categories:
- **System**: Multi-joint hardware (e.g., a complete robotic arm).
- **Actuator**: Simple single-joint hardware (e.g., a standalone motor).
- **Sensor**: Read-only hardware (e.g., an external encoder).
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field, replace

from ..constants import (
    CONTROL_TYPE_ACTUATOR,
    CONTROL_TYPE_SENSOR,
    CONTROL_TYPE_SYSTEM,
)
from ..exceptions import RobotValidationError, ValidationErrorCode


@dataclass(frozen=True)
class Ros2ControlJoint:
    """Joint configuration in a ros2_control block."""

    name: str
    command_interfaces: Sequence[str] = field(default_factory=tuple)
    state_interfaces: Sequence[str] = field(default_factory=tuple)
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
        object.__setattr__(self, "command_interfaces", tuple(self.command_interfaces))
        object.__setattr__(self, "state_interfaces", tuple(self.state_interfaces))

    def with_prefix(self, prefix: str) -> Ros2ControlJoint:
        """Create a new control joint with a prefixed name."""
        return replace(self, name=f"{prefix}{self.name}")

    def normalized(self) -> Ros2ControlJoint:
        """Return a new control joint with sorted interfaces."""
        return replace(
            self,
            command_interfaces=tuple(sorted(self.command_interfaces)),
            state_interfaces=tuple(sorted(self.state_interfaces)),
        )


@dataclass(frozen=True)
class Ros2ControlSensor:
    """Sensor configuration in a ros2_control block."""

    name: str
    state_interfaces: Sequence[str] = field(default_factory=tuple)
    parameters: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Validate sensor configuration."""
        if not self.name:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY,
                "ROS2 control sensor name cannot be empty",
                target="SensorName",
                value=self.name,
            )
        object.__setattr__(self, "state_interfaces", tuple(self.state_interfaces))

    def with_prefix(self, prefix: str) -> Ros2ControlSensor:
        """Create a new control sensor with a prefixed name."""
        return replace(self, name=f"{prefix}{self.name}")

    def normalized(self) -> Ros2ControlSensor:
        """Return a new control sensor with sorted interfaces."""
        return replace(
            self,
            state_interfaces=tuple(sorted(self.state_interfaces)),
        )


@dataclass(frozen=True)
class Ros2Control:
    """Hardware interface abstraction for ROS 2 Control.

    This model describes a hardware system (system, actuator, or sensor),
    its associated joints/sensors, and the hardware plugin used to
    communicate with the physical or simulated hardware.
    """

    name: str
    type: str = CONTROL_TYPE_SYSTEM  # "system", "actuator", or "sensor"
    hardware_plugin: str = ""
    joints: Sequence[Ros2ControlJoint] = field(default_factory=tuple)
    sensors: Sequence[Ros2ControlSensor] = field(default_factory=tuple)
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
        if self.type not in (CONTROL_TYPE_SYSTEM, CONTROL_TYPE_ACTUATOR, CONTROL_TYPE_SENSOR):
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

        # Ensure all sensors have unique names
        sensor_names = [s.name for s in self.sensors]
        if len(sensor_names) != len(set(sensor_names)):
            raise RobotValidationError(
                ValidationErrorCode.DUPLICATE_NAME,
                f"Duplicate sensor names found in ROS2 control system '{self.name}'",
                target="Ros2ControlSensors",
            )

        # Hardware sensors are read-only and do not accept command interfaces
        if self.type == CONTROL_TYPE_SENSOR:
            for joint in self.joints:
                if joint.command_interfaces:
                    raise RobotValidationError(
                        ValidationErrorCode.INVALID_VALUE,
                        "Hardware sensors cannot have command interfaces",
                        target="Ros2ControlMode",
                        value=self.type,
                    )

        # Hardware actuators are designed for exactly one joint
        if self.type == CONTROL_TYPE_ACTUATOR and len(self.joints) != 1:
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                "Actuator type must have exactly one joint",
                target="Ros2ControlJoints",
                value=len(self.joints),
            )
        object.__setattr__(self, "joints", tuple(self.joints))
        object.__setattr__(self, "sensors", tuple(self.sensors))

    def with_prefix(self, prefix: str) -> Ros2Control:
        """Create a new control block with prefixed name and joints."""
        return replace(
            self,
            name=f"{prefix}{self.name}",
            joints=tuple(j.with_prefix(prefix) for j in self.joints),
            sensors=tuple(s.with_prefix(prefix) for s in self.sensors),
        )

    def normalized(self) -> Ros2Control:
        """Return a new control block with sorted joints and sensors for comparison."""
        return replace(
            self,
            joints=tuple(sorted([j.normalized() for j in self.joints], key=lambda x: x.name)),
            sensors=tuple(sorted([s.normalized() for s in self.sensors], key=lambda x: x.name)),
        )
