"""Transmission models for ros2_control integration."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from enum import Enum

from ..exceptions import RobotModelError, RobotValidationError
from ..utils.string_utils import is_valid_urdf_name


class TransmissionType(str, Enum):
    """Standard transmission types in ros2_control."""

    SIMPLE = "transmission_interface/SimpleTransmission"
    DIFFERENTIAL = "transmission_interface/DifferentialTransmission"
    FOUR_BAR_LINKAGE = "transmission_interface/FourBarLinkageTransmission"
    CUSTOM = "custom"


class HardwareInterface(str, Enum):
    """Standard hardware interface types in ros2_control."""

    POSITION = "hardware_interface/PositionJointInterface"
    VELOCITY = "hardware_interface/VelocityJointInterface"
    EFFORT = "hardware_interface/EffortJointInterface"
    POSITION_VELOCITY = "hardware_interface/PositionVelocityJointInterface"

    # ROS2 control interfaces
    COMMAND_POSITION = "position"
    COMMAND_VELOCITY = "velocity"
    COMMAND_EFFORT = "effort"
    STATE_POSITION = "position"
    STATE_VELOCITY = "velocity"
    STATE_EFFORT = "effort"


@dataclass(frozen=True)
class TransmissionJoint:
    """Joint specification in a transmission.

    Defines how a joint is connected in the transmission with its hardware interface.
    """

    name: str
    hardware_interfaces: list[str] = field(default_factory=lambda: ["position"])
    mechanical_reduction: float = 1.0
    offset: float = 0.0

    def __post_init__(self) -> None:
        """Validate transmission joint."""
        if not self.name:
            raise RobotModelError()
        if not self.hardware_interfaces:
            raise RobotValidationError(
                check_name="HardwareInterfaces",
                value=self.name,
                reason="Must have at least one interface",
            )
        if self.mechanical_reduction == 0:
            raise RobotValidationError(
                check_name="MechanicalReduction", value=self.name, reason="Cannot be zero"
            )


@dataclass(frozen=True)
class TransmissionActuator:
    """Actuator specification in a transmission.

    Defines the actuator properties and its connection to the transmission.
    """

    name: str
    hardware_interfaces: list[str] = field(default_factory=lambda: ["position"])
    mechanical_reduction: float = 1.0
    offset: float = 0.0

    def __post_init__(self) -> None:
        """Validate transmission actuator."""
        if not self.name:
            raise RobotModelError()
        if not self.hardware_interfaces:
            raise RobotValidationError(
                check_name="HardwareInterfaces",
                value=self.name,
                reason="Must have at least one interface",
            )
        if self.mechanical_reduction == 0:
            raise RobotValidationError(
                check_name="MechanicalReduction", value=self.name, reason="Cannot be zero"
            )


@dataclass(frozen=True)
class Transmission:
    """Transmission definition for mapping between joints and actuators.

    Transmissions define the relationship between joints and actuators, handling
    mechanical reduction and other transformations. Used by ros_control/ros2_control.
    """

    name: str
    type: str  # Plugin name (e.g., TransmissionType enum or custom)
    joints: list[TransmissionJoint] = field(default_factory=list)
    actuators: list[TransmissionActuator] = field(default_factory=list)

    # Additional parameters for complex transmissions
    parameters: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Validate transmission configuration."""
        if not self.name:
            raise RobotModelError()
        if not self.type:
            raise RobotModelError()

        # Validate naming convention
        if not is_valid_urdf_name(self.name):
            raise RobotValidationError("TransmissionName", self.name, "Invalid characters")

        # Must have at least one joint
        if not self.joints:
            raise RobotValidationError(
                "TransmissionJoints", self.name, "Must have at least one joint"
            )

        # Check for duplicate joint names
        joint_names = [j.name for j in self.joints]
        duplicates = {name for name, count in Counter(joint_names).items() if count > 1}
        if duplicates:
            raise RobotValidationError("DuplicateJoints", self.name, str(duplicates))

        # Check for duplicate actuator names
        if self.actuators:
            actuator_names = [a.name for a in self.actuators]
            duplicates = {name for name, count in Counter(actuator_names).items() if count > 1}
            if duplicates:
                raise RobotValidationError("DuplicateActuators", self.name, str(duplicates))

    @classmethod
    def create_simple(
        cls,
        name: str,
        joint_name: str,
        actuator_name: str | None = None,
        mechanical_reduction: float = 1.0,
        hardware_interface: str = "position",
    ) -> Transmission:
        """Create a simple 1-to-1 transmission.

        Args:
            name: Transmission name
            joint_name: Name of the joint
            actuator_name: Name of the actuator (defaults to joint_name + "_motor")
            mechanical_reduction: Gear ratio (default 1.0)
            hardware_interface: Interface type (default "position")

        Returns:
            Configured simple transmission

        """
        if actuator_name is None:
            actuator_name = f"{joint_name}_motor"

        return cls(
            name=name,
            type=TransmissionType.SIMPLE.value,
            joints=[
                TransmissionJoint(
                    name=joint_name,
                    hardware_interfaces=[hardware_interface],
                    mechanical_reduction=mechanical_reduction,
                )
            ],
            actuators=[
                TransmissionActuator(
                    name=actuator_name,
                    hardware_interfaces=[hardware_interface],
                    mechanical_reduction=1.0,
                )
            ],
        )

    @classmethod
    def create_differential(
        cls,
        name: str,
        joint1_name: str,
        joint2_name: str,
        actuator1_name: str | None = None,
        actuator2_name: str | None = None,
        mechanical_reduction: float = 1.0,
        hardware_interface: str = "position",
    ) -> Transmission:
        """Create a differential transmission (2 actuators, 2 joints).

        Args:
            name: Transmission name
            joint1_name: First joint name
            joint2_name: Second joint name
            actuator1_name: First actuator name (defaults to joint1_name + "_motor")
            actuator2_name: Second actuator name (defaults to joint2_name + "_motor")
            mechanical_reduction: Gear ratio (default 1.0)
            hardware_interface: Interface type (default "position")

        Returns:
            Configured differential transmission

        """
        if actuator1_name is None:
            actuator1_name = f"{joint1_name}_motor"
        if actuator2_name is None:
            actuator2_name = f"{joint2_name}_motor"

        return cls(
            name=name,
            type=TransmissionType.DIFFERENTIAL.value,
            joints=[
                TransmissionJoint(
                    name=joint1_name,
                    hardware_interfaces=[hardware_interface],
                    mechanical_reduction=mechanical_reduction,
                ),
                TransmissionJoint(
                    name=joint2_name,
                    hardware_interfaces=[hardware_interface],
                    mechanical_reduction=mechanical_reduction,
                ),
            ],
            actuators=[
                TransmissionActuator(
                    name=actuator1_name,
                    hardware_interfaces=[hardware_interface],
                    mechanical_reduction=1.0,
                ),
                TransmissionActuator(
                    name=actuator2_name,
                    hardware_interfaces=[hardware_interface],
                    mechanical_reduction=1.0,
                ),
            ],
        )
