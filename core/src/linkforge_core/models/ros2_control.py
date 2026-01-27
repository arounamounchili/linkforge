"""ros2_control data models for ROS 2 control configuration."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Ros2ControlJoint:
    """Joint configuration in ros2_control block.

    Represents a joint's control interfaces in a ros2_control system.
    """

    name: str
    command_interfaces: list[str] = field(default_factory=list)
    state_interfaces: list[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        """Validate joint configuration."""
        if not self.name:
            raise ValueError("Joint name cannot be empty")
        if not self.command_interfaces:
            raise ValueError(f"Joint '{self.name}' must have at least one command interface")
        if not self.state_interfaces:
            raise ValueError(f"Joint '{self.name}' must have at least one state interface")


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
            raise ValueError("ros2_control name cannot be empty")
        if self.type not in ("system", "actuator", "sensor"):
            raise ValueError(f"Invalid ros2_control type: {self.type}")
        if not self.hardware_plugin:
            raise ValueError("Hardware plugin cannot be empty")
