"""Gazebo-specific URDF extensions and elements."""

from __future__ import annotations

from dataclasses import dataclass, field

from ..exceptions import RobotValidationError


@dataclass(frozen=True)
class GazeboElement:
    """Generic Gazebo element that can be applied to robot, link, or joint.

    The <gazebo> tag in URDF allows specification of Gazebo-specific properties
    that are not part of the standard URDF specification.
    """

    reference: str | None = None  # Link or joint name (None for robot-level)
    properties: dict[str, str] = field(default_factory=dict)
    plugins: list[GazeboPlugin] = field(default_factory=list)

    # Common properties for links
    material: str | None = None  # Gazebo material (e.g., "Gazebo/Red")
    self_collide: bool | None = None
    static: bool | None = None
    gravity: bool | None = None

    # Common properties for joints
    stop_cfm: float | None = None  # Constraint force mixing for joint stops
    stop_erp: float | None = None  # Error reduction parameter for joint stops
    provide_feedback: bool | None = None  # Enable force-torque feedback
    implicit_spring_damper: bool | None = None

    # Friction parameters
    mu1: float | None = None  # Friction coefficient in first friction direction
    mu2: float | None = None  # Friction coefficient in second friction direction
    kp: float | None = None  # Contact stiffness
    kd: float | None = None  # Contact damping

    def __post_init__(self) -> None:
        """Validate Gazebo element."""
        # If reference is specified, it must be non-empty
        if self.reference is not None and not self.reference:
            raise RobotValidationError("GazeboReference", self.reference, "cannot be empty string")


@dataclass(frozen=True)
class GazeboPlugin:
    """Gazebo plugin specification.

    Plugins can be applied at robot, link, or joint level to extend Gazebo functionality.
    """

    name: str
    filename: str
    parameters: dict[str, str] = field(default_factory=dict)
    raw_xml: str | None = None  # Store raw XML content for round-trip fidelity

    def __post_init__(self) -> None:
        """Validate plugin configuration."""
        if not self.name:
            raise RobotValidationError("PluginName", self.name, "cannot be empty")
        if not self.filename:
            raise RobotValidationError("PluginFilename", self.filename, "cannot be empty")
