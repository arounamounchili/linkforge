"""Gazebo simulation models and plugins.

This module provides data structures to represent Gazebo-specific properties,
bridging the gap between standard kinematic URDF and high-fidelity physics
simulation parameters.

Core Components:
- **Elements**: Container for link/joint specific physics (CFM, ERP, materials).
- **Plugins**: Functional extensions for sensors, controllers, and physics.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field, replace

from ..exceptions import RobotValidationError, ValidationErrorCode


@dataclass(frozen=True)
class GazeboElement:
    """Simulation-specific metadata container for Gazebo.

    Encapsulates parameters that are not part of the standard URDF spec but
    are required for high-fidelity physics (e.g., DART/ODE parameters) or
    visual appearance in Gazebo.
    """

    reference: str | None = None  # Link or joint name (None for robot-level)
    properties: dict[str, str] = field(default_factory=dict)
    plugins: Sequence[GazeboPlugin] = field(default_factory=tuple)

    # Common properties for links (platform-specific)
    material: str | None = None  # Gazebo material (e.g., "Gazebo/Red")
    static: bool | None = None

    # Common properties for joints (platform-specific)
    stop_cfm: float | None = None  # Constraint force mixing for joint stops
    stop_erp: float | None = None  # Error reduction parameter for joint stops
    provide_feedback: bool | None = None  # Enable force-torque feedback
    implicit_spring_damper: bool | None = None

    def __post_init__(self) -> None:
        """Validate Gazebo element."""
        # If reference is specified, it must be non-empty
        if self.reference is not None and not self.reference:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY,
                "Gazebo reference cannot be empty",
                target="GazeboReference",
            )
        object.__setattr__(self, "plugins", tuple(self.plugins))

    def with_prefix(self, prefix: str) -> GazeboElement:
        """Create a new gazebo element with prefixed reference and plugins."""
        return replace(
            self,
            reference=f"{prefix}{self.reference}" if self.reference else None,
            plugins=tuple(p.with_prefix(prefix) for p in self.plugins),
        )


@dataclass(frozen=True)
class GazeboPlugin:
    """Gazebo plugin specification for functional extensions."""

    name: str
    filename: str
    parameters: dict[str, str] = field(default_factory=dict)
    raw_xml: str | None = field(
        default=None, compare=False
    )  # Store raw XML content for round-trip fidelity

    def __post_init__(self) -> None:
        """Validate plugin configuration."""
        if not self.name:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY, "Plugin name cannot be empty", target="PluginName"
            )
        if not self.filename:
            raise RobotValidationError(
                ValidationErrorCode.VALUE_EMPTY,
                "Plugin filename cannot be empty",
                target="PluginFilename",
            )

    def with_prefix(self, prefix: str) -> GazeboPlugin:
        """Create a new plugin with a prefixed name."""
        return replace(self, name=f"{prefix}{self.name}")
