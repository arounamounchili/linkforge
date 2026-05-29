"""Gazebo simulation models and plugins.

This module provides data structures to represent Gazebo-specific properties,
bridging the gap between standard kinematic URDF and high-fidelity physics
simulation parameters.

Core Components:
- **Elements**: Container for link/joint specific physics (CFM, ERP, materials).
- **Plugins**: Functional extensions for sensors, controllers, and physics.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
from types import MappingProxyType
from typing import Any

from ..exceptions import RobotValidationError, ValidationErrorCode


@dataclass(frozen=True, slots=True)
class GazeboElement:
    """Simulation-specific metadata container for Gazebo.

    Encapsulates parameters that are not part of the standard URDF spec but
    are required for high-fidelity physics (e.g., DART/ODE parameters) or
    visual appearance in Gazebo.

    All mutable inputs (``properties`` dict, ``plugins`` sequence) are
    converted to immutable structures in ``__post_init__`` to guarantee
    true deep immutability even though the class is ``frozen=True``.
    """

    reference: str | None = None  # Link or joint name (None for robot-level)
    properties: Mapping[str, str] = field(default_factory=dict)
    plugins: Sequence[GazeboPlugin] = field(default_factory=tuple)

    # Common properties for links (platform-specific)
    material: str | None = None  # Gazebo material (e.g., "Gazebo/Red")
    static: bool | None = None

    # Common properties for joints (platform-specific)
    stop_cfm: float | None = None  # Constraint force mixing for joint stops (>= 0)
    stop_erp: float | None = None  # Error reduction parameter for joint stops (∈ [0, 1])
    provide_feedback: bool | None = None  # Enable force-torque feedback
    implicit_spring_damper: bool | None = None

    def __post_init__(self) -> None:
        """Validate and deeply immutabilise the Gazebo element."""
        # If reference is specified, it must be non-blank
        if self.reference is not None and not self.reference.strip():
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY,
                "Gazebo reference cannot be empty or whitespace-only",
                target="GazeboReference",
            )

        # Deep-immutabilise mutable fields
        object.__setattr__(self, "plugins", tuple(self.plugins))
        object.__setattr__(self, "properties", MappingProxyType(dict(self.properties)))

        # --- Physics parameter range validation ---
        # CFM (Constraint Force Mixing) must be non-negative.
        # A value of 0 means a perfectly rigid constraint; negative values are physically invalid.
        if self.stop_cfm is not None and self.stop_cfm < 0.0:
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                f"stop_cfm must be >= 0 (got {self.stop_cfm}). "
                "Negative CFM values are physically invalid for ODE/DART solvers.",
                target="GazeboStopCFM",
                value=str(self.stop_cfm),
            )

        # ERP (Error Reduction Parameter) must be in [0, 1].
        # 0 = no correction, 1 = full correction per step.
        if self.stop_erp is not None and not (0.0 <= self.stop_erp <= 1.0):
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                f"stop_erp must be in [0.0, 1.0] (got {self.stop_erp}). "
                "Values outside this range produce unstable physics simulation.",
                target="GazeboStopERP",
                value=str(self.stop_erp),
            )

    def with_prefix(self, prefix: str) -> GazeboElement:
        """Create a new gazebo element with prefixed reference and plugins."""
        return replace(
            self,
            reference=f"{prefix}{self.reference}" if self.reference else None,
            plugins=tuple(p.with_prefix(prefix) for p in self.plugins),
        )

    def __deepcopy__(self, memo: dict[int, Any]) -> GazeboElement:
        """Deep copy: Since this object is deeply immutable, return self."""
        return self


@dataclass(frozen=True, slots=True)
class GazeboPlugin:
    """Gazebo plugin specification for functional extensions.

    ``parameters`` is stored as an immutable :class:`~types.MappingProxyType`
    regardless of what is passed at construction time, ensuring that the
    ``frozen=True`` contract is not silently violated by callers mutating the dict.
    """

    name: str
    filename: str
    parameters: Mapping[str, str] = field(default_factory=dict)
    raw_xml: str | None = field(
        default=None, compare=False
    )  # Store raw XML content for round-trip fidelity

    def __post_init__(self) -> None:
        """Validate and deeply immutabilise plugin configuration."""
        if not self.name.strip():
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY,
                "Plugin name cannot be empty or whitespace-only",
                target="PluginName",
            )
        if not self.filename.strip():
            raise RobotValidationError(
                ValidationErrorCode.VALUE_EMPTY,
                "Plugin filename cannot be empty or whitespace-only",
                target="PluginFilename",
            )
        object.__setattr__(self, "parameters", MappingProxyType(dict(self.parameters)))

    def with_prefix(self, prefix: str) -> GazeboPlugin:
        """Create a new plugin with a prefixed name."""
        return replace(self, name=f"{prefix}{self.name}")

    def __deepcopy__(self, memo: dict[int, Any]) -> GazeboPlugin:
        """Deep copy: Since this object is deeply immutable, return self."""
        return self
