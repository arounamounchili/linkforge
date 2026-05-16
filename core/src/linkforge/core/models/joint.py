"""Joint model representing kinematic connections between robot links.

This module defines the relationships between parent and child links,
including motion types, limits, dynamics, and safety configurations.

Core Components:
- **Topology**: Parent-child relationship and naming.
- **Motion**: Joint types (Revolute, Prismatic, etc.) and axis definition.
- **Constraints**: Limits, dynamics (friction/damping), and safety controllers.
- **Specialized**: Mimic joints and calibration metadata.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import StrEnum

from .._utils.string_utils import is_valid_name
from ..constants import (
    DEFAULT_JOINT_DAMPING,
    DEFAULT_JOINT_EFFORT,
    DEFAULT_JOINT_FRICTION,
    DEFAULT_JOINT_VELOCITY,
    EPSILON,
    JOINT_CONTINUOUS,
    JOINT_FIXED,
    JOINT_FLOATING,
    JOINT_PLANAR,
    JOINT_PRISMATIC,
    JOINT_REVOLUTE,
)
from ..exceptions import RobotValidationError, ValidationErrorCode
from .geometry import Transform, Vector3


class JointType(StrEnum):
    """Standard robot joint types."""

    REVOLUTE = JOINT_REVOLUTE  # Rotates around axis with limits
    CONTINUOUS = JOINT_CONTINUOUS  # Rotates around axis without limits
    PRISMATIC = JOINT_PRISMATIC  # Slides along axis with limits
    FIXED = JOINT_FIXED  # No motion
    FLOATING = JOINT_FLOATING  # 6 DOF (free in space)
    PLANAR = JOINT_PLANAR  # 2D motion in a plane


@dataclass(frozen=True)
class JointLimits:
    """Joint limits for revolute/prismatic joints.

    For CONTINUOUS joints, lower/upper are optional (only effort/velocity used).
    """

    lower: float | None = None  # Lower limit (radians for revolute, meters for prismatic)
    upper: float | None = None  # Upper limit
    effort: float = DEFAULT_JOINT_EFFORT  # Maximum effort (N or Nm)
    velocity: float = DEFAULT_JOINT_VELOCITY  # Maximum velocity (rad/s or m/s)

    def __post_init__(self) -> None:
        """Validate limits."""
        # Only validate lower/upper relationship if both are provided
        if self.lower is not None and self.upper is not None and self.lower > self.upper:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Lower limit must be <= Upper limit",
                target="JointLimitRange",
                value=(self.lower, self.upper),
            )
        if self.effort < 0:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Effort must be non-negative",
                target="JointEffort",
                value=self.effort,
            )
        if self.velocity < 0:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Velocity must be non-negative",
                target="JointVelocity",
                value=self.velocity,
            )


@dataclass(frozen=True)
class JointDynamics:
    """Joint dynamics properties defining physical behavior."""

    damping: float = DEFAULT_JOINT_DAMPING
    friction: float = DEFAULT_JOINT_FRICTION

    def __post_init__(self) -> None:
        """Validate dynamics."""
        if self.damping < 0:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Damping must be non-negative",
                target="JointDamping",
                value=self.damping,
            )
        if self.friction < 0:
            raise RobotValidationError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Friction must be non-negative",
                target="JointFriction",
                value=self.friction,
            )


@dataclass(frozen=True)
class JointMimic:
    """Joint mimic configuration (this joint mimics another)."""

    joint: str  # Name of joint to mimic
    multiplier: float = 1.0
    offset: float = 0.0

    def with_prefix(self, prefix: str) -> JointMimic:
        """Create a new mimic with a prefixed joint name."""
        return replace(self, joint=f"{prefix}{self.joint}")


@dataclass(frozen=True)
class JointSafetyController:
    """Safety controller settings for the joint.

    Attributes:
        soft_lower_limit: Lower bound of the joint safety controller.
        soft_upper_limit: Upper bound of the joint safety controller.
        k_position: Position gain.
        k_velocity: Velocity gain.
    """

    soft_lower_limit: float | None = None
    soft_upper_limit: float | None = None
    k_position: float | None = None
    k_velocity: float = 0.0


@dataclass(frozen=True)
class JointCalibration:
    """Calibration settings for the joint.

    Attributes:
        rising: Position of the rising edge.
        falling: Position of the falling edge.
    """

    rising: float | None = None
    falling: float | None = None


@dataclass(frozen=True)
class Joint:
    """Robot joint defining the kinematic connection between two links.

    A joint couples a parent link to a child link with a specific degree
    of freedom (DOF) and mechanical limits. It defines the coordinate
    transformation from parent to child.
    """

    name: str
    type: JointType
    parent: str  # Parent link name
    child: str  # Child link name
    origin: Transform = Transform.identity()
    axis: Vector3 | None = None  # Joint axis (required for revolute/prismatic/planar)
    limits: JointLimits | None = None
    dynamics: JointDynamics | None = None
    mimic: JointMimic | None = None
    safety_controller: JointSafetyController | None = None
    calibration: JointCalibration | None = None

    def __post_init__(self) -> None:
        """Validate and normalize joint properties."""
        # Validate name
        if not self.name:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY,
                "Joint name cannot be empty",
                target="JointName",
                value=self.name,
            )
        if not is_valid_name(self.name):
            raise RobotValidationError(
                ValidationErrorCode.INVALID_NAME,
                "Invalid characters in joint name",
                target="JointName",
                value=self.name,
            )

        if not self.parent:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY,
                "Parent link name cannot be empty",
                target="ParentLink",
            )
        if not self.child:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY,
                "Child link name cannot be empty",
                target="ChildLink",
            )
        if self.parent == self.child:
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                "Parent and child links cannot be the same",
                target="JointTopology",
                value=self.parent,
            )

        # Validate Axis Requirements
        if self.type in (
            JointType.REVOLUTE,
            JointType.CONTINUOUS,
            JointType.PRISMATIC,
            JointType.PLANAR,
        ):
            if self.axis is None:
                raise RobotValidationError(
                    ValidationErrorCode.INVALID_VALUE,
                    f"Axis required for joint type '{self.type.value}'",
                    target="JointAxis",
                    value=self.type.value,
                )
        elif self.type in (JointType.FIXED, JointType.FLOATING) and self.axis is not None:
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                f"Axis not allowed for joint type '{self.type.value}'",
                target="JointAxis",
                value=self.type.value,
            )

        # Validate Limits
        if self.type in (JointType.REVOLUTE, JointType.PRISMATIC):
            if self.limits is None:
                raise RobotValidationError(
                    ValidationErrorCode.INVALID_VALUE,
                    f"Limits required for joint type '{self.type.value}'",
                    target="JointLimits",
                    value=self.type.value,
                )
        elif self.type == JointType.FIXED and self.limits is not None:
            raise RobotValidationError(
                ValidationErrorCode.INVALID_VALUE,
                f"Limits not allowed for joint type '{self.type.value}'",
                target="JointLimits",
                value=self.type.value,
            )

        # Validate and normalize axis if present
        if self.axis is not None:
            axis_sq_mag = self.axis.x**2 + self.axis.y**2 + self.axis.z**2
            axis_magnitude = math.sqrt(axis_sq_mag)
            if axis_magnitude < EPSILON:
                raise RobotValidationError(
                    ValidationErrorCode.INVALID_VALUE,
                    "Joint axis cannot be a zero vector",
                    target="JointAxis",
                    value=self.axis,
                )

            # Non-unit axis vectors are not allowed in the constructor for strict IR
            if abs(axis_magnitude - 1.0) > EPSILON:
                raise RobotValidationError(
                    ValidationErrorCode.INVALID_VALUE,
                    "Joint axis must be a unit vector",
                    target="JointAxisNormalization",
                    value=axis_magnitude,
                )

    @property
    def degrees_of_freedom(self) -> int:
        """Get number of degrees of freedom."""
        dof_map = {
            JointType.FIXED: 0,
            JointType.REVOLUTE: 1,
            JointType.CONTINUOUS: 1,
            JointType.PRISMATIC: 1,
            JointType.PLANAR: 2,
            JointType.FLOATING: 6,
        }
        return dof_map[self.type]

    def with_prefix(self, prefix: str) -> Joint:
        """Create a new joint with prefixed name, parent, child, and mimic."""
        return replace(
            self,
            name=f"{prefix}{self.name}",
            parent=f"{prefix}{self.parent}",
            child=f"{prefix}{self.child}",
            mimic=self.mimic.with_prefix(prefix) if self.mimic else None,
        )
