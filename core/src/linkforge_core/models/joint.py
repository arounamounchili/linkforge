"""Joint model representing connections between links."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..utils.string_utils import is_valid_urdf_name
from .geometry import Transform, Vector3


class JointType(Enum):
    """URDF joint types."""

    REVOLUTE = "revolute"  # Rotates around axis with limits
    CONTINUOUS = "continuous"  # Rotates around axis without limits
    PRISMATIC = "prismatic"  # Slides along axis with limits
    FIXED = "fixed"  # No motion
    FLOATING = "floating"  # 6 DOF (free in space)
    PLANAR = "planar"  # 2D motion in a plane


@dataclass(frozen=True)
class JointLimits:
    """Joint limits for revolute/prismatic joints.

    For CONTINUOUS joints, lower/upper are optional (only effort/velocity used).
    """

    lower: float | None = None  # Lower limit (radians for revolute, meters for prismatic)
    upper: float | None = None  # Upper limit
    effort: float = 0.0  # Maximum effort (N or Nm)
    velocity: float = 0.0  # Maximum velocity (rad/s or m/s)

    def __post_init__(self) -> None:
        """Validate limits."""
        # Only validate lower/upper relationship if both are provided
        if self.lower is not None and self.upper is not None and self.lower > self.upper:
            raise ValueError(f"Lower limit ({self.lower}) must be <= upper limit ({self.upper})")
        if self.effort < 0:
            raise ValueError(f"Effort must be non-negative, got {self.effort}")
        if self.velocity < 0:
            raise ValueError(f"Velocity must be non-negative, got {self.velocity}")


@dataclass(frozen=True)
class JointDynamics:
    """Joint dynamics properties."""

    damping: float = 0.0  # Damping coefficient
    friction: float = 0.0  # Friction coefficient

    def __post_init__(self) -> None:
        """Validate dynamics."""
        if self.damping < 0:
            raise ValueError(f"Damping must be non-negative, got {self.damping}")
        if self.friction < 0:
            raise ValueError(f"Friction must be non-negative, got {self.friction}")


@dataclass(frozen=True)
class JointMimic:
    """Joint mimic configuration (this joint mimics another)."""

    joint: str  # Name of joint to mimic
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass(frozen=True)
class JointSafetyController:
    """Safety controller settings for the joint.

    Attributes:
        soft_lower_limit: Lower bound of the joint safety controller.
        soft_upper_limit: Upper bound of the joint safety controller.
        k_position: Position gain.
        k_velocity: Velocity gain.
    """

    soft_lower_limit: float = 0.0
    soft_upper_limit: float = 0.0
    k_position: float = 0.0
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
    """Robot joint (connection between two links).

    Defines the kinematic relationship between parent and child links.
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
        """Validate joint configuration."""
        if not self.name:
            raise ValueError("Joint name cannot be empty")

        # Validate naming convention
        if not is_valid_urdf_name(self.name):
            raise ValueError(
                f"Joint name '{self.name}' contains invalid characters. "
                "Use only alphanumeric, underscore, or hyphen."
            )

        if not self.parent:
            raise ValueError("Parent link name cannot be empty")

        if not self.child:
            raise ValueError("Child link name cannot be empty")

        if self.parent == self.child:
            raise ValueError(f"Parent and child cannot be the same: {self.parent}")

        # Validate Axis requirements
        axis_required = self.type in (
            JointType.REVOLUTE,
            JointType.CONTINUOUS,
            JointType.PRISMATIC,
            JointType.PLANAR,
        )
        axis_forbidden = self.type in (JointType.FIXED, JointType.FLOATING)

        if axis_required and self.axis is None:
            raise ValueError(f"{self.type.value} joints require an axis")

        if axis_forbidden and self.axis is not None:
            raise ValueError(f"{self.type.value} joints must not have an axis")

        # Validate Limits
        # Revolute and prismatic joints require limits
        if self.type in (JointType.REVOLUTE, JointType.PRISMATIC) and self.limits is None:
            raise ValueError(f"{self.type.value} joints require limits")

        # Fixed and continuous joints should handle limits appropriately
        # Continuous joints use effort/velocity but not lower/upper
        # Fixed joints should not have limits element at all
        if self.type == JointType.FIXED and self.limits is not None:
            raise ValueError("Fixed joints must not have limits")

        # Validate and normalize axis if present
        if self.axis is not None:
            import math

            axis_magnitude = math.sqrt(self.axis.x**2 + self.axis.y**2 + self.axis.z**2)
            if axis_magnitude < 1e-10:
                raise ValueError(
                    f"Joint axis magnitude must be >= 1e-10, got {axis_magnitude:.2e} "
                    f"for axis=({self.axis.x}, {self.axis.y}, {self.axis.z})"
                )

            # Enforce normalized axis in model
            if abs(axis_magnitude - 1.0) > 1e-6:
                raise ValueError(
                    f"Joint axis must be a unit vector (magnitude 1.0), got {axis_magnitude:.4f}. "
                    "Normalize the axis vector before creating the Joint model."
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
