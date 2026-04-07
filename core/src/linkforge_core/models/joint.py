"""Joint model representing kinematic connections within the LinkForge Intermediate Representation (IR)."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ..exceptions import RobotValidationError
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
            raise RobotValidationError(
                "JointLimitRange", f"{self.lower} > {self.upper}", "Lower must be <= Upper"
            )
        if self.effort < 0:
            raise RobotValidationError("JointEffort", self.effort, "Must be non-negative")
        if self.velocity < 0:
            raise RobotValidationError("JointVelocity", self.velocity, "Must be non-negative")


@dataclass(frozen=True)
class JointDynamics:
    """Joint dynamics properties."""

    damping: float = 0.0  # Damping coefficient
    friction: float = 0.0  # Friction coefficient

    def __post_init__(self) -> None:
        """Validate dynamics."""
        if self.damping < 0:
            raise RobotValidationError("JointDamping", self.damping, "Must be non-negative")
        if self.friction < 0:
            raise RobotValidationError("JointFriction", self.friction, "Must be non-negative")


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
    The joint type determines whether an axis and limits are required or
    allowed.

    Note:
        - Revolute/Prismatic: Requires both `axis` and `limits`.
        - Continuous: Requires `axis`, limits are optional (no range).
        - Fixed: No axis or limits allowed.
        - Planar: Requires `axis`.
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
        """Validate joint configuration and kinematic constraints.

        Raises:
            RobotValidationError: If naming conventions, topology (parent==child),
                axis, or limit requirements for the joint type are violated.
        """
        if not self.name:
            raise RobotValidationError("JointName", self.name, "cannot be empty")

        # Validate naming convention
        if not is_valid_urdf_name(self.name):
            raise RobotValidationError("JointName", self.name, "Invalid characters")

        if not self.parent:
            raise RobotValidationError("ParentLink", self.parent, "cannot be empty")

        if not self.child:
            raise RobotValidationError("ChildLink", self.child, "cannot be empty")

        if self.parent == self.child:
            raise RobotValidationError(
                "JointTopology", self.parent, "Parent and child cannot be the same"
            )

        # Validate Axis requirements
        # Type-specific validation
        if self.type in (
            JointType.REVOLUTE,
            JointType.PRISMATIC,
            JointType.CONTINUOUS,
            JointType.PLANAR,
        ):
            if self.axis is None:
                raise RobotValidationError(
                    "JointAxis", self.type.value, "Axis required for this type"
                )
        elif self.type in (JointType.FIXED, JointType.FLOATING) and self.axis is not None:
            raise RobotValidationError(
                "JointAxis", self.type.value, "Axis not allowed for this type"
            )

        # Validate Limits
        if self.type in (JointType.REVOLUTE, JointType.PRISMATIC):
            if self.limits is None:
                raise RobotValidationError(
                    "JointLimits", self.type.value, "Limits required for this type"
                )
            # Limit validity is already checked in JointLimits.__post_init__ (RobotModelError)
        elif self.type == JointType.FIXED and self.limits is not None:
            raise RobotValidationError(
                "JointLimits", self.type.value, "Limits not allowed for this type"
            )

        # Validate and normalize axis if present
        if self.axis is not None:
            import math

            axis_magnitude = math.sqrt(self.axis.x**2 + self.axis.y**2 + self.axis.z**2)
            if axis_magnitude < 1e-10:
                raise RobotValidationError("JointAxisMagnitude", axis_magnitude, "Too small")

            # Enforce normalized axis in model
            if abs(axis_magnitude - 1.0) > 1e-6:
                raise RobotValidationError(
                    "JointAxisNormalization", axis_magnitude, "Must be unit vector"
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
