"""Link model representing rigid bodies and their physical properties.

This module provides the core data structures for robot links, including
inertial properties, collision geometry, visual appearance, and surface
physics parameters.

Core Components:
- **Physics**: Inertia tensors, mass, and surface properties (friction).
- **Geometry**: Visual and collision representations.
- **Traversal**: Support for namespacing and deep-copy transformations.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field, replace

from .._utils.string_utils import is_valid_name
from ..constants import (
    DEFAULT_CONTACT_KD,
    DEFAULT_CONTACT_KP,
    DEFAULT_FRICTION_MU,
    DEFAULT_FRICTION_MU2,
    DEFAULT_GRAVITY,
    DEFAULT_SELF_COLLIDE,
    EPSILON,
    MIN_REASONABLE_INERTIA,
)
from ..exceptions import RobotPhysicsError, RobotValidationError, ValidationErrorCode
from .geometry import Geometry, Transform
from .material import Material


@dataclass(frozen=True)
class InertiaTensor:
    """3x3 inertia tensor representation.

    Symmetric tensor with 6 unique components:
    [ ixx  ixy  ixz ]
    [ ixy  iyy  iyz ]
    [ ixz  iyz  izz ]

    The tensor must be physically plausible. Diagonals must be positive
    values, and the principal moments must satisfy the triangle
    inequality for rigid body mass distribution.
    """

    ixx: float
    ixy: float
    ixz: float
    iyy: float
    iyz: float
    izz: float

    def __post_init__(self) -> None:
        """Validate inertia tensor values."""
        # All diagonal elements must be positive
        if self.ixx <= 0 or self.iyy <= 0 or self.izz <= 0:
            raise RobotPhysicsError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Diagonal inertia components must be positive",
                target="DiagonalInertia",
                value=(self.ixx, self.iyy, self.izz),
            )

        # Principal moments triangle inequality
        # https://en.wikipedia.org/wiki/Moment_of_inertia#Principal_axes
        if not (
            self.ixx + self.iyy >= self.izz - EPSILON
            and self.iyy + self.izz >= self.ixx - EPSILON
            and self.izz + self.ixx >= self.iyy - EPSILON
        ):
            raise RobotPhysicsError(
                ValidationErrorCode.INERTIA_TRIANGLE_INEQUALITY,
                "Inertia tensor violates triangle inequality (unphysical)",
                target="InertiaTriangleInequality",
                value=(self.ixx, self.iyy, self.izz),
            )

    @classmethod
    def zero(cls) -> InertiaTensor:
        """Create a minimal valid inertia tensor (for massless links)."""
        return cls(
            MIN_REASONABLE_INERTIA,
            0.0,
            0.0,
            MIN_REASONABLE_INERTIA,
            0.0,
            MIN_REASONABLE_INERTIA,
        )


@dataclass(frozen=True)
class Inertial:
    """Inertial properties of a robot link."""

    mass: float
    origin: Transform = Transform.identity()
    inertia: InertiaTensor = field(default_factory=InertiaTensor.zero)

    def __post_init__(self) -> None:
        """Validate inertial properties.

        Raises:
            RobotPhysicsError: If mass is negative.
        """
        if self.mass < 0:
            raise RobotPhysicsError(
                ValidationErrorCode.OUT_OF_RANGE,
                "Mass must be non-negative",
                target="Mass",
                value=self.mass,
            )


@dataclass(frozen=True)
class LinkPhysics:
    """Surface and contact physics properties for a link.

    Defines how the link interacts with other objects in a physics simulator
    (e.g., friction, stiffness, damping).
    """

    self_collide: bool = DEFAULT_SELF_COLLIDE
    gravity: bool = DEFAULT_GRAVITY
    mu: float = DEFAULT_FRICTION_MU
    mu2: float = DEFAULT_FRICTION_MU2
    kp: float = DEFAULT_CONTACT_KP
    kd: float = DEFAULT_CONTACT_KD


@dataclass(frozen=True)
class Visual:
    """Visual representation of a link."""

    geometry: Geometry
    origin: Transform = Transform.identity()
    material: Material | None = None
    name: str | None = None

    def with_prefix(self, prefix: str) -> Visual:
        """Create a new visual with a prefixed name and material."""
        return replace(
            self,
            name=f"{prefix}{self.name}" if self.name else None,
            material=self.material.with_prefix(prefix) if self.material else None,
        )


@dataclass(frozen=True)
class Collision:
    """Collision representation of a link."""

    geometry: Geometry
    origin: Transform = Transform.identity()
    name: str | None = None

    def with_prefix(self, prefix: str) -> Collision:
        """Create a new collision with a prefixed name."""
        return replace(self, name=f"{prefix}{self.name}" if self.name else None)


@dataclass
class Link:
    """Robot link representing a rigid body in a kinematic chain.

    A link is a rigid body defined by its name, physical properties (inertial),
    and geometric representations (visual/collision). It serves as a node
    in the kinematic graph.
    """

    name: str
    inertial: Inertial | None = None
    physics: LinkPhysics = field(default_factory=LinkPhysics)
    visuals: Sequence[Visual] = field(default_factory=tuple)
    collisions: Sequence[Collision] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        """Validate link."""
        if not self.name:
            raise RobotValidationError(
                ValidationErrorCode.NAME_EMPTY, "Link name cannot be empty", target="LinkName"
            )

        # Standard naming convention: alphanumeric and underscores
        if not is_valid_name(self.name):
            raise RobotValidationError(
                ValidationErrorCode.INVALID_NAME,
                "Invalid characters in link name",
                target="LinkName",
                value=self.name,
            )

        # Enforce immutable collections
        self.visuals = tuple(self.visuals)
        self.collisions = tuple(self.collisions)

    def add_visual(self, visual: Visual) -> None:
        """Add a visual representation."""
        self.visuals = (*self.visuals, visual)

    def add_collision(self, collision: Collision) -> None:
        """Add a collision representation."""
        self.collisions = (*self.collisions, collision)

    @property
    def mass(self) -> float:
        """Get link mass (0.0 if no inertial properties are defined)."""
        return self.inertial.mass if self.inertial else 0.0

    @property
    def inertia(self) -> InertiaTensor:
        """Get link inertia tensor (zero tensor if not defined)."""
        return self.inertial.inertia if self.inertial else InertiaTensor.zero()

    @property
    def inertial_origin(self) -> Transform:
        """Get inertial origin (identity if not defined)."""
        return self.inertial.origin if self.inertial else Transform.identity()

    def with_prefix(self, prefix: str) -> Link:
        """Create a new link with a prefixed name and sub-elements."""
        return replace(
            self,
            name=f"{prefix}{self.name}",
            visuals=tuple(v.with_prefix(prefix) for v in self.visuals),
            collisions=tuple(c.with_prefix(prefix) for c in self.collisions),
        )
