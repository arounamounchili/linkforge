"""Link model representing a rigid body within the LinkForge Intermediate Representation (IR)."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import InitVar, dataclass, field, replace

from ..exceptions import RobotPhysicsError, RobotValidationError, ValidationErrorCode
from ..utils.string_utils import is_valid_name
from .geometry import Geometry, Transform
from .material import Material


@dataclass(frozen=True)
class InertiaTensor:
    """3x3 inertia tensor representation.

    Symmetric tensor with 6 unique components:
    [ ixx  ixy  ixz ]
    [ ixy  iyy  iyz ]
    [ ixz  iyz  izz ]

    The tensor must be physically plausible. Diagonals must be non-zero
    positive values, and the principal moments must satisfy the triangle
    inequality.
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

        # Triangle inequality for principal moments
        # https://en.wikipedia.org/wiki/Moment_of_inertia#Principal_axes
        # Add a small epsilon tolerance for float precision issues (e.g. from CAD or Blender)
        epsilon = 1e-9
        if not (
            self.ixx + self.iyy >= self.izz - epsilon
            and self.iyy + self.izz >= self.ixx - epsilon
            and self.izz + self.ixx >= self.iyy - epsilon
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
        epsilon = 1e-6
        return cls(epsilon, 0.0, 0.0, epsilon, 0.0, epsilon)


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
    """Robot link (rigid body in the kinematic chain).

    A link is a rigid body with visual, collision, and inertial properties.
    The LinkForge model allows multiple visual and collision elements per link,
    supporting high-fidelity formats like URDF and SDF.
    """

    name: str
    initial_visuals: InitVar[Sequence[Visual] | None] = None
    initial_collisions: InitVar[Sequence[Collision] | None] = None
    inertial: Inertial | None = None

    _visuals: list[Visual] = field(default_factory=list, init=False)
    _collisions: list[Collision] = field(default_factory=list, init=False)

    def __post_init__(
        self,
        initial_visuals: Sequence[Visual] | None = None,
        initial_collisions: Sequence[Collision] | None = None,
    ) -> None:
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

        if initial_visuals:
            self._visuals.extend(initial_visuals)
        if initial_collisions:
            self._collisions.extend(initial_collisions)

    @property
    def visuals(self) -> list[Visual]:
        """Get visual representations."""
        return list(self._visuals)

    @property
    def collisions(self) -> list[Collision]:
        """Get collision representations."""
        return list(self._collisions)

    def add_visual(self, visual: Visual) -> None:
        """Add a visual representation."""
        self._visuals.append(visual)

    def add_collision(self, collision: Collision) -> None:
        """Add a collision representation."""
        self._collisions.append(collision)

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
        return Link(
            name=f"{prefix}{self.name}",
            initial_visuals=[v.with_prefix(prefix) for v in self._visuals],
            initial_collisions=[c.with_prefix(prefix) for c in self._collisions],
            inertial=self.inertial,
        )
