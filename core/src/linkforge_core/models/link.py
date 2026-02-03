"""Link model representing a robot link in URDF."""

from __future__ import annotations

from dataclasses import dataclass, field

from ..utils.string_utils import is_valid_urdf_name
from .geometry import Geometry, Transform
from .material import Material


@dataclass(frozen=True)
class InertiaTensor:
    """3x3 inertia tensor representation.

    Symmetric tensor with 6 unique components:
    [ ixx  ixy  ixz ]
    [ ixy  iyy  iyz ]
    [ ixz  iyz  izz ]
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
            raise ValueError("Diagonal inertia elements must be positive")

        # Triangle inequality for principal moments
        # https://en.wikipedia.org/wiki/Moment_of_inertia#Principal_axes
        # Add a small epsilon tolerance for float precision issues (e.g. from CAD or Blender)
        epsilon = 1e-9
        if not (
            self.ixx + self.iyy >= self.izz - epsilon
            and self.iyy + self.izz >= self.ixx - epsilon
            and self.izz + self.ixx >= self.iyy - epsilon
        ):
            raise ValueError("Inertia tensor violates triangle inequality")

    @classmethod
    def zero(cls) -> InertiaTensor:
        """Create a minimal valid inertia tensor (for massless links)."""
        epsilon = 1e-6
        return cls(epsilon, 0.0, 0.0, epsilon, 0.0, epsilon)


@dataclass(frozen=True)
class Inertial:
    """Inertial properties of a link."""

    mass: float
    origin: Transform = Transform.identity()
    inertia: InertiaTensor = field(default_factory=InertiaTensor.zero)

    def __post_init__(self) -> None:
        """Validate mass is non-negative."""
        if self.mass < 0:
            raise ValueError(f"Mass must be non-negative, got {self.mass}")


@dataclass(frozen=True)
class Visual:
    """Visual representation of a link."""

    geometry: Geometry
    origin: Transform = Transform.identity()
    material: Material | None = None
    name: str | None = None


@dataclass(frozen=True)
class Collision:
    """Collision representation of a link."""

    geometry: Geometry
    origin: Transform = Transform.identity()
    name: str | None = None


@dataclass(frozen=True)
class Link:
    """Robot link (rigid body in the kinematic chain).

    A link is a rigid body with visual, collision, and inertial properties.
    URDF allows multiple <visual> and <collision> elements per link.
    """

    name: str
    visuals: list[Visual] = field(default_factory=list)
    collisions: list[Collision] = field(default_factory=list)
    inertial: Inertial | None = None

    def __post_init__(self) -> None:
        """Validate link."""
        if not self.name:
            raise ValueError("Link name cannot be empty")

        # URDF naming convention: lowercase with underscores
        if not is_valid_urdf_name(self.name):
            raise ValueError(
                f"Link name '{self.name}' contains invalid characters. "
                "Use only alphanumeric, underscore, or hyphen."
            )

    @property
    def mass(self) -> float:
        """Get link mass (0.0 if no inertial properties)."""
        return self.inertial.mass if self.inertial else 0.0
