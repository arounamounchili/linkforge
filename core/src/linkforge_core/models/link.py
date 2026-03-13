"""Link model representing a robot link in URDF."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import InitVar, dataclass, field

from ..exceptions import RobotModelError
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
            raise RobotModelError("Diagonal inertia elements must be positive")

        # Triangle inequality for principal moments
        # https://en.wikipedia.org/wiki/Moment_of_inertia#Principal_axes
        # Add a small epsilon tolerance for float precision issues (e.g. from CAD or Blender)
        epsilon = 1e-9
        if not (
            self.ixx + self.iyy >= self.izz - epsilon
            and self.iyy + self.izz >= self.ixx - epsilon
            and self.izz + self.ixx >= self.iyy - epsilon
        ):
            raise RobotModelError("Inertia tensor violates triangle inequality")

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
            raise RobotModelError(f"Mass must be non-negative, got {self.mass}")


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


@dataclass
class Link:
    """Robot link (rigid body in the kinematic chain).

    A link is a rigid body with visual, collision, and inertial properties.
    URDF allows multiple <visual> and <collision> elements per link.
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
            raise RobotModelError("Link name cannot be empty")

        # URDF naming convention: lowercase with underscores
        if not is_valid_urdf_name(self.name):
            raise RobotModelError(
                f"Link name '{self.name}' contains invalid characters. "
                "Use only alphanumeric, underscore, or hyphen."
            )

        if initial_visuals:
            self._visuals.extend(initial_visuals)
        if initial_collisions:
            self._collisions.extend(initial_collisions)

    @property
    def visuals(self) -> Sequence[Visual]:
        """Get visual representations."""
        return tuple(self._visuals)

    @property
    def collisions(self) -> Sequence[Collision]:
        """Get collision representations."""
        return tuple(self._collisions)

    def add_visual(self, visual: Visual) -> None:
        """Add a visual representation."""
        self._visuals.append(visual)

    def add_collision(self, collision: Collision) -> None:
        """Add a collision representation."""
        self._collisions.append(collision)

    @property
    def mass(self) -> float:
        """Get link mass (0.0 if no inertial properties)."""
        return self.inertial.mass if self.inertial else 0.0
