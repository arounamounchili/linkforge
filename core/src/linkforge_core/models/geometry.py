"""Geometry primitives for robot links."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path


class GeometryType(Enum):
    """Supported geometry types for URDF (standard specification only)."""

    BOX = "box"
    CYLINDER = "cylinder"
    SPHERE = "sphere"
    MESH = "mesh"


@dataclass(frozen=True)
class Vector3:
    """3D vector representation."""

    x: float
    y: float
    z: float

    def __iter__(self) -> Iterator[float]:
        """Allow unpacking: x, y, z = vector."""
        return iter((self.x, self.y, self.z))

    def to_tuple(self) -> tuple[float, float, float]:
        """Convert to tuple."""
        return (self.x, self.y, self.z)

    def __str__(self) -> str:
        """String representation for URDF."""
        return f"{self.x} {self.y} {self.z}"


@dataclass(frozen=True)
class Transform:
    """Spatial transformation (position + orientation).

    Uses XYZ position and RPY (Roll-Pitch-Yaw) orientation in radians.
    """

    xyz: Vector3 = Vector3(0.0, 0.0, 0.0)
    rpy: Vector3 = Vector3(0.0, 0.0, 0.0)  # Roll, Pitch, Yaw in radians

    @classmethod
    def identity(cls) -> Transform:
        """Create identity transform."""
        return cls()

    def __str__(self) -> str:
        """String representation."""
        return f"xyz: {self.xyz}, rpy: {self.rpy}"


# Geometry primitives


@dataclass(frozen=True)
class Box:
    """Box geometry (rectangular cuboid)."""

    size: Vector3  # width (x), depth (y), height (z)

    def __post_init__(self) -> None:
        """Validate box dimensions."""
        if self.size.x <= 0 or self.size.y <= 0 or self.size.z <= 0:
            raise ValueError(
                f"Box dimensions must be positive, got size=({self.size.x}, {self.size.y}, {self.size.z})"
            )

    @property
    def type(self) -> GeometryType:
        return GeometryType.BOX

    def volume(self) -> float:
        """Calculate volume."""
        return self.size.x * self.size.y * self.size.z


@dataclass(frozen=True)
class Cylinder:
    """Cylinder geometry (axis along Z)."""

    radius: float
    length: float  # height along Z axis

    def __post_init__(self) -> None:
        """Validate cylinder dimensions."""
        if self.radius <= 0:
            raise ValueError(f"Cylinder radius must be positive, got radius={self.radius}")
        if self.length <= 0:
            raise ValueError(f"Cylinder length must be positive, got length={self.length}")

    @property
    def type(self) -> GeometryType:
        return GeometryType.CYLINDER

    def volume(self) -> float:
        """Calculate volume."""
        import math

        return math.pi * self.radius**2 * self.length


@dataclass(frozen=True)
class Sphere:
    """Sphere geometry."""

    radius: float

    def __post_init__(self) -> None:
        """Validate sphere dimensions."""
        if self.radius <= 0:
            raise ValueError(f"Sphere radius must be positive, got radius={self.radius}")

    @property
    def type(self) -> GeometryType:
        return GeometryType.SPHERE

    def volume(self) -> float:
        """Calculate volume."""
        import math

        return (4.0 / 3.0) * math.pi * self.radius**3


@dataclass(frozen=True)
class Mesh:
    """Mesh geometry from file."""

    filepath: Path
    scale: Vector3 = field(default_factory=lambda: Vector3(1.0, 1.0, 1.0))

    def __post_init__(self) -> None:
        """Validate mesh scale."""
        if self.scale.x <= 0 or self.scale.y <= 0 or self.scale.z <= 0:
            raise ValueError(
                f"Mesh scale must be positive, got scale=({self.scale.x}, {self.scale.y}, {self.scale.z})"
            )

    @property
    def type(self) -> GeometryType:
        return GeometryType.MESH


# Type alias for any geometry (URDF standard primitives only)
Geometry = Box | Cylinder | Sphere | Mesh
