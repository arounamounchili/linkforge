"""Geometry primitives and spatial transformations for robot models.

Defines the structural building blocks for robot links and spatial pose
utilities.

Core Components:
    - Box, Cylinder, Sphere: Analytic primitive geometries.
    - Mesh: External geometry reference with scaling support.
    - Transform: 6-DOF spatial pose (XYZ + RPY).
    - Vector3: 3D spatial coordinate container.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass, field
from enum import StrEnum

from ..constants import (
    GEOM_BOX,
    GEOM_CYLINDER,
    GEOM_MESH,
    GEOM_SPHERE,
    PI,
)
from ..exceptions import RobotPhysicsError, ValidationErrorCode


class GeometryType(StrEnum):
    """Standard geometry primitives."""

    BOX = GEOM_BOX
    CYLINDER = GEOM_CYLINDER
    SPHERE = GEOM_SPHERE
    MESH = GEOM_MESH


@dataclass(frozen=True)
class Vector3:
    """3D vector representation for spatial coordinates, axes, and scaling."""

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
        """String representation."""
        return f"{self.x} {self.y} {self.z}"


@dataclass(frozen=True)
class Transform:
    """Spatial transformation representing a 6-DOF pose.

    Combines XYZ position (Vector3) and RPY (Roll-Pitch-Yaw) orientation
    in radians, following the standard robotics convention.
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
    """Box geometry representing a rectangular cuboid."""

    size: Vector3  # width (x), depth (y), height (z)

    def __post_init__(self) -> None:
        """Validate box dimensions."""
        if self.size.x <= 0 or self.size.y <= 0 or self.size.z <= 0:
            raise RobotPhysicsError(
                ValidationErrorCode.PHYSICS_VIOLATION,
                "Box dimensions must be positive",
                target="BoxSize",
                value=self.size,
            )

    @property
    def type(self) -> GeometryType:
        return GeometryType.BOX

    def volume(self) -> float:
        """Calculate volume."""
        return self.size.x * self.size.y * self.size.z


@dataclass(frozen=True)
class Cylinder:
    """Cylinder geometry aligned along the local Z-axis."""

    radius: float
    length: float  # height along Z axis

    def __post_init__(self) -> None:
        """Validate cylinder dimensions."""
        if self.radius <= 0:
            raise RobotPhysicsError(
                ValidationErrorCode.PHYSICS_VIOLATION,
                "Cylinder radius must be positive",
                target="CylinderRadius",
                value=self.radius,
            )
        if self.length <= 0:
            raise RobotPhysicsError(
                ValidationErrorCode.PHYSICS_VIOLATION,
                "Cylinder length must be positive",
                target="CylinderLength",
                value=self.length,
            )

    @property
    def type(self) -> GeometryType:
        return GeometryType.CYLINDER

    def volume(self) -> float:
        """Calculate volume of the cylinder."""
        return PI * (self.radius**2) * self.length


@dataclass(frozen=True)
class Sphere:
    """Sphere geometry."""

    radius: float

    def __post_init__(self) -> None:
        """Validate sphere dimensions."""
        if self.radius <= 0:
            raise RobotPhysicsError(
                ValidationErrorCode.PHYSICS_VIOLATION,
                "Sphere radius must be positive",
                target="SphereRadius",
                value=self.radius,
            )

    @property
    def type(self) -> GeometryType:
        return GeometryType.SPHERE

    def volume(self) -> float:
        """Calculate volume of the sphere."""
        return (4.0 / 3.0) * PI * (self.radius**3)


@dataclass(frozen=True)
class Mesh:
    """Mesh geometry from file or URI."""

    resource: str
    scale: Vector3 = field(default_factory=lambda: Vector3(1.0, 1.0, 1.0))

    def __post_init__(self) -> None:
        """Validate mesh scale."""
        if self.scale.x == 0 or self.scale.y == 0 or self.scale.z == 0:
            raise RobotPhysicsError(
                ValidationErrorCode.PHYSICS_VIOLATION,
                "Mesh scale components must be non-zero",
                target="MeshScale",
                value=self.scale,
            )

    @property
    def type(self) -> GeometryType:
        return GeometryType.MESH


# Type alias for any geometry primitive.
Geometry = Box | Cylinder | Sphere | Mesh
