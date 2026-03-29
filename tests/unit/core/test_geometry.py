"""Tests for geometry primitives."""

from __future__ import annotations

import math

import pytest
from linkforge_core.exceptions import RobotModelError
from linkforge_core.models import Box, Cylinder, Mesh, Sphere, Transform, Vector3


class TestVector3:
    """Tests for Vector3."""

    def test_creation(self) -> None:
        """Test creating a vector."""
        v = Vector3(1.0, 2.0, 3.0)
        assert v.x == 1.0
        assert v.y == 2.0
        assert v.z == 3.0

    def test_to_tuple(self) -> None:
        """Test converting to tuple."""
        v = Vector3(1.0, 2.0, 3.0)
        assert v.to_tuple() == (1.0, 2.0, 3.0)

    def test_iteration(self) -> None:
        """Test unpacking."""
        v = Vector3(1.0, 2.0, 3.0)
        x, y, z = v
        assert x == 1.0
        assert y == 2.0
        assert z == 3.0

    def test_string_representation(self) -> None:
        """Test string conversion for URDF."""
        v = Vector3(1.0, 2.0, 3.0)
        assert str(v) == "1.0 2.0 3.0"


class TestTransform:
    """Tests for Transform."""

    def test_identity(self) -> None:
        """Test identity transform."""
        t = Transform.identity()
        assert t.xyz == Vector3(0.0, 0.0, 0.0)
        assert t.rpy == Vector3(0.0, 0.0, 0.0)

    def test_creation(self) -> None:
        """Test creating a transform."""
        t = Transform(xyz=Vector3(1.0, 2.0, 3.0), rpy=Vector3(0.1, 0.2, 0.3))
        assert t.xyz.x == 1.0
        assert t.rpy.x == 0.1

    def test_string_representation(self) -> None:
        """Test string representation."""
        t = Transform(xyz=Vector3(1.0, 2.0, 3.0), rpy=Vector3(0.1, 0.2, 0.3))
        assert "xyz" in str(t)
        assert "rpy" in str(t)


class TestBox:
    """Tests for Box geometry."""

    def test_creation(self) -> None:
        """Test creating a box."""
        box = Box(size=Vector3(1.0, 2.0, 3.0))
        assert box.size.x == 1.0
        assert box.size.y == 2.0
        assert box.size.z == 3.0

    def test_volume(self) -> None:
        """Test volume calculation."""
        box = Box(size=Vector3(2.0, 3.0, 4.0))
        assert box.volume() == 24.0

    def test_type(self) -> None:
        """Test geometry type."""
        from linkforge_core.models import GeometryType

        box = Box(size=Vector3(1.0, 1.0, 1.0))
        assert box.type == GeometryType.BOX

    def test_negative_dimension_validation(self) -> None:
        """Test that negative dimensions are rejected."""
        with pytest.raises(RobotModelError, match="Box dimensions must be positive"):
            Box(size=Vector3(-1.0, 2.0, 3.0))

    def test_zero_dimension_validation(self) -> None:
        """Test that zero dimensions are rejected."""
        with pytest.raises(RobotModelError, match="Box dimensions must be positive"):
            Box(size=Vector3(1.0, 0.0, 3.0))


class TestCylinder:
    """Tests for Cylinder geometry."""

    def test_creation(self) -> None:
        """Test creating a cylinder."""
        cyl = Cylinder(radius=1.0, length=2.0)
        assert cyl.radius == 1.0
        assert cyl.length == 2.0

    def test_volume(self) -> None:
        """Test volume calculation."""
        cyl = Cylinder(radius=1.0, length=2.0)
        expected = math.pi * 1.0**2 * 2.0
        assert cyl.volume() == pytest.approx(expected)

    def test_type(self) -> None:
        """Test geometry type."""
        from linkforge_core.models import GeometryType

        cyl = Cylinder(radius=1.0, length=2.0)
        assert cyl.type == GeometryType.CYLINDER

    def test_negative_radius_validation(self) -> None:
        """Test that negative radius is rejected."""
        with pytest.raises(RobotModelError, match="Cylinder radius must be positive"):
            Cylinder(radius=-1.0, length=2.0)

    def test_zero_length_validation(self) -> None:
        """Test that zero length is rejected."""
        with pytest.raises(RobotModelError, match="Cylinder length must be positive"):
            Cylinder(radius=1.0, length=0.0)


class TestSphere:
    """Tests for Sphere geometry."""

    def test_creation(self) -> None:
        """Test creating a sphere."""
        sphere = Sphere(radius=1.0)
        assert sphere.radius == 1.0

    def test_volume(self) -> None:
        """Test volume calculation."""
        sphere = Sphere(radius=1.0)
        expected = (4.0 / 3.0) * math.pi * 1.0**3
        assert sphere.volume() == pytest.approx(expected)

    def test_type(self) -> None:
        """Test geometry type."""
        from linkforge_core.models import GeometryType

        sphere = Sphere(radius=1.0)
        assert sphere.type == GeometryType.SPHERE

    def test_negative_radius_validation(self) -> None:
        """Test that negative radius is rejected."""
        with pytest.raises(RobotModelError, match="Sphere radius must be positive"):
            Sphere(radius=-1.0)

    def test_zero_radius_validation(self) -> None:
        """Test that zero radius is rejected."""
        with pytest.raises(RobotModelError, match="Sphere radius must be positive"):
            Sphere(radius=0.0)


class TestMesh:
    """Tests for Mesh geometry."""

    def test_creation(self) -> None:
        """Test creating a mesh."""
        mesh = Mesh(resource="model.stl")
        assert mesh.resource == "model.stl"
        assert mesh.scale == Vector3(1.0, 1.0, 1.0)

    def test_creation_with_scale(self) -> None:
        """Test creating a mesh with custom scale."""
        mesh = Mesh(resource="model.stl", scale=Vector3(2.0, 2.0, 2.0))
        assert mesh.scale == Vector3(2.0, 2.0, 2.0)

    def test_type(self) -> None:
        """Test geometry type."""
        from linkforge_core.models import GeometryType

        mesh = Mesh(resource="model.stl")
        assert mesh.type == GeometryType.MESH

    def test_negative_scale_support(self) -> None:
        """Test that negative scale is now accepted for mirroring."""
        mesh = Mesh(resource="model.stl", scale=Vector3(-1.0, 1.0, 1.0))
        assert mesh.scale.x == -1.0

    def test_zero_scale_validation(self) -> None:
        """Test that zero scale is rejected."""
        with pytest.raises(RobotModelError, match="Mesh scale components must be non-zero"):
            Mesh(resource="model.stl", scale=Vector3(1.0, 0.0, 1.0))
