"""Tests for inertia calculations."""

from __future__ import annotations

import pytest
from linkforge_core.exceptions import RobotModelError, RobotPhysicsError
from linkforge_core.models import Box, Cylinder, InertiaTensor, Mesh, Sphere, Vector3
from linkforge_core.physics import (
    calculate_box_inertia,
    calculate_cylinder_inertia,
    calculate_inertia,
    calculate_mesh_inertia_approximation,
    calculate_mesh_inertia_from_triangles,
    calculate_sphere_inertia,
)


class TestInertiaTensor:
    """Tests for InertiaTensor."""

    def test_creation(self) -> None:
        """Test creating an inertia tensor."""
        inertia = InertiaTensor(ixx=1.0, ixy=0.0, ixz=0.0, iyy=1.0, iyz=0.0, izz=1.0)
        assert inertia.ixx == 1.0
        assert inertia.iyy == 1.0
        assert inertia.izz == 1.0

    def test_zero_inertia(self) -> None:
        """Test zero inertia tensor."""
        inertia = InertiaTensor.zero()
        assert inertia.ixx > 0
        assert inertia.iyy > 0
        assert inertia.izz > 0

    def test_invalid_negative_diagonal(self) -> None:
        """Test that negative diagonal elements raise error."""
        with pytest.raises(RobotModelError, match="positive"):
            InertiaTensor(ixx=-1.0, ixy=0.0, ixz=0.0, iyy=1.0, iyz=0.0, izz=1.0)

    def test_invalid_triangle_inequality(self) -> None:
        """Test that triangle inequality violation raises error."""
        with pytest.raises(RobotModelError, match="InertiaTriangleInequality"):
            # This violates Ixx + Iyy >= Izz
            InertiaTensor(ixx=1.0, ixy=0.0, ixz=0.0, iyy=1.0, iyz=0.0, izz=10.0)


class TestBoxInertia:
    """Tests for box inertia calculation."""

    def test_unit_cube(self) -> None:
        """Test inertia of unit cube with unit mass."""
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        inertia = calculate_box_inertia(box, mass=1.0)

        # For unit cube: I = (1/12) * m * (1² + 1²) = 1/6
        expected = 1.0 / 6.0
        assert inertia.ixx == pytest.approx(expected)
        assert inertia.iyy == pytest.approx(expected)
        assert inertia.izz == pytest.approx(expected)
        assert inertia.ixy == 0.0
        assert inertia.ixz == 0.0
        assert inertia.iyz == 0.0

    def test_rectangular_box(self) -> None:
        """Test inertia of rectangular box."""
        box = Box(size=Vector3(2.0, 3.0, 4.0))
        mass = 10.0
        inertia = calculate_box_inertia(box, mass)

        # Ixx = (1/12) * m * (y² + z²) = (1/12) * 10 * (9 + 16) = 20.833...
        ixx = (1.0 / 12.0) * mass * (3.0**2 + 4.0**2)
        iyy = (1.0 / 12.0) * mass * (2.0**2 + 4.0**2)
        izz = (1.0 / 12.0) * mass * (2.0**2 + 3.0**2)

        assert inertia.ixx == pytest.approx(ixx)
        assert inertia.iyy == pytest.approx(iyy)
        assert inertia.izz == pytest.approx(izz)

    def test_zero_mass(self) -> None:
        """Test that zero mass returns zero inertia."""
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        inertia = calculate_box_inertia(box, mass=0.0)
        assert inertia == InertiaTensor.zero()


class TestCylinderInertia:
    """Tests for cylinder inertia calculation."""

    def test_unit_cylinder(self) -> None:
        """Test inertia of unit cylinder."""
        cyl = Cylinder(radius=1.0, length=2.0)
        mass = 1.0
        inertia = calculate_cylinder_inertia(cyl, mass)

        # Ixx = Iyy = (1/12) * m * (3r² + h²) = (1/12) * 1 * (3 + 4) = 7/12
        # Izz = (1/2) * m * r² = 0.5
        ixx = (1.0 / 12.0) * mass * (3 * 1.0**2 + 2.0**2)
        izz = 0.5 * mass * 1.0**2

        assert inertia.ixx == pytest.approx(ixx)
        assert inertia.iyy == pytest.approx(ixx)
        assert inertia.izz == pytest.approx(izz)
        assert inertia.ixy == 0.0

    def test_zero_mass(self) -> None:
        """Test that zero mass returns zero inertia."""
        cyl = Cylinder(radius=1.0, length=1.0)
        inertia = calculate_cylinder_inertia(cyl, mass=0.0)
        assert inertia == InertiaTensor.zero()


class TestSphereInertia:
    """Tests for sphere inertia calculation."""

    def test_unit_sphere(self) -> None:
        """Test inertia of unit sphere."""
        sphere = Sphere(radius=1.0)
        mass = 1.0
        inertia = calculate_sphere_inertia(sphere, mass)

        # I = (2/5) * m * r² = 0.4
        expected = (2.0 / 5.0) * mass * 1.0**2

        assert inertia.ixx == pytest.approx(expected)
        assert inertia.iyy == pytest.approx(expected)
        assert inertia.izz == pytest.approx(expected)

    def test_zero_mass(self) -> None:
        """Test that zero mass returns zero inertia."""
        sphere = Sphere(radius=1.0)
        inertia = calculate_sphere_inertia(sphere, mass=0.0)
        assert inertia == InertiaTensor.zero()


class TestCalculateInertia:
    """Tests for generic calculate_inertia function."""

    def test_box(self) -> None:
        """Test calculation for box."""
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        inertia = calculate_inertia(box, mass=1.0)
        expected = calculate_box_inertia(box, mass=1.0)
        assert inertia == expected

    def test_cylinder(self) -> None:
        """Test calculation for cylinder."""
        cyl = Cylinder(radius=1.0, length=1.0)
        inertia = calculate_inertia(cyl, mass=1.0)
        expected = calculate_cylinder_inertia(cyl, mass=1.0)
        assert inertia == expected

    def test_sphere(self) -> None:
        """Test calculation for sphere."""
        sphere = Sphere(radius=1.0)
        inertia = calculate_inertia(sphere, mass=1.0)
        expected = calculate_sphere_inertia(sphere, mass=1.0)
        assert inertia == expected

    def test_mesh(self) -> None:
        """Test calculation for mesh."""
        mesh = Mesh(resource="test.stl")
        inertia = calculate_inertia(mesh, mass=1.0)
        expected = calculate_mesh_inertia_approximation(mesh, mass=1.0)
        assert inertia == expected

    def test_zero_mass(self) -> None:
        """Test that zero mass returns zero inertia."""
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        inertia = calculate_inertia(box, mass=0.0)
        assert inertia == InertiaTensor.zero()

    def test_negative_mass(self) -> None:
        """Test that negative mass returns zero inertia."""
        sphere = Sphere(radius=1.0)
        inertia = calculate_inertia(sphere, mass=-1.0)
        assert inertia == InertiaTensor.zero()

    def test_unsupported_geometry_type(self) -> None:
        """Test that unsupported geometry raises RobotModelError."""

        class UnsupportedGeometry:
            """Fake unsupported geometry."""

            pass

        with pytest.raises(RobotModelError, match="Unsupported geometry type"):
            calculate_inertia(UnsupportedGeometry(), mass=1.0)  # type: ignore


class TestMeshInertia:
    """Tests for mesh inertia calculation."""

    def test_mesh_default_approximation(self) -> None:
        """Test mesh inertia uses box approximation by default."""
        mesh = Mesh(resource="robot.stl")
        mass = 5.0
        # Use approximate calculation (bounding box)
        inertia = calculate_mesh_inertia_approximation(mesh, mass)

        # Should return non-zero inertia
        assert inertia.ixx > 0
        assert inertia.iyy > 0
        assert inertia.izz > 0

    def test_mesh_with_scale(self) -> None:
        """Test mesh inertia with custom scale."""
        mesh = Mesh(resource="model.dae", scale=Vector3(2.0, 2.0, 2.0))
        mass = 10.0
        inertia = calculate_mesh_inertia_approximation(mesh, mass)

        # Larger scale should give larger inertia
        assert inertia.ixx > 0
        assert inertia.iyy > 0
        assert inertia.izz > 0

    def test_mesh_zero_mass(self) -> None:
        """Test that zero mass returns zero inertia."""
        mesh = Mesh(resource="test.stl")
        inertia = calculate_mesh_inertia_approximation(mesh, mass=0.0)
        # Expect minimal stability inertia, not zero
        assert inertia.ixx == pytest.approx(1e-06)

    def test_mesh_negative_mass(self) -> None:
        """Test that negative mass returns zero inertia."""
        mesh = Mesh(resource="test.stl")
        inertia = calculate_mesh_inertia_approximation(mesh, mass=-1.0)
        assert inertia == InertiaTensor.zero()


class TestMeshInertiaFromTriangles:
    """Tests for triangle-based mesh inertia calculation."""

    def test_unit_cube_mesh(self) -> None:
        """Test inertia of unit cube mesh has reasonable values."""
        # Create vertices for unit cube centered at origin
        vertices = [
            (-0.5, -0.5, -0.5),  # 0
            (0.5, -0.5, -0.5),  # 1
            (0.5, 0.5, -0.5),  # 2
            (-0.5, 0.5, -0.5),  # 3
            (-0.5, -0.5, 0.5),  # 4
            (0.5, -0.5, 0.5),  # 5
            (0.5, 0.5, 0.5),  # 6
            (-0.5, 0.5, 0.5),  # 7
        ]

        # Create triangulated cube faces (Outward/CCW winding)
        triangles = [
            # Bottom face (z = -0.5) - looking from bottom CCW: 0-3-2, 0-2-1
            (0, 3, 2),
            (0, 2, 1),
            # Top face (z = 0.5) - looking from top CCW: 4-5-6, 4-6-7
            (4, 5, 6),
            (4, 6, 7),
            # Front face (y = -0.5) - looking from front CCW: 0-1-5, 0-5-4
            (0, 1, 5),
            (0, 5, 4),
            # Back face (y = 0.5) - looking from back CCW: 3-6-2, 3-7-6
            (3, 6, 2),
            (3, 7, 6),
            # Left face (x = -0.5) - looking from left CCW: 0-4-7, 0-7-3
            (0, 4, 7),
            (0, 7, 3),
            # Right face (x = 0.5) - looking from right CCW: 1-2-6, 1-6-5
            (1, 2, 6),
            (1, 6, 5),
        ]

        mass = 1.0
        mesh_inertia = calculate_mesh_inertia_from_triangles(vertices, triangles, mass)

        # For a unit cube, inertia should be in reasonable range
        # Analytical: I = (1/12) * m * (1² + 1²) = 1/6 ≈ 0.167
        # Mesh calculation gives reasonable approximation
        assert 0.05 < mesh_inertia.ixx < 0.3
        assert 0.05 < mesh_inertia.iyy < 0.3
        assert 0.05 < mesh_inertia.izz < 0.3

        # Diagonal values should be similar for a cube
        assert mesh_inertia.ixx == pytest.approx(mesh_inertia.iyy, rel=0.1)
        assert mesh_inertia.iyy == pytest.approx(mesh_inertia.izz, rel=0.1)

        # Off-diagonal terms should be small for centered cube (not exactly zero due to numerical integration)
        assert abs(mesh_inertia.ixy) < 0.01
        assert abs(mesh_inertia.ixz) < 0.01
        assert abs(mesh_inertia.iyz) < 0.01

    def test_rectangular_mesh(self) -> None:
        """Test inertia of rectangular box mesh has reasonable values."""
        # Create vertices for 2x3x4 box centered at origin
        x, y, z = 1.0, 1.5, 2.0  # Half-dimensions
        vertices = [
            (-x, -y, -z),  # 0
            (x, -y, -z),  # 1
            (x, y, -z),  # 2
            (-x, y, -z),  # 3
            (-x, -y, z),  # 4
            (x, -y, z),  # 5
            (x, y, z),  # 6
            (-x, y, z),  # 7
        ]

        # CCW/Outward winding order
        triangles = [
            (0, 3, 2),
            (0, 2, 1),  # Bottom
            (4, 5, 6),
            (4, 6, 7),  # Top
            (0, 1, 5),
            (0, 5, 4),  # Front
            (3, 7, 6),
            (3, 6, 2),  # Back
            (0, 4, 7),
            (0, 7, 3),  # Left
            (1, 2, 6),
            (1, 6, 5),  # Right
        ]

        mass = 10.0
        mesh_inertia = calculate_mesh_inertia_from_triangles(vertices, triangles, mass)

        # Verify inertia values are positive and reasonable
        assert mesh_inertia.ixx > 0
        assert mesh_inertia.iyy > 0
        assert mesh_inertia.izz > 0

    def test_tetrahedron_mesh(self) -> None:
        """Test inertia of simple tetrahedron mesh."""
        # Regular tetrahedron (points A, B, C, D)
        vertices = [
            (1.0, 1.0, 1.0),
            (1.0, -1.0, -1.0),
            (-1.0, 1.0, -1.0),
            (-1.0, -1.0, 1.0),
        ]

        # CCW winding
        triangles = [
            (0, 1, 2),
            (0, 3, 1),
            (0, 2, 3),
            (1, 3, 2),
        ]

        mass = 5.0
        inertia = calculate_mesh_inertia_from_triangles(vertices, triangles, mass)

        assert inertia.ixx > 0
        assert inertia.iyy > 0
        assert inertia.izz > 0

    def test_empty_mesh(self) -> None:
        """Test that empty mesh raises RobotPhysicsError."""
        vertices: list[tuple[float, float, float]] = []
        triangles: list[tuple[int, int, int]] = []
        with pytest.raises(RobotPhysicsError, match="Mesh is empty"):
            calculate_mesh_inertia_from_triangles(vertices, triangles, mass=1.0)

    def test_zero_mass(self) -> None:
        """Test that zero mass returns minimal stable inertia."""
        vertices = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
        triangles = [(0, 1, 2)]
        inertia = calculate_mesh_inertia_from_triangles(vertices, triangles, mass=0.0)
        # Should return minimal stable inertia defined in InertiaTensor.zero()
        assert inertia == InertiaTensor.zero()

    def test_negative_mass(self) -> None:
        """Test that negative mass returns minimal stable inertia."""
        vertices = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
        triangles = [(0, 1, 2)]
        inertia = calculate_mesh_inertia_from_triangles(vertices, triangles, mass=-1.0)
        assert inertia == InertiaTensor.zero()

    def test_degenerate_mesh(self) -> None:
        """Test mesh with zero volume raises RobotPhysicsError."""
        # Flat triangle (zero volume)
        vertices = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
        triangles = [(0, 1, 2)]
        # This will be caught by volume check or topology check
        with pytest.raises(RobotPhysicsError, match="not watertight|Degenerate mesh"):
            calculate_mesh_inertia_from_triangles(vertices, triangles, mass=1.0)
