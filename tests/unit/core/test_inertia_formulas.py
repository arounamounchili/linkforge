"""Verification tests for inertia formulas against analytical solutions.

These tests verify that the inertia calculations match known analytical formulas
from physics textbooks and Wikipedia's List of Moments of Inertia.
"""

from __future__ import annotations

import pytest
from linkforge_core.models import Box, Cylinder, Sphere, Vector3
from linkforge_core.physics import (
    calculate_box_inertia,
    calculate_cylinder_inertia,
    calculate_mesh_inertia_from_triangles,
    calculate_sphere_inertia,
)


class TestBoxFormulaVerification:
    """Verify box inertia against analytical formulas."""

    def test_unit_cube_analytical(self):
        """Unit cube: m=1, side=1 → I = (1/12) * 1 * (1² + 1²) = 1/6."""
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        inertia = calculate_box_inertia(box, mass=1.0)

        expected = 1.0 / 6.0  # 0.16666...

        assert inertia.ixx == pytest.approx(expected, rel=1e-6)
        assert inertia.iyy == pytest.approx(expected, rel=1e-6)
        assert inertia.izz == pytest.approx(expected, rel=1e-6)

    def test_rectangular_box_analytical(self):
        """Box 2x3x4 with mass 12kg."""
        box = Box(size=Vector3(2.0, 3.0, 4.0))
        mass = 12.0
        inertia = calculate_box_inertia(box, mass)

        # Ixx = (1/12) * 12 * (3² + 4²) = 1 * 25 = 25
        # Iyy = (1/12) * 12 * (2² + 4²) = 1 * 20 = 20
        # Izz = (1/12) * 12 * (2² + 3²) = 1 * 13 = 13

        assert inertia.ixx == pytest.approx(25.0, rel=1e-6)
        assert inertia.iyy == pytest.approx(20.0, rel=1e-6)
        assert inertia.izz == pytest.approx(13.0, rel=1e-6)


class TestCylinderFormulaVerification:
    """Verify cylinder inertia against analytical formulas."""

    def test_unit_cylinder_analytical(self):
        """Cylinder: r=1, h=2, m=1."""
        cyl = Cylinder(radius=1.0, length=2.0)
        mass = 1.0
        inertia = calculate_cylinder_inertia(cyl, mass)

        # Ixx = Iyy = (1/12) * 1 * (3*1² + 2²) = (1/12) * 7 = 0.5833...
        # Izz = (1/2) * 1 * 1² = 0.5

        ixx_expected = (1.0 / 12.0) * (3 * 1.0**2 + 2.0**2)
        izz_expected = 0.5

        assert inertia.ixx == pytest.approx(ixx_expected, rel=1e-6)
        assert inertia.iyy == pytest.approx(ixx_expected, rel=1e-6)
        assert inertia.izz == pytest.approx(izz_expected, rel=1e-6)

    def test_thin_disk_analytical(self):
        """Thin disk: r=2, h≈0, m=10."""
        cyl = Cylinder(radius=2.0, length=0.01)
        mass = 10.0
        inertia = calculate_cylinder_inertia(cyl, mass)

        # For thin disk, h→0:
        # Ixx = Iyy ≈ (1/4) * m * r²
        # Izz = (1/2) * m * r²

        ixx_expected = (1.0 / 12.0) * mass * (3 * 2.0**2 + 0.01**2)  # ≈ 10
        izz_expected = 0.5 * mass * 2.0**2  # = 20

        assert inertia.ixx == pytest.approx(ixx_expected, rel=1e-4)
        assert inertia.izz == pytest.approx(izz_expected, rel=1e-6)

    def test_long_rod_analytical(self):
        """Long thin rod: r≈0, h=10, m=5."""
        cyl = Cylinder(radius=0.01, length=10.0)
        mass = 5.0
        inertia = calculate_cylinder_inertia(cyl, mass)

        # For thin rod, r→0:
        # Ixx = Iyy ≈ (1/12) * m * h²
        # Izz ≈ 0

        ixx_expected = (1.0 / 12.0) * mass * 10.0**2  # ≈ 41.67
        izz_expected = 0.5 * mass * 0.01**2  # ≈ 0

        assert inertia.ixx == pytest.approx(ixx_expected, rel=1e-4)
        assert inertia.izz == pytest.approx(izz_expected, abs=1e-3)


class TestSphereFormulaVerification:
    """Verify sphere inertia against analytical formulas."""

    def test_unit_sphere_analytical(self):
        """Unit sphere: r=1, m=1 → I = (2/5) * 1 * 1² = 0.4."""
        sphere = Sphere(radius=1.0)
        mass = 1.0
        inertia = calculate_sphere_inertia(sphere, mass)

        expected = 0.4

        assert inertia.ixx == pytest.approx(expected, rel=1e-6)
        assert inertia.iyy == pytest.approx(expected, rel=1e-6)
        assert inertia.izz == pytest.approx(expected, rel=1e-6)

    def test_sphere_radius_2_mass_10_analytical(self):
        """Sphere: r=2, m=10 → I = (2/5) * 10 * 4 = 16."""
        sphere = Sphere(radius=2.0)
        mass = 10.0
        inertia = calculate_sphere_inertia(sphere, mass)

        expected = (2.0 / 5.0) * mass * 2.0**2  # = 16

        assert inertia.ixx == pytest.approx(expected, rel=1e-6)
        assert inertia.iyy == pytest.approx(expected, rel=1e-6)
        assert inertia.izz == pytest.approx(expected, rel=1e-6)


class TestMeshInertiaVerification:
    """Verify mesh inertia against analytical solutions for simple shapes."""

    def test_cube_mesh_vs_analytical_box(self):
        """Compare mesh-calculated cube inertia to analytical box formula."""
        # Unit cube centered at origin
        vertices = [
            (-0.5, -0.5, -0.5),
            (0.5, -0.5, -0.5),
            (0.5, 0.5, -0.5),
            (-0.5, 0.5, -0.5),
            (-0.5, -0.5, 0.5),
            (0.5, -0.5, 0.5),
            (0.5, 0.5, 0.5),
            (-0.5, 0.5, 0.5),
        ]

        triangles = [
            (0, 1, 2),
            (0, 2, 3),  # Bottom
            (4, 6, 5),
            (4, 7, 6),  # Top
            (0, 5, 1),
            (0, 4, 5),  # Front
            (3, 2, 6),
            (3, 6, 7),  # Back
            (0, 3, 7),
            (0, 7, 4),  # Left
            (1, 5, 6),
            (1, 6, 2),  # Right
        ]

        mass = 1.0
        mesh_inertia = calculate_mesh_inertia_from_triangles(vertices, triangles, mass)

        # Analytical box
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        calculate_box_inertia(box, mass)

        # Expected: I = (1/12) * 1 * (1² + 1²) = 1/6 ≈ 0.1667
        expected = 1.0 / 6.0

        # Mesh calculation should be close to analytical (within 10%)
        print(f"Mesh Ixx: {mesh_inertia.ixx}, Expected: {expected}")
        print(f"Mesh Iyy: {mesh_inertia.iyy}, Expected: {expected}")
        print(f"Mesh Izz: {mesh_inertia.izz}, Expected: {expected}")

        assert mesh_inertia.ixx == pytest.approx(expected, rel=0.1)
        assert mesh_inertia.iyy == pytest.approx(expected, rel=0.1)
        assert mesh_inertia.izz == pytest.approx(expected, rel=0.1)

    def test_rectangular_mesh_vs_analytical_box(self):
        """Compare mesh-calculated rectangular box to analytical formula."""
        # 2x3x4 box centered at origin
        x, y, z = 1.0, 1.5, 2.0
        vertices = [
            (-x, -y, -z),
            (x, -y, -z),
            (x, y, -z),
            (-x, y, -z),
            (-x, -y, z),
            (x, -y, z),
            (x, y, z),
            (-x, y, z),
        ]

        triangles = [
            (0, 1, 2),
            (0, 2, 3),
            (4, 6, 5),
            (4, 7, 6),
            (0, 5, 1),
            (0, 4, 5),
            (3, 2, 6),
            (3, 6, 7),
            (0, 3, 7),
            (0, 7, 4),
            (1, 5, 6),
            (1, 6, 2),
        ]

        mass = 10.0
        mesh_inertia = calculate_mesh_inertia_from_triangles(vertices, triangles, mass)

        # Analytical box (full dimensions: 2x3x4)
        box = Box(size=Vector3(2.0, 3.0, 4.0))
        box_inertia = calculate_box_inertia(box, mass)

        print(f"Mesh Ixx: {mesh_inertia.ixx}, Box Ixx: {box_inertia.ixx}")
        print(f"Mesh Iyy: {mesh_inertia.iyy}, Box Iyy: {box_inertia.iyy}")
        print(f"Mesh Izz: {mesh_inertia.izz}, Box Izz: {box_inertia.izz}")

        # Should match within 10%
        assert mesh_inertia.ixx == pytest.approx(box_inertia.ixx, rel=0.1)
        assert mesh_inertia.iyy == pytest.approx(box_inertia.iyy, rel=0.1)
        assert mesh_inertia.izz == pytest.approx(box_inertia.izz, rel=0.1)


class TestInertiaEdgeCases:
    """Test edge cases and error handling in inertia calculations."""

    def test_box_zero_mass(self):
        """Test that zero mass returns minimal inertia (1e-06 for numerical stability)."""
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        inertia = calculate_box_inertia(box, mass=0.0)

        # Returns minimal inertia for numerical stability
        assert inertia.ixx == pytest.approx(1e-06)
        assert inertia.iyy == pytest.approx(1e-06)
        assert inertia.izz == pytest.approx(1e-06)

    def test_box_negative_mass(self):
        """Test that negative mass returns minimal inertia."""
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        inertia = calculate_box_inertia(box, mass=-5.0)

        assert inertia.ixx == pytest.approx(1e-06)
        assert inertia.iyy == pytest.approx(1e-06)
        assert inertia.izz == pytest.approx(1e-06)

    def test_cylinder_zero_mass(self):
        """Test that zero mass returns minimal inertia."""
        cyl = Cylinder(radius=1.0, length=2.0)
        inertia = calculate_cylinder_inertia(cyl, mass=0.0)

        assert inertia.ixx == pytest.approx(1e-06)
        assert inertia.iyy == pytest.approx(1e-06)
        assert inertia.izz == pytest.approx(1e-06)

    def test_cylinder_negative_mass(self):
        """Test that negative mass returns minimal inertia."""
        cyl = Cylinder(radius=1.0, length=2.0)
        inertia = calculate_cylinder_inertia(cyl, mass=-3.0)

        assert inertia.ixx == pytest.approx(1e-06)
        assert inertia.iyy == pytest.approx(1e-06)
        assert inertia.izz == pytest.approx(1e-06)

    def test_sphere_zero_mass(self):
        """Test that zero mass returns minimal inertia."""
        sphere = Sphere(radius=1.0)
        inertia = calculate_sphere_inertia(sphere, mass=0.0)

        assert inertia.ixx == pytest.approx(1e-06)
        assert inertia.iyy == pytest.approx(1e-06)
        assert inertia.izz == pytest.approx(1e-06)

    def test_sphere_negative_mass(self):
        """Test that negative mass returns minimal inertia."""
        sphere = Sphere(radius=1.0)
        inertia = calculate_sphere_inertia(sphere, mass=-2.0)

        assert inertia.ixx == pytest.approx(1e-06)
        assert inertia.iyy == pytest.approx(1e-06)
        assert inertia.izz == pytest.approx(1e-06)

    def test_mesh_zero_mass(self):
        """Test that zero mass returns minimal inertia."""
        vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
        triangles = [(0, 1, 2)]
        inertia = calculate_mesh_inertia_from_triangles(vertices, triangles, mass=0.0)

        assert inertia.ixx == pytest.approx(1e-06)
        assert inertia.iyy == pytest.approx(1e-06)
        assert inertia.izz == pytest.approx(1e-06)

    def test_mesh_negative_mass(self):
        """Test that negative mass returns minimal inertia."""
        vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
        triangles = [(0, 1, 2)]
        inertia = calculate_mesh_inertia_from_triangles(vertices, triangles, mass=-1.0)

        assert inertia.ixx == pytest.approx(1e-06)
        assert inertia.iyy == pytest.approx(1e-06)
        assert inertia.izz == pytest.approx(1e-06)

    def test_mesh_empty_vertices(self):
        """Test that empty vertices raises ValueError."""
        vertices = []
        triangles = [(0, 1, 2)]

        with pytest.raises(ValueError, match="empty mesh.*no vertices"):
            calculate_mesh_inertia_from_triangles(vertices, triangles, mass=1.0)

    def test_mesh_empty_triangles(self):
        """Test that empty triangles raises ValueError."""
        vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
        triangles = []

        with pytest.raises(ValueError, match="empty mesh.*no triangles"):
            calculate_mesh_inertia_from_triangles(vertices, triangles, mass=1.0)

    def test_mesh_invalid_triangle_length(self):
        """Test that triangle with != 3 indices raises ValueError."""
        vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0)]
        triangles = [(0, 1, 2, 3)]  # 4 indices instead of 3

        with pytest.raises(ValueError, match="exactly 3 indices"):
            calculate_mesh_inertia_from_triangles(vertices, triangles, mass=1.0)

    def test_mesh_invalid_triangle_index(self):
        """Test that out-of-bounds triangle index raises ValueError."""
        vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
        triangles = [(0, 1, 5)]  # Index 5 is out of bounds

        with pytest.raises(ValueError, match="index.*out of bounds"):
            calculate_mesh_inertia_from_triangles(vertices, triangles, mass=1.0)

    def test_mesh_degenerate_zero_volume(self):
        """Test that mesh with zero volume raises ValueError."""
        # All vertices coplanar (in XY plane)
        vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0)]
        triangles = [(0, 1, 2), (1, 3, 2)]

        with pytest.raises(ValueError, match="zero volume"):
            calculate_mesh_inertia_from_triangles(vertices, triangles, mass=1.0)

    def test_calculate_inertia_unsupported_geometry(self):
        """Test that unsupported geometry type raises ValueError."""
        from linkforge_core.physics.inertia import calculate_inertia

        # Create a mock geometry object that's not Box, Cylinder, Sphere, or Mesh
        class UnsupportedGeometry:
            pass

        with pytest.raises(ValueError, match="Unsupported geometry type"):
            calculate_inertia(UnsupportedGeometry(), mass=1.0)

    def test_calculate_inertia_zero_mass(self):
        """Test that calculate_inertia with zero mass returns minimal inertia."""
        from linkforge_core.physics.inertia import calculate_inertia

        box = Box(size=Vector3(1.0, 1.0, 1.0))
        inertia = calculate_inertia(box, mass=0.0)

        assert inertia.ixx == pytest.approx(1e-06)
        assert inertia.iyy == pytest.approx(1e-06)
        assert inertia.izz == pytest.approx(1e-06)

    def test_calculate_mesh_inertia_zero_mass(self):
        """Test calculate_mesh_inertia (approximation) with zero mass."""
        from linkforge_core.models.geometry import Mesh
        from linkforge_core.physics.inertia import calculate_mesh_inertia

        mesh = Mesh(filepath="package://test/meshes/test.stl", scale=Vector3(1.0, 1.0, 1.0))
        inertia = calculate_mesh_inertia(mesh, mass=0.0)

        assert inertia.ixx == pytest.approx(1e-06)
        assert inertia.iyy == pytest.approx(1e-06)
        assert inertia.izz == pytest.approx(1e-06)

    def test_calculate_mesh_inertia_approximation(self):
        """Test that calculate_mesh_inertia uses bounding box approximation."""
        from linkforge_core.models.geometry import Mesh
        from linkforge_core.physics.inertia import calculate_mesh_inertia

        # Mesh with scale 2x3x4
        mesh = Mesh(filepath="package://test/meshes/test.stl", scale=Vector3(2.0, 3.0, 4.0))
        inertia = calculate_mesh_inertia(mesh, mass=10.0)

        # Should approximate as a box with the same scale
        box = Box(size=Vector3(2.0, 3.0, 4.0))
        box_inertia = calculate_box_inertia(box, mass=10.0)

        assert inertia.ixx == box_inertia.ixx
        assert inertia.iyy == box_inertia.iyy
        assert inertia.izz == box_inertia.izz


class TestInertiaTriangleInequality:
    """Verify that all calculated inertia tensors satisfy the triangle inequality.

    For a valid inertia tensor:
        Ixx + Iyy ≥ Izz
        Ixx + Izz ≥ Iyy
        Iyy + Izz ≥ Ixx
    """

    def test_box_satisfies_triangle_inequality(self):
        """Box inertia should satisfy triangle inequality."""
        box = Box(size=Vector3(2.0, 3.0, 4.0))
        inertia = calculate_box_inertia(box, mass=10.0)

        assert inertia.ixx + inertia.iyy >= inertia.izz - 1e-10
        assert inertia.ixx + inertia.izz >= inertia.iyy - 1e-10
        assert inertia.iyy + inertia.izz >= inertia.ixx - 1e-10

    def test_cylinder_satisfies_triangle_inequality(self):
        """Cylinder inertia should satisfy triangle inequality."""
        cyl = Cylinder(radius=2.0, length=5.0)
        inertia = calculate_cylinder_inertia(cyl, mass=8.0)

        assert inertia.ixx + inertia.iyy >= inertia.izz - 1e-10
        assert inertia.ixx + inertia.izz >= inertia.iyy - 1e-10
        assert inertia.iyy + inertia.izz >= inertia.ixx - 1e-10

    def test_sphere_satisfies_triangle_inequality(self):
        """Sphere inertia should satisfy triangle inequality."""
        sphere = Sphere(radius=3.0)
        inertia = calculate_sphere_inertia(sphere, mass=15.0)

        assert inertia.ixx + inertia.iyy >= inertia.izz - 1e-10
        assert inertia.ixx + inertia.izz >= inertia.iyy - 1e-10
        assert inertia.iyy + inertia.izz >= inertia.ixx - 1e-10

    def test_mesh_satisfies_triangle_inequality(self):
        """Mesh inertia should satisfy triangle inequality."""
        # Unit cube
        vertices = [
            (-0.5, -0.5, -0.5),
            (0.5, -0.5, -0.5),
            (0.5, 0.5, -0.5),
            (-0.5, 0.5, -0.5),
            (-0.5, -0.5, 0.5),
            (0.5, -0.5, 0.5),
            (0.5, 0.5, 0.5),
            (-0.5, 0.5, 0.5),
        ]
        triangles = [
            (0, 1, 2),
            (0, 2, 3),
            (4, 6, 5),
            (4, 7, 6),
            (0, 5, 1),
            (0, 4, 5),
            (3, 2, 6),
            (3, 6, 7),
            (0, 3, 7),
            (0, 7, 4),
            (1, 5, 6),
            (1, 6, 2),
        ]

        inertia = calculate_mesh_inertia_from_triangles(vertices, triangles, mass=5.0)

        assert inertia.ixx + inertia.iyy >= inertia.izz - 1e-10
        assert inertia.ixx + inertia.izz >= inertia.iyy - 1e-10
        assert inertia.iyy + inertia.izz >= inertia.ixx - 1e-10


class TestInertiaWrapper:
    """Test the calculate_inertia wrapper function."""

    def test_calculate_inertia_box(self):
        from linkforge_core.physics.inertia import calculate_inertia

        box = Box(size=Vector3(1.0, 1.0, 1.0))
        inertia = calculate_inertia(box, mass=1.0)
        assert inertia.ixx > 0

    def test_calculate_inertia_cylinder(self):
        from linkforge_core.physics.inertia import calculate_inertia

        cyl = Cylinder(radius=1.0, length=1.0)
        inertia = calculate_inertia(cyl, mass=1.0)
        assert inertia.ixx > 0

    def test_calculate_inertia_sphere(self):
        from linkforge_core.physics.inertia import calculate_inertia

        sphere = Sphere(radius=1.0)
        inertia = calculate_inertia(sphere, mass=1.0)
        assert inertia.ixx > 0

    def test_calculate_inertia_mesh(self):
        from linkforge_core.models import Vector3
        from linkforge_core.models.geometry import Mesh
        from linkforge_core.physics.inertia import calculate_inertia

        # calculate_mesh_inertia uses approximation if triangles not provided
        mesh = Mesh(filepath="package://test/test.stl", scale=Vector3(1.0, 1.0, 1.0))
        inertia = calculate_inertia(mesh, mass=1.0)
        # Should call calculate_mesh_inertia -> approx box
        assert inertia.ixx > 0
