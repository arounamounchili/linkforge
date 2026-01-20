"""Verification tests for inertia formulas against analytical solutions.

These tests verify that the inertia calculations match known analytical formulas
from physics textbooks and Wikipedia's List of Moments of Inertia.
"""

from __future__ import annotations

import pytest

from linkforge.core.models import Box, Cylinder, Sphere, Vector3
from linkforge.core.physics import (
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
