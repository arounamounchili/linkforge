"""Unit tests for Inertia models and analytical formulas."""

from __future__ import annotations

import pytest
from linkforge.core import (
    Box,
    InertiaTensor,
    RobotPhysicsError,
    Sphere,
    ValidationErrorCode,
    Vector3,
)
from linkforge.core.physics.inertia import (
    calculate_inertia,
    calculate_mesh_inertia_from_triangles,
)

# Inertia Model Tests


class TestInertiaModels:
    def test_valid_inertia_tensor(self) -> None:
        """Test creating a valid inertia tensor."""
        # Now requires all 6 components
        it = InertiaTensor(ixx=1.0, iyy=1.0, izz=1.0, ixy=0.0, ixz=0.0, iyz=0.0)
        assert it.ixx == 1.0
        assert it.ixy == 0.0

    def test_invalid_inertia_tensor(self) -> None:
        """Test that invalid diagonal elements raise error."""
        with pytest.raises(RobotPhysicsError) as exc:
            InertiaTensor(ixx=-1.0, iyy=1.0, izz=1.0, ixy=0.0, ixz=0.0, iyz=0.0)
        assert exc.value.code == ValidationErrorCode.OUT_OF_RANGE

    def test_triangle_inequality_validation(self) -> None:
        """Test that triangle inequality violation raises error."""
        with pytest.raises(RobotPhysicsError, match="triangle inequality"):
            InertiaTensor(ixx=10.0, iyy=1.0, izz=1.0, ixy=0.0, ixz=0.0, iyz=0.0)


# Analytical Formula Verification


class TestInertiaFormulas:
    def test_box_inertia_formula(self) -> None:
        """Verify analytical inertia for a box."""
        box = Box(size=Vector3(1, 2, 3))
        mass = 12.0
        # Formula: Ixx = 1/12 * mass * (y^2 + z^2) = 1/12 * 12 * (4 + 9) = 13
        inertia = calculate_inertia(box, mass)
        assert inertia.ixx == pytest.approx(13.0)
        # Iyy = 1/12 * 12 * (1 + 9) = 10
        assert inertia.iyy == pytest.approx(10.0)
        # Izz = 1/12 * 12 * (1 + 4) = 5
        assert inertia.izz == pytest.approx(5.0)

    def test_sphere_inertia_formula(self) -> None:
        """Verify analytical inertia for a sphere."""
        sphere = Sphere(radius=1.0)
        mass = 5.0
        # Formula: I = 2/5 * mass * r^2 = 2/5 * 5 * 1 = 2
        inertia = calculate_inertia(sphere, mass)
        assert inertia.ixx == pytest.approx(2.0)
        assert inertia.iyy == pytest.approx(2.0)
        assert inertia.izz == pytest.approx(2.0)


class TestInertiaIntegration:
    """Numerical verification of the Mirtich volume integration algorithm."""

    def create_box_mesh(
        self, size: tuple[float, float, float]
    ) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
        """Helper to create a simple box mesh with 8 vertices and 12 triangles."""
        x, y, z = [s / 2.0 for s in size]

        # Vertices (explicitly floats for type-checker)
        v = [
            (float(-x), float(-y), float(-z)),
            (float(x), float(-y), float(-z)),
            (float(x), float(y), float(-z)),
            (float(-x), float(y), float(-z)),
            (float(-x), float(-y), float(z)),
            (float(x), float(-y), float(z)),
            (float(x), float(y), float(z)),
            (float(-x), float(y), float(z)),
        ]

        # Triangles (standard CCW winding)
        f = [
            (0, 2, 1),
            (0, 3, 2),  # Bottom
            (4, 5, 6),
            (4, 6, 7),  # Top
            (0, 1, 5),
            (0, 5, 4),  # Front
            (1, 2, 6),
            (1, 6, 5),  # Right
            (2, 3, 7),
            (2, 7, 6),  # Back
            (3, 0, 4),
            (3, 4, 7),  # Left
        ]
        return v, f

    def test_mirtich_vs_analytic_box(self):
        """Verify that Mirtich integration for a box matches the analytic formula."""
        size = (1.0, 2.0, 3.0)
        mass = 12.0

        # Analytic
        analytic = calculate_inertia(Box(size=Vector3(*size)), mass)

        # Mirtich
        v, f = self.create_box_mesh(size)
        numerical = calculate_mesh_inertia_from_triangles(v, f, mass)

        # Check diagonals (within float precision)
        assert numerical.ixx == pytest.approx(analytic.ixx, rel=1e-6)
        assert numerical.iyy == pytest.approx(analytic.iyy, rel=1e-6)
        assert numerical.izz == pytest.approx(analytic.izz, rel=1e-6)

        # Check off-diagonals (should be near zero)
        assert abs(numerical.ixy) < 1e-9
        assert abs(numerical.ixz) < 1e-9
        assert abs(numerical.iyz) < 1e-9

    def test_mirtich_negative_volume_detection(self):
        """Verify that the algorithm detects inconsistent winding (inward-facing triangles)."""
        size = (1.0, 1.0, 1.0)
        mass = 1.0
        v, f = self.create_box_mesh(size)

        # Flip winding of ALL triangles to make total volume negative
        f = [(t[0], t[2], t[1]) for t in f]

        with pytest.raises(RobotPhysicsError) as exc:
            calculate_mesh_inertia_from_triangles(v, f, mass)

        assert exc.value.code == ValidationErrorCode.PHYSICS_VIOLATION
        assert "winding" in str(exc.value).lower()

    def test_mirtich_degenerate_mesh(self):
        """Verify that a zero-volume mesh (plane) raises a PhysicsError."""
        # A single triangle has zero volume
        v = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
        f = [(0, 1, 2)]

        with pytest.raises(RobotPhysicsError) as exc:
            calculate_mesh_inertia_from_triangles(v, f, 1.0)

        assert exc.value.code == ValidationErrorCode.INVALID_VALUE
        assert "degenerate" in str(exc.value).lower()

    def test_mirtich_stability_fallback(self):
        """Verify that near-zero mass returns a stable fallback tensor."""
        v, f = self.create_box_mesh((1, 1, 1))
        # Very small mass below threshold
        result = calculate_mesh_inertia_from_triangles(v, f, 1e-10)

        assert result.ixx > 0
        assert result.iyy > 0
        assert result.izz > 0

    def test_mirtich_mesh_scaling(self):
        """Verify that scaling the mesh coordinates scales inertia correctly (I ~ m * r²)."""
        size = (1.0, 1.0, 1.0)
        mass = 1.0
        v, f = self.create_box_mesh(size)

        i1 = calculate_mesh_inertia_from_triangles(v, f, mass)

        # Double the size (linear scale = 2)
        v2 = [(x * 2.0, y * 2.0, z * 2.0) for x, y, z in v]
        i2 = calculate_mesh_inertia_from_triangles(v2, f, mass)

        # Inertia should increase by factor of 4 (2²)
        assert i2.ixx == pytest.approx(i1.ixx * 4.0, rel=1e-6)
        assert i2.iyy == pytest.approx(i1.iyy * 4.0, rel=1e-6)
        assert i2.izz == pytest.approx(i1.izz * 4.0, rel=1e-6)
