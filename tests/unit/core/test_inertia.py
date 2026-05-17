"""Unit tests for Inertia models and analytical formulas."""

from __future__ import annotations

import importlib
from unittest.mock import patch

import linkforge.core.physics.inertia
import pytest
from linkforge.core import (
    Box,
    Cylinder,
    InertiaTensor,
    Mesh,
    RobotMathError,
    RobotPhysicsError,
    Sphere,
    ValidationErrorCode,
    Vector3,
)
from linkforge.core.constants import MIN_INERTIA_STABILITY_VALUE
from linkforge.core.physics.inertia import (
    _get_stability_fallback,
    calculate_box_inertia,
    calculate_cylinder_inertia,
    calculate_inertia,
    calculate_mesh_inertia_approximation,
    calculate_mesh_inertia_from_triangles,
    calculate_sphere_inertia,
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

    def test_cylinder_inertia_formula(self) -> None:
        """Verify analytical inertia for a cylinder."""
        cylinder = Cylinder(radius=0.5, length=2.0)
        mass = 12.0
        # Izz = 0.5 * mass * r^2 = 0.5 * 12 * 0.25 = 1.5
        # Ixx = Iyy = (1/12) * mass * (3r^2 + h^2) = (1/12) * 12 * (0.75 + 4) = 4.75
        it = calculate_cylinder_inertia(cylinder, mass)
        assert it.izz == pytest.approx(1.5)
        assert it.ixx == pytest.approx(4.75)
        assert it.iyy == pytest.approx(4.75)


class TestInertiaIntegration:
    """Numerical verification of the Mirtich volume integration algorithm."""

    def create_box_mesh(
        self, size: tuple[float, float, float]
    ) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
        """Helper to create a simple box mesh with 8 vertices and 12 triangles."""
        x, y, z = [s / 2.0 for s in size]

        # Vertices (explicitly floats for type-checker)
        v = [
            (-x, -y, -z),
            (x, -y, -z),
            (x, y, -z),
            (-x, y, -z),
            (-x, -y, z),
            (x, -y, z),
            (x, y, z),
            (-x, y, z),
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

    def test_mirtich_sylvester_failure(self):
        """Test Sylvester criterion safety guard in mesh integration."""

        v = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
        f = [(0, 2, 1), (0, 3, 2), (0, 1, 3), (1, 2, 3)]
        mass = 1.0

        # We use a huge NEGATIVE tolerance to force the condition delta < -eps
        with (
            patch("linkforge.core.physics.inertia.SYLVESTER_TOLERANCE_EPSILON", -1e20),
            pytest.raises(RobotPhysicsError, match="fails Sylvester criterion"),
        ):
            calculate_mesh_inertia_from_triangles(v, f, mass)

    def test_mirtich_validation_edge_cases(self):
        """Test mesh input validation for empty or corrupted data."""
        # Empty
        with pytest.raises(RobotPhysicsError, match="Mesh is empty"):
            calculate_mesh_inertia_from_triangles([], [], 1.0)
        # Non-finite vertex
        with pytest.raises(RobotMathError, match="contains non-finite value"):
            calculate_mesh_inertia_from_triangles([(float("nan"), 0.0, 0.0)], [(0, 0, 0)], 1.0)
        # Out of range index
        with pytest.raises(RobotPhysicsError, match="Invalid triangle at 0"):
            calculate_mesh_inertia_from_triangles([(0.0, 0.0, 0.0)], [(0, 1, 2)], 1.0)

    def test_mirtich_winding_cancellation(self):
        """Test detection of severe volume cancellation (mixed winding)."""
        mass = 1.0
        v = [
            (-1.0, -1.0, -1.0),
            (1.0, -1.0, -1.0),
            (1.0, 1.0, -1.0),
            (-1.0, 1.0, -1.0),
            (-1.0, -1.0, 1.0),
            (1.0, -1.0, 1.0),
            (1.0, 1.0, 1.0),
            (-1.0, 1.0, 1.0),
        ]
        # Standard box faces
        f = [
            (0, 2, 1),
            (0, 3, 2),
            (4, 5, 6),
            (4, 6, 7),
            (0, 1, 5),
            (0, 5, 4),
            (1, 2, 6),
            (1, 6, 5),
            (2, 3, 7),
            (2, 7, 6),
            (3, 0, 4),
            (3, 4, 7),
        ]
        # Flip 4 out of 12 triangles to trigger high cancellation (ratio < 0.5)
        f_bad = []
        for i, tri in enumerate(f):
            if i < 4:
                f_bad.append((tri[0], tri[2], tri[1]))
            else:
                f_bad.append(tri)
        with pytest.raises(RobotPhysicsError, match="inconsistent global winding"):
            calculate_mesh_inertia_from_triangles(v, f_bad, mass)

    def test_mirtich_non_finite_safety_guards(self, mocker):
        """Verify safety guards for non-finite results without brittle call counting."""
        v, f = self.create_box_mesh((1, 1, 1))
        mass = 1.0

        # Mock input validation to do nothing so we can control isfinite calls
        mocker.patch("linkforge.core.physics.inertia._validate_mesh_inputs")

        # 1. Trigger weighted_com non-finite check
        mocker.patch("linkforge.core.physics.inertia.isfinite", return_value=False)
        with pytest.raises(RobotPhysicsError, match="weighted center of mass is non-finite"):
            calculate_mesh_inertia_from_triangles(v, f, mass)

        # 2. Trigger final com non-finite check
        # We need isfinite to be True for weighted_com (3 calls) then False
        mocker.patch(
            "linkforge.core.physics.inertia.isfinite", side_effect=[True, True, True, False]
        )
        with pytest.raises(RobotPhysicsError, match="Computed center of mass is non-finite"):
            calculate_mesh_inertia_from_triangles(v, f, mass)


class TestInertiaUtils:
    """Tests for inertia utilities and unified wrapper."""

    def test_stability_fallback_values(self):
        """Verify the values of the stability fallback tensor."""
        it = _get_stability_fallback()
        assert it.ixx == MIN_INERTIA_STABILITY_VALUE
        assert it.iyy == MIN_INERTIA_STABILITY_VALUE
        assert it.izz == MIN_INERTIA_STABILITY_VALUE

    def test_inertia_stability_fallbacks(self):
        """Test that all shapes fallback to stability tensor when mass is near zero."""
        mass = 1e-12

        fallback = _get_stability_fallback()
        assert calculate_inertia(Box(size=Vector3(1, 1, 1)), mass) == fallback
        assert calculate_box_inertia(Box(size=Vector3(1, 1, 1)), mass) == fallback
        assert calculate_cylinder_inertia(Cylinder(radius=1, length=1), mass) == fallback
        assert calculate_sphere_inertia(Sphere(radius=1), mass) == fallback
        assert calculate_mesh_inertia_approximation(Mesh(resource="test.stl"), mass) == fallback

    def test_unified_calculate_inertia_types(self):
        """Verify the unified wrapper handles all supported and unsupported types."""
        mass = 1.0
        assert isinstance(calculate_inertia(Box(size=Vector3(1, 1, 1)), mass), InertiaTensor)
        assert isinstance(calculate_inertia(Cylinder(radius=1, length=1), mass), InertiaTensor)
        assert isinstance(calculate_inertia(Sphere(radius=1), mass), InertiaTensor)
        assert isinstance(calculate_inertia(Mesh(resource="test.stl"), mass), InertiaTensor)

        class FakeGeometry:
            pass

        with pytest.raises(RobotPhysicsError, match="Unsupported geometry type"):
            calculate_inertia(FakeGeometry(), mass)  # type: ignore

    def test_cache_size_env_var(self, monkeypatch):
        """Verify that the cache size environment variable is respected."""
        monkeypatch.setenv("LINKFORGE_INERTIA_CACHE_SIZE", "128")

        importlib.reload(linkforge.core.physics.inertia)
        assert linkforge.core.physics.inertia.DEFAULT_INERTIA_CACHE_SIZE_ENV == 128
