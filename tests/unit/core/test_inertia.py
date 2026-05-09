"""Unit tests for Inertia models and analytical formulas."""

from __future__ import annotations

import pytest
from linkforge_core.exceptions import RobotPhysicsError, ValidationErrorCode
from linkforge_core.models import InertiaTensor
from linkforge_core.models.geometry import Box, Sphere, Vector3
from linkforge_core.physics.inertia import calculate_inertia

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


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
