import pytest
from linkforge_core.exceptions import RobotPhysicsError
from linkforge_core.physics.inertia import (
    calculate_inertia,
    calculate_mesh_inertia_from_triangles,
)


def test_calculate_mesh_inertia_negative_diagonal() -> None:
    """Inverted winding order forces negative surface integrals, triggering a strict error."""
    vertices = [
        (1.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0),
        (0.0, 0.0, -1.0),
    ]
    # Inverted triangles (CW from outside)
    triangles = [
        (0, 4, 2),
        (0, 3, 4),
        (0, 5, 3),
        (0, 2, 5),
        (1, 2, 4),
        (1, 4, 3),
        (1, 3, 5),
        (1, 5, 2),
    ]
    with pytest.raises(RobotPhysicsError, match="Negative diagonal inertia"):
        calculate_mesh_inertia_from_triangles(vertices, triangles, mass=1.0)


def test_calculate_mesh_inertia_zero_volume() -> None:
    """Degenerate mesh (zero volume) raises RobotPhysicsError."""
    vertices = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
    triangles = [(0, 1, 2)]

    with pytest.raises(RobotPhysicsError, match="Degenerate mesh"):
        calculate_mesh_inertia_from_triangles(vertices, triangles, 1.0)


def test_calculate_inertia_unsupported_geometry_fallback() -> None:
    """Unsupported geometry types raise RobotPhysicsError in the inertia facade."""

    class UnsupportedShape:
        pass

    with pytest.raises(RobotPhysicsError, match="Unsupported geometry type"):
        calculate_inertia(UnsupportedShape(), mass=1.0)  # type: ignore


def test_mesh_inertia_robust_negative_diagonals_handling() -> None:
    """Verify that CCW meshes result in positive diagonal inertia."""
    # Simple tetrahedron with mass (CCW/Outward winding)
    vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
    triangles = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]

    res = calculate_mesh_inertia_from_triangles(vertices, triangles, mass=1.0)
    assert res.ixx >= 0
    assert res.iyy >= 0
    assert res.izz >= 0
