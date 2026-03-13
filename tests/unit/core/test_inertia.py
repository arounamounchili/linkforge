"""Unit tests for mesh inertia calculation."""

import logging

import pytest
from linkforge_core.exceptions import RobotModelError
from linkforge_core.physics.inertia import calculate_mesh_inertia_from_triangles


def test_calculate_mesh_inertia_negative_diagonal(caplog):
    """Inverted winding order forces negative surface integrals, triggering the warning fallback."""
    vertices = [
        (1.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0),
        (0.0, 0.0, -1.0),
    ]
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
    with caplog.at_level(logging.WARNING, logger="linkforge_core.physics.inertia"):
        calculate_mesh_inertia_from_triangles(vertices, triangles, mass=1.0)

    assert "Negative diagonal inertia detected" in caplog.text


def test_calculate_mesh_inertia_zero_volume():
    """Degenerate mesh (zero volume) raises RobotModelError."""
    vertices = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
    triangles = [(0, 1, 2)]

    with pytest.raises(RobotModelError, match="mesh has zero volume"):
        calculate_mesh_inertia_from_triangles(vertices, triangles, 1.0)


def test_calculate_inertia_unsupported_geometry_fallback():
    """Unsupported geometry types raise RobotModelError in the inertia facade."""

    class UnsupportedShape:
        pass

    with pytest.raises(RobotModelError, match="Unsupported geometry type"):
        from linkforge_core.physics.inertia import calculate_inertia

        calculate_inertia(UnsupportedShape(), mass=1.0)  # type: ignore


def test_mesh_inertia_robust_negative_diagonals_handling():
    """Verify robust handling of negative diagonal inertia elements during mesh calculation."""
    # Simple tetrahedron with mass
    vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
    triangles = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]

    from linkforge_core.physics.inertia import calculate_mesh_inertia_from_triangles

    res = calculate_mesh_inertia_from_triangles(vertices, triangles, mass=1.0)
    assert res.ixx >= 0
    assert res.iyy >= 0
    assert res.izz >= 0
