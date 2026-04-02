from unittest.mock import patch

import numpy as np
import pytest
from linkforge.blender.utils.physics import calculate_mesh_inertia_numpy
from linkforge_core.models.link import InertiaTensor


def get_cube_mesh(side=1.0, offset=(0, 0, 0), inverted=False):
    """Generate cube mesh vertices and triangles."""
    # Standard cube vertices
    s = side
    ox, oy, oz = offset
    vertices = np.array(
        [
            (ox, oy, oz),
            (ox + s, oy, oz),
            (ox + s, oy + s, oz),
            (ox, oy + s, oz),
            (ox, oy, oz + s),
            (ox + s, oy, oz + s),
            (ox + s, oy + s, oz + s),
            (ox, oy + s, oz + s),
        ],
        dtype=np.float64,
    )

    # Counter-clockwise winding (outward normals)
    if not inverted:
        triangles = np.array(
            [
                (0, 2, 1),
                (0, 3, 2),  # bottom
                (4, 5, 6),
                (4, 6, 7),  # top
                (0, 1, 5),
                (0, 5, 4),  # front
                (1, 2, 6),
                (1, 6, 5),  # right
                (2, 3, 7),
                (2, 7, 6),  # back
                (3, 0, 4),
                (3, 4, 7),  # left
            ],
            dtype=np.int32,
        )
    else:
        # Clockwise winding (inverted normals)
        triangles = np.array(
            [
                (0, 1, 2),
                (0, 2, 3),  # bottom
                (4, 6, 5),
                (4, 7, 6),  # top
                (0, 5, 1),
                (0, 4, 5),  # front
                (1, 6, 2),
                (1, 5, 6),  # right
                (2, 7, 3),
                (2, 6, 7),  # back
                (3, 4, 0),
                (3, 7, 4),  # left
            ],
            dtype=np.int32,
        )

    return vertices, triangles


def test_calculate_mesh_inertia_numpy_cube():
    """Test inertia calculation for a 1m unit cube (1kg)."""
    vertices, triangles = get_cube_mesh(side=1.0)
    mass = 1.0
    inertia = calculate_mesh_inertia_numpy(vertices, triangles, mass)

    assert inertia is not None
    # 1m cube, 1kg => Ixx = Iyy = Izz = 1/6 kg.m^2
    expected = 1.0 / 6.0
    assert pytest.approx(inertia.ixx, abs=1e-5) == expected
    assert pytest.approx(inertia.iyy, abs=1e-5) == expected
    assert pytest.approx(inertia.izz, abs=1e-5) == expected
    # Symmetry and COM alignment ensure zero cross-products
    assert pytest.approx(inertia.ixy, abs=1e-5) == 0.0


def test_calculate_mesh_inertia_numpy_inverted_cube():
    """Test that inverted normals (negative volume) still yield correct positive inertia."""
    vertices, triangles = get_cube_mesh(side=1.0, inverted=True)
    mass = 1.0
    inertia = calculate_mesh_inertia_numpy(vertices, triangles, mass)

    assert inertia is not None
    expected = 1.0 / 6.0
    assert pytest.approx(inertia.ixx, abs=1e-5) == expected


def test_calculate_mesh_inertia_numpy_offset_cube():
    """Test offset-invariance via Parallel Axis Theorem."""
    vertices, triangles = get_cube_mesh(side=1.0, offset=(10, 20, 30))
    mass = 1.0
    inertia = calculate_mesh_inertia_numpy(vertices, triangles, mass)

    assert inertia is not None
    expected = 1.0 / 6.0
    assert pytest.approx(inertia.ixx, abs=1e-5) == expected


def test_calculate_mesh_inertia_numpy_zero_mass():
    """Test that zero mass returns a zero inertia tensor."""
    vertices, triangles = get_cube_mesh(side=1.0)
    result = calculate_mesh_inertia_numpy(vertices, triangles, 0.0)
    expected = InertiaTensor.zero()
    assert result.ixx == expected.ixx
    assert result.iyy == expected.iyy
    assert result.izz == expected.izz
    assert result.ixy == expected.ixy
    assert result.ixz == expected.ixz
    assert result.iyz == expected.iyz


def test_calculate_mesh_inertia_numpy_empty_mesh():
    """Test handling of empty mesh data."""
    vertices = np.array([], dtype=np.float64).reshape((0, 3))
    triangles = np.array([], dtype=np.int32).reshape((0, 3))
    inertia = calculate_mesh_inertia_numpy(vertices, triangles, 1.0)
    assert inertia is None


def test_calculate_mesh_inertia_numpy_degenerate_mesh():
    """Test handling of (near) zero volume mesh."""
    # Single triangle is zero volume
    vertices = np.array([(0, 0, 0), (1, 0, 0), (0, 1, 0)], dtype=np.float64)
    triangles = np.array([(0, 1, 2)], dtype=np.int32)
    inertia = calculate_mesh_inertia_numpy(vertices, triangles, 1.0)
    assert inertia is None


def test_calculate_mesh_inertia_numpy_no_numpy():
    """Test fallback logic when NumPy is unavailable."""
    vertices, triangles = get_cube_mesh()
    with patch("linkforge.blender.utils.physics.np", None):
        inertia = calculate_mesh_inertia_numpy(vertices, triangles, 1.0)
        assert inertia is None
