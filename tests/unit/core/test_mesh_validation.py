"""Unit tests for mesh topology validation."""

from __future__ import annotations

import pytest
from linkforge.core import RobotPhysicsError, ValidationErrorCode, validate_mesh_topology


class TestMeshTopologyValidation:
    """Test suite for mesh topology validation utility."""

    def test_watertight_consistent_mesh(self) -> None:
        """A simple watertight, consistently oriented tetrahedron."""
        vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
        triangles = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]
        warnings = validate_mesh_topology(vertices, triangles, level=2)
        assert len(warnings) == 0

    def test_open_mesh_boundary_edge(self) -> None:
        """A mesh with a missing face should report boundary edges."""
        vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
        # Tetrahedron missing a face
        triangles = [
            (0, 2, 1),
            (0, 1, 3),
            (0, 3, 2),
            # (1, 2, 3)
        ]
        warnings = validate_mesh_topology(vertices, triangles, level=1)
        assert any(w.code == ValidationErrorCode.MESH_BOUNDARY_EDGE for w in warnings)

    def test_non_manifold_edge(self) -> None:
        """An edge shared by 3 triangles should report as non-manifold."""
        vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 1)]
        triangles = [
            (0, 2, 1),
            (0, 1, 3),
            (0, 3, 2),
            (1, 2, 3),
            (1, 2, 4),  # Extra face sharing edge (1, 2)
        ]
        warnings = validate_mesh_topology(vertices, triangles, level=1)
        assert any(w.code == ValidationErrorCode.MESH_NON_MANIFOLD for w in warnings)

    def test_degenerate_triangles_level_2(self) -> None:
        """Degenerate triangles (identical vertices) should yield a warning at level >= 2."""
        vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
        triangles = [
            (0, 2, 1),
            (1, 1, 2),  # Degenerate
        ]
        warnings = validate_mesh_topology(vertices, triangles, level=2)
        assert any(w.code == ValidationErrorCode.MESH_DEGENERATE for w in warnings)

        # Should be ignored at level 1 (only flags boundary/manifold issues)
        warnings_l1 = validate_mesh_topology(vertices, triangles, level=1)
        assert not any(w.code == ValidationErrorCode.MESH_DEGENERATE for w in warnings_l1)

    def test_duplicate_faces_level_2(self) -> None:
        """Duplicate faces should yield a warning at level >= 2."""
        vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
        triangles = [
            (0, 2, 1),
            (1, 0, 2),  # Duplicate of the same face ignoring index order
            (0, 1, 3),
        ]
        warnings = validate_mesh_topology(vertices, triangles, level=2)
        assert any(w.code == ValidationErrorCode.MESH_DUPLICATE_FACE for w in warnings)

    def test_inconsistent_orientation_level_2(self) -> None:
        """Inconsistent face winding should yield a warning at level >= 2."""
        vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
        triangles = [
            (0, 2, 1),
            (0, 3, 1),  # Flipped! Should be (0, 1, 3) according to ccw rule
            (0, 3, 2),
            (1, 2, 3),
        ]
        warnings = validate_mesh_topology(vertices, triangles, level=2)
        assert any(w.code == ValidationErrorCode.MESH_INCONSISTENT_WINDING for w in warnings)

    def test_unwelded_vertices(self) -> None:
        """Different indices sharing the same coordinates should yield a proximity warning."""
        # 0 and 4 are at the same location but have different indices
        vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 0.00000001, 0)]
        triangles = [(0, 2, 1), (4, 1, 3), (0, 3, 2), (1, 2, 3)]

        # Should warning at level 2 (proximity_threshold default is 6, so 0.00000001 matches 0)
        warnings = validate_mesh_topology(vertices, triangles, level=2)
        assert any(w.code == ValidationErrorCode.MESH_UNWELDED for w in warnings)

        # If we set proximity_threshold to 9, they should NOT match
        warnings_strict = validate_mesh_topology(
            vertices, triangles, level=2, proximity_threshold=9
        )
        assert not any(w.code == ValidationErrorCode.MESH_UNWELDED for w in warnings_strict)

    def test_strict_mode_raises(self) -> None:
        """Strict mode should raise RobotPhysicsError on topology issues."""
        vertices = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 1)]

        # Boundary
        with pytest.raises(RobotPhysicsError, match="boundary edge"):
            validate_mesh_topology(vertices, [(0, 1, 2)], strict=True)

        # Duplicate
        with pytest.raises(RobotPhysicsError, match="duplicate"):
            validate_mesh_topology(vertices, [(0, 1, 2), (0, 2, 1)], strict=True, level=2)

        # Degenerate
        with pytest.raises(RobotPhysicsError, match="degenerate"):
            validate_mesh_topology(vertices, [(0, 1, 1)], strict=True, level=2)

        # Non-manifold
        triangles_non_manifold = [
            (0, 1, 2),
            (0, 2, 3),
            (0, 3, 1),
            (1, 3, 2),
            (4, 1, 2),
            (4, 2, 5),
            (4, 5, 1),
            (1, 5, 2),
        ]
        vertices_nm = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 0, -1), (1, 1, 0)]
        with pytest.raises(RobotPhysicsError, match="non-manifold edge"):
            validate_mesh_topology(vertices_nm, triangles_non_manifold, strict=True, level=1)

        # Inconsistent winding
        triangles_inconsistent = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 3, 2)]
        with pytest.raises(RobotPhysicsError, match="inconsistent winding"):
            validate_mesh_topology(vertices, triangles_inconsistent, strict=True, level=2)

        # Unwelded
        vertices_unwelded = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 0, 0)]
        with pytest.raises(RobotPhysicsError, match="unwelded"):
            validate_mesh_topology(vertices_unwelded, triangles_inconsistent, strict=True, level=2)

        # Iterator validation
        warnings = validate_mesh_topology(None, None, level=1)
        assert len(warnings) == 1
        assert warnings[0].code == ValidationErrorCode.INVALID_VALUE

        with pytest.raises(RobotPhysicsError, match="iterable"):
            validate_mesh_topology(None, None, strict=True)

        # Short faces cause IndexError, which trips invalid_count
        vertices = [(0, 0, 0), (1, 0, 0)]
        warnings = validate_mesh_topology(vertices, [(0, 1)], level=1)
        assert any(w.code == ValidationErrorCode.INVALID_VALUE for w in warnings)

        with pytest.raises(RobotPhysicsError, match="invalid"):
            validate_mesh_topology(vertices, [(0, 1)], strict=True, level=1)

        # Test string characters trip ValueError -> invalid_count
        warnings = validate_mesh_topology(vertices, [("a", "b", "c")], level=1)
        assert any(w.code == ValidationErrorCode.INVALID_VALUE for w in warnings)

    def test_sliver_triangles(self) -> None:
        """Sliver triangles should yield a warning at level >= 2."""
        # Base = 1, Height = 0.0001 => Aspect ratio = 1/(2*0.5*0.0001) = 10000
        vertices = [(0, 0, 0), (1, 0, 0), (0.5, 0.0001, 0)]
        triangles = [(0, 1, 2)]

        # Should warning at level 2 with default threshold (1000)
        warnings = validate_mesh_topology(vertices, triangles, level=2)
        assert any(w.code == ValidationErrorCode.MESH_SLIVER for w in warnings)

        # Should NOT warning if we increase threshold to 20000
        warnings_clean = validate_mesh_topology(
            vertices, triangles, level=2, sliver_threshold=20000
        )
        assert not any(w.code == ValidationErrorCode.MESH_SLIVER for w in warnings_clean)
