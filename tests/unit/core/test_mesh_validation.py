"""Unit tests for mesh topology validation."""

from __future__ import annotations

import pytest
from linkforge_core.exceptions import RobotPhysicsError
from linkforge_core.physics.mesh_validation import validate_mesh_topology


class TestMeshTopologyValidation:
    """Test suite for mesh topology validation utility."""

    def test_watertight_consistent_mesh(self) -> None:
        """A simple watertight, consistently oriented tetrahedron."""
        triangles = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]
        warnings = validate_mesh_topology(triangles, level=2)
        assert len(warnings) == 0

    def test_open_mesh_boundary_edge(self) -> None:
        """A mesh with a missing face should report boundary edges."""
        # Tetrahedron missing a face
        triangles = [
            (0, 2, 1),
            (0, 1, 3),
            (0, 3, 2),
            # (1, 2, 3)
        ]
        warnings = validate_mesh_topology(triangles, level=1)
        assert any("boundary edge" in w.lower() for w in warnings)

    def test_non_manifold_edge(self) -> None:
        """An edge shared by 3 triangles should report as non-manifold."""
        triangles = [
            (0, 2, 1),
            (0, 1, 3),
            (0, 3, 2),
            (1, 2, 3),
            (1, 2, 4),  # Extra face sharing edge (1, 2)
        ]
        warnings = validate_mesh_topology(triangles, level=1)
        assert any("non-manifold edge" in w.lower() for w in warnings)

    def test_degenerate_triangles_level_2(self) -> None:
        """Degenerate triangles (identical vertices) should yield a warning at level >= 2."""
        triangles = [
            (0, 2, 1),
            (1, 1, 2),  # Degenerate
        ]
        warnings = validate_mesh_topology(triangles, level=2)
        assert any("degenerate" in w.lower() for w in warnings)

        # Should be ignored at level 1 (only flags boundary/manifold issues)
        warnings_l1 = validate_mesh_topology(triangles, level=1)
        assert not any("degenerate" in w.lower() for w in warnings_l1)

    def test_duplicate_faces_level_2(self) -> None:
        """Duplicate faces should yield a warning at level >= 2."""
        triangles = [
            (0, 2, 1),
            (1, 0, 2),  # Duplicate of the same face ignoring index order
            (0, 1, 3),
        ]
        warnings = validate_mesh_topology(triangles, level=2)
        assert any("duplicate" in w.lower() for w in warnings)

    def test_inconsistent_orientation_level_2(self) -> None:
        """Inconsistent face winding should yield a warning at level >= 2."""
        triangles = [
            (0, 2, 1),
            (0, 3, 1),  # Flipped! Should be (0, 1, 3) according to ccw rule
            (0, 3, 2),
            (1, 2, 3),
        ]
        warnings = validate_mesh_topology(triangles, level=2)
        assert any("inconsistent winding" in w.lower() for w in warnings)

    def test_strict_mode_raises(self) -> None:
        """Strict mode should raise RobotPhysicsError on topology issues."""
        triangles = [(0, 1, 2)]  # Boundary edges
        with pytest.raises(RobotPhysicsError, match="boundary edge"):
            validate_mesh_topology(triangles, strict=True)

        triangles_dup = [(0, 1, 2), (0, 2, 1)]
        with pytest.raises(RobotPhysicsError, match="duplicate"):
            validate_mesh_topology(triangles_dup, strict=True, level=2)

        # Test degenerate triangles in strict mode
        with pytest.raises(RobotPhysicsError, match="degenerate"):
            validate_mesh_topology([(0, 1, 1)], strict=True, level=2)

        # Two tetrahedrons sharing one edge (1, 2). Completely closed, but edge (1, 2) is non-manifold (4 faces).
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
        with pytest.raises(RobotPhysicsError, match="non-manifold edge"):
            validate_mesh_topology(triangles_non_manifold, strict=True, level=1)

        # Standard tetrahedron but the last face is reversed
        triangles_inconsistent = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 3, 2)]
        with pytest.raises(RobotPhysicsError, match="inconsistent winding"):
            validate_mesh_topology(triangles_inconsistent, strict=True, level=2)

    def test_failing_iterator_and_short_faces(self) -> None:
        """Test fallback type casting and short length faces at level 1."""
        warnings = validate_mesh_topology(None, level=1)
        assert len(warnings) == 1
        assert "iterable" in warnings[0]

        with pytest.raises(RobotPhysicsError, match="iterable"):
            validate_mesh_topology(None, strict=True)

        # Short faces cause IndexError, which trips invalid_count
        warnings = validate_mesh_topology([(0, 1)], level=1)
        assert len(warnings) == 1

        with pytest.raises(RobotPhysicsError, match="invalid"):
            validate_mesh_topology([(0, 1)], strict=True, level=1)

        # Test string characters trip ValueError -> invalid_count
        warnings = validate_mesh_topology([("a", "b", "c")], level=1)
        assert len(warnings) == 1
