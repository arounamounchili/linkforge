"""Unit tests for mesh topology validation."""

from __future__ import annotations

import pytest
from linkforge_core.exceptions import RobotPhysicsError
from linkforge_core.physics.mesh_validation import validate_mesh_topology


class TestMeshTopologyValidation:
    """Test suite for mesh topology validation utility."""

    def test_watertight_cube(self) -> None:
        """A standard watertight cube should return no warnings."""
        triangles = [
            (0, 1, 2),
            (0, 2, 3),  # Bottom
            (4, 6, 5),
            (4, 7, 6),  # Top
            (0, 1, 5),
            (0, 5, 4),  # Front
            (2, 3, 7),
            (2, 7, 6),  # Back
            (0, 4, 7),
            (0, 7, 3),  # Left
            (1, 5, 6),
            (1, 6, 2),  # Right
        ]

        warnings = validate_mesh_topology(triangles)
        assert len(warnings) == 0

    def test_open_mesh_boundary_edge(self) -> None:
        """A mesh with a missing face should report boundary edges."""
        # Cube missing the top face
        triangles = [
            (0, 1, 2),
            (0, 2, 3),  # Bottom
            # (4, 6, 5), (4, 7, 6),  # Missing Top
            (0, 1, 5),
            (0, 5, 4),  # Front
            (2, 3, 7),
            (2, 7, 6),  # Back
            (0, 4, 7),
            (0, 7, 3),  # Left
            (1, 5, 6),
            (1, 6, 2),  # Right
        ]

        warnings = validate_mesh_topology(triangles)
        assert any("boundary edge" in w.lower() for w in warnings)

    def test_non_manifold_edge(self) -> None:
        """An edge shared by 3 triangles should report as non-manifold."""
        triangles = [
            (0, 1, 2),
            (0, 1, 3),
            (0, 1, 2),  # Duplicate triangle sharing edge (0, 1)
        ]

        warnings = validate_mesh_topology(triangles)
        assert any("non-manifold edge" in w.lower() for w in warnings)

    def test_strict_mode_raises(self) -> None:
        """Strict mode should raise RobotPhysicsError on topology issues."""
        triangles = [(0, 1, 2)]  # Single triangle is essentially all boundary edges

        with pytest.raises(RobotPhysicsError, match="boundary edge"):
            validate_mesh_topology(triangles, strict=True)
