"""Non-manifold mesh validation utilities.

Provides topology checks for triangle meshes before physics calculations.
A valid mesh for inertia calculation must be:
  - Closed (watertight): every edge shared by exactly 2 triangles
  - Manifold: no T-junctions, no self-intersections
  - Consistently oriented: adjacent triangles share edges in opposite order
"""

from __future__ import annotations

from typing import Any

from ..exceptions import RobotPhysicsError, ValidationErrorCode
from ..logging_config import get_logger

logger = get_logger(__name__)


def validate_mesh_topology(
    triangles: list[tuple[int, int, int]] | Any,
    *,
    strict: bool = False,
    name: str | None = None,
) -> list[str]:
    """Check mesh topology for non-manifold issues.

    Args:
        triangles: Triangle index list or (M, 3) array
        strict: If True, raise on first issue. If False, collect all warnings.
        name: Optional mesh name for logging context.

    Returns:
        List of warning messages (empty = clean mesh)

    Raises:
        RobotPhysicsError: If strict=True and issues are found
    """
    warnings = []

    # Build edge → triangle map
    # Edge is represented as a sorted tuple (min_idx, max_idx)
    edge_map: dict[tuple[int, int], list[int]] = {}
    for tri_idx, tri in enumerate(triangles):
        edges = [
            (min(tri[0], tri[1]), max(tri[0], tri[1])),
            (min(tri[1], tri[2]), max(tri[1], tri[2])),
            (min(tri[2], tri[0]), max(tri[2], tri[0])),
        ]
        for edge in edges:
            edge_map.setdefault(edge, []).append(tri_idx)

    # Every edge must be shared by exactly 2 triangles (watertight)
    boundary_edges = [e for e, tris in edge_map.items() if len(tris) == 1]
    non_manifold_edges = [e for e, tris in edge_map.items() if len(tris) > 2]

    if boundary_edges:
        prefix = f"Mesh '{name}'" if name else "Mesh"
        msg = f"{prefix} has {len(boundary_edges)} boundary edge(s) — not watertight. Inertia calculation may be inaccurate."
        warnings.append(msg)
        if strict:
            raise RobotPhysicsError(
                ValidationErrorCode.PHYSICS_VIOLATION,
                msg,
                target="MeshTopology",
                value=len(boundary_edges),
            )
        logger.warning(msg)

    if non_manifold_edges:
        prefix = f"Mesh '{name}'" if name else "Mesh"
        msg = f"{prefix} has {len(non_manifold_edges)} non-manifold edge(s) (shared by >2 triangles). Mesh may be self-intersecting."
        warnings.append(msg)
        if strict:
            raise RobotPhysicsError(
                ValidationErrorCode.PHYSICS_VIOLATION,
                msg,
                target="MeshTopology",
                value=len(non_manifold_edges),
            )
        logger.warning(msg)

    return warnings
