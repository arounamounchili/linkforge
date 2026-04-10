"""Non-manifold mesh validation utilities.

Provides topology checks for triangle meshes before physics calculations.
A valid mesh for inertia calculation must be:
  - Closed (watertight): every edge shared by exactly 2 triangles
  - Manifold: no T-junctions, no self-intersections
  - Consistently oriented: adjacent triangles share edges in opposite order
"""

from __future__ import annotations

from ..exceptions import RobotPhysicsError, ValidationErrorCode
from ..logging_config import get_logger

logger = get_logger(__name__)

# Validation message templates
WATERTIGHT_WARNING_TEMPLATE = (
    "Mesh has {count} boundary edge(s) — not watertight. Inertia calculation may be inaccurate."
)
NON_MANIFOLD_WARNING_TEMPLATE = (
    "Mesh has {count} non-manifold edge(s) (shared by >2 triangles). Mesh may be self-intersecting."
)


def validate_mesh_topology(
    triangles: list[tuple[int, int, int]],
    *,
    strict: bool = False,
) -> list[str]:
    """Check mesh topology for non-manifold issues.

    Args:
        triangles: Triangle index list
        strict: If True, raise on first issue. If False, collect all warnings.

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
        msg = WATERTIGHT_WARNING_TEMPLATE.format(count=len(boundary_edges))
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
        msg = NON_MANIFOLD_WARNING_TEMPLATE.format(count=len(non_manifold_edges))
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
