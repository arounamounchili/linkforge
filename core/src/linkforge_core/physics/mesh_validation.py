"""Non-manifold mesh validation utilities.

Provides topology checks for triangle meshes before physics calculations.
A valid mesh for inertia calculation must be:
  - Closed (watertight): every edge shared by exactly 2 triangles
  - Manifold: no edges shared by >2 triangles
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
    level: int = 2,
    name: str | None = None,
) -> list[str]:
    """Check mesh topology for structural issues.

    Args:
        triangles: Triangle index list or (M, 3) array
        strict: If True, raise on first issue. If False, collect all warnings.
        level: Validation strictness level.
               1: Basic topology (boundary & non-manifold edges)
               2: Plus degenerate triangles, duplicate faces, and orientation consistency
        name: Optional mesh name for logging context.

    Returns:
        List of warning messages (empty = clean mesh)

    Raises:
        RobotPhysicsError: If strict=True and issues are found
    """
    warnings: list[str] = []
    prefix = f"Mesh '{name}'" if name else "Mesh"

    # Normalize generic iterators or numpy arrays to list form if needed
    try:
        triangles_list = list(triangles)
    except TypeError:
        triangles_list = []

    # Level 2 pre-checks
    if level >= 2:
        seen_faces = set()
        duplicate_count = 0
        degenerate_count = 0

        for tri in triangles_list:
            if len(tri) < 3 or len(set(tri[:3])) < 3:
                degenerate_count += 1
                continue

            sorted_tri = tuple(sorted(list(tri)[:3]))
            if sorted_tri in seen_faces:
                duplicate_count += 1
            seen_faces.add(sorted_tri)

        if degenerate_count > 0:
            msg = f"{prefix} has {degenerate_count} degenerate triangle(s) (missing or identical vertices)."
            warnings.append(msg)
            if strict:
                raise RobotPhysicsError(
                    ValidationErrorCode.PHYSICS_VIOLATION,
                    msg,
                    target="MeshTopology",
                    value=degenerate_count,
                )
            logger.warning(msg)

        if duplicate_count > 0:
            msg = f"{prefix} has {duplicate_count} duplicate triangle(s)."
            warnings.append(msg)
            if strict:
                raise RobotPhysicsError(
                    ValidationErrorCode.PHYSICS_VIOLATION,
                    msg,
                    target="MeshTopology",
                    value=duplicate_count,
                )
            logger.warning(msg)

    # Edge tracking
    edge_map: dict[tuple[int, int], list[int]] = {}
    directed_edges: set[tuple[int, int]] = set()
    inconsistent_edges_count = 0

    for tri_idx, tri in enumerate(triangles_list):
        if len(tri) < 3:
            continue
        u, v, w = int(tri[0]), int(tri[1]), int(tri[2])

        # Undirected edges
        undirected_edges = [
            (min(u, v), max(u, v)),
            (min(v, w), max(v, w)),
            (min(w, u), max(w, u)),
        ]
        for edge in undirected_edges:
            edge_map.setdefault(edge, []).append(tri_idx)

        # Directed edges for orientation consistency
        if level >= 2:
            dir_edges = [(u, v), (v, w), (w, u)]
            for de in dir_edges:
                if de in directed_edges:
                    inconsistent_edges_count += 1
                directed_edges.add(de)

    # Watertight / Manifold checks
    boundary_edges = [e for e, tris in edge_map.items() if len(tris) == 1]
    non_manifold_edges = [e for e, tris in edge_map.items() if len(tris) > 2]

    if boundary_edges:
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

    if level >= 2 and inconsistent_edges_count > 0:
        msg = f"{prefix} has {inconsistent_edges_count} edge(s) with inconsistent winding (orientation mismatch)."
        warnings.append(msg)
        if strict:
            raise RobotPhysicsError(
                ValidationErrorCode.PHYSICS_VIOLATION,
                msg,
                target="MeshTopology",
                value=inconsistent_edges_count,
            )
        logger.warning(msg)

    return warnings
