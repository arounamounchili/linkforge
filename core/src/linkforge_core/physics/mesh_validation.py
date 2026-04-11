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

    # Strict normalization and type checking
    try:
        triangles_list = list(triangles)
    except TypeError as e:
        msg = f"{prefix} failed validation: Triangle array must be iterable."
        if strict:
            raise RobotPhysicsError(
                ValidationErrorCode.INVALID_VALUE, msg, target="MeshTopology"
            ) from e
        warnings.append(msg)
        return warnings

    seen_faces = set()
    duplicate_count = 0
    degenerate_count = 0
    invalid_count = 0

    edge_counts: dict[tuple[int, int], int] = {}
    edge_directions: dict[tuple[int, int], list[tuple[int, int]]] = {}

    for tri in triangles_list:
        try:
            a, b, c = int(tri[0]), int(tri[1]), int(tri[2])
        except Exception:
            invalid_count += 1
            continue

        # Degenerate check (skipped from all edge mapping)
        if a == b or b == c or c == a:
            if level >= 2:
                degenerate_count += 1
            continue

        # Duplicate check (skipped from all edge mapping)
        if level >= 2:
            sorted_tri = tuple(sorted((a, b, c)))
            if sorted_tri in seen_faces:
                duplicate_count += 1
                continue
            seen_faces.add(sorted_tri)

        # Edge registration
        undirected_edges = [
            (min(a, b), max(a, b)),
            (min(b, c), max(b, c)),
            (min(c, a), max(c, a)),
        ]
        directed_edges = [(a, b), (b, c), (c, a)]

        for i in range(3):
            undirected_edge = undirected_edges[i]
            edge_counts[undirected_edge] = edge_counts.get(undirected_edge, 0) + 1

            if level >= 2:
                edge_directions.setdefault(undirected_edge, []).append(directed_edges[i])

    if invalid_count > 0:
        msg = f"{prefix} has {invalid_count} invalid triangle(s) (unparsable or missing indices)."
        warnings.append(msg)
        if strict:
            raise RobotPhysicsError(
                ValidationErrorCode.INVALID_VALUE,
                msg,
                target="MeshTopology",
                value=invalid_count,
            )
        logger.warning(msg)

    if level >= 2:
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

    # Topology Evaluation
    boundary_edges = []
    non_manifold_edges = []
    inconsistent_edges_count = 0

    for undirected_edge, count in edge_counts.items():
        if count == 1:
            boundary_edges.append(undirected_edge)
        elif count > 2:
            non_manifold_edges.append(undirected_edge)

        # Orientation evaluation (Level 2)
        # We only evaluate orientation for manifold edges (exactly 2 faces).
        # Non-manifold edges fail validation independently.
        if level >= 2 and count == 2:
            dirs = edge_directions[undirected_edge]
            # Consistent winding yields two unique directed edges (len == 2).
            # Identical (invalid) winding overwrites the same directed edge in the set (len == 1).
            if len(dirs) != 2:
                inconsistent_edges_count += 1
            else:
                (a, b), (c, d) = tuple(dirs)
                if not (a == d and b == c):
                    inconsistent_edges_count += 1

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
