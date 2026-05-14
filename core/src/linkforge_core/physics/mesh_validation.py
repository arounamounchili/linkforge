"""Mesh topology and numerical validation utilities.

Provides checks for the 'Top Six Pathologies' of triangle meshes to ensure
physical accuracy and simulation stability.

Core Components:
    - validate_mesh_topology: Comprehensive check for manifoldness, winding, and slivers.
    - Severity: Enum for differentiating advisory warnings from fatal errors.
"""

from __future__ import annotations

import math
from typing import Any

from ..exceptions import RobotPhysicsError, ValidationErrorCode
from ..logging_config import get_logger
from ..validation.result import Severity, ValidationIssue

logger = get_logger(__name__)


def validate_mesh_topology(
    vertices: list[tuple[float, float, float]] | Any,
    triangles: list[tuple[int, int, int]] | Any,
    *,
    strict: bool = False,
    level: int = 2,
    name: str | None = None,
    proximity_threshold: int = 6,
    sliver_threshold: float = 1000.0,
) -> list[ValidationIssue]:
    """Check mesh topology for structural and numerical issues.

    Args:
        vertices: Vertex coordinate list or (N, 3) array (meters)
        triangles: Triangle index list or (M, 3) array
        strict: If True, raise on first issue. If False, collect all warnings.
        level: Validation strictness level.
               1: Basic topology (boundary & non-manifold edges)
               2: Comprehensive (adds degenerate, slivers, duplicates, winding, and vertex proximity)
        name: Optional mesh name for logging context.
        proximity_threshold: Decimal precision for vertex proximity check (default 6).
        sliver_threshold: Aspect ratio threshold for sliver triangle detection (default 1000).

    Returns:
        List of ValidationIssue objects (empty = clean mesh)

    Raises:
        RobotPhysicsError: If strict=True and issues are found
    """
    issues: list[ValidationIssue] = []
    prefix = f"Mesh '{name}'" if name else "Mesh"

    # --- 1. Basic Input Normalization ---
    try:
        triangles_list = list(triangles)
        vertices_list = list(vertices)
    except TypeError as e:
        issue = ValidationIssue(
            severity=Severity.ERROR,
            title="Invalid Input",
            message=f"{prefix} failed validation: Vertices and Triangles must be iterable.",
            code=ValidationErrorCode.INVALID_VALUE,
        )
        if strict and issue.code:
            raise RobotPhysicsError(issue.code, issue.message, target="MeshTopology") from e
        issues.append(issue)
        return issues

    # --- 2. Vertex Proximity Check (Level 2) ---
    # Detects "unwelded" vertices where different indices share the same coordinate.
    if level >= 2:
        coord_map: dict[tuple[float, float, float], list[int]] = {}
        for i, v in enumerate(vertices_list):
            # Round to avoid floating point noise from CAD exporters
            rounded_v = (
                round(float(v[0]), proximity_threshold),
                round(float(v[1]), proximity_threshold),
                round(float(v[2]), proximity_threshold),
            )
            coord_map.setdefault(rounded_v, []).append(i)

        unwelded_groups = [indices for indices in coord_map.values() if len(indices) > 1]
        if unwelded_groups:
            count = sum(len(g) for g in unwelded_groups)
            issue = ValidationIssue(
                severity=Severity.WARNING,
                title="Unwelded Vertices",
                message=f"{prefix} has {count} unwelded vertices (duplicate coordinates with different indices).",
                code=ValidationErrorCode.MESH_UNWELDED,
                suggestion="Consider welding vertices in your CAD tool or using the 'Weld' modifier.",
            )
            issues.append(issue)
            if strict and issue.code:
                raise RobotPhysicsError(
                    issue.code,
                    issue.message,
                    target="MeshTopology",
                    value=count,
                )
            logger.warning(issue.message)

    # --- 3. Edge Registration & Triangle-Level Filtering ---
    seen_faces = set()
    duplicate_count = 0
    degenerate_count = 0
    sliver_count = 0
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

        # Sliver check (Level 2)
        # Skinny triangles cause numerical instability in physics engines.
        if level >= 2:
            va, vb, vc = vertices_list[a], vertices_list[b], vertices_list[c]

            # Edge vectors
            ab = (vb[0] - va[0], vb[1] - va[1], vb[2] - va[2])
            bc = (vc[0] - vb[0], vc[1] - vb[1], vc[2] - vb[2])
            ca = (va[0] - vc[0], va[1] - vc[1], va[2] - vc[2])

            # Squared edge lengths
            l2_ab = ab[0] ** 2 + ab[1] ** 2 + ab[2] ** 2
            l2_bc = bc[0] ** 2 + bc[1] ** 2 + bc[2] ** 2
            l2_ca = ca[0] ** 2 + ca[1] ** 2 + ca[2] ** 2

            l2_max = max(l2_ab, l2_bc, l2_ca)

            # Area = 0.5 * |AB x AC|
            # (Reusable for normal but we only need magnitude)
            ac = (vc[0] - va[0], vc[1] - va[1], vc[2] - va[2])
            cp_x = ab[1] * ac[2] - ab[2] * ac[1]
            cp_y = ab[2] * ac[0] - ab[0] * ac[2]
            cp_z = ab[0] * ac[1] - ab[1] * ac[0]

            area = 0.5 * math.sqrt(cp_x**2 + cp_y**2 + cp_z**2)

            # Aspect Ratio = L_max / H_min
            # Area = 0.5 * L_max * H_min  => H_min = 2 * Area / L_max
            # S = L_max / (2 * Area / L_max) = L_max^2 / (2 * Area)
            if area > 1e-15:  # Avoid division by zero (handled by degenerate check)
                aspect_ratio = l2_max / (2 * area)
                if aspect_ratio > sliver_threshold:
                    sliver_count += 1

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

    # --- 4. Preliminary Warnings (Invalid/Degenerate/Duplicate) ---
    if invalid_count > 0:
        issue = ValidationIssue(
            severity=Severity.ERROR,
            title="Invalid Triangles",
            message=f"{prefix} has {invalid_count} invalid triangle(s) (unparsable or missing indices).",
            code=ValidationErrorCode.INVALID_VALUE,
        )
        issues.append(issue)
        if strict and issue.code:
            raise RobotPhysicsError(
                issue.code,
                issue.message,
                target="MeshTopology",
                value=invalid_count,
            )
        logger.warning(issue.message)

    if level >= 2:
        if degenerate_count > 0:
            issue = ValidationIssue(
                severity=Severity.WARNING,
                title="Degenerate Triangles",
                message=f"{prefix} has {degenerate_count} degenerate triangle(s) (missing or identical vertices).",
                code=ValidationErrorCode.MESH_DEGENERATE,
            )
            issues.append(issue)
            if strict and issue.code:
                raise RobotPhysicsError(
                    issue.code,
                    issue.message,
                    target="MeshTopology",
                    value=degenerate_count,
                )
            logger.warning(issue.message)

        if sliver_count > 0:
            issue = ValidationIssue(
                severity=Severity.WARNING,
                title="Sliver Triangles",
                message=f"{prefix} has {sliver_count} sliver triangle(s) (aspect ratio > {sliver_threshold}).",
                code=ValidationErrorCode.MESH_SLIVER,
                suggestion="Skinny triangles cause numerical instability. Refine the mesh to improve quality.",
            )
            issues.append(issue)
            # Sliver check NEVER raises even in strict mode - only a warning
            logger.warning(issue.message)

        if duplicate_count > 0:
            issue = ValidationIssue(
                severity=Severity.WARNING,
                title="Duplicate Faces",
                message=f"{prefix} has {duplicate_count} duplicate triangle(s).",
                code=ValidationErrorCode.MESH_DUPLICATE_FACE,
            )
            issues.append(issue)
            if strict and issue.code:
                raise RobotPhysicsError(
                    issue.code,
                    issue.message,
                    target="MeshTopology",
                    value=duplicate_count,
                )
            logger.warning(issue.message)

    # --- 5. Topology Evaluation (Boundary/Manifold/Winding) ---
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
            # Consistent winding yields two unique directed edges (len == 2)
            # and MUST be in opposite directions: (A,B) and (B,A)
            (u, v), (x, y) = tuple(dirs)
            if not (u == y and v == x):
                inconsistent_edges_count += 1

    if boundary_edges:
        issue = ValidationIssue(
            severity=Severity.WARNING,
            title="Non-Watertight Mesh",
            message=f"{prefix} has {len(boundary_edges)} boundary edge(s).",
            code=ValidationErrorCode.MESH_BOUNDARY_EDGE,
            suggestion="Ensure the mesh is closed (watertight) for accurate inertia calculation.",
        )
        issues.append(issue)
        if strict and issue.code:
            raise RobotPhysicsError(
                issue.code,
                issue.message,
                target="MeshTopology",
                value=len(boundary_edges),
            )
        logger.warning(issue.message)

    if non_manifold_edges:
        issue = ValidationIssue(
            severity=Severity.WARNING,
            title="Non-Manifold Mesh",
            message=f"{prefix} has {len(non_manifold_edges)} non-manifold edge(s) (shared by >2 triangles).",
            code=ValidationErrorCode.MESH_NON_MANIFOLD,
        )
        issues.append(issue)
        if strict and issue.code:
            raise RobotPhysicsError(
                issue.code,
                issue.message,
                target="MeshTopology",
                value=len(non_manifold_edges),
            )
        logger.warning(issue.message)

    if level >= 2 and inconsistent_edges_count > 0:
        issue = ValidationIssue(
            severity=Severity.WARNING,
            title="Inconsistent Winding",
            message=f"{prefix} has {inconsistent_edges_count} edge(s) with inconsistent winding.",
            code=ValidationErrorCode.MESH_INCONSISTENT_WINDING,
            suggestion="Ensure all normals point outward. Some faces may be flipped.",
        )
        issues.append(issue)
        if strict and issue.code:
            raise RobotPhysicsError(
                issue.code,
                issue.message,
                target="MeshTopology",
                value=inconsistent_edges_count,
            )
        logger.warning(issue.message)

    return issues
