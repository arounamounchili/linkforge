"""Inertia tensor calculation for primitive geometries.

Based on standard formulas for common shapes:
https://en.wikipedia.org/wiki/List_of_moments_of_inertia
"""

from __future__ import annotations

import os
from functools import lru_cache
from math import isfinite

from ..exceptions import RobotMathError, RobotPhysicsError, ValidationErrorCode
from ..logging_config import get_logger
from ..models.geometry import Box, Cylinder, Geometry, Mesh, Sphere
from ..models.link import InertiaTensor
from .mesh_validation import validate_mesh_topology

logger = get_logger(__name__)

# Numerical thresholds for physical stability and mesh integrity
NEGATIVE_INERTIA_THRESHOLD = -1e-06
MIN_MASS_STABILITY_THRESHOLD = 0.01  # kg
DEGENERATE_VOL_THRESHOLD = 1e-12  # m³
MIN_INERTIA_STABILITY_VALUE = 1e-06  # kg·m²

# Configurable cache size for inertia calculations
DEFAULT_INERTIA_CACHE_SIZE = int(os.environ.get("LINKFORGE_INERTIA_CACHE_SIZE", "512"))


def _get_stability_fallback() -> InertiaTensor:
    """Return a minimal inertia tensor for numerical stability."""
    val = MIN_INERTIA_STABILITY_VALUE
    return InertiaTensor(ixx=val, ixy=0.0, ixz=0.0, iyy=val, iyz=0.0, izz=val)


@lru_cache(maxsize=DEFAULT_INERTIA_CACHE_SIZE)
def _calculate_box_inertia_cached(x: float, y: float, z: float, mass: float) -> InertiaTensor:
    """Cached calculation of box inertia tensor."""
    ixx = (1.0 / 12.0) * mass * (y * y + z * z)
    iyy = (1.0 / 12.0) * mass * (x * x + z * z)
    izz = (1.0 / 12.0) * mass * (x * x + y * y)
    return InertiaTensor(ixx=ixx, ixy=0.0, ixz=0.0, iyy=iyy, iyz=0.0, izz=izz)


def calculate_box_inertia(box: Box, mass: float) -> InertiaTensor:
    """Calculate inertia tensor for a box (rectangular cuboid).

    Args:
        box: Box geometry with size (x, y, z)
        mass: Total mass in kg

    Returns:
        Inertia tensor about center of mass
    """
    if mass < MIN_MASS_STABILITY_THRESHOLD:
        return _get_stability_fallback()

    return _calculate_box_inertia_cached(box.size.x, box.size.y, box.size.z, mass)


@lru_cache(maxsize=DEFAULT_INERTIA_CACHE_SIZE)
def _calculate_cylinder_inertia_cached(radius: float, length: float, mass: float) -> InertiaTensor:
    """Cached calculation of cylinder inertia tensor."""
    ixx = iyy = (1.0 / 12.0) * mass * (3 * radius * radius + length * length)
    izz = 0.5 * mass * radius * radius
    return InertiaTensor(ixx=ixx, ixy=0.0, ixz=0.0, iyy=iyy, iyz=0.0, izz=izz)


def calculate_cylinder_inertia(cylinder: Cylinder, mass: float) -> InertiaTensor:
    """Calculate inertia tensor for a cylinder (axis along Z).

    Args:
        cylinder: Cylinder geometry with radius and length
        mass: Total mass in kg

    Returns:
        Inertia tensor about center of mass
    """
    if mass < MIN_MASS_STABILITY_THRESHOLD:
        return _get_stability_fallback()

    return _calculate_cylinder_inertia_cached(cylinder.radius, cylinder.length, mass)


@lru_cache(maxsize=DEFAULT_INERTIA_CACHE_SIZE)
def _calculate_sphere_inertia_cached(radius: float, mass: float) -> InertiaTensor:
    """Cached calculation of sphere inertia tensor."""
    i = (2.0 / 5.0) * mass * radius * radius
    return InertiaTensor(ixx=i, ixy=0.0, ixz=0.0, iyy=i, iyz=0.0, izz=i)


def calculate_sphere_inertia(sphere: Sphere, mass: float) -> InertiaTensor:
    """Calculate inertia tensor for a sphere.

    Args:
        sphere: Sphere geometry with radius
        mass: Total mass in kg

    Returns:
        Inertia tensor about center of mass
    """
    if mass < MIN_MASS_STABILITY_THRESHOLD:
        return _get_stability_fallback()

    return _calculate_sphere_inertia_cached(sphere.radius, mass)


def calculate_mesh_inertia_from_triangles(
    vertices: list[tuple[float, float, float]],
    triangles: list[tuple[int, int, int]],
    mass: float,
) -> InertiaTensor:
    """Calculate inertia tensor for a triangle mesh using the Mirtich algorithm.

    Based on: Brian Mirtich, "Fast and Accurate Computation of Polyhedral Mass Properties,"
    Journal of Graphics Tools, volume 1, number 2, pages 31-50, 1996.

    This implementation uses the Divergence Theorem to convert volume integrals into
    surface integrals across triangles. The calculation follows 4 phases:

    1. Validation: Ensures mesh topology and numerical integrity.
    2. Conditioning: Translates mesh to a local mean origin to preserve floating-point precision.
    3. Integration: Accumulates signed volume and moments across all tetrahedra.
    4. Normalization: Applies Parallel Axis Theorem and density scaling to produce
       the final tensor about the Center of Mass (CoM).

    Args:
        vertices: List of (x, y, z) vertex coordinates in meters
        triangles: List of (v0, v1, v2) triangle indices
        mass: Total mass in kg

    Returns:
        Inertia tensor about center of mass in kg·m²

    Raises:
        RobotPhysicsError: If mesh is non-manifold, zero-volume, or physically unstable
    """
    if mass < MIN_MASS_STABILITY_THRESHOLD:
        return _get_stability_fallback()

    # --- 1. Topology & Numerical Validation ---
    _validate_mesh_inputs(vertices, triangles)
    validate_mesh_topology(vertices, triangles, strict=False)

    # --- 2. Numerical Conditioning ---
    # Translate mesh to local mean origin to improve integration precision
    mean = [sum(v[i] for v in vertices) / len(vertices) for i in range(3)]
    vertices = [(v[0] - mean[0], v[1] - mean[1], v[2] - mean[2]) for v in vertices]

    # --- 3. Geometric Integration (Divergence Theorem) ---
    total_volume = 0.0
    abs_volume = 0.0
    weighted_com = [0.0, 0.0, 0.0]

    # Canonical inertia integrals (about local origin)
    i_xx = i_yy = i_zz = 0.0
    i_xy = i_xz = i_yz = 0.0

    for tri in triangles:
        a, b, c = vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]

        # Volume formula: V = (1/6) * (a dot (b cross c))
        det = (
            a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0])
        )
        tet_vol = det / 6.0

        # Skip purely numerical noise from slivers or degenerate triangles
        if abs(tet_vol) < DEGENERATE_VOL_THRESHOLD:
            continue

        total_volume += tet_vol
        abs_volume += abs(tet_vol)

        # Centroid of tetrahedron
        tet_com = [(a[i] + b[i] + c[i]) / 4.0 for i in range(3)]
        for i in range(3):
            weighted_com[i] += tet_vol * tet_com[i]

        # Integration coefficients
        c_x2, c_xy = tet_vol / 10.0, tet_vol / 20.0

        # Second moment integrals (x², y², z²) across tetrahedron
        x2 = c_x2 * (a[0] ** 2 + b[0] ** 2 + c[0] ** 2 + a[0] * b[0] + a[0] * c[0] + b[0] * c[0])
        y2 = c_x2 * (a[1] ** 2 + b[1] ** 2 + c[1] ** 2 + a[1] * b[1] + a[1] * c[1] + b[1] * c[1])
        z2 = c_x2 * (a[2] ** 2 + b[2] ** 2 + c[2] ** 2 + a[2] * b[2] + a[2] * c[2] + b[2] * c[2])

        # Product moment integrals (xy, xz, yz)
        xy = c_xy * (
            2 * a[0] * a[1]
            + 2 * b[0] * b[1]
            + 2 * c[0] * c[1]
            + a[0] * b[1]
            + a[0] * c[1]
            + b[0] * a[1]
            + b[0] * c[1]
            + c[0] * a[1]
            + c[0] * b[1]
        )
        xz = c_xy * (
            2 * a[0] * a[2]
            + 2 * b[0] * b[2]
            + 2 * c[0] * c[2]
            + a[0] * b[2]
            + a[0] * c[2]
            + b[0] * a[2]
            + b[0] * c[2]
            + c[0] * a[2]
            + c[0] * b[2]
        )
        yz = c_xy * (
            2 * a[1] * a[2]
            + 2 * b[1] * b[2]
            + 2 * c[1] * c[2]
            + a[1] * b[2]
            + a[1] * c[2]
            + b[1] * a[2]
            + b[1] * c[2]
            + c[1] * a[2]
            + c[1] * b[2]
        )

        # Accumulate origin-relative components
        i_xx += y2 + z2
        i_yy += x2 + z2
        i_zz += x2 + y2
        i_xy -= xy
        i_xz -= xz
        i_yz -= yz

    # --- 4. Normalization & Stability Checks ---
    if abs(total_volume) < DEGENERATE_VOL_THRESHOLD:
        raise RobotPhysicsError(
            ValidationErrorCode.INVALID_VALUE,
            "Degenerate mesh: Total volume is zero or near-zero.",
            target="MeshVolume",
            value=total_volume,
        )

    if total_volume < 0:
        raise RobotPhysicsError(
            ValidationErrorCode.PHYSICS_VIOLATION,
            "Mesh has inward or inconsistent winding (negative total volume).",
            target="MeshVolume",
            value=total_volume,
        )

    # Detect high-cancellation meshes (mixed winding/internal intersections)
    if abs_volume > 0 and abs(total_volume) / abs_volume < 0.5:
        raise RobotPhysicsError(
            ValidationErrorCode.PHYSICS_VIOLATION,
            "Mesh has inconsistent global winding (severe volume cancellation detected).",
            target="MeshVolume",
        )

    # Center of mass and density
    if not all(isfinite(w) for w in weighted_com):
        raise RobotPhysicsError(
            ValidationErrorCode.PHYSICS_VIOLATION,
            "Mesh weighted center of mass is non-finite.",
            target="MeshCOM",
        )

    cx, cy, cz = (w / total_volume for w in weighted_com)
    if not all(isfinite(c) for c in (cx, cy, cz)):
        raise RobotPhysicsError(
            ValidationErrorCode.PHYSICS_VIOLATION,
            "Computed center of mass is non-finite.",
            target="MeshCOM",
        )

    density = mass / total_volume

    i_xx, i_yy, i_zz = (i * density for i in (i_xx, i_yy, i_zz))
    i_xy, i_xz, i_yz = (i * density for i in (i_xy, i_xz, i_yz))

    # Shift to center of mass
    i_xx -= mass * (cy**2 + cz**2)
    i_yy -= mass * (cx**2 + cz**2)
    i_zz -= mass * (cx**2 + cy**2)
    i_xy += mass * cx * cy
    i_xz += mass * cx * cz
    i_yz += mass * cy * cz

    # 4. Physicality check
    # Check positive semi-definiteness using Sylvester's criterion (principal minors)
    # This avoids adding a heavy dependency on numpy just for an eigenvalue check.
    delta1 = i_xx
    delta2 = i_xx * i_yy - i_xy**2
    delta3 = (
        i_xx * (i_yy * i_zz - i_yz**2)
        - i_xy * (i_xy * i_zz - i_xz * i_yz)
        + i_xz * (i_xy * i_yz - i_yy * i_xz)
    )

    eps = 1e-9 * max(i_xx, i_yy, i_zz, 1.0)
    if delta1 < -eps or delta2 < -eps or delta3 < -eps:
        raise RobotPhysicsError(
            ValidationErrorCode.PHYSICS_VIOLATION,
            f"Inertia tensor is not positive semi-definite (fails Sylvester criterion). "
            f"Often indicates numerical corruption or extreme non-manifold shapes. "
            f"Minors: D1={delta1:.6f}, D2={delta2:.6f}, D3={delta3:.6f}",
            target="InertiaTensor",
            value=(delta1, delta2, delta3),
        )

    return InertiaTensor(
        ixx=max(i_xx, MIN_INERTIA_STABILITY_VALUE),
        ixy=i_xy,
        ixz=i_xz,
        iyy=max(i_yy, MIN_INERTIA_STABILITY_VALUE),
        iyz=i_yz,
        izz=max(i_zz, MIN_INERTIA_STABILITY_VALUE),
    )


def _validate_mesh_inputs(
    vertices: list[tuple[float, float, float]],
    triangles: list[tuple[int, int, int]],
) -> None:
    """Numerical and index validation."""
    if len(vertices) == 0 or len(triangles) == 0:
        raise RobotPhysicsError(ValidationErrorCode.VALUE_EMPTY, "Mesh is empty")

    for i, v in enumerate(vertices):
        if any(not isfinite(c) for c in v):
            raise RobotMathError(
                ValidationErrorCode.INVALID_VALUE,
                f"Vertex {i} contains non-finite value (NaN or Inf): {v}",
                target="Vertices",
                value=v,
            )

    n_verts = len(vertices)
    for i, tri in enumerate(triangles):
        if len(tri) != 3 or any(not (0 <= idx < n_verts) for idx in tri):
            raise RobotPhysicsError(ValidationErrorCode.OUT_OF_RANGE, f"Invalid triangle at {i}")


def calculate_mesh_inertia_approximation(mesh: Mesh, mass: float) -> InertiaTensor:
    """Calculate approximate (bounding box) inertia for a mesh.

    This is a lightweight fallback that treats the mesh as an axis-aligned
    bounding box based on its scale. It does not require triangle data.

    Args:
        mesh: Mesh geometry with scale
        mass: Total mass in kg

    Returns:
        Approximate inertia tensor using bounding box approximation
    """
    if mass < MIN_MASS_STABILITY_THRESHOLD:
        return _get_stability_fallback()

    return calculate_box_inertia(Box(size=mesh.scale), mass)


def calculate_inertia(geometry: Geometry, mass: float) -> InertiaTensor:
    """Unified wrapper for any geometry type."""
    if mass < MIN_MASS_STABILITY_THRESHOLD:
        return _get_stability_fallback()

    if isinstance(geometry, Box):
        return calculate_box_inertia(geometry, mass)
    if isinstance(geometry, Cylinder):
        return calculate_cylinder_inertia(geometry, mass)
    if isinstance(geometry, Sphere):
        return calculate_sphere_inertia(geometry, mass)
    if isinstance(geometry, Mesh):
        return calculate_mesh_inertia_approximation(geometry, mass)

    raise RobotPhysicsError(
        ValidationErrorCode.INVALID_VALUE,
        f"Unsupported geometry type: {type(geometry).__name__}",
        target="Geometry",
        value=type(geometry),
    )
