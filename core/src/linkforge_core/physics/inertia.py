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

logger = get_logger(__name__)

# Configurable cache size for inertia calculations
# Can be overridden via environment variable LINKFORGE_INERTIA_CACHE_SIZE
# Default: 512 (supports robots with 500+ links)
# For memory-constrained environments, set to 128 or less
DEFAULT_INERTIA_CACHE_SIZE = int(os.environ.get("LINKFORGE_INERTIA_CACHE_SIZE", "512"))


@lru_cache(maxsize=DEFAULT_INERTIA_CACHE_SIZE)
def _calculate_box_inertia_cached(x: float, y: float, z: float, mass: float) -> InertiaTensor:
    """Cached calculation of box inertia tensor.

    Args:
        x, y, z: Box dimensions
        mass: Mass in kg

    Returns:
        Inertia tensor about the center of mass
    """
    ixx = (1.0 / 12.0) * mass * (y * y + z * z)
    iyy = (1.0 / 12.0) * mass * (x * x + z * z)
    izz = (1.0 / 12.0) * mass * (x * x + y * y)
    return InertiaTensor(ixx=ixx, ixy=0.0, ixz=0.0, iyy=iyy, iyz=0.0, izz=izz)


def calculate_box_inertia(box: Box, mass: float) -> InertiaTensor:
    """Calculate inertia tensor for a box (rectangular cuboid).

    Args:
        box: Box geometry
        mass: Mass in kg

    Returns:
        Inertia tensor about the center of mass

    Formula for box with dimensions (x, y, z):
        Ixx = (1/12) * m * (y² + z²)
        Iyy = (1/12) * m * (x² + z²)
        Izz = (1/12) * m * (x² + y²)
        Ixy = Ixz = Iyz = 0 (off-diagonal terms are zero for symmetrical shapes)

    """
    if mass <= 0:
        return InertiaTensor.zero()

    return _calculate_box_inertia_cached(box.size.x, box.size.y, box.size.z, mass)


@lru_cache(maxsize=DEFAULT_INERTIA_CACHE_SIZE)
def _calculate_cylinder_inertia_cached(radius: float, length: float, mass: float) -> InertiaTensor:
    """Cached calculation of cylinder inertia tensor.

    Args:
        radius: Cylinder radius
        length: Cylinder length (height)
        mass: Mass in kg

    Returns:
        Inertia tensor about the center of mass
    """
    ixx = iyy = (1.0 / 12.0) * mass * (3 * radius * radius + length * length)
    izz = 0.5 * mass * radius * radius
    return InertiaTensor(ixx=ixx, ixy=0.0, ixz=0.0, iyy=iyy, iyz=0.0, izz=izz)


def calculate_cylinder_inertia(cylinder: Cylinder, mass: float) -> InertiaTensor:
    """Calculate inertia tensor for a cylinder (axis along Z).

    Args:
        cylinder: Cylinder geometry
        mass: Mass in kg

    Returns:
        Inertia tensor about the center of mass

    Formula for cylinder with radius r and height h (along Z):
        Ixx = Iyy = (1/12) * m * (3r² + h²)
        Izz = (1/2) * m * r²
        Ixy = Ixz = Iyz = 0

    """
    if mass <= 0:
        return InertiaTensor.zero()

    return _calculate_cylinder_inertia_cached(cylinder.radius, cylinder.length, mass)


@lru_cache(maxsize=DEFAULT_INERTIA_CACHE_SIZE)
def _calculate_sphere_inertia_cached(radius: float, mass: float) -> InertiaTensor:
    """Cached calculation of sphere inertia tensor.

    Args:
        radius: Sphere radius
        mass: Mass in kg

    Returns:
        Inertia tensor about the center of mass
    """
    i = (2.0 / 5.0) * mass * radius * radius
    return InertiaTensor(ixx=i, ixy=0.0, ixz=0.0, iyy=i, iyz=0.0, izz=i)


def calculate_sphere_inertia(sphere: Sphere, mass: float) -> InertiaTensor:
    """Calculate inertia tensor for a sphere.

    Args:
        sphere: Sphere geometry
        mass: Mass in kg

    Returns:
        Inertia tensor about the center of mass

    Formula for sphere with radius r:
        Ixx = Iyy = Izz = (2/5) * m * r²
        Ixy = Ixz = Iyz = 0

    """
    if mass <= 0:
        return InertiaTensor.zero()

    return _calculate_sphere_inertia_cached(sphere.radius, mass)


def calculate_mesh_inertia_from_triangles(
    vertices: list[tuple[float, float, float]],
    triangles: list[tuple[int, int, int]],
    mass: float,
) -> InertiaTensor:
    """Calculate inertia tensor for a triangle mesh using tetrahedralization.

    This implementation uses the **tetrahedral decomposition** method:
    each triangle (A, B, C) forms a signed tetrahedron with apex at the origin.
    The volume integrals are computed analytically per tetrahedron using
    the closed-form formulas from Mirtich (1996), then accumulated over all triangles.

    This is different from the projection integral approach in the original
    reference C code — both methods derive from the divergence theorem and
    produce identical results for valid meshes.

    Algorithm steps:
    1. Validate input consistency and numerical finiteness
    2. Decompose mesh into tetrahedra (origin + each triangle)
    3. Compute volume-weighted center of mass
    4. Integrate inertia tensor components about origin
    5. Apply parallel axis theorem to shift to center of mass

    Args:
        vertices: List of (x, y, z) vertex coordinates in meters
        triangles: List of (v0, v1, v2) triangle indices (into vertices list)
                  Indices must be valid (0 <= index < len(vertices))
        mass: Total mass in kg (must be positive)

    Returns:
        Inertia tensor about center of mass in kg⋅m²
        Returns zero tensor if mass <= 0 or mesh is degenerate

    Example:
        >>> # Simple cube vertices (1m sides)
        >>> vertices = [
        ...     (0, 0, 0),
        ...     (1, 0, 0),
        ...     (1, 1, 0),
        ...     (0, 1, 0),  # bottom
        ...     (0, 0, 1),
        ...     (1, 0, 1),
        ...     (1, 1, 1),
        ...     (0, 1, 1),  # top
        ... ]
        >>> # 12 triangles (2 per face, 6 faces)
        >>> triangles = [
        ...     (0, 1, 2),
        ...     (0, 2, 3),  # bottom
        ...     (4, 6, 5),
        ...     (4, 7, 6),  # top
        ...     # ... (sides omitted for brevity)
        ... ]
        >>> inertia = calculate_mesh_inertia_from_triangles(vertices, triangles, mass=1.0)
        >>> # Result will be approximately I = (1/6) for a 1kg 1m cube

    Note:
        - Mesh MUST be closed (watertight) with consistent outward-facing
          triangle winding for accurate results. This is guaranteed by Blender
          when using evaluated/triangulated meshes.
        - The signed volume method assumes consistent winding. If triangles
          are inconsistently wound, density and inertia integrals become
          mathematically inconsistent. The negative diagonal check below
          is the safety net for this case.

    """
    if mass <= 0:
        return InertiaTensor.zero()

    _validate_mesh_inputs(vertices, triangles)

    # Accumulators for volume-weighted properties
    total_volume = 0.0
    weighted_com = [0.0, 0.0, 0.0]

    # Canonical inertia integrals (about origin)
    # These represent: ∫∫∫ (coordinate products) dV
    i_xx = 0.0  # ∫∫∫ (y² + z²) dV
    i_yy = 0.0  # ∫∫∫ (x² + z²) dV
    i_zz = 0.0  # ∫∫∫ (x² + y²) dV
    i_xy = 0.0  # ∫∫∫ xy dV
    i_xz = 0.0  # ∫∫∫ xz dV
    i_yz = 0.0  # ∫∫∫ yz dV

    # Process each triangle as a tetrahedron with apex at origin
    for tri in triangles:
        # Get vertices
        a = vertices[tri[0]]
        b = vertices[tri[1]]
        c = vertices[tri[2]]

        # Compute signed volume of tetrahedron (origin, a, b, c)
        # Volume formula: V = (1/6) * det([a, b, c])
        det = (
            a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0])
        )
        tet_vol = det / 6.0
        total_volume += tet_vol

        # Centroid of tetrahedron (origin + a + b + c) / 4
        tet_com = [(a[i] + b[i] + c[i]) / 4.0 for i in range(3)]

        # Accumulate volume-weighted center of mass
        for i in range(3):
            weighted_com[i] += tet_vol * tet_com[i]

        # Canonical inertia tensor integrals for tetrahedron with one vertex at origin
        # Based on "Polyhedral Mass Properties" formulas
        # For a tetrahedron with vertices at origin, a, b, c:
        # ∫∫∫ x² dV = (V/10) * (ax² + bx² + cx² + ax·bx + ax·cx + bx·cx)

        ax, ay, az = a[0], a[1], a[2]
        bx, by, bz = b[0], b[1], b[2]
        cx, cy, cz = c[0], c[1], c[2]

        # Coefficients for integration
        # For a tetrahedron with one vertex at origin, the second moment integrals are:
        # ∫∫∫ x² dV = (V/10) * (sum of squared coords + pairwise products)
        # ∫∫∫ xy dV = (V/20) * (sum of pairwise products)
        coeff_x2 = tet_vol / 10.0
        coeff_xy = tet_vol / 20.0

        # Compute second moment integrals
        # ∫∫∫ x² dV
        x2 = coeff_x2 * (ax * ax + bx * bx + cx * cx + ax * bx + ax * cx + bx * cx)
        # ∫∫∫ y² dV
        y2 = coeff_x2 * (ay * ay + by * by + cy * cy + ay * by + ay * cy + by * cy)
        # ∫∫∫ z² dV
        z2 = coeff_x2 * (az * az + bz * bz + cz * cz + az * bz + az * cz + bz * cz)

        # Compute product moment integrals
        # ∫∫∫ xy dV
        xy = coeff_xy * (
            2 * ax * ay
            + 2 * bx * by
            + 2 * cx * cy
            + ax * by
            + ax * cy
            + bx * ay
            + bx * cy
            + cx * ay
            + cx * by
        )
        # ∫∫∫ xz dV
        xz = coeff_xy * (
            2 * ax * az
            + 2 * bx * bz
            + 2 * cx * cz
            + ax * bz
            + ax * cz
            + bx * az
            + bx * cz
            + cx * az
            + cx * bz
        )
        # ∫∫∫ yz dV
        yz = coeff_xy * (
            2 * ay * az
            + 2 * by * bz
            + 2 * cy * cz
            + ay * bz
            + ay * cz
            + by * az
            + by * cz
            + cy * az
            + cy * bz
        )

        # Accumulate inertia tensor components
        # I_xx = ∫∫∫ (y² + z²) dV
        i_xx += y2 + z2
        # I_yy = ∫∫∫ (x² + z²) dV
        i_yy += x2 + z2
        # I_zz = ∫∫∫ (x² + y²) dV
        i_zz += x2 + y2
        # I_xz = -∫∫∫ xz dV
        i_xz -= xz
        # I_yz = -∫∫∫ yz dV
        i_yz -= yz
        # I_xy = -∫∫∫ xy dV
        i_xy -= xy

    # Check for degenerate mesh (all triangles are coplanar or have zero area)
    if abs(total_volume) < 1e-12:
        raise RobotPhysicsError(
            ValidationErrorCode.PHYSICS_VIOLATION,
            "Degenerate mesh forms zero volume",
            target="TotalVolume",
            value=total_volume,
        )

    # Compute center of mass
    cx = weighted_com[0] / total_volume
    cy = weighted_com[1] / total_volume
    cz = weighted_com[2] / total_volume

    # Compute density from mass and volume
    density = mass / abs(total_volume)

    # Scale inertia by density
    i_xx *= density
    i_yy *= density
    i_zz *= density
    i_xy *= density
    i_xz *= density
    i_yz *= density

    # Apply parallel axis theorem to translate to center of mass
    # I_cm = I_origin - mass * (distance terms)
    i_xx -= mass * (cy * cy + cz * cz)
    i_yy -= mass * (cx * cx + cz * cz)
    i_zz -= mass * (cx * cx + cy * cy)
    i_xy += mass * cx * cy
    i_xz += mass * cx * cz
    i_yz += mass * cy * cz

    # Validate diagonal elements (must be positive for physical correctness)
    # Negative values indicate mesh winding issues or calculation errors
    negative_inertia_threshold = -1e-6
    if (
        i_xx < negative_inertia_threshold
        or i_yy < negative_inertia_threshold
        or i_zz < negative_inertia_threshold
    ):
        raise RobotPhysicsError(
            ValidationErrorCode.PHYSICS_VIOLATION,
            f"Negative diagonal inertia indicates incorrect mesh winding "
            f"or a non-manifold mesh: Ixx={i_xx:.6f}, Iyy={i_yy:.6f}, Izz={i_zz:.6f}",
            target="InertiaDiagonal",
            value=(i_xx, i_yy, i_zz),
        )

    return InertiaTensor(
        ixx=max(i_xx, 0.0),  # clamp tiny floating-point noise
        ixy=i_xy,
        ixz=i_xz,
        iyy=max(i_yy, 0.0),
        iyz=i_yz,
        izz=max(i_zz, 0.0),
    )


def _validate_mesh_inputs(
    vertices: list[tuple[float, float, float]],
    triangles: list[tuple[int, int, int]],
) -> None:
    """Validate mesh topology and numerical finiteness before computation."""
    if not vertices:
        raise RobotPhysicsError(
            ValidationErrorCode.VALUE_EMPTY,
            "Cannot calculate inertia for empty mesh vertices",
            target="Vertices",
            value=0,
        )

    if not triangles:
        raise RobotPhysicsError(
            ValidationErrorCode.VALUE_EMPTY,
            "Cannot calculate inertia for empty mesh triangles",
            target="Triangles",
            value=0,
        )

    for i, v in enumerate(vertices):
        if any(not isfinite(c) for c in v):
            raise RobotMathError(
                ValidationErrorCode.INVALID_VALUE,
                f"Vertex {i} contains non-finite value (NaN or Inf)",
                target="Vertices",
                value=v,
            )

    n_verts = len(vertices)
    for i, tri in enumerate(triangles):
        # Validate triangle indices
        if len(tri) != 3:
            raise RobotPhysicsError(
                ValidationErrorCode.INVALID_VALUE,
                f"Expected exactly 3 indices for triangle at index {i}",
                target="TriangleIndices",
                value=len(tri),
            )

        for idx in tri:
            if not (0 <= idx < n_verts):
                raise RobotPhysicsError(
                    ValidationErrorCode.OUT_OF_RANGE,
                    f"Triangle index {idx} at triangle {i} out of range (0-{n_verts - 1})",
                    target="TriangleIndex",
                    value=idx,
                )

        # Check for degenerate (zero-area) triangles
        v0, v1, v2 = vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
        # Edges
        e1 = (v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2])
        e2 = (v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2])
        # Cross product: magnitude squared = 4 * area squared
        cross_mag_sq = (
            (e1[1] * e2[2] - e1[2] * e2[1]) ** 2
            + (e1[2] * e2[0] - e1[0] * e2[2]) ** 2
            + (e1[0] * e2[1] - e1[1] * e2[0]) ** 2
        )
        if cross_mag_sq < 1e-24:
            logger.warning(
                f"Degenerate triangle at index {i} (zero area): {tri}. "
                f"This triangle contributes nothing and may indicate a mesh export error."
            )


def calculate_mesh_inertia(mesh: Mesh, mass: float) -> InertiaTensor:
    """Calculate approximate inertia tensor for a mesh.

    Args:
        mesh: Mesh geometry
        mass: Mass in kg

    Returns:
        Approximate inertia tensor

    Note:
        This function is for use in the core layer when mesh geometry is not available.
        In the Blender integration layer, use calculate_mesh_inertia_from_triangles()
        with actual mesh data for accurate results.

        For now, we approximate using bounding box (scaled by mesh.scale).

    Warning:
        This is a PLACEHOLDER approximation using a bounding box proxy.
        It is physically inaccurate for any non-box mesh shape.
        For accurate results, use calculate_mesh_inertia_from_triangles()
        with actual vertex and triangle data from the platform adapter.
        Inertia errors from this function can cause unstable simulation dynamics.

    """
    if mass <= 0:
        return InertiaTensor.zero()

    # Placeholder: return conservative estimate
    # In practice, this will be computed from actual mesh geometry in Blender
    # Default to a scaled unit cube for safety
    unit_box = Box(size=mesh.scale)
    return calculate_box_inertia(unit_box, mass)


def calculate_inertia(geometry: Geometry, mass: float) -> InertiaTensor:
    """Calculate inertia tensor for any geometry type.

    Args:
        geometry: Geometry (Box, Cylinder, Sphere, or Mesh)
        mass: Mass in kg

    Returns:
        Inertia tensor about the center of mass

    Raises:
        RobotPhysicsError: If geometry type is not supported

    """
    if mass <= 0:
        return InertiaTensor.zero()

    if isinstance(geometry, Box):
        return calculate_box_inertia(geometry, mass)
    elif isinstance(geometry, Cylinder):
        return calculate_cylinder_inertia(geometry, mass)
    elif isinstance(geometry, Sphere):
        return calculate_sphere_inertia(geometry, mass)
    elif isinstance(geometry, Mesh):
        return calculate_mesh_inertia(geometry, mass)
    else:
        raise RobotPhysicsError(
            ValidationErrorCode.INVALID_VALUE,
            f"Unsupported geometry type: {type(geometry)}",
            target="Geometry",
            value=type(geometry),
        )
