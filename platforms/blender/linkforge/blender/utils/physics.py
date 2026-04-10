"""Vectorized mesh physics for Blender platform."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np  # type: ignore[import-not-found]

    from ...linkforge_core.models.link import InertiaTensor
else:
    try:
        import numpy as np  # type: ignore[import-not-found]
    except ImportError:
        np = None

from ...linkforge_core.exceptions import RobotMathError, RobotPhysicsError, ValidationErrorCode
from ...linkforge_core.logging_config import get_logger
from ...linkforge_core.models.link import InertiaTensor
from ...linkforge_core.physics.inertia import (
    DEGENERATE_VOL_THRESHOLD,
    MIN_INERTIA_STABILITY_VALUE,
    MIN_MASS_STABILITY_THRESHOLD,
    NEGATIVE_INERTIA_THRESHOLD,
)

logger = get_logger(__name__)


def calculate_mesh_inertia_numpy(
    vertices: np.ndarray,
    triangles: np.ndarray,
    mass: float,
) -> InertiaTensor | None:
    """Calculate inertia tensor for a triangle mesh.

    Vectorized implementation of Mirtich (1996) algorithm using NumPy.

    Args:
        vertices: (N, 3) float array of vertex coordinates in meters.
        triangles: (M, 3) int array of vertex indices.
        mass: Total mass in kg.

    Returns:
        InertiaTensor about the center of mass, or None if calculation fails.
    """
    if np is None:
        logger.error("NumPy not found. Cannot perform vectorized inertia calculation.")
        return None

    # 1. Input validation matching Core standards
    if mass < MIN_MASS_STABILITY_THRESHOLD:
        val = MIN_INERTIA_STABILITY_VALUE
        return InertiaTensor(ixx=val, ixy=0.0, ixz=0.0, iyy=val, iyz=0.0, izz=val)

    if vertices.size == 0 or triangles.size == 0:
        return None

    if not np.isfinite(vertices).all():
        raise RobotMathError(
            ValidationErrorCode.INVALID_VALUE,
            "Mesh contains non-finite vertex coordinates (NaN or Inf).",
            target="Vertices",
            value=vertices,
        )

    # 2. Extract triangle geometry
    # Get vertices for all triangles
    try:
        a_verts = vertices[triangles[:, 0]]
        b_verts = vertices[triangles[:, 1]]
        c_verts = vertices[triangles[:, 2]]
    except IndexError as e:
        raise RobotPhysicsError(
            ValidationErrorCode.OUT_OF_RANGE,
            f"Triangle indices are out of range for vertex array: {e}",
            target="Triangles",
            value=triangles,
        ) from e

    # 3. Compute signed volumes of tetrahedra (origin, a_verts, b_verts, c_verts)
    # Using cross product for triple product: det(a_verts, b_verts, c_verts) = a_verts . (b_verts x c_verts)
    det = np.sum(a_verts * np.cross(b_verts, c_verts), axis=1)
    volumes = det / 6.0
    total_volume = np.sum(volumes)

    if abs(total_volume) < DEGENERATE_VOL_THRESHOLD:
        logger.warning(
            f"Degenerate mesh: Total volume {total_volume:.2e} is near zero. "
            "Inertia may be inaccurate."
        )
        return None

    # 4. Compute volume-weighted center of mass
    # Tetrahedron COM = (a_verts + b_verts + c_verts) / 4.0
    tets_com = (a_verts + b_verts + c_verts) / 4.0
    weighted_com = np.sum(volumes[:, np.newaxis] * tets_com, axis=0)
    com = weighted_com / total_volume

    # 5. Compute second moment integrals (integrals about origin)
    # Using symmetric form:
    # ∫∫∫ x² dV = (V/10) * (ax² + bx² + cx² + ax*bx + ax*cx + bx*cx)
    coeff = volumes / 10.0

    # Second moments: x2, y2, z2 integrals about origin
    x2_sum = np.sum(
        coeff
        * (
            a_verts[:, 0] ** 2
            + b_verts[:, 0] ** 2
            + c_verts[:, 0] ** 2
            + a_verts[:, 0] * b_verts[:, 0]
            + a_verts[:, 0] * c_verts[:, 0]
            + b_verts[:, 0] * c_verts[:, 0]
        )
    )
    y2_sum = np.sum(
        coeff
        * (
            a_verts[:, 1] ** 2
            + b_verts[:, 1] ** 2
            + c_verts[:, 1] ** 2
            + a_verts[:, 1] * b_verts[:, 1]
            + a_verts[:, 1] * c_verts[:, 1]
            + b_verts[:, 1] * c_verts[:, 1]
        )
    )
    z2_sum = np.sum(
        coeff
        * (
            a_verts[:, 2] ** 2
            + b_verts[:, 2] ** 2
            + c_verts[:, 2] ** 2
            + a_verts[:, 2] * b_verts[:, 2]
            + a_verts[:, 2] * c_verts[:, 2]
            + b_verts[:, 2] * c_verts[:, 2]
        )
    )

    # Product moments: xy, xz, yz integrals about origin
    # Integration formula for cross products in a tetrahedron
    xy_sum = (
        np.sum(
            coeff
            * (
                2 * a_verts[:, 0] * a_verts[:, 1]
                + 2 * b_verts[:, 0] * b_verts[:, 1]
                + 2 * c_verts[:, 0] * c_verts[:, 1]
                + a_verts[:, 0] * b_verts[:, 1]
                + a_verts[:, 0] * c_verts[:, 1]
                + b_verts[:, 0] * a_verts[:, 1]
                + b_verts[:, 0] * c_verts[:, 1]
                + c_verts[:, 0] * a_verts[:, 1]
                + c_verts[:, 0] * b_verts[:, 1]
            )
        )
        / 2.0
    )

    xz_sum = (
        np.sum(
            coeff
            * (
                2 * a_verts[:, 0] * a_verts[:, 2]
                + 2 * b_verts[:, 0] * b_verts[:, 2]
                + 2 * c_verts[:, 0] * c_verts[:, 2]
                + a_verts[:, 0] * b_verts[:, 2]
                + a_verts[:, 0] * c_verts[:, 2]
                + b_verts[:, 0] * a_verts[:, 2]
                + b_verts[:, 0] * c_verts[:, 2]
                + c_verts[:, 0] * a_verts[:, 2]
                + c_verts[:, 0] * b_verts[:, 2]
            )
        )
        / 2.0
    )

    yz_sum = (
        np.sum(
            coeff
            * (
                2 * a_verts[:, 1] * a_verts[:, 2]
                + 2 * b_verts[:, 1] * b_verts[:, 2]
                + 2 * c_verts[:, 1] * c_verts[:, 2]
                + a_verts[:, 1] * b_verts[:, 2]
                + a_verts[:, 1] * c_verts[:, 2]
                + b_verts[:, 1] * a_verts[:, 2]
                + b_verts[:, 1] * c_verts[:, 2]
                + c_verts[:, 1] * a_verts[:, 2]
                + c_verts[:, 1] * b_verts[:, 2]
            )
        )
        / 2.0
    )

    # 6. Final normalization and parallel axis theorem
    density = mass / abs(total_volume)

    # Second moments about origin
    i_xx_orig = density * (y2_sum + z2_sum)
    i_yy_orig = density * (x2_sum + z2_sum)
    i_zz_orig = density * (x2_sum + y2_sum)
    i_xy_orig = -density * xy_sum
    i_xz_orig = -density * xz_sum
    i_yz_orig = -density * yz_sum

    # Translate to center of mass
    cx, cy, cz = com
    i_xx = i_xx_orig - mass * (cy**2 + cz**2)
    i_yy = i_yy_orig - mass * (cx**2 + cz**2)
    i_zz = i_zz_orig - mass * (cx**2 + cy**2)
    i_xy = i_xy_orig + mass * cx * cy
    i_xz = i_xz_orig + mass * cx * cz
    i_yz = i_yz_orig + mass * cy * cz

    # 7. Physicality check
    if any(i < NEGATIVE_INERTIA_THRESHOLD for i in (i_xx, i_yy, i_zz)):
        raise RobotPhysicsError(
            ValidationErrorCode.PHYSICS_VIOLATION,
            f"Negative diagonal inertia (vectorized path) indicates incorrect mesh winding "
            f"or a non-manifold mesh: Ixx={i_xx:.6f}, Iyy={i_yy:.6f}, Izz={i_zz:.6f}",
            target="InertiaDiagonal",
            value=(i_xx, i_yy, i_zz),
        )

    # Return validated InertiaTensor
    return InertiaTensor(
        ixx=max(i_xx, 0.0),
        ixy=i_xy,
        ixz=i_xz,
        iyy=max(i_yy, 0.0),
        iyz=i_yz,
        izz=max(i_zz, 0.0),
    )
