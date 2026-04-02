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

from ...linkforge_core.logging_config import get_logger
from ...linkforge_core.models.link import InertiaTensor

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

    if mass <= 0:
        return InertiaTensor.zero()

    if vertices.size == 0 or triangles.size == 0:
        return None

    # Get vertices for all triangles
    a_verts = vertices[triangles[:, 0]]
    b_verts = vertices[triangles[:, 1]]
    c_verts = vertices[triangles[:, 2]]

    # Compute signed volumes of tetrahedra (origin, a_verts, b_verts, c_verts)
    # Using cross product for triple product: det(a_verts, b_verts, c_verts) = a_verts . (b_verts x c_verts)
    det = np.sum(a_verts * np.cross(b_verts, c_verts), axis=1)
    volumes = det / 6.0
    total_volume = np.sum(volumes)

    if abs(total_volume) < 1e-10:
        logger.warning("Degenerate mesh: Total volume is near zero.")
        return None

    # Compute volume-weighted center of mass
    # Tetrahedron COM = (a_verts + b_verts + c_verts + o_verts) / 4 = (a_verts + b_verts + c_verts) / 4
    tets_com = (a_verts + b_verts + c_verts) / 4.0
    weighted_com = np.sum(volumes[:, np.newaxis] * tets_com, axis=0)
    com = weighted_com / total_volume

    # Compute second moment integrals (integrals about origin)
    # ∫∫∫ x² dV = (V/60) * (A_x² + B_x² + C_x² + (A_x+B_x+C_x)² )
    # But a common symmetric form is:
    # ∫∫∫ x² dV = (V/10) * (ax² + bx² + cx² + ax*bx + ax*cx + bx*cx)
    # where coeff = V / 10

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
    xy_sum = np.sum(
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
        / 2.0
    )

    xz_sum = np.sum(
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
        / 2.0
    )

    yz_sum = np.sum(
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
        / 2.0
    )

    # Compute density
    density = mass / total_volume

    # Second moments about origin
    i_xx_orig = density * (y2_sum + z2_sum)
    i_yy_orig = density * (x2_sum + z2_sum)
    i_zz_orig = density * (x2_sum + y2_sum)
    i_xy_orig = -density * xy_sum
    i_xz_orig = -density * xz_sum
    i_yz_orig = -density * yz_sum

    # Translate to center of mass (Parallel Axis Theorem)
    cx, cy, cz = com
    i_xx = i_xx_orig - mass * (cy**2 + cz**2)
    i_yy = i_yy_orig - mass * (cx**2 + cz**2)
    i_zz = i_zz_orig - mass * (cx**2 + cy**2)
    i_xy = i_xy_orig + mass * cx * cy
    i_xz = i_xz_orig + mass * cx * cz
    i_yz = i_yz_orig + mass * cy * cz

    # Return validated InertiaTensor
    return InertiaTensor(
        ixx=abs(i_xx),
        ixy=i_xy,
        ixz=i_xz,
        iyy=abs(i_yy),
        iyz=i_yz,
        izz=abs(i_zz),
    )
