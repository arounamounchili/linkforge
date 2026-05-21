"""Utilities for managing object transforms and parenting."""

from __future__ import annotations

from typing import Any

from linkforge.core import Transform, Vector3
from linkforge.core._utils.math_utils import clean_float

try:
    from mathutils import Matrix
except ImportError:
    Matrix = None  # type: ignore[assignment,misc]


def matrix_to_transform(matrix: Any) -> Transform:
    """Convert a Blender 4x4 matrix to a Core Transform.

    Args:
        matrix: Blender mathutils.Matrix (4x4)

    Returns:
        Core Transform with XYZ position and RPY rotation.
    """
    if matrix is None or Matrix is None:
        return Transform.identity()

    translation = matrix.to_translation()
    rotation = matrix.to_euler("XYZ")

    return Transform(
        xyz=Vector3(
            clean_float(translation.x),
            clean_float(translation.y),
            clean_float(translation.z),
        ),
        rpy=Vector3(
            clean_float(rotation.x),
            clean_float(rotation.y),
            clean_float(rotation.z),
        ),
    )


def set_parent_keep_transform(child_obj: Any, parent_obj: Any) -> None:
    """Set object parent while preserving its world transform (visual location/rotation).

    This matches standard Blender 'Object (Keep Transform)' behavior by setting
    matrix_parent_inverse to the inverse of the parent's world matrix.

    Args:
        child_obj: The Blender object to be parented
        parent_obj: The Blender object to become the parent
    """
    if not child_obj or not parent_obj:
        return

    # Store current world state
    pk_mw = child_obj.matrix_world.copy()

    # Apply parenting
    child_obj.parent = parent_obj

    # Set the inverse matrix to cancel out the parent's current world transform.
    # This effectively isolates the child from the parent's scale.
    child_obj.matrix_parent_inverse = parent_obj.matrix_world.inverted()

    # Restore world transform to ensure no drift
    child_obj.matrix_world = pk_mw


def clear_parent_keep_transform(child_obj: Any) -> None:
    """Clear object parent while preserving its world transform.

    Args:
        child_obj: The Blender object to unparent
    """
    if not child_obj:
        return

    # Store current world state
    pk_mw = child_obj.matrix_world.copy()

    # Remove parent
    child_obj.parent = None

    # Restore world transform (since parent is now None, World == Local)
    child_obj.matrix_world = pk_mw
