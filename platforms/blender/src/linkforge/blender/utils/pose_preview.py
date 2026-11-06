"""Utilities for non-destructive visual pose previewing and capturing in Blender."""

from __future__ import annotations

import contextlib
import math
from typing import Any

import bpy

try:
    from mathutils import Matrix, Vector
except ImportError:
    Matrix = None  # type: ignore[assignment,misc]
    Vector = None  # type: ignore[assignment,misc]

from ..constants import (
    AXIS_CUSTOM,
    AXIS_X,
    AXIS_Y,
    AXIS_Z,
)
from ..core._utils.math_utils import normalize_vector
from ..core.constants import (
    JOINT_CONTINUOUS,
    JOINT_PRISMATIC,
    JOINT_REVOLUTE,
)
from .property_helpers import get_joint_props, get_robot_props

REST_MATRIX_KEY = "_linkforge_rest_matrix"


def get_joint_axis_vector(jp: Any) -> tuple[float, float, float]:
    """Return normalized 3D axis vector for a robot joint in its local frame.

    Args:
        jp: JointPropertyGroup instance.

    Returns:
        tuple[float, float, float]: Unit axis vector (x, y, z).
    """
    axis = getattr(jp, "axis", AXIS_Z)
    if axis == AXIS_X:
        return (1.0, 0.0, 0.0)
    if axis == AXIS_Y:
        return (0.0, 1.0, 0.0)
    if axis == AXIS_Z:
        return (0.0, 0.0, 1.0)
    if axis == AXIS_CUSTOM:
        cx = float(getattr(jp, "custom_axis_x", 0.0))
        cy = float(getattr(jp, "custom_axis_y", 0.0))
        cz = float(getattr(jp, "custom_axis_z", 1.0))
        norm = normalize_vector(cx, cy, cz)
        return norm if any(norm) else (0.0, 0.0, 1.0)
    return (0.0, 0.0, 1.0)


def _matrix_to_rows(matrix: Any) -> list[list[float]]:
    """Convert Blender matrix or mock matrix to list of 4 float rows."""
    if matrix is None:
        return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    rows = []
    for i in range(4):
        row = []
        for j in range(4):
            val = float(matrix[i][j])
            row.append(val)
        rows.append(row)
    return rows


def _rows_to_matrix(rows: list[list[float]]) -> Any:
    """Convert list of 4 float rows to Matrix instance."""
    if Matrix is None:
        return None
    return Matrix(rows)


def cache_rest_transform(joint_obj: Any) -> Any:
    """Ensure the rest transform of a joint Empty is cached and return it.

    Args:
        joint_obj: Blender object representing the joint.

    Returns:
        Matrix instance of the rest local transform.
    """
    if joint_obj is None or Matrix is None:
        return None

    if REST_MATRIX_KEY in joint_obj:
        raw_rows = joint_obj[REST_MATRIX_KEY]
        # In Blender RNA custom properties, arrays or IDPropertyArray might be returned
        rows = [list(r) for r in raw_rows]
        return _rows_to_matrix(rows)

    local_m = getattr(joint_obj, "matrix_local", None)
    if local_m is None:
        local_m = Matrix.Identity(4)

    rows = _matrix_to_rows(local_m)
    joint_obj[REST_MATRIX_KEY] = rows
    return _rows_to_matrix(rows)


def get_rest_matrix(joint_obj: Any) -> Any:
    """Return the cached rest matrix for a joint object.

    Args:
        joint_obj: Blender object representing the joint.

    Returns:
        Matrix: The rest local transformation matrix.
    """
    return cache_rest_transform(joint_obj)


def apply_joint_value(joint_obj: Any, value: float, visited: set[str] | None = None) -> None:
    """Apply an angular or linear joint offset to a joint object in the 3D viewport.

    Args:
        joint_obj: Blender object representing the joint Empty.
        value: Angular displacement (radians) or linear displacement (meters).
        visited: Set of visited joint names to prevent infinite recursion with cyclic mimics.
    """
    if joint_obj is None or Matrix is None:
        return

    jp = get_joint_props(joint_obj)
    if not jp or not jp.is_robot_joint:
        return

    visited_set = visited if visited is not None else set()
    j_name = str(getattr(joint_obj, "name", ""))
    if j_name in visited_set:
        return
    visited_set.add(j_name)

    rest_m = get_rest_matrix(joint_obj)
    if rest_m is None:
        return

    j_type = str(getattr(jp, "joint_type", "")).lower()
    axis_vec = get_joint_axis_vector(jp)

    if j_type in (JOINT_REVOLUTE, JOINT_CONTINUOUS):
        # Local rotation around joint axis
        rot_m = Matrix.Rotation(float(value), 4, axis_vec)
        joint_obj.matrix_local = rest_m @ rot_m
    elif j_type == JOINT_PRISMATIC:
        # Local translation along joint axis
        disp_vec = (
            axis_vec[0] * float(value),
            axis_vec[1] * float(value),
            axis_vec[2] * float(value),
        )
        vec_obj = Vector(disp_vec) if Vector is not None else disp_vec
        trans_m = Matrix.Translation(vec_obj)
        joint_obj.matrix_local = rest_m @ trans_m

    # Handle mimic joints parented elsewhere that track this joint
    scene = getattr(bpy.context, "scene", None)
    if scene:
        for obj in getattr(scene, "objects", []):
            if obj == joint_obj:
                continue
            other_jp = get_joint_props(obj)
            if (
                other_jp
                and other_jp.is_robot_joint
                and getattr(other_jp, "use_mimic", False)
                and getattr(other_jp, "mimic_joint", None) == joint_obj
            ):
                mult = float(getattr(other_jp, "mimic_multiplier", 1.0))
                offset = float(getattr(other_jp, "mimic_offset", 0.0))
                mimic_val = value * mult + offset
                apply_joint_value(obj, mimic_val, visited=visited_set)


def restore_robot_rest_pose(scene: bpy.types.Scene | None = None) -> None:
    """Restore all articulated joints in the scene back to their pristine rest pose.

    Args:
        scene: The active Blender scene (defaults to bpy.context.scene).
    """
    if scene is None:
        scene = getattr(bpy.context, "scene", None)
    if not scene:
        return

    for obj in getattr(scene, "objects", []):
        if REST_MATRIX_KEY in obj:
            raw_rows = obj[REST_MATRIX_KEY]
            rows = [list(r) for r in raw_rows]
            rest_m = _rows_to_matrix(rows)
            if rest_m is not None:
                obj.matrix_local = rest_m
            with contextlib.suppress(Exception):
                del obj[REST_MATRIX_KEY]

    props = get_robot_props(scene)
    if props:
        with contextlib.suppress(Exception):
            props.preview_active_state_name = ""
        for state in getattr(props, "group_states", []):
            if getattr(state, "is_previewing", False):
                state.is_previewing = False

    view_layer = getattr(getattr(bpy.context, "view_layer", None), "update", None)
    if callable(view_layer):
        view_layer()


def apply_group_state_pose(_scene: bpy.types.Scene | None, state_item: Any) -> None:
    """Apply all joint values of a GroupStateProperty item to scene objects.

    Args:
        _scene: The active Blender scene.
        state_item: GroupStateProperty item containing joint_values.
    """
    if state_item is None:
        return

    for jv in getattr(state_item, "joint_values", []):
        j_obj = getattr(jv, "joint_obj", None)
        if j_obj:
            apply_joint_value(j_obj, float(jv.value))

    view_layer = getattr(getattr(bpy.context, "view_layer", None), "update", None)
    if callable(view_layer):
        view_layer()


def capture_scene_pose_to_state(_scene: bpy.types.Scene | None, state_item: Any) -> int:
    """Capture current joint transforms from 3D scene objects into a Group State.

    Args:
        _scene: The active Blender scene.
        state_item: Target GroupStateProperty item to receive joint values.

    Returns:
        int: Number of joints successfully captured.
    """
    if not state_item or Matrix is None:
        return 0

    captured_count = 0
    for jv in getattr(state_item, "joint_values", []):
        j_obj = getattr(jv, "joint_obj", None)
        if not j_obj:
            continue

        jp = get_joint_props(j_obj)
        if not jp or not jp.is_robot_joint:
            continue

        rest_m = get_rest_matrix(j_obj)
        if rest_m is None:
            continue

        curr_m = getattr(j_obj, "matrix_local", None)
        if curr_m is None:
            continue

        j_type = str(getattr(jp, "joint_type", "")).lower()
        axis_vec = get_joint_axis_vector(jp)

        try:
            delta_m = rest_m.inverted() @ curr_m
        except Exception:
            continue

        if j_type in (JOINT_REVOLUTE, JOINT_CONTINUOUS):
            axis = getattr(jp, "axis", AXIS_Z)
            if axis == AXIS_X:
                angle = math.atan2(delta_m[2][1], delta_m[1][1])
            elif axis == AXIS_Y:
                angle = math.atan2(delta_m[0][2], delta_m[2][2])
            elif axis == AXIS_Z:
                angle = math.atan2(delta_m[1][0], delta_m[0][0])
            else:
                # Custom axis: calculate trace and project onto axis vector
                t0 = delta_m[0][0]
                t1 = delta_m[1][1]
                t2 = delta_m[2][2]
                tr = (t0 + t1 + t2 - 1.0) / 2.0
                clamped_tr = max(-1.0, min(1.0, tr))
                angle_mag = math.acos(clamped_tr)
                # Compute sign using skew-symmetric parts
                rx = delta_m[2][1] - delta_m[1][2]
                ry = delta_m[0][2] - delta_m[2][0]
                rz = delta_m[1][0] - delta_m[0][1]
                sign_dot = rx * axis_vec[0] + ry * axis_vec[1] + rz * axis_vec[2]
                angle = angle_mag if sign_dot >= 0 else -angle_mag

            if j_type == JOINT_REVOLUTE:
                low = float(getattr(jp, "limit_lower", -math.pi))
                high = float(getattr(jp, "limit_upper", math.pi))
                if low <= high:
                    angle = max(low, min(high, angle))
            jv.value = angle
            captured_count += 1

        elif j_type == JOINT_PRISMATIC:
            dx = delta_m[0][3]
            dy = delta_m[1][3]
            dz = delta_m[2][3]
            disp = dx * axis_vec[0] + dy * axis_vec[1] + dz * axis_vec[2]
            low = float(getattr(jp, "limit_lower", -1.0))
            high = float(getattr(jp, "limit_upper", 1.0))
            if low <= high:
                disp = max(low, min(high, disp))
            jv.value = disp
            captured_count += 1

    return captured_count


def is_pose_preview_active(scene: bpy.types.Scene | None = None) -> bool:
    """Return True if any group state is currently previewing in the scene.

    Args:
        scene: Active Blender scene.

    Returns:
        bool: True if preview is active.
    """
    if scene is None:
        scene = getattr(bpy.context, "scene", None)
    if not scene:
        return False

    props = get_robot_props(scene)
    if not props:
        return False

    if getattr(props, "preview_active_state_name", ""):
        return True

    return any(getattr(s, "is_previewing", False) for s in getattr(props, "group_states", []))
